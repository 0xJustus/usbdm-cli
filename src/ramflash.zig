//! USBDM RAM flash-routine engine: download a routine into target RAM and run it
//! on the target CPU. ABI from usbdm-flash-routines / eclipse-makefiles-build;
//! target multi-byte fields are BIG-ENDIAN.

const std = @import("std");
const hexfile = @import("hexfile.zig");

/// Param area (`program` offset, <=255) + one data chunk (<=256); 512 fits both.
pub const out_buf_size: usize = 512;

/// SmallTargetImageHeader.flags bits (byte 0 of a small-code image).
pub const OPT_SMALL_CODE: u8 = 0x80;
pub const OPT_PAGED_ADDRESSES: u8 = 0x40;

/// BDCSCR.BDMACT: set when the target self-halts (BGND).
pub const BDCSCR_BDMACT: u8 = 0x40;

/// FlashDriverError_t: routine writes it (BE u16) at the header addr on exit; 0 = ok.
pub const DriverError = enum(u16) {
    ok = 0,
    locked = 1,
    illegal_params = 2,
    prog_failed = 3,
    prog_wprot = 4,
    verify_failed = 5,
    erase_failed = 6,
    trap = 7,
    prog_accerr = 8,
    prog_fpviol = 9,
    prog_mgstat0 = 10,
    clkdiv = 11,
    illegal_security = 12,
    unknown = 13,
    timeout = 14, // host-synthesized on poll timeout
    _,

    pub fn name(self: DriverError) []const u8 {
        return switch (self) {
            .ok => "OK",
            .locked => "device secured/locked",
            .illegal_params => "illegal parameters",
            .prog_failed => "program failed",
            .prog_wprot => "write-protected",
            .verify_failed => "verify failed",
            .erase_failed => "erase/blank-check failed",
            .trap => "target routine trapped",
            .prog_accerr => "flash access error",
            .prog_fpviol => "flash protection violation",
            .prog_mgstat0 => "flash MGSTAT0 error",
            .clkdiv => "invalid flash clock divider",
            .illegal_security => "illegal security setting",
            .timeout => "routine timed out",
            else => "unknown routine error",
        };
    }
};

pub const Op = enum { program, block_erase, blank_check, selective_erase, verify };

pub const Error = error{
    NotSmallCode,
    BadImage,
    NoRamForData, // maxDataSize < 40
    Timeout,
    RoutineError, // non-OK DriverError; see last_error
} || anyerror;

/// Parsed HCS08 small-code routine blob.
pub const Routine = struct {
    load_address: u32,
    /// Per-op byte offsets into the image (SmallTargetImageHeader).
    program: u8,
    mass_erase: u8,
    blank_check: u8,
    selective_erase: u8,
    verify: u8,
    flags: u8,
    image: []const u8,

    /// [start,end) image offsets of `op`'s code slice. Zero end -> end-of-image;
    /// zero start -> op absent/unsupported (see `hasOp`). Ref loadSmallTargetProgram.
    pub fn slice(self: Routine, op: Op) struct { start: u8, end: u8 } {
        const size: u8 = @intCast(self.image.len);
        const raw_end: u8 = switch (op) {
            .program => self.mass_erase,
            .block_erase => self.blank_check,
            .blank_check => self.selective_erase,
            .selective_erase => self.verify,
            .verify => size,
        };
        const start: u8 = switch (op) {
            .program => self.program,
            .block_erase => self.mass_erase,
            .blank_check => self.blank_check,
            .selective_erase => self.selective_erase,
            .verify => self.verify,
        };
        return .{ .start = start, .end = if (raw_end == 0) size else raw_end };
    }

    /// Routine for `op` present (start offset != 0).
    pub fn hasOp(self: Routine, op: Op) bool {
        return self.slice(op).start != 0;
    }
};

/// Parse a small-code routine from an S-record blob (6-byte SmallTargetImageHeader at front).
pub fn parseSmall(gpa: std.mem.Allocator, srec: []const u8) Error!struct { routine: Routine, image_owned: []u8 } {
    var img = try hexfile.parse(gpa, srec);
    errdefer img.deinit(gpa);
    // Code = lowest-address segment; ignore the reset-vector sliver at 0xFFFE.
    var code: ?hexfile.Segment = null;
    for (img.segments) |seg| {
        if (seg.addr < 0x8000) { // low RAM, not vector space
            if (code == null or seg.addr < code.?.addr) code = seg;
        }
    }
    const c = code orelse return error.BadImage;
    if (c.data.len < 6) return error.BadImage;
    if (c.data[0] & OPT_SMALL_CODE == 0) return error.NotSmallCode;

    // Own a copy: outlives the parsed hexfile.Image.
    const owned = try gpa.dupe(u8, c.data);
    errdefer gpa.free(owned);
    const load_address = c.addr;
    img.deinit(gpa);

    return .{
        .image_owned = owned,
        .routine = .{
            .load_address = load_address,
            .flags = owned[0],
            .program = owned[1],
            .mass_erase = owned[2],
            .blank_check = owned[3],
            .selective_erase = owned[4],
            .verify = owned[5],
            .image = owned,
        },
    };
}

/// Live target access (BDM session in production, mock in tests).
pub const Target = struct {
    ctx: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        writeMem: *const fn (ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void,
        readMem: *const fn (ctx: *anyopaque, addr: u32, buf: []u8) anyerror!void,
        writePc: *const fn (ctx: *anyopaque, pc: u32) anyerror!void,
        go: *const fn (ctx: *anyopaque) anyerror!void,
        halt: *const fn (ctx: *anyopaque) anyerror!void,
        /// Read BDCSCR (BDM status/control register).
        readStatus: *const fn (ctx: *anyopaque) anyerror!u8,
        sleepMs: *const fn (ctx: *anyopaque, ms: u32) void,
        /// Re-establish BDM sync after the routine ran (it can disturb the BDM clock). Best-effort.
        reconnect: *const fn (ctx: *anyopaque) anyerror!void,
    };
};

pub const Params = struct {
    ram_start: u32,
    ram_end: u32, // inclusive
    /// Flash controller register base. u32 for CFV1 (high, e.g. 0xFF9820); HCS08/HCS12 truncate to u16.
    controller: u32,
    sector_size: u16,
    frequency_khz: u16, // target bus clock
    /// Watchdog/COP control-register addr. CFV1 SOPT 0xFF9802; HCS12 copctl 0x003C (u16).
    watchdog_addr: u32 = 0,
    /// Erase/program alignment in bytes.
    alignment: u32 = 1,
    poll_iterations: u32 = 400, // ~4s at 10ms
};

pub const Driver = struct {
    target: Target,
    routine: Routine,
    params: Params,
    last_error: DriverError = .ok,

    /// Data-buffer capacity: codeLoad - dataAddr (code at top of RAM, params+data at bottom).
    fn maxDataSize(self: *const Driver, op: Op) Error!u32 {
        const s = self.routine.slice(op);
        if (s.end <= s.start) return error.BadImage;
        const code_len: u32 = @as(u32, s.end) - s.start;
        const code_load = self.params.ram_end - code_len + 1;
        const data_addr = self.routine.load_address + self.routine.program;
        if (code_load <= data_addr) return error.NoRamForData;
        const max = code_load - data_addr;
        if (max < 40) return error.NoRamForData;
        return max;
    }

    /// Run one op; callers chunk. `data` = program/verify buffer (empty for
    /// erase/blank-check); `region_size` = span for blank-check/selective-erase.
    fn runOnce(self: *Driver, op: Op, flash_addr: u32, data: []const u8, region_size: u32) Error!void {
        const t = self.target;
        const r = self.routine;
        const s = r.slice(op);
        if (s.end <= s.start) return error.BadImage;
        const code_len: u32 = @as(u32, s.end) - s.start;
        const code_load = self.params.ram_end - code_len + 1;
        const entry = code_load; // small-code entry = start of loaded slice
        const header_addr = r.load_address; // param block at image base
        const data_offset: u32 = r.program; // param area = `program` bytes
        const data_addr = header_addr + data_offset;

        try t.vtable.writeMem(t.ctx, code_load, r.image[s.start..s.end]);

        // SmallTargetFlashDataHeader: 11 bytes; param area = data_offset >= 11.
        if (data_offset < 11) return error.BadImage;
        if (@as(usize, data_offset) + data.len > out_buf_size) return error.BadImage;

        var hdr: [11]u8 = undefined;
        std.mem.writeInt(u16, hdr[0..2], @truncate(flash_addr), .big); // flashAddress (BE)
        std.mem.writeInt(u16, hdr[2..4], @truncate(self.params.controller), .big); // controller (BE)
        switch (op) {
            .program, .verify => {
                std.mem.writeInt(u16, hdr[4..6], @truncate(data_addr), .big); // dataAddress
                std.mem.writeInt(u16, hdr[6..8], @intCast(data.len), .big); // dataSize
            },
            .blank_check => {
                std.mem.writeInt(u16, hdr[4..6], @truncate(data_addr), .big); // dataAddress
                std.mem.writeInt(u16, hdr[6..8], @intCast(region_size), .big); // bytes to check
            },
            .selective_erase => {
                const ss = self.params.sector_size;
                const first = flash_addr / ss;
                const last = (flash_addr + region_size - 1) / ss;
                std.mem.writeInt(u16, hdr[4..6], ss, .big); // sectorSize
                std.mem.writeInt(u16, hdr[6..8], @intCast(last - first + 1), .big); // sectorCount
            },
            .block_erase => {
                std.mem.writeInt(u16, hdr[4..6], 0, .big);
                std.mem.writeInt(u16, hdr[6..8], 0, .big);
            },
        }
        // page_wdog_Address + pageNum (0 unless paged/wdog)
        std.mem.writeInt(u16, hdr[8..10], @truncate(self.params.watchdog_addr), .big);
        hdr[10] = @truncate(flash_addr >> 16);

        var out: [out_buf_size]u8 = undefined;
        const total = @as(usize, data_offset) + data.len;
        @memset(out[0..data_offset], 0);
        @memcpy(out[0..11], &hdr);
        @memcpy(out[data_offset..][0..data.len], data);
        try t.vtable.writeMem(t.ctx, header_addr, out[0..total]);

        try t.vtable.writePc(t.ctx, entry);
        try t.vtable.go(t.ctx);

        const halted = try pollUntilHalted(t, self.params.poll_iterations, BDCSCR_BDMACT);

        // result: DriverError, BE u16 @ header_addr
        var res: [2]u8 = undefined;
        try t.vtable.readMem(t.ctx, header_addr, &res);
        // timed-out routine wrote no result -> force timeout
        var err = std.mem.readInt(u16, &res, .big);
        if (!halted) err = @intFromEnum(DriverError.timeout);
        self.last_error = @enumFromInt(err);
        if (self.last_error != .ok) return error.RoutineError;
    }

    /// Program `data` at `flash_addr`, chunked.
    pub fn program(self: *Driver, flash_addr: u32, data: []const u8) Error!void {
        const max = try self.maxDataSize(.program);
        const chunk = @min(max, 256); // engine buffer cap
        var off: usize = 0;
        while (off < data.len) {
            const n = @min(data.len - off, chunk);
            try self.runOnce(.program, flash_addr + @as(u32, @intCast(off)), data[off..][0..n], 0);
            off += n;
        }
    }

    /// Mass-erase the whole flash block. `flash_addr` must be a real FLASH address,
    /// not the RAM base: HCS08 latches the command by writing to a flash location.
    pub fn massErase(self: *Driver, flash_addr: u32) Error!void {
        try self.runOnce(.block_erase, flash_addr, &.{}, 0);
    }
    pub fn eraseRange(self: *Driver, flash_addr: u32, size: u32) Error!void {
        try self.runOnce(.selective_erase, flash_addr, &.{}, size);
    }
    /// Blank-check `size` bytes at `flash_addr` (routine reads flash directly; no data buffer).
    pub fn blankCheck(self: *Driver, flash_addr: u32, size: u32) Error!void {
        try self.runOnce(.blank_check, flash_addr, &.{}, size);
    }
    pub fn verify(self: *Driver, flash_addr: u32, data: []const u8) Error!void {
        // Some HCS08 images have no verify routine (offset 0); fall back to
        // BDM readback-compare (blob-independent).
        if (!self.routine.hasOp(.verify)) return self.verifyReadback(flash_addr, data);
        const max = try self.maxDataSize(.verify);
        const chunk = @min(max, 256);
        var off: usize = 0;
        while (off < data.len) {
            const n = @min(data.len - off, chunk);
            try self.runOnce(.verify, flash_addr + @as(u32, @intCast(off)), data[off..][0..n], 0);
            off += n;
        }
    }

    fn verifyReadback(self: *Driver, flash_addr: u32, data: []const u8) Error!void {
        var buf: [256]u8 = undefined;
        var off: usize = 0;
        while (off < data.len) {
            const n = @min(data.len - off, buf.len);
            try self.target.vtable.readMem(self.target.ctx, flash_addr + @as(u32, @intCast(off)), buf[0..n]);
            if (!std.mem.eql(u8, buf[0..n], data[off..][0..n])) {
                self.last_error = .verify_failed;
                return error.RoutineError;
            }
            off += n;
        }
    }
};

// Large-code path (HCS12; CFV1 uses this shape with 32-bit fields). Op selected
// by the DO_* opcode in LargeTargetFlashDataHeader.flags (small code: by entry offset).

/// LargeTargetImageHeader.capabilities bits.
pub const CAP_RELOCATABLE: u16 = 1 << 15;
pub const CAP_DATA_FIXED: u16 = 1 << 12;
/// Op opcodes for LargeTargetFlashDataHeader.flags.
pub const DO_INIT_FLASH: u16 = 1 << 0;
pub const DO_ERASE_BLOCK: u16 = 1 << 1;
pub const DO_ERASE_RANGE: u16 = 1 << 2;
pub const DO_BLANK_CHECK_RANGE: u16 = 1 << 3;
pub const DO_PROGRAM_RANGE: u16 = 1 << 4;
pub const DO_VERIFY_RANGE: u16 = 1 << 5;
pub const IS_COMPLETE: u16 = 1 << 15;

/// sizeof(LargeTargetFlashDataHeader).
const LARGE_HEADER_SIZE: u32 = 18;
/// program = blank-check + program + verify in one pass (ref programOperation).
const LARGE_PROGRAM_OP: u16 = DO_BLANK_CHECK_RANGE | DO_PROGRAM_RANGE | DO_VERIFY_RANGE;

/// Poll `readStatus` every 10 ms until `mask` shows self-halt, then halt+reconnect.
/// Sleeps before the first read: at `go` the target still reads as halted, so an
/// immediate read could look like completion (ref do-while).
fn pollUntilHalted(t: Target, iters: u32, mask: u8) anyerror!bool {
    var halted = false;
    var i: u32 = 0;
    while (i < iters) : (i += 1) {
        t.vtable.sleepMs(t.ctx, 10);
        if ((try t.vtable.readStatus(t.ctx)) & mask != 0) {
            halted = true;
            break;
        }
    }
    t.vtable.halt(t.ctx) catch {};
    t.vtable.reconnect(t.ctx) catch {};
    return halted;
}

/// Parsed 16-bit large-code routine (HCS12). 14-byte LargeTargetImageHeader; see parseLarge for layout.
pub const LargeRoutine = struct {
    image: []u8, // mutable: loadAddress/flashData/copctlAddress patched pre-download
    image_address: u32, // == loadAddress in the blob
    entry: u32,
    capabilities: u16,
    flash_data: u32, // flashData; data-header addr when CAP_DATA_FIXED
    header_offset: usize, // offset of the 14-byte image header within `image`
};

/// Parse a 16-bit large-code routine (HCS12). Free via `gpa.free(routine.image)`.
pub fn parseLarge(gpa: std.mem.Allocator, srec: []const u8) Error!LargeRoutine {
    var img = try hexfile.parse(gpa, srec);
    errdefer img.deinit(gpa);
    // code = lowest-address segment
    var code: ?hexfile.Segment = null;
    for (img.segments) |seg| {
        if (seg.addr < 0x8000 and (code == null or seg.addr < code.?.addr)) code = seg;
    }
    const c = code orelse return error.BadImage;
    if (c.data.len < 16) return error.BadImage;
    const image_address = c.addr;
    const header_address = std.mem.readInt(u16, c.data[0..2], .big);
    if (header_address < image_address) return error.BadImage;
    const off: usize = header_address - image_address;
    if (off + 14 > c.data.len) return error.BadImage;
    const owned = try gpa.dupe(u8, c.data);
    errdefer gpa.free(owned);
    // LargeTargetImageHeader: loadAddress@0 entry@2 capabilities@4
    // copctlAddress@6 calibFactor@8(u32) flashData@12
    const entry = std.mem.readInt(u16, owned[off + 2 ..][0..2], .big);
    const capabilities = std.mem.readInt(u16, owned[off + 4 ..][0..2], .big);
    const flash_data = std.mem.readInt(u16, owned[off + 12 ..][0..2], .big);
    img.deinit(gpa);
    return .{
        .image = owned,
        .image_address = image_address,
        .entry = entry,
        .capabilities = capabilities,
        .flash_data = flash_data,
        .header_offset = off,
    };
}

pub const LargeDriver = struct {
    target: Target,
    routine: LargeRoutine,
    params: Params,
    last_error: DriverError = .ok,
    code_loaded: bool = false,

    const Layout = struct { code_load: u32, entry: u32, data_header: u32, data_addr: u32, max_data: u32 };

    /// RAM placement (relocation-aware); ref loadLargeTargetProgram.
    fn layout(self: *const LargeDriver) Layout {
        const cap = self.routine.capabilities;
        const relocatable = cap & CAP_RELOCATABLE != 0;
        const code_load: u32 = if (relocatable) (self.params.ram_start + 3) & ~@as(u32, 3) else self.routine.image_address;
        const delta = code_load -% self.routine.image_address;
        const image_size: u32 = @intCast(self.routine.image.len);
        // data header after code unless CAP_DATA_FIXED
        const data_header: u32 = if (cap & CAP_DATA_FIXED != 0) self.routine.flash_data else code_load + image_size;
        const data_addr = data_header + LARGE_HEADER_SIZE;
        const max_data = if (self.params.ram_end >= data_addr) self.params.ram_end - data_addr + 1 else 0;
        return .{ .code_load = code_load, .entry = self.routine.entry +% delta, .data_header = data_header, .data_addr = data_addr, .max_data = max_data };
    }

    /// Patch header fields, download code once (reused by later ops).
    fn loadCode(self: *LargeDriver) Error!void {
        if (self.code_loaded) return;
        const l = self.layout();
        const off = self.routine.header_offset;
        std.mem.writeInt(u16, self.routine.image[off..][0..2], @truncate(l.code_load), .big); // loadAddress
        std.mem.writeInt(u16, self.routine.image[off + 6 ..][0..2], @truncate(self.params.watchdog_addr), .big); // copctlAddress
        std.mem.writeInt(u16, self.routine.image[off + 12 ..][0..2], @truncate(l.data_header), .big); // flashData
        try self.target.vtable.writeMem(self.target.ctx, l.code_load, self.routine.image);
        self.code_loaded = true;
    }

    /// Marshal the 18-byte LargeTargetFlashDataHeader + data, run, read ResultStruct { flags@0, errorCode@2 }.
    fn runOnce(self: *LargeDriver, op_flags: u16, flash_addr: u32, size: u32, data: []const u8) Error!void {
        const t = self.target;
        const l = self.layout();
        if (LARGE_HEADER_SIZE + data.len > LARGE_HEADER_SIZE + out_buf_size) return error.BadImage;
        var buf: [LARGE_HEADER_SIZE + out_buf_size]u8 = undefined;
        @memset(buf[0..LARGE_HEADER_SIZE], 0);
        // LargeTargetFlashDataHeader: flags@0 errorCode@2 controller@4
        // frequency@6 sectorSize@8 address@10(u32) dataSize@14 dataAddress@16
        std.mem.writeInt(u16, buf[0..2], DO_INIT_FLASH | op_flags, .big);
        std.mem.writeInt(u16, buf[2..4], 0xFFFF, .big); // errorCode sentinel (-1)
        std.mem.writeInt(u16, buf[4..6], @truncate(self.params.controller), .big);
        std.mem.writeInt(u16, buf[6..8], self.params.frequency_khz, .big);
        std.mem.writeInt(u16, buf[8..10], self.params.sector_size, .big);
        std.mem.writeInt(u32, buf[10..14], flash_addr, .big);
        std.mem.writeInt(u16, buf[14..16], @intCast(size), .big);
        std.mem.writeInt(u16, buf[16..18], @truncate(l.data_addr), .big);
        @memcpy(buf[LARGE_HEADER_SIZE..][0..data.len], data);
        try t.vtable.writeMem(t.ctx, l.data_header, buf[0 .. LARGE_HEADER_SIZE + data.len]);

        try t.vtable.writePc(t.ctx, l.entry);
        try t.vtable.go(t.ctx);

        const halted = try pollUntilHalted(t, self.params.poll_iterations, BDCSCR_BDMACT);

        var res: [4]u8 = undefined;
        try t.vtable.readMem(t.ctx, l.data_header, &res);
        const flags = std.mem.readInt(u16, res[0..2], .big);
        var err = std.mem.readInt(u16, res[2..4], .big);
        // timeout: sentinel never overwritten -> force timeout
        if (!halted) err = @intFromEnum(DriverError.timeout);
        // success requires IS_COMPLETE set (action bits cleared)
        if (err == 0 and flags != IS_COMPLETE) err = @intFromEnum(DriverError.unknown);
        self.last_error = @enumFromInt(err);
        if (self.last_error != .ok) return error.RoutineError;
    }

    fn chunkSize(self: *const LargeDriver) Error!u32 {
        const n = @min(@min(self.layout().max_data, out_buf_size), 256);
        if (n == 0) return error.NoRamForData;
        return @intCast(n);
    }

    /// Align `flash_addr` down and 0xFF-pad head/tail so misaligned/odd segments
    /// form valid aligned program/verify units (ref doFlashBlock; 0xFF is a no-op on erased flash).
    fn runPadded(self: *LargeDriver, op: u16, flash_addr: u32, data: []const u8) Error!void {
        const alignb: u32 = @max(self.params.alignment, 1);
        const mask: u32 = alignb - 1;
        const start = flash_addr & ~mask;
        const data_end: u32 = flash_addr + @as(u32, @intCast(data.len));
        const total: u32 = (((data_end + mask) & ~mask)) - start; // aligned end - aligned start
        const chunk = @max((try self.chunkSize()) & ~mask, alignb);
        var buf: [256]u8 = undefined;
        var off: u32 = 0;
        while (off < total) {
            const n = @min(total - off, chunk);
            var k: u32 = 0;
            while (k < n) : (k += 1) {
                const abs = start + off + k;
                buf[k] = if (abs >= flash_addr and abs < data_end) data[abs - flash_addr] else 0xFF;
            }
            try self.runOnce(op, start + off, n, buf[0..n]);
            off += n;
        }
    }

    /// Program: blank-check + program + verify per chunk.
    pub fn program(self: *LargeDriver, flash_addr: u32, data: []const u8) Error!void {
        try self.loadCode();
        try self.runPadded(LARGE_PROGRAM_OP, flash_addr, data);
    }
    pub fn verify(self: *LargeDriver, flash_addr: u32, data: []const u8) Error!void {
        try self.loadCode();
        try self.runPadded(DO_VERIFY_RANGE, flash_addr, data);
    }
    pub fn massErase(self: *LargeDriver, flash_addr: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_ERASE_BLOCK, flash_addr, 0, &.{});
    }
    pub fn eraseRange(self: *LargeDriver, flash_addr: u32, size: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_ERASE_RANGE, flash_addr, size, &.{});
    }
    pub fn blankCheck(self: *LargeDriver, flash_addr: u32, size: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_BLANK_CHECK_RANGE, flash_addr, size, &.{});
    }
};

// CFV1 (ColdFire V1): 32-bit variant of the large-code ABI (all-u32 header,
// different field order; completion via XCSR RUNSTATE). Ref FlashProgrammer_CFV1.cpp.

/// CFV1 capability bits (different positions from HCS12).
pub const CAP_CFV1_RELOCATABLE: u32 = 1 << 31;
pub const CAP_CFV1_DATA_FIXED: u32 = 1 << 12;
/// XCSR run-state: HALT 0x80, STOP 0x40 (self-halt/stop).
pub const CFV1_XCSR_RUNSTATE: u8 = 0xC0;
/// Completion sentinel = bit 31 of the 32-bit flags (HCS12 uses bit 15 of 16-bit).
pub const CFV1_IS_COMPLETE: u32 = 1 << 31;
/// sizeof(CFV1 LargeTargetFlashDataHeader).
const CFV1_HEADER_SIZE: u32 = 32;

pub const Cfv1Routine = struct {
    image: []u8, // mutable: loadAddress/flashData patched pre-download
    image_address: u32,
    entry: u32,
    capabilities: u32,
    flash_data: u32,
    header_offset: usize, // offset of the 16-byte image header within `image`
};

/// Parse a 32-bit CFV1 large-code routine. Free via `gpa.free(routine.image)`.
pub fn parseCfv1(gpa: std.mem.Allocator, srec: []const u8) Error!Cfv1Routine {
    var img = try hexfile.parse(gpa, srec);
    errdefer img.deinit(gpa);
    // code = lowest-address segment; no 0x8000 filter (CFV1 RAM is high, ~0x00800000).
    var code: ?hexfile.Segment = null;
    for (img.segments) |seg| {
        if (code == null or seg.addr < code.?.addr) code = seg;
    }
    const c = code orelse return error.BadImage;
    if (c.data.len < 20) return error.BadImage;
    const image_address = c.addr;
    const header_address = std.mem.readInt(u32, c.data[0..4], .big);
    if (header_address < image_address) return error.BadImage;
    const off: usize = header_address - image_address;
    if (off + 16 > c.data.len) return error.BadImage;
    const owned = try gpa.dupe(u8, c.data);
    errdefer gpa.free(owned);
    // LargeTargetImageHeader: loadAddress@0, entry@4, capabilities@8, flashData@12 (all u32).
    const entry = std.mem.readInt(u32, owned[off + 4 ..][0..4], .big);
    const capabilities = std.mem.readInt(u32, owned[off + 8 ..][0..4], .big);
    const flash_data = std.mem.readInt(u32, owned[off + 12 ..][0..4], .big);
    img.deinit(gpa);
    return .{
        .image = owned,
        .image_address = image_address,
        .entry = entry,
        .capabilities = capabilities,
        .flash_data = flash_data,
        .header_offset = off,
    };
}

pub const Cfv1Driver = struct {
    target: Target,
    routine: Cfv1Routine,
    params: Params,
    last_error: DriverError = .ok,
    code_loaded: bool = false,

    const Layout = struct { code_load: u32, entry: u32, data_header: u32, data_addr: u32, max_data: u32 };

    fn layout(self: *const Cfv1Driver) Layout {
        const cap = self.routine.capabilities;
        const relocatable = cap & CAP_CFV1_RELOCATABLE != 0;
        const code_load: u32 = if (relocatable) (self.params.ram_start + 3) & ~@as(u32, 3) else self.routine.image_address;
        const delta = code_load -% self.routine.image_address;
        const image_size: u32 = @intCast(self.routine.image.len);
        const data_header: u32 = if (cap & CAP_CFV1_DATA_FIXED != 0) self.routine.flash_data else code_load + image_size;
        // 2-byte data alignment (procAlignmentMask = 1)
        const data_addr = (data_header + CFV1_HEADER_SIZE + 1) & ~@as(u32, 1);
        const max_data = if (self.params.ram_end >= data_addr) self.params.ram_end - data_addr + 1 else 0;
        return .{ .code_load = code_load, .entry = self.routine.entry +% delta, .data_header = data_header, .data_addr = data_addr, .max_data = max_data };
    }

    fn loadCode(self: *Cfv1Driver) Error!void {
        if (self.code_loaded) return;
        const l = self.layout();
        const off = self.routine.header_offset;
        std.mem.writeInt(u32, self.routine.image[off..][0..4], l.code_load, .big); // loadAddress
        std.mem.writeInt(u32, self.routine.image[off + 12 ..][0..4], l.data_header, .big); // flashData
        try self.target.vtable.writeMem(self.target.ctx, l.code_load, self.routine.image);
        self.code_loaded = true;
    }

    /// Marshal the 32-byte all-u32 LargeTargetFlashDataHeader + data, run, read ResultStruct { flags u32@0, errorCode u16@4 }.
    fn runOnce(self: *Cfv1Driver, op_flags: u32, flash_addr: u32, size: u32, data: []const u8) Error!void {
        const t = self.target;
        const l = self.layout();
        if (data.len > out_buf_size) return error.BadImage;
        var buf: [CFV1_HEADER_SIZE + out_buf_size]u8 = undefined;
        @memset(buf[0..CFV1_HEADER_SIZE], 0);
        std.mem.writeInt(u32, buf[0..4], DO_INIT_FLASH | op_flags, .big); // flags
        std.mem.writeInt(u16, buf[4..6], 0xFFFF, .big); // errorCode sentinel (-1)
        std.mem.writeInt(u16, buf[6..8], self.params.sector_size, .big); // sectorSize
        std.mem.writeInt(u32, buf[8..12], self.params.watchdog_addr, .big); // watchdogAddress
        std.mem.writeInt(u32, buf[12..16], self.params.controller, .big); // controller
        std.mem.writeInt(u32, buf[16..20], self.params.frequency_khz, .big); // frequency
        std.mem.writeInt(u32, buf[20..24], flash_addr, .big); // address
        std.mem.writeInt(u32, buf[24..28], size, .big); // dataSize
        std.mem.writeInt(u32, buf[28..32], l.data_addr, .big); // dataAddress
        @memcpy(buf[CFV1_HEADER_SIZE..][0..data.len], data);
        try t.vtable.writeMem(t.ctx, l.data_header, buf[0 .. CFV1_HEADER_SIZE + data.len]);

        try t.vtable.writePc(t.ctx, l.entry);
        try t.vtable.go(t.ctx);

        const halted = try pollUntilHalted(t, self.params.poll_iterations, CFV1_XCSR_RUNSTATE);

        var res: [8]u8 = undefined;
        try t.vtable.readMem(t.ctx, l.data_header, &res);
        const flags = std.mem.readInt(u32, res[0..4], .big);
        var err = std.mem.readInt(u16, res[4..6], .big);
        if (!halted) err = @intFromEnum(DriverError.timeout);
        if (err == 0 and flags != CFV1_IS_COMPLETE) err = @intFromEnum(DriverError.unknown);
        self.last_error = @enumFromInt(err);
        if (self.last_error != .ok) return error.RoutineError;
    }

    fn chunkSize(self: *const Cfv1Driver) Error!u32 {
        const n = @min(@min(self.layout().max_data, out_buf_size), 256);
        if (n == 0) return error.NoRamForData;
        return @intCast(n);
    }

    /// Like the HCS12 runPadded; CFV1 flash is longword-aligned (align 4).
    fn runPadded(self: *Cfv1Driver, op: u32, flash_addr: u32, data: []const u8) Error!void {
        const alignb: u32 = @max(self.params.alignment, 1);
        const mask: u32 = alignb - 1;
        const start = flash_addr & ~mask;
        const data_end: u32 = flash_addr + @as(u32, @intCast(data.len));
        const total: u32 = (((data_end + mask) & ~mask)) - start;
        const chunk = @max((try self.chunkSize()) & ~mask, alignb);
        var buf: [256]u8 = undefined;
        var off: u32 = 0;
        while (off < total) {
            const n = @min(total - off, chunk);
            var k: u32 = 0;
            while (k < n) : (k += 1) {
                const abs = start + off + k;
                buf[k] = if (abs >= flash_addr and abs < data_end) data[abs - flash_addr] else 0xFF;
            }
            try self.runOnce(op, start + off, n, buf[0..n]);
            off += n;
        }
    }

    pub fn program(self: *Cfv1Driver, flash_addr: u32, data: []const u8) Error!void {
        try self.loadCode();
        try self.runPadded(DO_BLANK_CHECK_RANGE | DO_PROGRAM_RANGE | DO_VERIFY_RANGE, flash_addr, data);
    }
    pub fn verify(self: *Cfv1Driver, flash_addr: u32, data: []const u8) Error!void {
        try self.loadCode();
        try self.runPadded(DO_VERIFY_RANGE, flash_addr, data);
    }
    pub fn massErase(self: *Cfv1Driver, flash_addr: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_ERASE_BLOCK, flash_addr, 0, &.{});
    }
    pub fn eraseRange(self: *Cfv1Driver, flash_addr: u32, size: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_ERASE_RANGE, flash_addr, size, &.{});
    }
    pub fn blankCheck(self: *Cfv1Driver, flash_addr: u32, size: u32) Error!void {
        try self.loadCode();
        try self.runOnce(DO_BLANK_CHECK_RANGE, flash_addr, size, &.{});
    }
};

const testing = std.testing;

const MockTarget = struct {
    mem: std.AutoHashMap(u32, u8),
    pc: u32 = 0,
    bdmact: bool = false,
    last_header: [32]u8 = undefined,
    header_addr: u32 = 0,
    result_error: u16 = 0,
    result_flags: u16 = 0, // large: exit flags (IS_COMPLETE on success)
    large: bool = false, // large result layout: flags@0, errorCode@2
    cfv1: bool = false, // CFV1 result: flags u32@0, errorCode u16@4; XCSR halt
    result_flags32: u32 = 0, // CFV1 exit flags (CFV1_IS_COMPLETE on success)
    never_halt: bool = false, // never self-halts -> poll timeout
    ran_entry: u32 = 0,
    reconnects: u32 = 0,

    fn init(gpa: std.mem.Allocator) MockTarget {
        return .{ .mem = std.AutoHashMap(u32, u8).init(gpa) };
    }
    fn deinit(self: *MockTarget) void {
        self.mem.deinit();
    }
    fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        for (data, 0..) |b, i| try self.mem.put(addr + @as(u32, @intCast(i)), b);
        // capture header before the routine clobbers bytes 0-1
        if (addr == self.header_addr and data.len >= 11) {
            const n = @min(data.len, self.last_header.len);
            @memcpy(self.last_header[0..n], data[0..n]);
        }
    }
    fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        for (buf, 0..) |*b, i| b.* = self.mem.get(addr + @as(u32, @intCast(i))) orelse 0;
    }
    fn writePc(ctx: *anyopaque, pc: u32) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.pc = pc;
    }
    fn go(ctx: *anyopaque) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        // model: run, self-halt (BDMACT), write result BE at header addr
        self.ran_entry = self.pc;
        if (self.never_halt) return; // never halts -> host times out
        self.bdmact = true;
        if (self.cfv1) {
            // CFV1 ResultStruct: flags u32@0 (BE), errorCode u16@4.
            self.mem.put(self.header_addr + 0, @truncate(self.result_flags32 >> 24)) catch {};
            self.mem.put(self.header_addr + 1, @truncate(self.result_flags32 >> 16)) catch {};
            self.mem.put(self.header_addr + 2, @truncate(self.result_flags32 >> 8)) catch {};
            self.mem.put(self.header_addr + 3, @truncate(self.result_flags32)) catch {};
            self.mem.put(self.header_addr + 4, @truncate(self.result_error >> 8)) catch {};
            self.mem.put(self.header_addr + 5, @truncate(self.result_error)) catch {};
            return;
        }
        // small: errorCode @0. large: flags@0, errorCode@2.
        const err_off: u32 = if (self.large) 2 else 0;
        self.mem.put(self.header_addr + err_off, @truncate(self.result_error >> 8)) catch {};
        self.mem.put(self.header_addr + err_off + 1, @truncate(self.result_error)) catch {};
        if (self.large) {
            self.mem.put(self.header_addr, @truncate(self.result_flags >> 8)) catch {};
            self.mem.put(self.header_addr + 1, @truncate(self.result_flags)) catch {};
        }
    }
    fn halt(ctx: *anyopaque) anyerror!void {
        _ = ctx;
    }
    fn readStatus(ctx: *anyopaque) anyerror!u8 {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        if (self.cfv1) return if (self.bdmact) CFV1_XCSR_RUNSTATE else 0;
        return if (self.bdmact) BDCSCR_BDMACT else 0;
    }
    fn sleepMs(ctx: *anyopaque, ms: u32) void {
        _ = ctx;
        _ = ms;
    }
    fn reconnect(ctx: *anyopaque) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.reconnects += 1;
    }
    const vtable = Target.VTable{
        .writeMem = writeMem,
        .readMem = readMem,
        .writePc = writePc,
        .go = go,
        .halt = halt,
        .readStatus = readStatus,
        .sleepMs = sleepMs,
        .reconnect = reconnect,
    };
    fn target(self: *MockTarget) Target {
        return .{ .ctx = self, .vtable = &vtable };
    }
};

// Synthetic small-code image; offsets below are slice boundaries, code is filler.
fn synthImage(gpa: std.mem.Allocator) ![]u8 {
    var img = try gpa.alloc(u8, 52);
    @memset(img, 0xAA);
    img[0] = OPT_SMALL_CODE | OPT_PAGED_ADDRESSES; // 0xC0
    img[1] = 12; // program
    img[2] = 20; // massErase
    img[3] = 28; // blankCheck
    img[4] = 36; // selectiveErase
    img[5] = 44; // verify
    return img;
}

fn synthRoutine(image: []const u8) Routine {
    return .{
        .load_address = 0x0080,
        .flags = image[0],
        .program = image[1],
        .mass_erase = image[2],
        .blank_check = image[3],
        .selective_erase = image[4],
        .verify = image[5],
        .image = image,
    };
}

test "slice selection picks the right code range per op" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    const r = synthRoutine(img);
    try testing.expectEqual(@as(u8, 12), r.slice(.program).start);
    try testing.expectEqual(@as(u8, 20), r.slice(.program).end);
    try testing.expectEqual(@as(u8, 44), r.slice(.verify).start);
    try testing.expectEqual(@as(u8, 52), r.slice(.verify).end); // == image size
}

test "program marshals the small param block and runs the top-of-RAM slice" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.header_addr = 0x0080; // load_address
    mock.result_error = 0;

    var drv = Driver{
        .target = mock.target(),
        .routine = synthRoutine(img),
        .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000 },
    };
    try drv.program(0xE000, &.{ 0x11, 0x22, 0x33, 0x44 });

    // captured header: flashAddress BE, controller BE, dataAddr, dataSize
    const hdr = mock.last_header;
    try testing.expectEqual(@as(u16, 0xE000), std.mem.readInt(u16, hdr[0..2], .big));
    try testing.expectEqual(@as(u16, 0x1820), std.mem.readInt(u16, hdr[2..4], .big));
    // dataAddress = load_address + program(12) = 0x008C
    try testing.expectEqual(@as(u16, 0x008C), std.mem.readInt(u16, hdr[4..6], .big));
    try testing.expectEqual(@as(u16, 4), std.mem.readInt(u16, hdr[6..8], .big));
    try testing.expectEqual(@as(u8, 0x11), mock.mem.get(0x008C).?);
    try testing.expectEqual(@as(u8, 0x44), mock.mem.get(0x008F).?);
    // entry = ramEnd - codeLen + 1; program slice len = 20-12 = 8 -> 0x107F-8+1 = 0x1078
    try testing.expectEqual(@as(u32, 0x1078), mock.ran_entry);
}

test "a non-OK routine result surfaces as RoutineError with the code" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.header_addr = 0x0080;
    mock.result_error = @intFromEnum(DriverError.verify_failed);
    var drv = Driver{
        .target = mock.target(),
        .routine = synthRoutine(img),
        .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000 },
    };
    try testing.expectError(error.RoutineError, drv.blankCheck(0xE000, 0x2000));
    try testing.expectEqual(DriverError.verify_failed, drv.last_error);
}

test "blank check sends the region byte count, not zero" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.header_addr = 0x0080;
    mock.result_error = 0;
    var drv = Driver{
        .target = mock.target(),
        .routine = synthRoutine(img),
        .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000 },
    };
    try drv.blankCheck(0xE000, 0x2000);
    // dataSize (BE u16 @6) = checked span
    try testing.expectEqual(@as(u16, 0x2000), std.mem.readInt(u16, mock.last_header[6..8], .big));
}

test "slice clamps a zero end offset to image size; hasOp reports absence" {
    const img = try testing.allocator.alloc(u8, 60);
    defer testing.allocator.free(img);
    @memset(img, 0xAA);
    img[0] = OPT_SMALL_CODE;
    img[1] = 12; // program
    img[2] = 20; // mass_erase
    img[3] = 28; // blank_check
    img[4] = 36; // selective_erase
    img[5] = 0; // verify absent
    const r = synthRoutine(img);
    // selective_erase end = verify(0) -> clamped to image size (60).
    try testing.expectEqual(@as(u8, 36), r.slice(.selective_erase).start);
    try testing.expectEqual(@as(u8, 60), r.slice(.selective_erase).end);
    try testing.expect(!r.hasOp(.verify)); // offset 0 -> absent
    try testing.expect(r.hasOp(.program));
}

test "verify falls back to readback-compare when the image has no verify routine" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    img[5] = 0; // no verify routine
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    // seed flash at 0xE000
    try mock.mem.put(0xE000, 0x11);
    try mock.mem.put(0xE001, 0x22);
    var drv = Driver{ .target = mock.target(), .routine = synthRoutine(img), .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000 } };
    try drv.verify(0xE000, &.{ 0x11, 0x22 }); // matches -> ok, no routine run
    // mismatch -> verify_failed
    try testing.expectError(error.RoutineError, drv.verify(0xE000, &.{ 0x11, 0x33 }));
    try testing.expectEqual(DriverError.verify_failed, drv.last_error);
}

test "small mass-erase latches a real flash address (not the RAM load addr) and reconnects" {
    const img = try synthImage(testing.allocator);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.header_addr = 0x0080;
    mock.result_error = 0;
    var drv = Driver{
        .target = mock.target(),
        .routine = synthRoutine(img),
        .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000 },
    };
    try drv.massErase(0xE000); // caller passes a flash address
    // flashAddress (BE u16 @0) = flash addr, not 0x0080
    try testing.expectEqual(@as(u16, 0xE000), std.mem.readInt(u16, mock.last_header[0..2], .big));
    // reconnects between halt and readback
    try testing.expect(mock.reconnects >= 1);
}

test "parseSmall reads the SmallTargetImageHeader from a real vendored blob" {
    // The HCS08 default 0x80 blob (vendored) begins C0 0B 49 74 92 00 at 0x0080.
    const srec =
        "S027000048435330382D64656661756C742D307838302D666C6173682D70726F6772616D2E6162737F\n" ++
        "S1230080C00B4974920000000000005588B68AF75586272AAFFF35865584F6AF013584550C\n" ++
        "S105FFFE008B72\n" ++
        "S9030000FC\n";
    const parsed = try parseSmall(testing.allocator, srec);
    defer testing.allocator.free(parsed.image_owned);
    const r = parsed.routine;
    try testing.expectEqual(@as(u32, 0x0080), r.load_address);
    try testing.expectEqual(@as(u8, 0xC0), r.flags);
    try testing.expectEqual(@as(u8, 0x0B), r.program); // program offset (== entry-in-blob 0x8B-0x80)
    try testing.expectEqual(@as(u8, 0x49), r.mass_erase);
    try testing.expectEqual(@as(u8, 0x74), r.blank_check);
    try testing.expectEqual(@as(u8, 0x92), r.selective_erase);
    try testing.expect(r.flags & OPT_SMALL_CODE != 0);
}

// Synthetic large-code image at 0x1000, 14-byte header at offset 2 (fields below).
fn synthLargeImage(gpa: std.mem.Allocator, size: usize) ![]u8 {
    std.debug.assert(size >= 16);
    var img = try gpa.alloc(u8, size);
    @memset(img, 0xAA);
    std.mem.writeInt(u16, img[0..2], 0x1002, .big); // headerAddress
    std.mem.writeInt(u16, img[2..4], 0x1000, .big); // loadAddress
    std.mem.writeInt(u16, img[4..6], 0x1050, .big); // entry
    std.mem.writeInt(u16, img[6..8], CAP_RELOCATABLE | 0x3E, .big); // capabilities
    std.mem.writeInt(u16, img[8..10], 0, .big); // copctlAddress
    std.mem.writeInt(u32, img[10..14], 0, .big); // calibFactor
    std.mem.writeInt(u16, img[14..16], 0, .big); // flashData
    return img;
}

fn synthLargeRoutine(image: []u8) LargeRoutine {
    return .{
        .image = image,
        .image_address = 0x1000,
        .entry = 0x1050,
        .capabilities = CAP_RELOCATABLE | 0x3E,
        .flash_data = 0,
        .header_offset = 2,
    };
}

test "parseLarge reads the image header from the vendored HCS12 blob" {
    // First data record of HCS12-MMCV4-FTS-flash-program.s19, verbatim.
    const srec =
        "S12310001002100012FAA03E00001439000000000000000000000000000000000000000073\n" ++
        "S9030000FC\n";
    const r = try parseLarge(testing.allocator, srec);
    defer testing.allocator.free(r.image);
    try testing.expectEqual(@as(u32, 0x1000), r.image_address);
    try testing.expectEqual(@as(usize, 2), r.header_offset);
    try testing.expectEqual(@as(u32, 0x12FA), r.entry);
    try testing.expectEqual(@as(u16, 0xA03E), r.capabilities);
    try testing.expect(r.capabilities & CAP_RELOCATABLE != 0);
    try testing.expect(r.capabilities & CAP_DATA_FIXED == 0);
}

test "large program relocates code+entry and marshals the 18-byte header" {
    const img = try synthLargeImage(testing.allocator, 0x40); // 64-byte image
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.large = true;
    // code_load = (ram_start+3)&~3 = 0x0800; data_header = 0x0800 + 0x40 = 0x0840
    mock.header_addr = 0x0840;
    mock.result_error = 0;
    mock.result_flags = IS_COMPLETE;

    var drv = LargeDriver{
        .target = mock.target(),
        .routine = synthLargeRoutine(img),
        .params = .{ .ram_start = 0x0800, .ram_end = 0x0FFF, .controller = 0x0100, .sector_size = 512, .frequency_khz = 4000, .watchdog_addr = 0x003C },
    };
    try drv.program(0x4000, &.{ 0xDE, 0xAD, 0xBE, 0xEF });

    // Relocation: entry = 0x1050 + (0x0800 - 0x1000) = 0x0850.
    try testing.expectEqual(@as(u32, 0x0850), mock.ran_entry);
    // Image header patched at download: loadAddress@0x0802, copctl@0x0808, flashData@0x080E.
    try testing.expectEqual(@as(u8, 0x08), mock.mem.get(0x0802).?);
    try testing.expectEqual(@as(u8, 0x00), mock.mem.get(0x0803).?);
    try testing.expectEqual(@as(u8, 0x00), mock.mem.get(0x0808).?);
    try testing.expectEqual(@as(u8, 0x3C), mock.mem.get(0x0809).?); // copctl = watchdog addr
    try testing.expectEqual(@as(u8, 0x08), mock.mem.get(0x080E).?);
    try testing.expectEqual(@as(u8, 0x40), mock.mem.get(0x080F).?); // flashData = data_header
    // captured header:
    const h = mock.last_header;
    try testing.expectEqual(DO_INIT_FLASH | LARGE_PROGRAM_OP, std.mem.readInt(u16, h[0..2], .big));
    try testing.expectEqual(@as(u16, 0x0100), std.mem.readInt(u16, h[4..6], .big)); // controller
    try testing.expectEqual(@as(u16, 4000), std.mem.readInt(u16, h[6..8], .big)); // frequency
    try testing.expectEqual(@as(u16, 512), std.mem.readInt(u16, h[8..10], .big)); // sectorSize
    try testing.expectEqual(@as(u32, 0x4000), std.mem.readInt(u32, h[10..14], .big)); // address
    try testing.expectEqual(@as(u16, 4), std.mem.readInt(u16, h[14..16], .big)); // dataSize
    try testing.expectEqual(@as(u16, 0x0852), std.mem.readInt(u16, h[16..18], .big)); // dataAddress
    try testing.expectEqual(@as(u8, 0xDE), mock.mem.get(0x0852).?);
    try testing.expectEqual(@as(u8, 0xEF), mock.mem.get(0x0855).?);
}

test "large routine reports errorCode@2 and requires IS_COMPLETE" {
    const img = try synthLargeImage(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.large = true;
    mock.header_addr = 0x0840;

    // A non-zero error surfaces even with IS_COMPLETE set.
    mock.result_error = @intFromEnum(DriverError.verify_failed);
    mock.result_flags = IS_COMPLETE;
    var drv = LargeDriver{
        .target = mock.target(),
        .routine = synthLargeRoutine(img),
        .params = .{ .ram_start = 0x0800, .ram_end = 0x0FFF, .controller = 0x0100, .sector_size = 512, .frequency_khz = 4000 },
    };
    try testing.expectError(error.RoutineError, drv.blankCheck(0x4000, 0x100));
    try testing.expectEqual(DriverError.verify_failed, drv.last_error);

    // errorCode OK but flags != IS_COMPLETE -> synthesized 'unknown'.
    mock.result_error = 0;
    mock.result_flags = 0;
    drv.code_loaded = false;
    try testing.expectError(error.RoutineError, drv.blankCheck(0x4000, 0x100));
    try testing.expectEqual(DriverError.unknown, drv.last_error);
}

// Synthetic CFV1 image at 0x00800000, 16-byte all-u32 header at offset 4 (fields below).
fn synthCfv1Image(gpa: std.mem.Allocator, size: usize) ![]u8 {
    std.debug.assert(size >= 20);
    var img = try gpa.alloc(u8, size);
    @memset(img, 0xAA);
    std.mem.writeInt(u32, img[0..4], 0x00800004, .big); // headerAddress
    std.mem.writeInt(u32, img[4..8], 0x00800000, .big); // loadAddress
    std.mem.writeInt(u32, img[8..12], 0x00800020, .big); // entry
    std.mem.writeInt(u32, img[12..16], 0x3E, .big); // capabilities
    std.mem.writeInt(u32, img[16..20], 0, .big); // flashData
    return img;
}

fn synthCfv1Routine(image: []u8) Cfv1Routine {
    return .{ .image = image, .image_address = 0x00800000, .entry = 0x00800020, .capabilities = 0x3E, .flash_data = 0, .header_offset = 4 };
}

test "parseCfv1 reads the 32-bit image header via a 4-byte BE header pointer" {
    const srec =
        "S0030000FC\n" ++
        "S32500800000" ++ "0080000400800000008000200000003E" ++ "00000000000000000000000000000000" ++ "78\n" ++
        "S70500000000FA\n";
    const r = try parseCfv1(testing.allocator, srec);
    defer testing.allocator.free(r.image);
    try testing.expectEqual(@as(u32, 0x00800000), r.image_address);
    try testing.expectEqual(@as(usize, 4), r.header_offset);
    try testing.expectEqual(@as(u32, 0x00800020), r.entry);
    try testing.expectEqual(@as(u32, 0x3E), r.capabilities);
    try testing.expect(r.capabilities & CAP_CFV1_RELOCATABLE == 0);
}

test "cfv1 program marshals the 32-byte all-u32 header and completes via XCSR RUNSTATE" {
    const img = try synthCfv1Image(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.cfv1 = true;
    // non-relocatable -> code at 0x800000; data_header = 0x800000 + 0x40 = 0x800040
    mock.header_addr = 0x00800040;
    mock.result_error = 0;
    mock.result_flags32 = CFV1_IS_COMPLETE;
    var drv = Cfv1Driver{
        .target = mock.target(),
        .routine = synthCfv1Routine(img),
        .params = .{ .ram_start = 0x00800000, .ram_end = 0x00807FFF, .controller = 0xFF9820, .sector_size = 1024, .frequency_khz = 6000, .watchdog_addr = 0xFF9802, .alignment = 4 },
    };
    try drv.program(0x1000, &.{ 0xDE, 0xAD, 0xBE, 0xEF });

    try testing.expectEqual(@as(u32, 0x00800020), mock.ran_entry); // entry (not relocated)
    const h = mock.last_header;
    try testing.expectEqual(DO_INIT_FLASH | DO_BLANK_CHECK_RANGE | DO_PROGRAM_RANGE | DO_VERIFY_RANGE, std.mem.readInt(u32, h[0..4], .big));
    try testing.expectEqual(@as(u16, 0xFFFF), std.mem.readInt(u16, h[4..6], .big)); // errorCode sentinel
    try testing.expectEqual(@as(u16, 1024), std.mem.readInt(u16, h[6..8], .big)); // sectorSize
    try testing.expectEqual(@as(u32, 0xFF9802), std.mem.readInt(u32, h[8..12], .big)); // watchdogAddress
    try testing.expectEqual(@as(u32, 0xFF9820), std.mem.readInt(u32, h[12..16], .big)); // controller
    try testing.expectEqual(@as(u32, 6000), std.mem.readInt(u32, h[16..20], .big)); // frequency
    try testing.expectEqual(@as(u32, 0x1000), std.mem.readInt(u32, h[20..24], .big)); // address
    try testing.expectEqual(@as(u32, 4), std.mem.readInt(u32, h[24..28], .big)); // dataSize
    try testing.expectEqual(@as(u32, 0x00800060), std.mem.readInt(u32, h[28..32], .big)); // dataAddress
    // Image header patched at download: loadAddress@0x800004, flashData@0x800010.
    try testing.expectEqual(@as(u32, 0x00800000), std.mem.readInt(u32, &[_]u8{ mock.mem.get(0x800004).?, mock.mem.get(0x800005).?, mock.mem.get(0x800006).?, mock.mem.get(0x800007).? }, .big));
    try testing.expectEqual(@as(u32, 0x00800040), std.mem.readInt(u32, &[_]u8{ mock.mem.get(0x800010).?, mock.mem.get(0x800011).?, mock.mem.get(0x800012).?, mock.mem.get(0x800013).? }, .big));
    try testing.expectEqual(@as(u8, 0xDE), mock.mem.get(0x800060).?);
    try testing.expectEqual(@as(u8, 0xEF), mock.mem.get(0x800063).?);
}

test "cfv1 routine error surfaces and IS_COMPLETE is required" {
    const img = try synthCfv1Image(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.cfv1 = true;
    mock.header_addr = 0x00800040;
    mock.result_error = @intFromEnum(DriverError.verify_failed);
    mock.result_flags32 = CFV1_IS_COMPLETE;
    var drv = Cfv1Driver{
        .target = mock.target(),
        .routine = synthCfv1Routine(img),
        .params = .{ .ram_start = 0x00800000, .ram_end = 0x00807FFF, .controller = 0xFF9820, .sector_size = 1024, .frequency_khz = 6000, .watchdog_addr = 0xFF9802, .alignment = 4 },
    };
    try testing.expectError(error.RoutineError, drv.blankCheck(0x0000, 0x2000));
    try testing.expectEqual(DriverError.verify_failed, drv.last_error);
    try testing.expect(mock.reconnects >= 1);
}

test "cfv1 completion requires the 32-bit sentinel, not the 16-bit one" {
    const img = try synthCfv1Image(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.cfv1 = true;
    mock.header_addr = 0x00800040;
    mock.result_error = 0;
    // 16-bit sentinel in the low half must NOT satisfy CFV1 completion (bit15-vs-bit31 guard).
    mock.result_flags32 = 0x0000_8000;
    var drv = Cfv1Driver{
        .target = mock.target(),
        .routine = synthCfv1Routine(img),
        .params = .{ .ram_start = 0x00800000, .ram_end = 0x00807FFF, .controller = 0xFF9820, .sector_size = 1024, .frequency_khz = 6000, .watchdog_addr = 0xFF9802, .alignment = 4 },
    };
    try testing.expectError(error.RoutineError, drv.blankCheck(0x0000, 0x2000));
    try testing.expectEqual(DriverError.unknown, drv.last_error);
}

test "a routine that never self-halts times out (all three drivers)" {
    // small
    {
        const img = try synthImage(testing.allocator);
        defer testing.allocator.free(img);
        var mock = MockTarget.init(testing.allocator);
        defer mock.deinit();
        mock.header_addr = 0x0080;
        mock.never_halt = true;
        var drv = Driver{ .target = mock.target(), .routine = synthRoutine(img), .params = .{ .ram_start = 0x0080, .ram_end = 0x107F, .controller = 0x1820, .sector_size = 512, .frequency_khz = 4000, .poll_iterations = 2 } };
        try testing.expectError(error.RoutineError, drv.blankCheck(0xE000, 0x100));
        try testing.expectEqual(DriverError.timeout, drv.last_error);
    }
    // large
    {
        const img = try synthLargeImage(testing.allocator, 0x40);
        defer testing.allocator.free(img);
        var mock = MockTarget.init(testing.allocator);
        defer mock.deinit();
        mock.large = true;
        mock.header_addr = 0x0840;
        mock.never_halt = true;
        var drv = LargeDriver{ .target = mock.target(), .routine = synthLargeRoutine(img), .params = .{ .ram_start = 0x0800, .ram_end = 0x0FFF, .controller = 0x0100, .sector_size = 512, .frequency_khz = 4000, .poll_iterations = 2 } };
        try testing.expectError(error.RoutineError, drv.blankCheck(0x4000, 0x100));
        try testing.expectEqual(DriverError.timeout, drv.last_error);
    }
    // cfv1
    {
        const img = try synthCfv1Image(testing.allocator, 0x40);
        defer testing.allocator.free(img);
        var mock = MockTarget.init(testing.allocator);
        defer mock.deinit();
        mock.cfv1 = true;
        mock.header_addr = 0x00800040;
        mock.never_halt = true;
        var drv = Cfv1Driver{ .target = mock.target(), .routine = synthCfv1Routine(img), .params = .{ .ram_start = 0x00800000, .ram_end = 0x00807FFF, .controller = 0xFF9820, .sector_size = 1024, .frequency_khz = 6000, .poll_iterations = 2 } };
        try testing.expectError(error.RoutineError, drv.blankCheck(0x0000, 0x2000));
        try testing.expectEqual(DriverError.timeout, drv.last_error);
    }
}

test "large program carries a paged 24-bit address (page in the top byte)" {
    const img = try synthLargeImage(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.large = true;
    mock.header_addr = 0x0840;
    mock.result_error = 0;
    mock.result_flags = IS_COMPLETE;
    var drv = LargeDriver{
        .target = mock.target(),
        .routine = synthLargeRoutine(img),
        .params = .{ .ram_start = 0x0800, .ram_end = 0x0FFF, .controller = 0x0100, .sector_size = 512, .frequency_khz = 4000, .watchdog_addr = 0x003C, .alignment = 2 },
    };
    // banked address: page 0x30, window offset 0x8000
    try drv.program(0x308000, &.{ 0x01, 0x02 });
    // address (u32 BE @10) = 0x00308000, page 0x30 in top byte
    try testing.expectEqual(@as(u32, 0x00308000), std.mem.readInt(u32, mock.last_header[10..14], .big));
}

test "large program rounds a misaligned/odd segment to the write alignment and 0xFF-pads" {
    const img = try synthLargeImage(testing.allocator, 0x40);
    defer testing.allocator.free(img);
    var mock = MockTarget.init(testing.allocator);
    defer mock.deinit();
    mock.large = true;
    mock.header_addr = 0x0840; // data_header = code_load(0x0800) + imageSize(0x40)
    mock.result_error = 0;
    mock.result_flags = IS_COMPLETE;
    var drv = LargeDriver{
        .target = mock.target(),
        .routine = synthLargeRoutine(img),
        .params = .{ .ram_start = 0x0800, .ram_end = 0x0FFF, .controller = 0x0100, .sector_size = 512, .frequency_khz = 4000, .alignment = 2 },
    };
    // Odd start (0x4001) and odd length (3) -> aligned to [0x4000, 0x4004).
    try drv.program(0x4001, &.{ 0x11, 0x22, 0x33 });
    const h = mock.last_header;
    try testing.expectEqual(@as(u32, 0x4000), std.mem.readInt(u32, h[10..14], .big)); // rounded-down start
    try testing.expectEqual(@as(u16, 4), std.mem.readInt(u16, h[14..16], .big)); // padded to word
    // Data buffer at dataAddress (0x0840 + 18 = 0x0852): FF pad, data, then aligned.
    try testing.expectEqual(@as(u8, 0xFF), mock.mem.get(0x0852).?); // leading pad (0x4000)
    try testing.expectEqual(@as(u8, 0x11), mock.mem.get(0x0853).?); // 0x4001
    try testing.expectEqual(@as(u8, 0x22), mock.mem.get(0x0854).?); // 0x4002
    try testing.expectEqual(@as(u8, 0x33), mock.mem.get(0x0855).?); // 0x4003
}

// Parsers must never panic on malformed bytes; arena per iteration avoids leaks.
test "fuzz: blob parsers never panic on malformed input" {
    var prng = std.Random.DefaultPrng.init(0x1CE0FF1C);
    const rand = prng.random();
    var raw: [512]u8 = undefined;
    const srec_chars = "0123456789ABCDEF\r\nS";
    var it: usize = 0;
    while (it < 20000) : (it += 1) {
        var arena_inst = std.heap.ArenaAllocator.init(testing.allocator);
        defer arena_inst.deinit();
        const a = arena_inst.allocator();
        const len = rand.intRangeAtMost(usize, 0, raw.len);
        const p = raw[0..len];
        if (rand.boolean()) {
            for (p) |*b| b.* = srec_chars[rand.intRangeAtMost(usize, 0, srec_chars.len - 1)];
        } else rand.bytes(p);
        _ = parseSmall(a, p) catch {};
        _ = parseLarge(a, p) catch {};
        _ = parseCfv1(a, p) catch {};
    }
}
