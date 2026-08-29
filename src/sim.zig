//! Software virtual USBDM dongle + target: models the USBDM protocol and the
//! flash-routine ABI in memory, so any command can run with no hardware.

const std = @import("std");
const protocol = @import("protocol.zig");
const transport = @import("transport.zig");
const target = @import("target.zig");
const device = @import("device.zig");
const ramflash = @import("ramflash.zig");
const cpu08 = @import("cpu08.zig");
const cpu12 = @import("cpu12.zig");
const cfv1 = @import("cfv1.zig");

const Rc = protocol.Rc;
const Cmd = protocol.Command;

/// Optional execution backend (a real CPU core) so `--sim` runs the ACTUAL
/// vendored flash routine instead of the hand-coded flash model (the default).
pub const CpuBackend = union(enum) {
    hcs08: *cpu08.Cpu,
    hcs12: *cpu12.Cpu,
    cfv1: *cfv1.Cpu,

    fn readByte(self: CpuBackend, a: u32) u8 {
        return switch (self) {
            .hcs08 => |c| blk: {
                var b: [1]u8 = undefined;
                c.hostRead(@intCast(a & 0xFFFF), &b);
                break :blk b[0];
            },
            .hcs12 => |c| c.hostReadByte(@intCast(a & 0xFFFF)),
            .cfv1 => |c| c.hostReadByte(a),
        };
    }
    fn writeByte(self: CpuBackend, a: u32, v: u8) void {
        switch (self) {
            .hcs08 => |c| c.hostWrite(@intCast(a & 0xFFFF), &.{v}),
            .hcs12 => |c| c.hostWrite(@intCast(a & 0xFFFF), &.{v}),
            .cfv1 => |c| c.hostWrite(a, &.{v}),
        }
    }
    fn readReg(self: CpuBackend, no: u16) u32 {
        return switch (self) {
            .hcs08 => |c| switch (no) {
                8 => c.a,
                9 => c.ccr,
                0xB => c.pc,
                0xC => (@as(u32, c.h) << 8) | c.x,
                0xF => c.sp,
                else => 0,
            },
            .hcs12 => |c| switch (no) {
                3 => c.pc,
                4 => (@as(u32, c.a) << 8) | c.b,
                5 => c.x,
                6 => c.y,
                7 => c.sp,
                else => 0,
            },
            .cfv1 => |c| if (no < 8) c.d[no] else if (no < 16) c.a[no - 8] else 0,
        };
    }
    fn writeReg(self: CpuBackend, no: u16, v: u32) void {
        switch (self) {
            .hcs08 => |c| switch (no) {
                8 => c.a = @truncate(v),
                9 => c.ccr = @truncate(v),
                0xB => c.pc = @truncate(v),
                0xC => {
                    c.h = @truncate(v >> 8);
                    c.x = @truncate(v);
                },
                0xF => c.sp = @truncate(v),
                else => {},
            },
            .hcs12 => |c| switch (no) {
                3 => c.pc = @truncate(v),
                4 => {
                    c.a = @truncate(v >> 8);
                    c.b = @truncate(v);
                },
                5 => c.x = @truncate(v),
                6 => c.y = @truncate(v),
                7 => c.sp = @truncate(v),
                else => {},
            },
            .cfv1 => |c| {
                if (no < 8) c.d[no] = v else if (no < 16) c.a[no - 8] = v;
            },
        }
    }
    fn readCReg(self: CpuBackend, no: u16) u32 {
        return switch (self) {
            .cfv1 => |c| switch (no) {
                15 => c.pc,
                14 => (@as(u32, c.sr_hi) << 8) | c.ccr,
                1 => c.vbr,
                else => 0,
            },
            else => 0,
        };
    }
    fn writeCReg(self: CpuBackend, no: u16, v: u32) void {
        switch (self) {
            .cfv1 => |c| switch (no) {
                15 => c.pc = v,
                14 => {
                    c.sr_hi = @truncate(v >> 8);
                    c.ccr = @truncate(v);
                },
                1 => c.vbr = v,
                else => {},
            },
            else => {},
        }
    }
    fn statusReg(self: CpuBackend) u32 {
        return switch (self) {
            .cfv1 => |c| if (c.halted) @as(u32, cfv1.XCSR_RUNSTATE) else 0,
            inline else => |c| if (c.halted) @as(u32, ramflash.BDCSCR_BDMACT) else 0,
        };
    }
    fn halted(self: CpuBackend) bool {
        return switch (self) {
            inline else => |c| c.halted,
        };
    }
    fn setHalted(self: CpuBackend, h: bool) void {
        switch (self) {
            inline else => |c| c.halted = h,
        }
    }
    /// Run until self-halt (BGND) or the step cap; a cap hit is reported as halted.
    fn go(self: CpuBackend) void {
        switch (self) {
            .hcs08 => |c| c.run(4_000_000) catch {
                c.halted = true;
            },
            inline .hcs12, .cfv1 => |c| c.run(8_000_000) catch {
                c.halted = true;
            },
        }
    }
    fn stepOne(self: CpuBackend) void {
        switch (self) {
            inline else => |c| {
                c.halted = false; // clear so a prior BGND doesn't block the step
                _ = c.step() catch {};
            },
        }
    }
};

pub const Sim = struct {
    gpa: std.mem.Allocator,
    dev: device.Device,
    mem: std.AutoHashMap(u32, u8),
    reg: [16]u32 = [_]u32{0} ** 16, // read/write_reg
    creg: [16]u32 = [_]u32{0} ** 16, // read/write_creg
    dreg: std.AutoHashMap(u16, u32),
    connected: bool = false,
    halted: bool = true, // starts halted (special mode)
    speed_sync: u16 = 1920, // ~4 MHz bus (128*60e6/1920)
    command_buffer_size: u16 = 254,
    /// HCS08 small-code flash routine; small code selects the op by entry PC,
    /// not a flags field. Null for pure protocol/memory tests.
    small_routine: ?ramflash.Routine = null,
    /// Arms flash emulation on `go`; unarmed, a plain `go`/gdb `continue` would
    /// reinterpret RAM as a parameter block and scribble the flash image.
    flash_armed: bool = false,
    /// When set, `go` runs the ACTUAL routine on a real core; execFlash unused.
    cpu: ?CpuBackend = null,

    // Flash param header = base of the LAST writeMem *call*. A call splits into
    // contiguous chunks (all but last "full"), so a partial chunk ends a call.
    last_write_addr: u32 = 0,
    write_prev_end: u32 = 0,
    write_prev_full: bool = false,

    resp: [protocol.max_command_size + 1]u8 = undefined,
    resp_len: usize = 0,
    frame: [protocol.max_command_size]u8 = undefined,
    frame_len: usize = 0,
    awaiting_cont: bool = false,

    pub fn init(gpa: std.mem.Allocator, dev: device.Device) Sim {
        return .{
            .gpa = gpa,
            .dev = dev,
            .mem = std.AutoHashMap(u32, u8).init(gpa),
            .dreg = std.AutoHashMap(u16, u32).init(gpa),
        };
    }
    pub fn deinit(self: *Sim) void {
        self.mem.deinit();
        self.dreg.deinit();
    }

    pub fn asTransport(self: *Sim, trace: protocol.Trace) transport.Transport {
        return .{ .ctx = self, .vtable = &vtable, .trace = trace };
    }

    // Unwritten flash reads 0xFF, RAM reads 0. With a CPU backend the emulator's
    // memory IS the target memory.
    fn memGet(self: *Sim, a: u32) u8 {
        if (self.mem.get(a)) |v| return v; // explicit writes win
        // SDID is a peripheral ID the CPU emulators don't model, so the sim serves
        // it (even with a backend attached) to drive identify/auto-detect.
        if (self.sdidByte(a)) |v| return v;
        if (self.cpu) |c| return c.readByte(a);
        return if (self.dev.inFlash(a)) 0xFF else 0;
    }

    /// Device's own SDID at its family SDID address; an explicit write overrides it.
    fn sdidByte(self: *Sim, a: u32) ?u8 {
        const addr = device.familySdidAddr(self.dev.family);
        if (a != addr and a != addr +% 1) return null;
        for (device.sdid_table) |e| {
            if (e.family == self.dev.family and std.ascii.eqlIgnoreCase(e.name, self.dev.name)) {
                return if (a == addr) @truncate(e.sdid >> 8) else @truncate(e.sdid);
            }
        }
        return null;
    }
    fn memPut(self: *Sim, a: u32, v: u8) void {
        if (self.cpu) |c| {
            c.writeByte(a, v);
            return;
        }
        self.mem.put(a, v) catch {};
    }

    fn rd16(self: *Sim, a: u32) u16 {
        return (@as(u16, self.memGet(a)) << 8) | self.memGet(a + 1);
    }
    fn rd32(self: *Sim, a: u32) u32 {
        return (@as(u32, self.rd16(a)) << 16) | self.rd16(a + 2);
    }
    fn wr16(self: *Sim, a: u32, v: u16) void {
        self.memPut(a, @truncate(v >> 8));
        self.memPut(a + 1, @truncate(v));
    }
    fn wr32(self: *Sim, a: u32, v: u32) void {
        self.wr16(a, @truncate(v >> 16));
        self.wr16(a + 2, @truncate(v));
    }

    // Loops use wrapping address add so a decoded garbage size can't overflow u32.
    fn eraseRange(self: *Sim, addr: u32, size: u32) void {
        var a = addr;
        var n = size;
        while (n != 0) : (n -= 1) {
            _ = self.mem.remove(a); // reads 0xFF again
            a +%= 1;
        }
    }
    fn eraseBlock(self: *Sim) void {
        // unwritten flash already reads 0xFF
        var keys: std.ArrayList(u32) = .empty;
        defer keys.deinit(self.gpa);
        var it = self.mem.keyIterator();
        while (it.next()) |k| {
            if (self.dev.inFlash(k.*)) keys.append(self.gpa, k.*) catch return;
        }
        for (keys.items) |k| _ = self.mem.remove(k);
    }
    /// Flash write-once: program only clears bits (val &= data).
    fn program(self: *Sim, flash_addr: u32, data_addr: u32, size: u32) void {
        var i: u32 = 0;
        while (i < size) : (i += 1) {
            const cur = self.memGet(flash_addr +% i);
            self.memPut(flash_addr +% i, cur & self.memGet(data_addr +% i));
        }
    }
    fn blankCheck(self: *Sim, addr: u32, size: u32) bool {
        var i: u32 = 0;
        while (i < size) : (i += 1) if (self.memGet(addr +% i) != 0xFF) return false;
        return true;
    }
    fn verify(self: *Sim, flash_addr: u32, data_addr: u32, size: u32) bool {
        var i: u32 = 0;
        while (i < size) : (i += 1) if (self.memGet(flash_addr +% i) != self.memGet(data_addr +% i)) return false;
        return true;
    }

    /// One run of the downloaded flash routine (on `go`); only when armed.
    fn execFlash(self: *Sim) void {
        if (!self.flash_armed) return;
        switch (self.dev.family) {
            .hcs12 => self.execLarge(false),
            .cfv1 => self.execLarge(true),
            .hcs08 => self.execSmall(),
        }
    }

    fn execLarge(self: *Sim, is_cfv1: bool) void {
        const h = self.last_write_addr;
        var flags: u32 = undefined;
        var address: u32 = undefined;
        var data_size: u32 = undefined;
        var data_addr: u32 = undefined;
        if (is_cfv1) {
            flags = self.rd32(h + 0);
            address = self.rd32(h + 20);
            data_size = self.rd32(h + 24);
            data_addr = self.rd32(h + 28);
        } else {
            flags = self.rd16(h + 0);
            address = self.rd32(h + 10);
            data_size = self.rd16(h + 14);
            data_addr = self.rd16(h + 16);
        }
        var err: u16 = 0;
        if (flags & ramflash.DO_ERASE_BLOCK != 0) self.eraseBlock();
        if (flags & ramflash.DO_ERASE_RANGE != 0) self.eraseRange(address, data_size);
        if (flags & ramflash.DO_BLANK_CHECK_RANGE != 0 and !self.blankCheck(address, data_size))
            err = @intFromEnum(ramflash.DriverError.erase_failed);
        if (err == 0 and flags & ramflash.DO_PROGRAM_RANGE != 0) self.program(address, data_addr, data_size);
        if (err == 0 and flags & ramflash.DO_VERIFY_RANGE != 0 and !self.verify(address, data_addr, data_size))
            err = @intFromEnum(ramflash.DriverError.verify_failed);
        if (is_cfv1) {
            self.wr32(h + 0, ramflash.CFV1_IS_COMPLETE);
            self.wr16(h + 4, err);
        } else {
            self.wr16(h + 0, ramflash.IS_COMPLETE);
            self.wr16(h + 2, err);
        }
    }

    fn execSmall(self: *Sim) void {
        const r = self.small_routine orelse return;
        const h = self.last_write_addr;
        const flash_addr = self.rd16(h + 0);
        const field4 = self.rd16(h + 4);
        const field6 = self.rd16(h + 6);
        // Op selected by entry PC: code_load = ram_end-len+1.
        const pc = self.reg[@intFromEnum(target.Hcs08Reg.pc)];
        const op: ?ramflash.Op = for ([_]ramflash.Op{ .program, .block_erase, .blank_check, .selective_erase, .verify }) |o| {
            const s = r.slice(o);
            if (s.end <= s.start) continue; // op absent from this image
            const code_len: u32 = @as(u32, s.end) - s.start;
            if (self.dev.ram_end - code_len + 1 == pc) break o;
        } else null;
        var err: u16 = 0;
        switch (op orelse return) {
            .program => self.program(flash_addr, field4, field6),
            .verify => if (!self.verify(flash_addr, field4, field6)) {
                err = @intFromEnum(ramflash.DriverError.verify_failed);
            },
            .blank_check => if (!self.blankCheck(flash_addr, field6)) {
                err = @intFromEnum(ramflash.DriverError.erase_failed);
            },
            .block_erase => self.eraseBlock(),
            .selective_erase => self.eraseRange(flash_addr, field4 * field6), // sectorSize*count
        }
        self.wr16(h + 0, err); // small: errorCode overwrites flashAddress
    }

    fn statusReg(self: *Sim) u32 {
        return switch (self.dev.family) {
            .cfv1 => if (self.halted) @as(u32, ramflash.CFV1_XCSR_RUNSTATE) else 0,
            else => if (self.halted) @as(u32, ramflash.BDCSCR_BDMACT) else 0,
        };
    }

    fn regIdx(reg_no: u16) usize {
        return @min(reg_no, 15);
    }

    /// Handle a reassembled frame [size, cmd|toggle, params...]; fills resp
    /// (resp[0] = rc | echoed toggle).
    fn handle(self: *Sim, frame: []const u8) void {
        const toggle: u8 = frame[1] & 0x80;
        const cmd: Cmd = @enumFromInt(frame[1] & ~@as(u8, transport.sequence_mask));
        const p = frame[2..];
        var rc: Rc = .ok;
        var data: []const u8 = &.{};
        var scratch: [protocol.max_command_size]u8 = undefined;

        switch (cmd) {
            .set_target, .set_options, .set_vdd, .write_control_reg => {},
            .target_step => {
                if (self.cpu) |c| {
                    c.stepOne();
                    self.halted = c.halted();
                }
            },
            .connect => self.connected = true,
            .target_reset, .target_halt => {
                self.halted = true;
                if (self.cpu) |c| c.setHalted(true); // keep core status in sync
            },
            .target_go => {
                if (self.cpu) |c| {
                    c.go();
                    self.halted = c.halted();
                } else {
                    self.halted = false;
                    self.execFlash();
                    self.halted = true; // routine self-halts on completion
                }
            },
            .set_speed => self.speed_sync = rd16p(p),
            .get_speed => {
                std.mem.writeInt(u16, scratch[0..2], self.speed_sync, .big);
                data = scratch[0..2];
            },
            .get_capabilities => {
                std.mem.writeInt(u16, scratch[0..2], 0, .big); // caps (unused by flash path)
                std.mem.writeInt(u16, scratch[2..4], self.command_buffer_size, .big);
                scratch[4] = 4;
                scratch[5] = 12;
                scratch[6] = 0;
                data = scratch[0..7];
            },
            .get_bdm_status => {
                // 0x89 = ackn(bit0)|sync_done(bits3..4=01)|internal power(bits6..7=10);
                // S_HALT (0x20) when halted (CFV1 gdb continue polls this).
                var w: u16 = if (self.connected) 0x0089 else 0x0000;
                if (self.halted) w |= 0x0020;
                std.mem.writeInt(u16, scratch[0..2], w, .big);
                data = scratch[0..2];
            },
            .control_pins => {
                std.mem.writeInt(u16, scratch[0..2], 0, .big);
                data = scratch[0..2];
            },
            .read_status_reg => {
                const st = if (self.cpu) |c| c.statusReg() else self.statusReg();
                std.mem.writeInt(u32, scratch[0..4], st, .big);
                data = scratch[0..4];
            },
            .write_reg => if (self.cpu) |c| c.writeReg(rd16p(p), rd32p(p[2..])) else {
                self.reg[regIdx(rd16p(p))] = rd32p(p[2..]);
            },
            .read_reg => {
                const v = if (self.cpu) |c| c.readReg(rd16p(p)) else self.reg[regIdx(rd16p(p))];
                std.mem.writeInt(u32, scratch[0..4], v, .big);
                data = scratch[0..4];
            },
            .write_creg => if (self.cpu) |c| c.writeCReg(rd16p(p), rd32p(p[2..])) else {
                self.creg[regIdx(rd16p(p))] = rd32p(p[2..]);
            },
            .read_creg => {
                const v = if (self.cpu) |c| c.readCReg(rd16p(p)) else self.creg[regIdx(rd16p(p))];
                std.mem.writeInt(u32, scratch[0..4], v, .big);
                data = scratch[0..4];
            },
            .write_dreg => self.dreg.put(rd16p(p), rd32p(p[2..])) catch {},
            .read_dreg => {
                std.mem.writeInt(u32, scratch[0..4], self.dreg.get(rd16p(p)) orelse 0, .big);
                data = scratch[0..4];
            },
            .write_mem => {
                // [0]=space, [1]=count, [2..6]=addr, [6..]=data
                const count: u32 = p[1];
                const addr = rd32p(p[2..]);
                // New call when this chunk isn't contiguous or the previous was
                // partial; last_write_addr = base of the call (parameter header).
                const max_chunk: u32 = (@as(u32, self.command_buffer_size) - 8) & ~@as(u32, 3);
                if (addr != self.write_prev_end or !self.write_prev_full) self.last_write_addr = addr;
                self.write_prev_end = addr +% count;
                self.write_prev_full = (count == max_chunk);
                var i: u32 = 0;
                while (i < count) : (i += 1) self.memPut(addr +% i, p[6 + i]);
            },
            .read_mem => {
                const count: u32 = p[1];
                const addr = rd32p(p[2..]);
                var i: u32 = 0;
                while (i < count) : (i += 1) scratch[i] = self.memGet(addr +% i);
                data = scratch[0..count];
            },
            else => rc = .illegal_command,
        }

        self.resp[0] = @intFromEnum(rc) | toggle;
        if (data.len > 0) @memcpy(self.resp[1..][0..data.len], data);
        self.resp_len = 1 + data.len;
    }

    fn rd16p(b: []const u8) u16 {
        return std.mem.readInt(u16, b[0..2], .big);
    }
    fn rd32p(b: []const u8) u32 {
        return std.mem.readInt(u32, b[0..4], .big);
    }

    const vtable = transport.Transport.VTable{ .send = send, .recv = recv, .sleep_ms = sleepMs };

    fn send(ctx: *anyopaque, data: []const u8, _: u32) @import("usb.zig").Error!void {
        const self: *Sim = @ptrCast(@alignCast(ctx));
        if (self.awaiting_cont) {
            // Continuation transfer: [0x00 marker, rest of frame...].
            const rest = data[1..];
            @memcpy(self.frame[self.frame_len..][0..rest.len], rest);
            self.frame_len += rest.len;
            self.awaiting_cont = false;
            self.handle(self.frame[0..self.frame[0]]);
            return;
        }
        const total = data[0];
        if (total <= transport.max_first_transaction) {
            self.handle(data[0..total]);
        } else {
            @memcpy(self.frame[0..data.len], data); // first 62 bytes
            self.frame_len = data.len;
            self.awaiting_cont = true;
        }
    }
    fn recv(ctx: *anyopaque, buf: []u8, _: u32) @import("usb.zig").Error!usize {
        const self: *Sim = @ptrCast(@alignCast(ctx));
        @memcpy(buf[0..self.resp_len], self.resp[0..self.resp_len]);
        return self.resp_len;
    }
    fn sleepMs(_: *anyopaque, _: u32) void {}
};

const testing = std.testing;
const Session = @import("session.zig").Session;

fn hcs08Dev() device.Device {
    return device.lookup("mc9s08qg8").?;
}

test "sim: connect, memory round-trip, and registers through the real Session" {
    var sim = Sim.init(testing.allocator, hcs08Dev());
    defer sim.deinit();
    var s = Session.init(sim.asTransport(.{}));

    try s.setTarget(.hcs08);
    try s.connect();
    try testing.expect(sim.connected);

    try s.writeMem(.byte, 0x0080, &.{ 0xDE, 0xAD, 0xBE, 0xEF });
    var back: [4]u8 = undefined;
    try s.readMem(.byte, 0x0080, &back);
    try testing.expectEqualSlices(u8, &.{ 0xDE, 0xAD, 0xBE, 0xEF }, &back);

    try s.writeReg(@intFromEnum(target.Hcs08Reg.pc), 0x1234);
    try testing.expectEqual(@as(u32, 0x1234), try s.readReg(@intFromEnum(target.Hcs08Reg.pc)));

    var f: [2]u8 = undefined;
    try s.readMem(.byte, hcs08Dev().flash_start, &f);
    try testing.expectEqualSlices(u8, &.{ 0xFF, 0xFF }, &f);
}

test "sim: a large chunked memory write reassembles across the 62-byte split" {
    var sim = Sim.init(testing.allocator, hcs08Dev());
    defer sim.deinit();
    var s = Session.init(sim.asTransport(.{}));
    s.command_buffer_size = 254; // 244-byte chunks -> frames exceed 62 bytes
    var big: [200]u8 = undefined;
    for (&big, 0..) |*b, i| b.* = @truncate(i * 7 + 1);
    try s.writeMem(.byte, 0x0080, &big);
    var back: [200]u8 = undefined;
    try s.readMem(.byte, 0x0080, &back);
    try testing.expectEqualSlices(u8, &big, &back);
}

test "sim: an unarmed `go` (gdb continue) leaves memory untouched" {
    // Regression: an unarmed large-code `go` used to decode RAM as a param
    // block and scribble flash (could u32-overflow).
    var sim = Sim.init(testing.allocator, device.lookup("mc9s12c32").?);
    defer sim.deinit();
    var s = Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs12);
    try s.connect();
    try s.writeMem(.byte, 0x0800, &.{ 0x11, 0x22, 0x33, 0x44 });
    try s.go();
    var back: [4]u8 = undefined;
    try s.readMem(.byte, 0x0800, &back);
    try testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33, 0x44 }, &back);
    // CFV1 all-0xFF garbage-header overflow path can't be reached.
    var cf = Sim.init(testing.allocator, device.lookup("mcf51qe128").?);
    defer cf.deinit();
    var cs = Session.init(cf.asTransport(.{}));
    try cs.setTarget(.cfv1);
    try cs.go(); // last_write_addr=0, flash@0=0xFF -> would overflow if armed+unguarded
}
