//! GDB RSP command dispatch for HCS08/HCS12/ColdFire V1 (pure `Target`-vtable
//! dispatcher). `g`-block order + register sizes MUST match the target XML.

const std = @import("std");
const rsp = @import("gdbstub.zig");

/// HCS08 register set: `g`-block/target.xml order, big-endian per register.
pub const RegDef = struct { name: []const u8, bytes: u8, is_pc: bool = false };
pub const hcs08_regs = [_]RegDef{
    .{ .name = "pc", .bytes = 2, .is_pc = true },
    .{ .name = "sp", .bytes = 2 },
    .{ .name = "hx", .bytes = 2 },
    .{ .name = "a", .bytes = 1 },
    .{ .name = "ccr", .bytes = 1 },
};

pub const target_xml =
    \\<?xml version="1.0"?>
    \\<!DOCTYPE target SYSTEM "gdb-target.dtd">
    \\<target version="1.0">
    \\<feature name="org.gnu.gdb.hcs08.core">
    \\<reg name="pc" bitsize="16" type="code_ptr"/>
    \\<reg name="sp" bitsize="16" type="data_ptr"/>
    \\<reg name="hx" bitsize="16"/>
    \\<reg name="a" bitsize="8"/>
    \\<reg name="ccr" bitsize="8"/>
    \\</feature>
    \\</target>
;

/// HCS12 native m68hc12 register set (gdbarch order, 14B, big-endian).
/// m68hc11-tdep lacks target-description support -> native arch, no <feature>.
pub const hcs12_regs = [_]RegDef{
    .{ .name = "x", .bytes = 2 },
    .{ .name = "d", .bytes = 2 },
    .{ .name = "y", .bytes = 2 },
    .{ .name = "sp", .bytes = 2 },
    .{ .name = "pc", .bytes = 2, .is_pc = true },
    .{ .name = "a", .bytes = 1 },
    .{ .name = "b", .bytes = 1 },
    .{ .name = "ccr", .bytes = 1 },
    .{ .name = "page", .bytes = 1 },
};
pub const hcs12_target_xml =
    \\<?xml version="1.0"?>
    \\<!DOCTYPE target SYSTEM "gdb-target.dtd">
    \\<target version="1.0"><architecture>m68hc12</architecture></target>
;

/// ColdFire V1 (m68k): coldfire.core (no coldfire.fp) = 72B block d0-d7, a0-a5,
/// fp(a6), sp(a7), ps(SR), pc, 4B big-endian; names MUST match m68k_register_names.
pub const cfv1_regs = blk: {
    var r: [18]RegDef = undefined;
    for (0..8) |i| r[i] = .{ .name = std.fmt.comptimePrint("d{d}", .{i}), .bytes = 4 };
    for (0..6) |i| r[8 + i] = .{ .name = std.fmt.comptimePrint("a{d}", .{i}), .bytes = 4 };
    r[14] = .{ .name = "fp", .bytes = 4 };
    r[15] = .{ .name = "sp", .bytes = 4 };
    r[16] = .{ .name = "ps", .bytes = 4 };
    r[17] = .{ .name = "pc", .bytes = 4, .is_pc = true };
    break :blk r;
};
pub const cfv1_target_xml =
    \\<?xml version="1.0"?>
    \\<!DOCTYPE target SYSTEM "gdb-target.dtd">
    \\<target version="1.0"><architecture>m68k:isa-a</architecture>
    \\<feature name="org.gnu.gdb.coldfire.core">
    \\<reg name="d0" bitsize="32" type="int"/><reg name="d1" bitsize="32" type="int"/>
    \\<reg name="d2" bitsize="32" type="int"/><reg name="d3" bitsize="32" type="int"/>
    \\<reg name="d4" bitsize="32" type="int"/><reg name="d5" bitsize="32" type="int"/>
    \\<reg name="d6" bitsize="32" type="int"/><reg name="d7" bitsize="32" type="int"/>
    \\<reg name="a0" bitsize="32" type="data_ptr"/><reg name="a1" bitsize="32" type="data_ptr"/>
    \\<reg name="a2" bitsize="32" type="data_ptr"/><reg name="a3" bitsize="32" type="data_ptr"/>
    \\<reg name="a4" bitsize="32" type="data_ptr"/><reg name="a5" bitsize="32" type="data_ptr"/>
    \\<reg name="fp" bitsize="32" type="data_ptr"/><reg name="sp" bitsize="32" type="data_ptr"/>
    \\<reg name="ps" bitsize="32" type="int"/><reg name="pc" bitsize="32" type="code_ptr"/>
    \\</feature></target>
;

pub const sig_trap: u8 = 5;
pub const sig_int: u8 = 2;

pub const Action = enum { none, detach, kill };

pub const Target = struct {
    ctx: *anyopaque,
    vtable: *const VTable,
    /// `g`/`G` block order + register sizes; must match `xml`. Default HCS08.
    regs: []const RegDef = &hcs08_regs,
    /// The target-description XML served via qXfer:features:read.
    xml: []const u8 = target_xml,
    /// Optional memory-map XML; when set, advertises qXfer:memory-map:read+ and
    /// routes `load` through vFlash* instead of raw memory writes.
    memory_map: ?[]const u8 = null,

    pub const VTable = struct {
        /// Read register `index`; value in the low bytes.
        readReg: *const fn (ctx: *anyopaque, index: usize) anyerror!u32,
        writeReg: *const fn (ctx: *anyopaque, index: usize, value: u32) anyerror!void,
        /// Read up to buf.len bytes at addr; return bytes read.
        readMem: *const fn (ctx: *anyopaque, addr: u32, buf: []u8) anyerror!usize,
        writeMem: *const fn (ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void,
        /// Resume: single-step (step=true) or continue. Return the stop signal.
        resume_: *const fn (ctx: *anyopaque, step: bool) anyerror!u8,
        /// Optional HW exec breakpoints (HCS08 DBG module); null -> Z1/z1 unsupported.
        setHwBreakpoint: ?*const fn (ctx: *anyopaque, addr: u32) anyerror!void = null,
        clearHwBreakpoint: ?*const fn (ctx: *anyopaque, addr: u32) anyerror!void = null,
        /// Optional vFlash* flash-load hooks (need a memory-map; drive GDB `load`).
        flashErase: ?*const fn (ctx: *anyopaque, addr: u32, len: u32) anyerror!void = null,
        flashWrite: ?*const fn (ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void = null,
        flashDone: ?*const fn (ctx: *anyopaque) anyerror!void = null,
        /// Optional `monitor` (qRcmd) handler; write output to `w`. false =
        /// unknown -> empty reply ("not supported").
        monitor: ?*const fn (ctx: *anyopaque, cmd: []const u8, w: *std.Io.Writer) anyerror!bool = null,
    };

    fn readReg(self: Target, i: usize) anyerror!u32 {
        return self.vtable.readReg(self.ctx, i);
    }
    fn writeReg(self: Target, i: usize, v: u32) anyerror!void {
        return self.vtable.writeReg(self.ctx, i, v);
    }
    fn readMem(self: Target, addr: u32, buf: []u8) anyerror!usize {
        return self.vtable.readMem(self.ctx, addr, buf);
    }
    fn writeMem(self: Target, addr: u32, data: []const u8) anyerror!void {
        return self.vtable.writeMem(self.ctx, addr, data);
    }
    fn resume_(self: Target, step: bool) anyerror!u8 {
        return self.vtable.resume_(self.ctx, step);
    }
};

pub const max_packet: usize = 0x400;

/// Dispatch packet `pkt` -> response payload `out` (caller frames via
/// rsp.writePacket); returns the server-loop action.
pub fn dispatch(t: Target, pkt: []const u8, out: *std.Io.Writer) !Action {
    if (pkt.len == 0) return .none;
    switch (pkt[0]) {
        '?' => {
            try stopReply(out, sig_trap);
        },
        'g' => try readAllRegs(t, out),
        'G' => try writeAllRegs(t, pkt[1..], out),
        'p' => try readOneReg(t, pkt[1..], out),
        'P' => try writeOneReg(t, pkt[1..], out),
        'm' => try readMemory(t, pkt[1..], out),
        'M' => try writeMemory(t, pkt[1..], out),
        'c' => try resumeTarget(t, false, out),
        's' => try resumeTarget(t, true, out),
        'k' => return .kill, // no reply
        'D' => {
            try out.writeAll("OK");
            return .detach;
        },
        'q' => try query(t, pkt, out),
        'Z', 'z' => try breakpoint(t, pkt, out),
        'v' => try vCommand(t, pkt, out),
        else => {
            // unknown packet -> empty reply
        },
    }
    return .none;
}

/// `v` packets: vCont resume and the vFlash* load protocol; else empty reply.
fn vCommand(t: Target, pkt: []const u8, out: *std.Io.Writer) !void {
    if (std.mem.eql(u8, pkt, "vCont?")) {
        // resume actions (single-context)
        try out.writeAll("vCont;c;C;s;S");
        return;
    }
    if (std.mem.startsWith(u8, pkt, "vCont;")) {
        // single-threaded: first action only; c/C=continue, s/S=step
        const spec = pkt["vCont;".len..];
        const step = spec.len > 0 and (spec[0] == 's' or spec[0] == 'S');
        const sig = t.resume_(step) catch return err(out, 14);
        try stopReply(out, sig);
        return;
    }
    if (std.mem.startsWith(u8, pkt, "vFlashErase:")) {
        const al = parseAddrLen(pkt["vFlashErase:".len..]) orelse return err(out, 1);
        if (al.len > std.math.maxInt(u32)) return err(out, 1); // guard the u32 cast
        const f = t.vtable.flashErase orelse return; // unsupported -> ""
        f(t.ctx, al.addr, @intCast(al.len)) catch return err(out, 14);
        try out.writeAll("OK");
        return;
    }
    if (std.mem.startsWith(u8, pkt, "vFlashWrite:")) {
        const rest = pkt["vFlashWrite:".len..];
        const colon = std.mem.indexOfScalar(u8, rest, ':') orelse return err(out, 1);
        const a = rsp.parseHex(rest[0..colon]) orelse return err(out, 1);
        if (a > std.math.maxInt(u32)) return err(out, 1);
        const f = t.vtable.flashWrite orelse return;
        // after the 2nd ':' = raw binary flash data (escapes already resolved)
        f(t.ctx, @intCast(a), rest[colon + 1 ..]) catch return err(out, 14);
        try out.writeAll("OK");
        return;
    }
    if (std.mem.eql(u8, pkt, "vFlashDone")) {
        if (t.vtable.flashDone) |f| f(t.ctx) catch return err(out, 14);
        try out.writeAll("OK");
        return;
    }
    // unhandled v-packet -> empty reply
}

fn stopReply(out: *std.Io.Writer, signal: u8) !void {
    try out.print("S{x:0>2}", .{signal});
}

fn readAllRegs(t: Target, out: *std.Io.Writer) !void {
    for (t.regs, 0..) |r, i| {
        const v = t.readReg(i) catch {
            // unavailable register -> 'xx' pairs
            for (0..r.bytes) |_| try out.writeAll("xx");
            continue;
        };
        try rsp.appendHexInt(out, v, r.bytes);
    }
}

fn writeAllRegs(t: Target, args: []const u8, out: *std.Io.Writer) !void {
    var pos: usize = 0;
    for (t.regs, 0..) |r, i| {
        const hexlen = r.bytes * 2;
        if (pos + hexlen > args.len) return err(out, 1);
        const v = rsp.parseHex(args[pos .. pos + hexlen]) orelse return err(out, 1);
        t.writeReg(i, @intCast(v)) catch return err(out, 2);
        pos += hexlen;
    }
    try out.writeAll("OK");
}

fn readOneReg(t: Target, args: []const u8, out: *std.Io.Writer) !void {
    const n = rsp.parseHex(args) orelse return err(out, 1);
    if (n >= t.regs.len) return err(out, 1);
    const v = t.readReg(@intCast(n)) catch return err(out, 2);
    try rsp.appendHexInt(out, v, t.regs[@intCast(n)].bytes);
}

fn writeOneReg(t: Target, args: []const u8, out: *std.Io.Writer) !void {
    const eqi = std.mem.indexOfScalar(u8, args, '=') orelse return err(out, 1);
    const n = rsp.parseHex(args[0..eqi]) orelse return err(out, 1);
    if (n >= t.regs.len) return err(out, 1);
    const v = rsp.parseHex(args[eqi + 1 ..]) orelse return err(out, 1);
    t.writeReg(@intCast(n), @intCast(v)) catch return err(out, 2);
    try out.writeAll("OK");
}

/// Parse "addr,len"; rejects addr > 32-bit (null, not truncation). Caller
/// clamps len to its buffer.
fn parseAddrLen(args: []const u8) ?struct { addr: u32, len: usize } {
    const comma = std.mem.indexOfScalar(u8, args, ',') orelse return null;
    const addr = rsp.parseHex(args[0..comma]) orelse return null;
    const len = rsp.parseHex(args[comma + 1 ..]) orelse return null;
    if (addr > std.math.maxInt(u32) or len > std.math.maxInt(usize)) return null;
    return .{ .addr = @intCast(addr), .len = @intCast(len) };
}

fn readMemory(t: Target, args: []const u8, out: *std.Io.Writer) !void {
    const al = parseAddrLen(args) orelse return err(out, 1);
    if (al.len == 0) return;
    var buf: [max_packet / 2]u8 = undefined;
    const want = @min(al.len, buf.len);
    const n = t.readMem(al.addr, buf[0..want]) catch return err(out, 14);
    try rsp.writeHexBytes(out, buf[0..n]);
}

fn writeMemory(t: Target, args: []const u8, out: *std.Io.Writer) !void {
    const colon = std.mem.indexOfScalar(u8, args, ':') orelse return err(out, 1);
    const al = parseAddrLen(args[0..colon]) orelse return err(out, 1);
    var buf: [max_packet / 2]u8 = undefined;
    const nbytes = rsp.parseHexBytes(args[colon + 1 ..], &buf) orelse return err(out, 1);
    if (nbytes != al.len) return err(out, 1);
    t.writeMem(al.addr, buf[0..nbytes]) catch return err(out, 14);
    try out.writeAll("OK");
}

fn resumeTarget(t: Target, step: bool, out: *std.Io.Writer) !void {
    const sig = t.resume_(step) catch return err(out, 14);
    try stopReply(out, sig);
}

/// Z/z<type>,addr,kind. Only type 1 (HW exec breakpoint) handled; else empty
/// reply -> GDB uses software breakpoints (memory writes).
fn breakpoint(t: Target, pkt: []const u8, out: *std.Io.Writer) !void {
    if (pkt.len < 4 or pkt[2] != ',') return; // malformed/empty -> unsupported
    const insert = pkt[0] == 'Z';
    if (pkt[1] != '1') return; // type 1 (HW exec) only
    const rest = pkt[3..];
    const comma = std.mem.indexOfScalar(u8, rest, ',') orelse rest.len;
    const a = rsp.parseHex(rest[0..comma]) orelse return err(out, 1);
    if (a > std.math.maxInt(u32)) return err(out, 1);
    const addr: u32 = @intCast(a);
    if (insert) {
        const f = t.vtable.setHwBreakpoint orelse return; // unsupported -> ""
        f(t.ctx, addr) catch return err(out, 2);
    } else {
        const f = t.vtable.clearHwBreakpoint orelse return;
        f(t.ctx, addr) catch return err(out, 2);
    }
    try out.writeAll("OK");
}

fn query(t: Target, pkt: []const u8, out: *std.Io.Writer) !void {
    if (std.mem.startsWith(u8, pkt, "qSupported")) {
        try out.print("PacketSize={x};qXfer:features:read+;qAttached+", .{max_packet});
        if (t.memory_map != null) try out.writeAll(";qXfer:memory-map:read+");
        return;
    }
    if (std.mem.eql(u8, pkt, "qAttached") or std.mem.startsWith(u8, pkt, "qAttached:")) {
        try out.writeAll("1"); // bare-metal target always pre-exists
        return;
    }
    if (std.mem.startsWith(u8, pkt, "qXfer:features:read:")) {
        try qXferFeatures(t.xml, pkt["qXfer:features:read:".len..], out);
        return;
    }
    if (std.mem.startsWith(u8, pkt, "qXfer:memory-map:read:")) {
        const mm = t.memory_map orelse {
            try out.writeAll("E00");
            return;
        };
        // memory-map: empty annex, rest is ":<offset>,<len>"
        const rest = pkt["qXfer:memory-map:read:".len..];
        const colon = std.mem.indexOfScalar(u8, rest, ':') orelse return err(out, 1);
        try serveXferChunk(mm, rest[colon + 1 ..], out);
        return;
    }
    if (std.mem.startsWith(u8, pkt, "qRcmd,")) {
        try monitorCmd(t, pkt["qRcmd,".len..], out);
        return;
    }
    // unhandled query -> empty reply
}

/// `monitor` (qRcmd,<hex>): decode, dispatch to the monitor hook, hex-encode
/// its output as the reply. Unknown/absent -> empty reply.
fn monitorCmd(t: Target, hex: []const u8, out: *std.Io.Writer) !void {
    const f = t.vtable.monitor orelse return;
    var cmdbuf: [256]u8 = undefined;
    const n = rsp.parseHexBytes(hex, &cmdbuf) orelse return err(out, 1);
    var msgbuf: [512]u8 = undefined;
    var mw = std.Io.Writer.fixed(&msgbuf);
    const handled = f(t.ctx, cmdbuf[0..n], &mw) catch return err(out, 1);
    if (!handled) return; // -> GDB "not supported"
    if (mw.buffered().len == 0) {
        try out.writeAll("OK");
        return;
    }
    try rsp.writeHexBytes(out, mw.buffered());
}

/// qXfer:features:read:<annex>:<offset>,<length>.
fn qXferFeatures(xml: []const u8, rest: []const u8, out: *std.Io.Writer) !void {
    const colon = std.mem.indexOfScalar(u8, rest, ':') orelse return err(out, 1);
    const annex = rest[0..colon];
    if (!std.mem.eql(u8, annex, "target.xml")) {
        try out.writeAll("E00");
        return;
    }
    try serveXferChunk(xml, rest[colon + 1 ..], out);
}

/// Serve a qXfer window of `blob`; prefix 'l' (last) or 'm' (more follows).
fn serveXferChunk(blob: []const u8, off_len: []const u8, out: *std.Io.Writer) !void {
    const ol = parseAddrLen(off_len) orelse return err(out, 1);
    if (ol.addr >= blob.len) {
        try out.writeAll("l");
        return;
    }
    // clamp via subtraction: addr+len could overflow (hostile len up to
    // usize-max); ol.addr < blob.len above, so `remaining` can't underflow.
    const remaining = blob.len - ol.addr;
    const end = ol.addr + @min(ol.len, remaining);
    try out.writeByte(if (end == blob.len) 'l' else 'm');
    try out.writeAll(blob[ol.addr..end]);
}

fn err(out: *std.Io.Writer, code: u8) !void {
    try out.print("E{x:0>2}", .{code});
}

const testing = std.testing;

const MockTarget = struct {
    regs: [hcs08_regs.len]u32 = .{ 0x1234, 0x0250, 0xABCD, 0x7F, 0x60 },
    mem: [0x10000]u8 = [_]u8{0} ** 0x10000,
    resumed_step: ?bool = null,
    breakpoint: ?u32 = null,
    flash_erased: ?struct { addr: u32, len: u32 } = null,
    flash_written: ?struct { addr: u32, len: usize } = null,
    flash_done: bool = false,

    fn readReg(ctx: *anyopaque, i: usize) anyerror!u32 {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        return self.regs[i];
    }
    fn writeReg(ctx: *anyopaque, i: usize, v: u32) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.regs[i] = v;
    }
    fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!usize {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        @memcpy(buf, self.mem[addr..][0..buf.len]);
        return buf.len;
    }
    fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        @memcpy(self.mem[addr..][0..data.len], data);
    }
    fn resume_(ctx: *anyopaque, step: bool) anyerror!u8 {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.resumed_step = step;
        return sig_trap;
    }
    fn setBp(ctx: *anyopaque, addr: u32) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.breakpoint = addr;
    }
    fn clearBp(ctx: *anyopaque, addr: u32) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        if (self.breakpoint == addr) self.breakpoint = null;
    }
    fn flashErase(ctx: *anyopaque, addr: u32, len: u32) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.flash_erased = .{ .addr = addr, .len = len };
    }
    fn flashWrite(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.flash_written = .{ .addr = addr, .len = data.len };
        @memcpy(self.mem[addr..][0..data.len], data);
    }
    fn flashDone(ctx: *anyopaque) anyerror!void {
        const self: *MockTarget = @ptrCast(@alignCast(ctx));
        self.flash_done = true;
    }
    fn monitor(ctx: *anyopaque, cmd: []const u8, w: *std.Io.Writer) anyerror!bool {
        _ = ctx;
        if (std.mem.eql(u8, cmd, "reset")) {
            try w.writeAll("ok");
            return true;
        }
        return false;
    }

    const vtable = Target.VTable{
        .readReg = readReg,
        .writeReg = writeReg,
        .readMem = readMem,
        .writeMem = writeMem,
        .resume_ = resume_,
        .setHwBreakpoint = setBp,
        .clearHwBreakpoint = clearBp,
        .flashErase = flashErase,
        .flashWrite = flashWrite,
        .flashDone = flashDone,
        .monitor = monitor,
    };
    const memory_map_xml =
        \\<memory-map><memory type="flash" start="0xe000" length="0x2000"><property name="blocksize">0x200</property></memory></memory-map>
    ;
    fn targetWithFlash(self: *MockTarget) Target {
        return .{ .ctx = self, .vtable = &vtable, .memory_map = memory_map_xml };
    }
    fn target(self: *MockTarget) Target {
        return .{ .ctx = self, .vtable = &vtable };
    }
};

fn MockTargetN(comptime n: usize) type {
    return struct {
        const Self = @This();
        regs: [n]u32 = [_]u32{0} ** n,
        fn readReg(ctx: *anyopaque, i: usize) anyerror!u32 {
            const self: *Self = @ptrCast(@alignCast(ctx));
            return self.regs[i];
        }
        fn writeReg(ctx: *anyopaque, i: usize, v: u32) anyerror!void {
            const self: *Self = @ptrCast(@alignCast(ctx));
            self.regs[i] = v;
        }
        fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!usize {
            _ = ctx;
            _ = addr;
            @memset(buf, 0);
            return buf.len;
        }
        fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
            _ = ctx;
            _ = addr;
            _ = data;
        }
        fn resume_(ctx: *anyopaque, step: bool) anyerror!u8 {
            _ = ctx;
            _ = step;
            return sig_trap;
        }
        const vtable = Target.VTable{
            .readReg = readReg,
            .writeReg = writeReg,
            .readMem = readMem,
            .writeMem = writeMem,
            .resume_ = resume_,
        };
    };
}

fn run(t: Target, pkt: []const u8, buf: []u8) !struct { resp: []u8, action: Action } {
    var w = std.Io.Writer.fixed(buf);
    const action = try dispatch(t, pkt, &w);
    return .{ .resp = w.buffered(), .action = action };
}

test "register-set g-block sizes match each architecture" {
    var hcs08_bytes: usize = 0;
    for (hcs08_regs) |r| hcs08_bytes += r.bytes;
    try testing.expectEqual(@as(usize, 8), hcs08_bytes); // pc,sp,hx (2) + a,ccr (1)

    var hcs12_bytes: usize = 0;
    for (hcs12_regs) |r| hcs12_bytes += r.bytes;
    try testing.expectEqual(@as(usize, 14), hcs12_bytes); // native m68hc12

    var cfv1_bytes: usize = 0;
    for (cfv1_regs) |r| cfv1_bytes += r.bytes;
    try testing.expectEqual(@as(usize, 72), cfv1_bytes); // 18 * 4, coldfire.core
}

test "g works with the CFV1 register set (18 x 4 bytes)" {
    var m = MockTargetN(18){};
    for (&m.regs, 0..) |*r, i| r.* = @intCast(i);
    var buf: [256]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    const t = Target{ .ctx = &m, .vtable = &MockTargetN(18).vtable, .regs = &cfv1_regs, .xml = cfv1_target_xml };
    _ = try dispatch(t, "g", &w);
    try testing.expectEqual(@as(usize, 72 * 2), w.buffered().len); // 2 hex chars/byte
}

test "? reports SIGTRAP" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "?", &buf);
    try testing.expectEqualStrings("S05", r.resp);
}

test "g reads all registers big-endian in declared order" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "g", &buf);
    // pc=1234 sp=0250 hx=abcd a=7f ccr=60
    try testing.expectEqualStrings("12340250abcd7f60", r.resp);
}

test "G writes all registers" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "G0001000200037f60", &buf);
    try testing.expectEqualStrings("OK", r.resp);
    try testing.expectEqual(@as(u32, 0x0001), m.regs[0]);
    try testing.expectEqual(@as(u32, 0x0002), m.regs[1]);
    try testing.expectEqual(@as(u32, 0x0003), m.regs[2]);
    try testing.expectEqual(@as(u32, 0x7f), m.regs[3]);
    try testing.expectEqual(@as(u32, 0x60), m.regs[4]);
}

test "p reads a single register" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "p0", &buf); // pc
    try testing.expectEqualStrings("1234", r.resp);
    const r2 = try run(m.target(), "p3", &buf); // a (1 byte)
    try testing.expectEqualStrings("7f", r2.resp);
}

test "P writes a single register" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "P1=beef", &buf);
    try testing.expectEqualStrings("OK", r.resp);
    try testing.expectEqual(@as(u32, 0xbeef), m.regs[1]);
}

test "p on an out-of-range register errors" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "p9", &buf);
    try testing.expectEqualStrings("E01", r.resp);
}

test "m reads memory as a byte stream" {
    var m = MockTarget{};
    m.mem[0x1000] = 0xDE;
    m.mem[0x1001] = 0xAD;
    m.mem[0x1002] = 0xBE;
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "m1000,3", &buf);
    try testing.expectEqualStrings("deadbe", r.resp);
}

test "M writes memory" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "M2000,2:cafe", &buf);
    try testing.expectEqualStrings("OK", r.resp);
    try testing.expectEqual(@as(u8, 0xca), m.mem[0x2000]);
    try testing.expectEqual(@as(u8, 0xfe), m.mem[0x2001]);
}

test "m with an address wider than 32 bits errors instead of crashing" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "m1234567890,4", &buf); // 0x1234567890 > u32
    try testing.expectEqualStrings("E01", r.resp);
}

test "M with mismatched length errors" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const r = try run(m.target(), "M2000,3:cafe", &buf); // says 3 bytes, gives 2
    try testing.expectEqualStrings("E01", r.resp);
}

test "s single-steps and c continues, both returning a stop reply" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const rs = try run(m.target(), "s", &buf);
    try testing.expectEqualStrings("S05", rs.resp);
    try testing.expectEqual(@as(?bool, true), m.resumed_step);

    m.resumed_step = null;
    const rc = try run(m.target(), "c", &buf);
    try testing.expectEqualStrings("S05", rc.resp);
    try testing.expectEqual(@as(?bool, false), m.resumed_step);
}

test "qSupported advertises PacketSize and target.xml" {
    var m = MockTarget{};
    var buf: [128]u8 = undefined;
    const r = try run(m.target(), "qSupported:multiprocess+;swbreak+", &buf);
    try testing.expectEqualStrings("PacketSize=400;qXfer:features:read+;qAttached+", r.resp);
}

test "qAttached reports 1 for a pre-existing target" {
    var m = MockTarget{};
    var buf: [16]u8 = undefined;
    const r = try run(m.target(), "qAttached", &buf);
    try testing.expectEqualStrings("1", r.resp);
}

test "qXfer serves target.xml in one final chunk" {
    var m = MockTarget{};
    var buf: [1024]u8 = undefined;
    const r = try run(m.target(), "qXfer:features:read:target.xml:0,3e8", &buf);
    try testing.expectEqual(@as(u8, 'l'), r.resp[0]);
    try testing.expectEqualStrings(target_xml, r.resp[1..]);
}

test "qXfer past the end returns l" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    var offbuf: [64]u8 = undefined;
    const req = std.fmt.bufPrint(&offbuf, "qXfer:features:read:target.xml:{x},10", .{target_xml.len}) catch unreachable;
    const r = try run(m.target(), req, &buf);
    try testing.expectEqualStrings("l", r.resp);
}

test "detach replies OK and signals detach; kill signals kill with no reply" {
    var m = MockTarget{};
    var buf: [16]u8 = undefined;
    const rd = try run(m.target(), "D", &buf);
    try testing.expectEqualStrings("OK", rd.resp);
    try testing.expectEqual(Action.detach, rd.action);

    const rk = try run(m.target(), "k", &buf);
    try testing.expectEqualStrings("", rk.resp);
    try testing.expectEqual(Action.kill, rk.action);
}

test "unknown and software-breakpoint packets get an empty reply" {
    var m = MockTarget{};
    var buf: [16]u8 = undefined;
    try testing.expectEqualStrings("", (try run(m.target(), "vMustReplyEmpty", &buf)).resp);
    // Z0 (software bp) unsupported -> empty
    try testing.expectEqualStrings("", (try run(m.target(), "Z0,1000,1", &buf)).resp);
    try testing.expectEqualStrings("", (try run(m.target(), "H", &buf)).resp);
}

test "Z1/z1 set and clear a hardware breakpoint" {
    var m = MockTarget{};
    var buf: [16]u8 = undefined;
    const ri = try run(m.target(), "Z1,e00a,1", &buf);
    try testing.expectEqualStrings("OK", ri.resp);
    try testing.expectEqual(@as(?u32, 0xE00A), m.breakpoint);
    const rr = try run(m.target(), "z1,e00a,1", &buf);
    try testing.expectEqualStrings("OK", rr.resp);
    try testing.expectEqual(@as(?u32, null), m.breakpoint);
}

test "qSupported advertises memory-map only when the target has one" {
    var m = MockTarget{};
    var buf: [128]u8 = undefined;
    const r1 = try run(m.target(), "qSupported", &buf);
    try testing.expect(std.mem.indexOf(u8, r1.resp, "memory-map") == null);
    const r2 = try run(m.targetWithFlash(), "qSupported", &buf);
    try testing.expect(std.mem.indexOf(u8, r2.resp, "qXfer:memory-map:read+") != null);
}

test "qXfer:memory-map serves the map (empty annex)" {
    var m = MockTarget{};
    var buf: [256]u8 = undefined;
    const r = try run(m.targetWithFlash(), "qXfer:memory-map:read::0,3e8", &buf);
    try testing.expectEqual(@as(u8, 'l'), r.resp[0]);
    try testing.expectEqualStrings(MockTarget.memory_map_xml, r.resp[1..]);
}

test "vFlashErase / vFlashWrite / vFlashDone drive the flash hooks" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    const re = try run(m.targetWithFlash(), "vFlashErase:e000,2000", &buf);
    try testing.expectEqualStrings("OK", re.resp);
    try testing.expectEqual(@as(u32, 0xE000), m.flash_erased.?.addr);
    try testing.expectEqual(@as(u32, 0x2000), m.flash_erased.?.len);
    // payload after the 2nd ':' = raw bytes
    const rw = try run(m.targetWithFlash(), "vFlashWrite:e000:\x11\x22\x33", &buf);
    try testing.expectEqualStrings("OK", rw.resp);
    try testing.expectEqual(@as(u32, 0xE000), m.flash_written.?.addr);
    try testing.expectEqual(@as(usize, 3), m.flash_written.?.len);
    try testing.expectEqual(@as(u8, 0x11), m.mem[0xE000]);
    const rd = try run(m.targetWithFlash(), "vFlashDone", &buf);
    try testing.expectEqualStrings("OK", rd.resp);
    try testing.expect(m.flash_done);
}

test "vFlash* is unsupported (empty reply) when the target has no flash hooks" {
    var m = MockTargetN(5){};
    var buf: [32]u8 = undefined;
    const t = Target{ .ctx = &m, .vtable = &MockTargetN(5).vtable };
    try testing.expectEqualStrings("", (try run(t, "vFlashErase:e000,10", &buf)).resp);
}

test "vFlashErase with a >u32 length errors instead of panicking" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    // > u32 max - must not panic on the cast
    try testing.expectEqualStrings("E01", (try run(m.targetWithFlash(), "vFlashErase:0,100000000", &buf)).resp);
}

test "qXfer with a near-u64::MAX length doesn't overflow/panic" {
    var m = MockTarget{};
    var buf: [512]u8 = undefined;
    const r = try run(m.targetWithFlash(), "qXfer:memory-map:read::1,ffffffffffffffff", &buf);
    try testing.expect(r.resp[0] == 'l' or r.resp[0] == 'm'); // served, not crashed
    const r2 = try run(m.target(), "qXfer:features:read:target.xml:1,ffffffffffffffff", &buf);
    try testing.expect(r2.resp[0] == 'l' or r2.resp[0] == 'm');
}

test "vCont? advertises actions; vCont;c continues; vCont;s steps" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    try testing.expectEqualStrings("vCont;c;C;s;S", (try run(m.target(), "vCont?", &buf)).resp);
    m.resumed_step = null;
    try testing.expectEqualStrings("S05", (try run(m.target(), "vCont;c", &buf)).resp);
    try testing.expectEqual(@as(?bool, false), m.resumed_step);
    try testing.expectEqualStrings("S05", (try run(m.target(), "vCont;s:1", &buf)).resp);
    try testing.expectEqual(@as(?bool, true), m.resumed_step);
}

test "qRcmd (monitor) dispatches; unknown command -> empty reply" {
    var m = MockTarget{};
    var buf: [64]u8 = undefined;
    // "reset"=7265736574; handler "ok" -> 6f6b
    try testing.expectEqualStrings("6f6b", (try run(m.target(), "qRcmd,7265736574", &buf)).resp);
    // unknown -> empty reply
    try testing.expectEqualStrings("", (try run(m.target(), "qRcmd,626f677573", &buf)).resp);
}

// dispatch() runs on attacker-controlled payloads: must never panic on any bytes.
test "fuzz: dispatch never panics on adversarial packet payloads" {
    var prng = std.Random.DefaultPrng.init(0x5AFED00D);
    const rand = prng.random();
    var m = MockTarget{};
    const t = m.targetWithFlash();
    var pkt: [512]u8 = undefined;
    var outbuf: [2 * max_packet]u8 = undefined;
    const cmds = "gGpPmMcsvqZzD?kH"; // bias the first byte toward real commands
    var it: usize = 0;
    while (it < 40000) : (it += 1) {
        const len = rand.intRangeAtMost(usize, 0, pkt.len);
        const p = pkt[0..len];
        rand.bytes(p);
        if (len > 0 and rand.boolean()) p[0] = cmds[rand.intRangeAtMost(usize, 0, cmds.len - 1)];
        var w = std.Io.Writer.fixed(&outbuf);
        _ = dispatch(t, p, &w) catch {};
    }
}

test "Z1 reports unsupported when the target has no hw breakpoints" {
    var m = MockTarget{};
    var t = m.target();
    // target build without breakpoint support
    const NoBp = struct {
        const vt = Target.VTable{
            .readReg = MockTarget.readReg,
            .writeReg = MockTarget.writeReg,
            .readMem = MockTarget.readMem,
            .writeMem = MockTarget.writeMem,
            .resume_ = MockTarget.resume_,
        };
    };
    t.vtable = &NoBp.vt;
    var buf: [16]u8 = undefined;
    try testing.expectEqualStrings("", (try run(t, "Z1,e00a,1", &buf)).resp);
}
