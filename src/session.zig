//! BDM session: typed USBDM commands over a Transport. Layouts follow the V4.12
//! firmware handlers (CmdProcessing*.c, cited per method); wire is big-endian.

const std = @import("std");
const protocol = @import("protocol.zig");
const target = @import("target.zig");
const transport = @import("transport.zig");

pub const Error = transport.Error || error{
    /// Non-OK BDM_RC code; see Session.last_rc.
    BdmError,
    UnalignedLength,
};

/// Firmware default 0x1801 (HCS08_SBDFR_DEFAULT).
pub const hcs08_sbdfr_default: u16 = 0x1801;

/// BDM_Option_t image for CMD_USBDM_SET_OPTIONS; firmware memcpy's it over its
/// struct, so layout must match BDM.h exactly: [flags, targetVdd, useAltBDMClock,
/// autoReconnect, SBDFRaddress.hi, SBDFRaddress.lo, reserved x3].
pub const Options = struct {
    cycle_vdd_on_reset: bool = false,
    cycle_vdd_on_connect: bool = false,
    leave_target_powered: bool = false,
    guess_speed: bool = true,
    use_reset_signal: bool = false,
    target_vdd: target.VddSelect = .off,
    /// ClkSwValues_t: 0xFF = don't touch target clock selection.
    use_alt_bdm_clock: u8 = 0xFF,
    /// AutoConnect_t: reconnect on USBDM_ReadStatusReg().
    auto_reconnect: u8 = 1,
    sbdfr_address: u16 = hcs08_sbdfr_default,

    pub fn serialize(self: Options) [9]u8 {
        // HC08 CodeWarrior packs bitfields from the LSB (cycleVddOnReset = bit 0).
        var flags: u8 = 0;
        if (self.cycle_vdd_on_reset) flags |= 1 << 0;
        if (self.cycle_vdd_on_connect) flags |= 1 << 1;
        if (self.leave_target_powered) flags |= 1 << 2;
        if (self.guess_speed) flags |= 1 << 3;
        if (self.use_reset_signal) flags |= 1 << 4;
        return .{
            flags,
            @intFromEnum(self.target_vdd),
            self.use_alt_bdm_clock,
            self.auto_reconnect,
            @intCast(self.sbdfr_address >> 8),
            @truncate(self.sbdfr_address),
            0,
            0,
            0,
        };
    }
};

pub const CapabilitiesInfo = struct {
    caps: target.Capabilities,
    /// Firmware command buffer size; bounds transfer chunks.
    max_command_size: u16,
    /// Extended firmware version (major, minor, micro).
    version: [3]u8,
};

/// SYNC value = length of the 128-BDM-clock SYNC pulse in 60 MHz ticks, so
/// value = 128*60e6/f (matches USBDM_API.cpp SetSpeed/GetSpeedHz). NOTE: BDM.h's
/// SYNC_MULTIPLE has an extra x2 (xtal, not BDM clock) - NOT this formula.
pub const sync_numerator: u64 = 128 * 60_000_000;

pub fn syncToHz(sync: u16) u32 {
    if (sync == 0) return 0;
    // firmware returns sync==1 for "unknown speed"; clamp so it doesn't overflow u32.
    return @intCast(@min(sync_numerator / sync, std.math.maxInt(u32)));
}

pub fn hzToSync(hz: u32) error{OutOfRange}!u16 {
    if (hz == 0) return error.OutOfRange;
    const sync = sync_numerator / hz;
    if (sync == 0 or sync > std.math.maxInt(u16)) return error.OutOfRange;
    return @intCast(sync);
}

pub const Session = struct {
    tp: transport.Transport,
    last_rc: protocol.Rc = .ok,
    buf: [256]u8 = undefined,
    /// Device command-buffer size from GET_CAPABILITIES. Variant-specific (254
    /// JMxx/Kinetis, 145 JS16/FZ0622C), so chunks are sized from the reported
    /// value. Default 128 (DEFAULT_PACKET_SIZE = 2*64) until capabilities() runs,
    /// so a transfer can't overflow an unqueried device.
    command_buffer_size: u16 = 128,

    pub fn init(tp: transport.Transport) Session {
        return .{ .tp = tp };
    }

    fn run(self: *Session, cmd: protocol.Command, params: []const u8) Error![]u8 {
        const resp = try self.tp.transact(cmd, params, &self.buf);
        self.last_rc = resp.rc;
        if (resp.rc != .ok) return error.BdmError;
        return resp.data;
    }

    /// f_CMD_SET_TARGET: [2] = target type.
    pub fn setTarget(self: *Session, t: target.TargetType) Error!void {
        _ = try self.run(.set_target, &.{@intFromEnum(t)});
    }

    /// f_CMD_SET_OPTIONS: [2..10] = BDM_Option_t image.
    pub fn setOptions(self: *Session, options: Options) Error!void {
        _ = try self.run(.set_options, &options.serialize());
    }

    /// f_CMD_GET_CAPABILITIES: returns [1..2] caps, [3..4] buffer size,
    /// [5..7] extended version - all big-endian.
    pub fn capabilities(self: *Session) Error!CapabilitiesInfo {
        const d = try self.run(.get_capabilities, &.{});
        if (d.len < 7) return error.ResponseTooShort;
        const info: CapabilitiesInfo = .{
            .caps = target.Capabilities.decode(std.mem.readInt(u16, d[0..2], .big)),
            .max_command_size = std.mem.readInt(u16, d[2..4], .big),
            .version = .{ d[4], d[5], d[6] },
        };
        // clamp reported size to a sane range and our frame buffer.
        self.command_buffer_size = std.math.clamp(info.max_command_size, 64, protocol.max_command_size);
        return info;
    }

    /// f_CMD_GET_BDM_STATUS: returns [1..2] = 16-bit status word.
    pub fn bdmStatus(self: *Session) Error!target.BdmStatus {
        const d = try self.run(.get_bdm_status, &.{});
        if (d.len < 2) return error.ResponseTooShort;
        return target.BdmStatus.decode(std.mem.readInt(u16, d[0..2], .big));
    }

    /// f_CMD_SET_VDD: [2..3] = 16-bit control, only LSB used.
    pub fn setVdd(self: *Session, vdd: target.VddSelect) Error!void {
        _ = try self.run(.set_vdd, &.{ 0, @intFromEnum(vdd) });
    }

    /// f_CMD_CONNECT: no parameters.
    pub fn connect(self: *Session) Error!void {
        _ = try self.run(.connect, &.{});
    }

    /// Reference connect ladder (targetConnectWithRetry): try, one quiet retry,
    /// then up to two reset cycles + reconnect. Each cycle resets TWICE - a
    /// watchdog workaround for HCS08/RS08/CFV1 (set `allow_reset` for those).
    pub fn connectWithRetry(self: *Session, allow_reset: bool) Error!void {
        var attempt: usize = 0;
        while (true) : (attempt += 1) {
            if (self.connect()) |_| {
                return;
            } else |err| {
                if (err != error.BdmError) return err;
                if (attempt == 0) continue; // quiet retry
                if (allow_reset and attempt <= 2) {
                    self.reset(.special, .all) catch {};
                    self.reset(.special, .all) catch {};
                    continue;
                }
                return err;
            }
        }
    }

    /// f_CMD_SET_SPEED: [2..3] = 16-bit SYNC value (0 re-enables auto).
    pub fn setSpeedSync(self: *Session, sync: u16) Error!void {
        var p: [2]u8 = undefined;
        std.mem.writeInt(u16, &p, sync, .big);
        _ = try self.run(.set_speed, &p);
    }

    /// f_CMD_GET_SPEED: returns [1..2] = 16-bit SYNC value.
    pub fn getSpeedSync(self: *Session) Error!u16 {
        const d = try self.run(.get_speed, &.{});
        if (d.len < 2) return error.ResponseTooShort;
        return std.mem.readInt(u16, d[0..2], .big);
    }

    /// f_CMD_READ_STATUS_REG: returns [1..4] = 32-bit value (BDCSCR/BDMSTS/XCSR).
    pub fn readStatusReg(self: *Session) Error!u32 {
        const d = try self.run(.read_status_reg, &.{});
        if (d.len < 4) return error.ResponseTooShort;
        return std.mem.readInt(u32, d[0..4], .big);
    }

    /// f_CMD_WRITE_CONTROL_REG: [2..5] = 32-bit value, only LSB used for HCS.
    pub fn writeControlReg(self: *Session, value: u32) Error!void {
        var p: [4]u8 = undefined;
        std.mem.writeInt(u32, &p, value, .big);
        _ = try self.run(.write_control_reg, &p);
    }

    /// f_CMD_RESET: [2] = TargetMode_t. NOTE: on success firmware invalidates
    /// the connection speed; reconnect before further ops.
    pub fn reset(self: *Session, mode: target.ResetMode, method: target.ResetMethod) Error!void {
        _ = try self.run(.target_reset, &.{target.resetByte(mode, method)});
    }

    pub fn step(self: *Session) Error!void {
        _ = try self.run(.target_step, &.{});
    }

    pub fn go(self: *Session) Error!void {
        _ = try self.run(.target_go, &.{});
    }

    pub fn halt(self: *Session) Error!void {
        _ = try self.run(.target_halt, &.{});
    }

    /// f_CMD_*_READ_REG: [2..3] = 16-bit regNo; returns [1..4] = 32-bit value.
    pub fn readReg(self: *Session, reg_no: u16) Error!u32 {
        return self.readRegLike(.read_reg, reg_no);
    }

    pub fn readCReg(self: *Session, reg_no: u16) Error!u32 {
        return self.readRegLike(.read_creg, reg_no);
    }

    pub fn readDReg(self: *Session, reg_no: u16) Error!u32 {
        return self.readRegLike(.read_dreg, reg_no);
    }

    fn readRegLike(self: *Session, cmd: protocol.Command, reg_no: u16) Error!u32 {
        var p: [2]u8 = undefined;
        std.mem.writeInt(u16, &p, reg_no, .big);
        const d = try self.run(cmd, &p);
        if (d.len < 4) return error.ResponseTooShort;
        return std.mem.readInt(u32, d[0..4], .big);
    }

    /// f_CMD_*_WRITE_REG: [2..3] = 16-bit regNo, [4..7] = 32-bit value.
    pub fn writeReg(self: *Session, reg_no: u16, value: u32) Error!void {
        return self.writeRegLike(.write_reg, reg_no, value);
    }

    pub fn writeCReg(self: *Session, reg_no: u16, value: u32) Error!void {
        return self.writeRegLike(.write_creg, reg_no, value);
    }

    pub fn writeDReg(self: *Session, reg_no: u16, value: u32) Error!void {
        return self.writeRegLike(.write_dreg, reg_no, value);
    }

    fn writeRegLike(self: *Session, cmd: protocol.Command, reg_no: u16, value: u32) Error!void {
        var p: [6]u8 = undefined;
        std.mem.writeInt(u16, p[0..2], reg_no, .big);
        std.mem.writeInt(u32, p[2..6], value, .big);
        _ = try self.run(cmd, &p);
    }

    /// f_CMD_CONTROL_PINS: [2..3] = 16-bit control; returns [1..2] = pin state.
    pub fn controlPins(self: *Session, control: u16) Error!u16 {
        var p: [2]u8 = undefined;
        std.mem.writeInt(u16, &p, control, .big);
        const d = try self.run(.control_pins, &p);
        if (d.len < 2) return error.ResponseTooShort;
        return std.mem.readInt(u16, d[0..2], .big);
    }

    /// Max data bytes per chunk, per reference host: reads (bufSize-1)&~3,
    /// writes (bufSize-8)&~3. Capped at protocol.max_command_size.
    fn maxReadChunk(self: *const Session) usize {
        return (@as(usize, self.command_buffer_size) - 1) & ~@as(usize, 3);
    }
    fn maxWriteChunk(self: *const Session) usize {
        return (@as(usize, self.command_buffer_size) - 8) & ~@as(usize, 3);
    }

    /// f_CMD_*_READ_MEM: [2] = size/space, [3] = count, [4..7] = addr; returns
    /// [1..count] data. Chunked to buffer size, aligned to element size.
    pub fn readMem(self: *Session, space: target.MemSpace, addr: u32, out: []u8) Error!void {
        const elem: usize = space.size;
        if (out.len % elem != 0) return error.UnalignedLength;
        const rc = self.maxReadChunk();
        const chunk_max = rc - (rc % elem);

        var offset: usize = 0;
        while (offset < out.len) {
            const n = @min(out.len - offset, chunk_max);
            var p: [6]u8 = undefined;
            p[0] = space.toByte();
            p[1] = @intCast(n);
            std.mem.writeInt(u32, p[2..6], addr +% @as(u32, @intCast(offset)), .big);
            const d = try self.run(.read_mem, &p);
            if (d.len < n) return error.ResponseTooShort;
            @memcpy(out[offset..][0..n], d[0..n]);
            offset += n;
        }
    }

    /// f_CMD_*_WRITE_MEM: [2] = size/space, [3] = count, [4..7] = addr, [8..] = data.
    pub fn writeMem(self: *Session, space: target.MemSpace, addr: u32, data: []const u8) Error!void {
        const elem: usize = space.size;
        if (data.len % elem != 0) return error.UnalignedLength;
        const wc = self.maxWriteChunk();
        const chunk_max = wc - (wc % elem);

        // sized for the largest device (JMxx, 254).
        const max_write_chunk = (protocol.max_command_size - 8) & ~@as(usize, 3);
        var p: [6 + max_write_chunk]u8 = undefined;
        var offset: usize = 0;
        while (offset < data.len) {
            const n = @min(data.len - offset, chunk_max);
            p[0] = space.toByte();
            p[1] = @intCast(n);
            std.mem.writeInt(u32, p[2..6], addr +% @as(u32, @intCast(offset)), .big);
            @memcpy(p[6..][0..n], data[offset..][0..n]);
            _ = try self.run(.write_mem, p[0 .. 6 + n]);
            offset += n;
        }
    }
};

const Mock = transport.Mock;

fn mockSession(mock: *Mock) Session {
    return Session.init(mock.transport());
}

test "setTarget frame" {
    var mock = Mock.init(std.testing.allocator, &.{&.{0}});
    defer mock.deinit();
    var s = mockSession(&mock);
    try s.setTarget(.hcs08);
    try std.testing.expectEqualSlices(u8, &.{ 3, 1, 1 }, mock.sent.items[0]);
}

test "setOptions frame matches firmware BDM_Option_t image" {
    var mock = Mock.init(std.testing.allocator, &.{&.{0}});
    defer mock.deinit();
    var s = mockSession(&mock);
    try s.setOptions(.{}); // firmware defaults: only guessSpeed set
    try std.testing.expectEqualSlices(u8, &.{
        11, 6, // size, CMD_USBDM_SET_OPTIONS
        0x08, // flags: guessSpeed (bit 3)
        0, // targetVdd off
        0xFF, // useAltBDMClock = CS_DEFAULT
        1, // autoReconnect = AUTOCONNECT_STATUS
        0x18, 0x01, // SBDFRaddress 0x1801 big-endian
        0, 0, 0, // reserved
    }, mock.sent.items[0]);
}

test "capabilities decode" {
    var mock = Mock.init(std.testing.allocator, &.{&.{
        0, // rc
        0x00, 0x65, // caps: HCS12|VDDCONTROL|HCS08(inv)|CFV1(inv)
        0x00, 0xFE, // max command 254
        4, 12, 1, // version 4.12.1
    }});
    defer mock.deinit();
    var s = mockSession(&mock);
    const info = try s.capabilities();
    try std.testing.expect(info.caps.hcs12);
    try std.testing.expect(info.caps.vdd_control);
    try std.testing.expect(!info.caps.hcs08); // inverted bit set -> unsupported
    try std.testing.expectEqual(@as(u16, 254), info.max_command_size);
    try std.testing.expectEqualSlices(u8, &.{ 4, 12, 1 }, &info.version);
}

test "status word decode from wire" {
    var mock = Mock.init(std.testing.allocator, &.{&.{ 0, 0x00, 0x89 }});
    defer mock.deinit();
    var s = mockSession(&mock);
    const st = try s.bdmStatus();
    try std.testing.expect(st.ackn); // bit 0
    try std.testing.expectEqual(target.BdmStatus.Connection.sync_done, st.connection); // bits 3..4 = 1
    try std.testing.expectEqual(target.BdmStatus.Power.internal, st.power); // bits 6..7 = 2
}

test "register access frames" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{ 0, 0x00, 0x00, 0x12, 0x34 },
        &.{0x80}, // second transaction echoes toggle 1
    });
    defer mock.deinit();
    var s = mockSession(&mock);
    const pc = try s.readReg(@intFromEnum(target.Hcs08Reg.pc));
    try std.testing.expectEqual(@as(u32, 0x1234), pc);
    try std.testing.expectEqualSlices(u8, &.{ 4, 27, 0, 0x0B }, mock.sent.items[0]);

    try s.writeReg(@intFromEnum(target.Hcs08Reg.sp), 0x0250);
    // 0x9A = CMD_USBDM_WRITE_REG (26) with toggle bit set.
    try std.testing.expectEqualSlices(u8, &.{ 8, 0x9A, 0, 0x0F, 0, 0, 0x02, 0x50 }, mock.sent.items[1]);
}

test "memory read frame and chunking" {
    var big: [300]u8 = undefined;
    var resp1: [253]u8 = undefined; // rc + 252 data
    resp1[0] = 0;
    for (resp1[1..], 0..) |*b, i| b.* = @truncate(i);
    var resp2: [49]u8 = undefined; // rc (toggle 1) + 48 data
    resp2[0] = 0x80;
    for (resp2[1..], 0..) |*b, i| b.* = @truncate(252 + i);

    var mock = Mock.init(std.testing.allocator, &.{ &resp1, &resp2 });
    defer mock.deinit();
    var s = mockSession(&mock);
    s.command_buffer_size = 254; // JMxx-class buffer -> 252-byte read chunks
    try s.readMem(.byte, 0x1000, &big);

    try std.testing.expectEqual(@as(usize, 2), mock.sent.items.len);
    try std.testing.expectEqualSlices(u8, &.{ 8, 33, 1, 252, 0, 0, 0x10, 0x00 }, mock.sent.items[0]);
    // 0xA1 = CMD_USBDM_READ_MEM (33) with toggle bit set.
    try std.testing.expectEqualSlices(u8, &.{ 8, 0xA1, 1, 48, 0, 0, 0x10, 0xFC }, mock.sent.items[1]);
    try std.testing.expectEqual(@as(u8, 0), big[0]);
    try std.testing.expectEqual(@as(u8, @truncate(299)), big[299]);
}

test "connect with retry follows the reference ladder" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{5}, // connect -> no_connection (toggle 0)
        &.{0x85}, // quiet retry connect -> no_connection (toggle 1)
        &.{0}, // reset special #1 -> ok (toggle 0)
        &.{0x80}, // reset special #2 -> ok (toggle 1)
        &.{0}, // connect -> ok (toggle 0)
    });
    defer mock.deinit();
    var s = mockSession(&mock);
    try s.connectWithRetry(true);
    // try, quiet retry, TWO resets (watchdog), connect.
    try std.testing.expectEqual(@as(usize, 5), mock.sent.items.len);
    try std.testing.expectEqual(@as(u8, 22), mock.sent.items[2][1] & 0x7F); // TARGET_RESET
    try std.testing.expectEqual(@as(u8, 22), mock.sent.items[3][1] & 0x7F); // TARGET_RESET
}

test "capabilities adopts the device buffer size and shrinks chunks (JS16=145)" {
    var big: [288]u8 = undefined; // exactly two 144-byte chunks
    // capabilities reports MAX_COMMAND_SIZE 145 (JS16) -> read chunk (145-1)&~3 = 144.
    var caps: [8]u8 = .{ 0, 0x00, 0x00, 0x00, 145, 4, 12, 2 };
    var r1: [145]u8 = undefined; // rc(toggle 1) + 144 data
    r1[0] = 0x80;
    var r2: [145]u8 = undefined; // rc(toggle 0) + 144 data
    r2[0] = 0;
    var mock = Mock.init(std.testing.allocator, &.{ &caps, &r1, &r2 });
    defer mock.deinit();
    var s = mockSession(&mock);
    const info = try s.capabilities();
    try std.testing.expectEqual(@as(u16, 145), info.max_command_size);
    try std.testing.expectEqual(@as(u16, 145), s.command_buffer_size);
    try s.readMem(.byte, 0x2000, &big);
    try std.testing.expectEqual(@as(usize, 2), mock.sent.items.len - 1); // + the caps cmd
    try std.testing.expectEqual(@as(u8, 144), mock.sent.items[1][3]); // chunk count 144, not 252
    try std.testing.expectEqual(@as(u8, 144), mock.sent.items[2][3]);
}

test "memory write frame carries header and data" {
    var mock = Mock.init(std.testing.allocator, &.{&.{0}});
    defer mock.deinit();
    var s = mockSession(&mock);
    try s.writeMem(.byte, 0x00C0, &.{ 0xAA, 0xBB });
    try std.testing.expectEqualSlices(
        u8,
        &.{ 10, 32, 1, 2, 0, 0, 0x00, 0xC0, 0xAA, 0xBB },
        mock.sent.items[0],
    );
}

test "word access requires aligned length" {
    var mock = Mock.init(std.testing.allocator, &.{});
    defer mock.deinit();
    var s = mockSession(&mock);
    var out: [3]u8 = undefined;
    try std.testing.expectError(error.UnalignedLength, s.readMem(.word, 0, &out));
}

test "bdm error surfaces rc" {
    var mock = Mock.init(std.testing.allocator, &.{&.{5}});
    defer mock.deinit();
    var s = mockSession(&mock);
    try std.testing.expectError(error.BdmError, s.connect());
    try std.testing.expectEqual(protocol.Rc.no_connection, s.last_rc);
}

test "sync/frequency conversions" {
    // 4 MHz -> value 1920 (0x0780), matching the reference host.
    try std.testing.expectEqual(@as(u16, 1920), try hzToSync(4_000_000));
    try std.testing.expectEqual(@as(u32, 4_000_000), syncToHz(1920));
    try std.testing.expectError(error.OutOfRange, hzToSync(0));
    // 1 MHz -> 7680 ticks; round trip
    try std.testing.expectEqual(@as(u16, 7680), try hzToSync(1_000_000));
    try std.testing.expectEqual(@as(u32, 1_000_000), syncToHz(try hzToSync(1_000_000)));
    // firmware "unknown speed" sentinel must not overflow
    try std.testing.expectEqual(@as(u32, std.math.maxInt(u32)), syncToHz(1));
}
