//! BDM command transport over bulk endpoints (JMxx/JS16 firmware). Wire format
//! from firmware USB.c + reference host low_level_usb.cpp; framing notes inline.
//! TX EP1 = [size, cmd, params...]; RX EP2 = [rc, data...] (oversized read).

const std = @import("std");
const usb = @import("usb.zig");
const protocol = @import("protocol.zig");

/// EP1 OUT / EP2 IN (firmware USB descriptors).
pub const bulk_ep_out: u8 = 0x01;
pub const bulk_ep_in: u8 = 0x82;
/// MaxFirstTransaction: longer commands are split.
pub const max_first_transaction: usize = 62;
/// Toggle/sequence bits in the rc byte.
pub const sequence_mask: u8 = 0xC0;
/// Total BDM_RC_BUSY polling budget.
const busy_budget_ms: u32 = 5000;

pub const Error = usb.Error || error{
    CommandTooLong,
    EmptyResponse,
    ResponseTooShort,
    BdmBusyTimeout,
    ToggleMismatch,
};

pub const Response = struct {
    rc: protocol.Rc,
    data: []u8,
};

/// Byte-level transport interface. `send` = one bulk OUT; `recv` = one bulk IN
/// read, returns bytes received.
pub const Transport = struct {
    ctx: *anyopaque,
    vtable: *const VTable,
    trace: protocol.Trace = .{},
    timeout_ms: u32 = 1500,
    toggle: u1 = 0,

    pub const VTable = struct {
        send: *const fn (ctx: *anyopaque, data: []const u8, timeout_ms: u32) usb.Error!void,
        recv: *const fn (ctx: *anyopaque, buf: []u8, timeout_ms: u32) usb.Error!usize,
        sleep_ms: *const fn (ctx: *anyopaque, ms: u32) void,
    };

    /// One command round trip. `response_buf` must be >= MAX_COMMAND_SIZE+1
    /// (protocol requires oversized reads). Returns payload after the rc byte.
    pub fn transact(
        self: *Transport,
        cmd: protocol.Command,
        params: []const u8,
        response_buf: []u8,
    ) Error!Response {
        var frame_buf: [protocol.max_command_size]u8 = undefined;
        const total = params.len + 2;
        if (total > protocol.max_command_size) return error.CommandTooLong;

        // cmd bit 7 = per-transaction toggle, echoed in the rc byte;
        // GET_CAPABILITIES resets it on both sides.
        if (cmd == .get_capabilities) self.toggle = 0;
        const sent_toggle: u8 = @as(u8, self.toggle) << 7;
        self.toggle +%= 1;

        frame_buf[0] = @intCast(total);
        frame_buf[1] = @intFromEnum(cmd) | sent_toggle;
        @memcpy(frame_buf[2..][0..params.len], params);
        const frame = frame_buf[0..total];

        self.trace.packet("EP1 -> command", frame);
        if (total <= max_first_transaction) {
            try self.vtable.send(self.ctx, frame, self.timeout_ms);
        } else {
            try self.vtable.send(self.ctx, frame[0..max_first_transaction], self.timeout_ms);
            var cont_buf: [protocol.max_command_size + 1]u8 = undefined;
            cont_buf[0] = 0; // continuation marker at pos 61; firmware restores the overlapped byte
            const rest = frame[max_first_transaction..];
            @memcpy(cont_buf[1..][0..rest.len], rest);
            self.trace.packet("EP1 -> continuation", cont_buf[0 .. rest.len + 1]);
            try self.vtable.send(self.ctx, cont_buf[0 .. rest.len + 1], self.timeout_ms);
        }

        var busy_waited: u32 = 0;
        var busy_backoff: u32 = 1;
        var timeout_retried = false;
        var toggle_retried = false;
        while (true) {
            const n = self.vtable.recv(self.ctx, response_buf, self.timeout_ms) catch |err| switch (err) {
                error.Timeout => blk: {
                    // one retry at 4x timeout (reference host).
                    if (timeout_retried) return err;
                    timeout_retried = true;
                    break :blk try self.vtable.recv(self.ctx, response_buf, 4 * self.timeout_ms);
                },
                else => return err,
            };
            if (n == 0) return error.EmptyResponse;
            self.trace.packet("EP2 <- response", response_buf[0..n]);

            const raw = response_buf[0];
            const rc: protocol.Rc = @enumFromInt(raw & ~sequence_mask);
            if (rc == .busy) {
                if (busy_waited >= busy_budget_ms) return error.BdmBusyTimeout;
                self.vtable.sleep_ms(self.ctx, busy_backoff);
                busy_waited += busy_backoff;
                busy_backoff = @min(busy_backoff * 2, 512);
                continue;
            }
            if ((raw & 0x80) != sent_toggle) {
                // Stale queued response: re-read once after 100 ms, then fail
                // rather than return another command's data (ref: BDM_RC_USB_ERROR).
                if (toggle_retried) return error.ToggleMismatch;
                toggle_retried = true;
                self.vtable.sleep_ms(self.ctx, 100);
                continue;
            }
            return .{ .rc = rc, .data = response_buf[1..n] };
        }
    }
};

/// libusb bulk endpoint pair.
pub const BulkUsb = struct {
    handle: usb.DeviceHandle,
    ep_out: u8 = bulk_ep_out,
    ep_in: u8 = bulk_ep_in,

    pub fn transport(self: *BulkUsb, trace: protocol.Trace) Transport {
        return .{ .ctx = self, .vtable = &vtable, .trace = trace };
    }

    const vtable: Transport.VTable = .{ .send = send, .recv = recv, .sleep_ms = sleepMs };

    fn send(ctx: *anyopaque, data: []const u8, timeout_ms: u32) usb.Error!void {
        const self: *BulkUsb = @ptrCast(@alignCast(ctx));
        _ = try self.handle.bulkWrite(self.ep_out, data, timeout_ms);
    }

    fn recv(ctx: *anyopaque, buf: []u8, timeout_ms: u32) usb.Error!usize {
        const self: *BulkUsb = @ptrCast(@alignCast(ctx));
        return self.handle.bulkRead(self.ep_in, buf, timeout_ms);
    }

    extern "c" fn usleep(us: c_uint) c_int;

    fn sleepMs(ctx: *anyopaque, ms: u32) void {
        _ = ctx;
        _ = usleep(ms * 1000);
    }
};

pub const Mock = struct {
    gpa: std.mem.Allocator,
    sent: std.ArrayList([]u8) = .empty,
    responses: []const []const u8,
    recv_i: usize = 0,
    slept_ms: u32 = 0,

    pub fn init(gpa: std.mem.Allocator, responses: []const []const u8) Mock {
        return .{ .gpa = gpa, .responses = responses };
    }

    pub fn deinit(self: *Mock) void {
        for (self.sent.items) |s| self.gpa.free(s);
        self.sent.deinit(self.gpa);
    }

    pub fn transport(self: *Mock) Transport {
        return .{ .ctx = self, .vtable = &vtable };
    }

    const vtable: Transport.VTable = .{ .send = send, .recv = recv, .sleep_ms = sleepMs };

    fn send(ctx: *anyopaque, data: []const u8, timeout_ms: u32) usb.Error!void {
        _ = timeout_ms;
        const self: *Mock = @ptrCast(@alignCast(ctx));
        const copy = self.gpa.dupe(u8, data) catch return error.NoMem;
        self.sent.append(self.gpa, copy) catch return error.NoMem;
    }

    fn recv(ctx: *anyopaque, buf: []u8, timeout_ms: u32) usb.Error!usize {
        _ = timeout_ms;
        const self: *Mock = @ptrCast(@alignCast(ctx));
        if (self.recv_i >= self.responses.len) return error.Timeout;
        const r = self.responses[self.recv_i];
        self.recv_i += 1;
        const n = @min(r.len, buf.len);
        @memcpy(buf[0..n], r[0..n]);
        return n;
    }

    fn sleepMs(ctx: *anyopaque, ms: u32) void {
        const self: *Mock = @ptrCast(@alignCast(ctx));
        self.slept_ms += ms;
    }
};

test "small command is a single frame with leading size byte" {
    var mock = Mock.init(std.testing.allocator, &.{&.{0}});
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    const resp = try tp.transact(.set_target, &.{1}, &buf);
    try std.testing.expectEqual(protocol.Rc.ok, resp.rc);
    try std.testing.expectEqual(@as(usize, 0), resp.data.len);
    try std.testing.expectEqual(@as(usize, 1), mock.sent.items.len);
    try std.testing.expectEqualSlices(u8, &.{ 3, 1, 1 }, mock.sent.items[0]);
}

test "toggle bit alternates and is echoed" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{0}, // toggle 0 echoed
        &.{0x80}, // toggle 1 echoed
        &.{0}, // toggle 0 again
    });
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    _ = try tp.transact(.connect, &.{}, &buf);
    _ = try tp.transact(.connect, &.{}, &buf);
    _ = try tp.transact(.connect, &.{}, &buf);
    try std.testing.expectEqual(@as(u8, 15), mock.sent.items[0][1]);
    try std.testing.expectEqual(@as(u8, 15 | 0x80), mock.sent.items[1][1]);
    try std.testing.expectEqual(@as(u8, 15), mock.sent.items[2][1]);
    try std.testing.expectEqual(@as(u32, 0), mock.slept_ms);
}

test "get_capabilities resets the toggle" {
    var mock = Mock.init(std.testing.allocator, &.{ &.{0}, &.{0}, &.{0x80} });
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    _ = try tp.transact(.connect, &.{}, &buf); // toggle 0 -> next would be 1
    _ = try tp.transact(.get_capabilities, &.{}, &buf); // forced back to 0
    _ = try tp.transact(.connect, &.{}, &buf); // toggle 1
    try std.testing.expectEqual(@as(u8, 5), mock.sent.items[1][1]);
    try std.testing.expectEqual(@as(u8, 15 | 0x80), mock.sent.items[2][1]);
}

test "toggle mismatch triggers one delayed re-read" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{ 0x80, 0xEE }, // stale response with wrong toggle
        &.{ 0x00, 0x42 }, // real response
    });
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    const resp = try tp.transact(.get_speed, &.{}, &buf);
    try std.testing.expectEqual(protocol.Rc.ok, resp.rc);
    try std.testing.expectEqualSlices(u8, &.{0x42}, resp.data);
    try std.testing.expectEqual(@as(u32, 100), mock.slept_ms);
}

test "persistent toggle mismatch fails instead of returning stale data" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{ 0x80, 0xEE }, // wrong toggle
        &.{ 0x80, 0xEF }, // still wrong toggle on the re-read
    });
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    try std.testing.expectError(error.ToggleMismatch, tp.transact(.get_speed, &.{}, &buf));
    try std.testing.expectEqual(@as(u32, 100), mock.slept_ms);
}

test "oversize command splits at 62 bytes with marker continuation" {
    var mock = Mock.init(std.testing.allocator, &.{&.{0}});
    defer mock.deinit();
    var params: [100]u8 = undefined;
    for (&params, 0..) |*p, i| p.* = @intCast(i);
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    _ = try tp.transact(.write_mem, &params, &buf);

    try std.testing.expectEqual(@as(usize, 2), mock.sent.items.len);
    const head = mock.sent.items[0];
    const cont = mock.sent.items[1];
    try std.testing.expectEqual(@as(usize, 62), head.len);
    try std.testing.expectEqual(@as(u8, 102), head[0]); // whole-frame size
    try std.testing.expectEqual(@as(u8, 32), head[1]); // CMD_USBDM_WRITE_MEM
    try std.testing.expectEqualSlices(u8, params[0..60], head[2..]);
    // Continuation: marker + frame[62..] (i.e. params[60..]).
    try std.testing.expectEqual(@as(usize, 1 + 102 - 62), cont.len);
    try std.testing.expectEqual(@as(u8, 0), cont[0]);
    try std.testing.expectEqualSlices(u8, params[60..], cont[1..]);
}

test "busy responses are polled with backoff until the real response" {
    var mock = Mock.init(std.testing.allocator, &.{
        &.{ 3, 1, 2, 3 }, // BDM_RC_BUSY filler
        &.{ 3, 1, 2, 3 },
        &.{ 0, 0xAB, 0xCD }, // real response
    });
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    const resp = try tp.transact(.get_speed, &.{}, &buf);
    try std.testing.expectEqual(protocol.Rc.ok, resp.rc);
    try std.testing.expectEqualSlices(u8, &.{ 0xAB, 0xCD }, resp.data);
    try std.testing.expectEqual(@as(u32, 3), mock.slept_ms); // 1 + 2
}

test "error rc is returned with sequence bits masked" {
    var mock = Mock.init(std.testing.allocator, &.{&.{5}});
    defer mock.deinit();
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    const resp = try tp.transact(.connect, &.{}, &buf);
    try std.testing.expectEqual(protocol.Rc.no_connection, resp.rc);
}

test "command too long is rejected" {
    var mock = Mock.init(std.testing.allocator, &.{});
    defer mock.deinit();
    var params: [protocol.max_command_size]u8 = undefined;
    var tp = mock.transport();
    var buf: [256]u8 = undefined;
    try std.testing.expectError(
        error.CommandTooLong,
        tp.transact(.write_mem, &params, &buf),
    );
}
