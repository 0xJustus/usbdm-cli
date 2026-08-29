//! Byte-write access over BDM (hwbreak.zig programs the HCS08 DBG registers);
//! a thin vtable so breakpoint logic is testable against a mock.

const session = @import("session.zig");

pub const Mem = struct {
    ctx: *anyopaque,
    writeFn: *const fn (ctx: *anyopaque, addr: u32, value: u8) anyerror!void,

    pub fn write(self: Mem, addr: u32, value: u8) anyerror!void {
        return self.writeFn(self.ctx, addr, value);
    }
};

/// `Mem` backed by a live BDM session.
pub fn sessionMem(s: *session.Session) Mem {
    const Adapter = struct {
        fn write(ctx: *anyopaque, addr: u32, value: u8) anyerror!void {
            const sess: *session.Session = @ptrCast(@alignCast(ctx));
            try sess.writeMem(.byte, addr, &.{value});
        }
    };
    return .{ .ctx = s, .writeFn = Adapter.write };
}
