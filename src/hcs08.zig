//! HCS08 flash bring-up, the part of UsbdmFlashProgrammer that happens before any
//! erase: reset into special mode, stop the COP, put the clock at a known speed,
//! optionally trim it, load FCDIV from the measured bus clock, and keep the
//! oscillator trim bytes alive across the sector erase that would otherwise wipe them.
//!
//! The small-code flash routine does none of this; it assumes a halted target with a
//! working flash clock divider and a COP that is not about to reset it. Reference:
//! USBDM FlashProgrammer_HCS08.cpp (programFlash, resetAndConnectTarget,
//! initialiseTargetFlash) and HCS08-default-flash-scripts.tcl.

const std = @import("std");
const session = @import("session.zig");
const device = @import("device.zig");
const clock = @import("clock.zig");
const hexfile = @import("hexfile.zig");

/// BDCSCR bits (HCS08).
const BDCSCR_ENBDM: u8 = 0x80;
const BDCSCR_CLKSW: u8 = 0x08;
const BDCSCR_BDMACT: u8 = 0x40;

/// SOPT1 value the reference scripts write: STOPE | BKGDPE, COP off. SOPT is write-once
/// after reset, so this has to be the first thing after the reset.
const SOPT_INIT: u8 = 0x22;

/// FCDIV and FPROT offsets from the flash controller base.
const FCDIV_OFFSET: u32 = 0;
const FPROT_OFFSET: u32 = 4;
const FCDIV_DIVLD: u8 = 0x80;

pub const Ctx = struct {
    sess: *session.Session,
    dev: device.Device,
    sleep_ms: *const fn (u32) void,
};

pub const Request = struct {
    /// Trim the internal reference to this frequency (Hz) and write the result into the
    /// image. null = do not trim; the existing trim is preserved instead.
    trim_hz: ?u32 = null,
    /// Where the two trim bytes live; null = the clock type's default.
    nvloc: ?u32 = null,
    /// Use this bus clock (Hz) for FCDIV instead of measuring it.
    bus_hz: ?u32 = null,
    /// false = carry on past a failed step (mass-erasing a secured part must not be
    /// stopped by a COP write the secured part refuses). The first failure is reported.
    strict: bool = true,
};

pub const Result = struct {
    bus_hz: u32,
    fcdiv: u8,
    trim: ?clock.TrimResult = null,
    trim_bytes: ?clock.TrimBytes = null,
    /// The chip's own trim bytes copied into the image so the erase does not lose them.
    preserved: ?[2]u8 = null,
    preserved_at: u32 = 0,
    /// First non-strict failure, if any.
    soft_error: ?anyerror = null,
};

pub const Error = error{
    NoFlashDivider,
    FcdivNotLoaded,
    TrimNotSupported,
    BadTrimLocation,
    SpeedUnknown,
    ClockRegisterVerify,
    NotHalted,
};

/// BDM options for a HCS08 flash session: the BDC runs from the bus clock (CLKSW=1) so
/// the pod's speed measurement IS the bus clock. Without that it reports the alternate
/// clock, twice the bus on the ICS parts. The reset line is left alone (the default
/// special reset is a software one and needs no RESET wire).
pub fn options(base: session.Options) session.Options {
    var o = base;
    o.use_alt_bdm_clock = 1; // CS_NORMAL
    return o;
}

fn writeReg(c: *Ctx, addr: u32, v: u8) anyerror!void {
    // A clock-register write can change the BDM clock under the pod's feet: retry once
    // after re-syncing (the reference's writeClockRegister).
    c.sess.writeMem(.byte, addr, &.{v}) catch {
        try c.sess.connect();
        try c.sess.writeMem(.byte, addr, &.{v});
    };
}

fn readByte(c: *Ctx, addr: u32) anyerror!u8 {
    var b: [1]u8 = undefined;
    try c.sess.readMem(.byte, addr, &b);
    return b[0];
}

/// Re-sync and return the BDC clock in Hz (== bus clock once CLKSW=1).
pub fn measureHz(c: *Ctx) anyerror!u32 {
    try c.sess.connect();
    const sync = try c.sess.getSpeedSync();
    if (sync <= 1) return error.SpeedUnknown;
    return session.syncToHz(sync);
}

/// Reset into special mode (BDM enabled, flash controller at its reset values) and halt
/// the CPU, as the reference does: a software reset (the device DB's default for HCS08),
/// then `connect; halt`. The halt is not optional: BDCSCR reads 0x88 (ENBDM|CLKSW,
/// BDMACT clear) straight after a reset on an SH16, i.e. the firmware is already running.
///
/// Measured on a bench SH16: a *hardware* special reset left the CPU running with FCDIV
/// already loaded by the firmware (DIVLD set, writes ignored) even when followed by a
/// halt, while the software one gives a clean halted part with FCDIV unloaded. The cause
/// of the hardware case was not established. Falls back to every reset strategy the pod
/// has.
pub fn specialReset(c: *Ctx) anyerror!void {
    c.sess.reset(.special, .software) catch {
        try c.sess.connect();
        try c.sess.reset(.special, .all);
    };
    try c.sess.connect();
    try c.sess.halt();
    const st: u8 = @truncate(try c.sess.readStatusReg());
    if (st & BDCSCR_BDMACT == 0) return error.NotHalted;
}

fn forceNormalClock(c: *Ctx) anyerror!void {
    const st: u8 = @truncate(try c.sess.readStatusReg());
    if (st & BDCSCR_CLKSW != 0) return;
    try c.sess.writeControlReg((st & 0xB8) | BDCSCR_ENBDM | BDCSCR_CLKSW);
    try c.sess.connect();
}

/// Write an ICS register set the way the reference does (trim first, re-sync after the
/// writes that can change the clock, FLL settle) and return the resulting bus clock.
pub fn configureIcs(c: *Ctx, regs: clock.IcsRegs) anyerror!u32 {
    const base = c.dev.clock_addr;
    try writeReg(c, base + clock.ics_trm, regs.trim);
    try c.sess.connect();
    try writeReg(c, base + clock.ics_c1, regs.c1);
    try writeReg(c, base + clock.ics_c2, regs.c2);
    try c.sess.connect();
    try writeReg(c, base + clock.ics_sc, regs.sc);
    c.sleep_ms(100); // FLL lock
    try c.sess.connect();
    return measureHz(c);
}

const IcsProbe = struct {
    c: *Ctx,

    fn setTrim(ctx: *anyopaque, msb: ?u8, lsb: ?u8) anyerror!void {
        const self: *IcsProbe = @ptrCast(@alignCast(ctx));
        const base = self.c.dev.clock_addr;
        // Re-sync before each verify read: the write just moved the target clock, and a
        // read at the old sync can come back as garbage (0x00 read back as 0x24 on the bench).
        if (lsb) |l| {
            try writeReg(self.c, base + clock.ics_sc, l);
            try self.c.sess.connect();
            if ((try readByte(self.c, base + clock.ics_sc)) & 1 != l) return error.ClockRegisterVerify;
        }
        if (msb) |m| {
            try writeReg(self.c, base + clock.ics_trm, m);
            try self.c.sess.connect();
            if ((try readByte(self.c, base + clock.ics_trm)) != m) return error.ClockRegisterVerify;
        }
    }
    fn measure(ctx: *anyopaque) anyerror!u32 {
        const self: *IcsProbe = @ptrCast(@alignCast(ctx));
        return measureHz(self.c);
    }
    const vtable = clock.Probe.VTable{ .setTrim = setTrim, .measureHz = measure };
    fn probe(self: *IcsProbe) clock.Probe {
        return .{ .ctx = self, .vtable = &vtable };
    }
};

/// The chip's trim bytes into the image, wherever the image leaves them unset: the
/// page erase that precedes programming takes the whole 512-byte sector, trim included,
/// and erased trim (0xFF) leaves the part running ~30% slow.
fn preserveTrim(gpa: std.mem.Allocator, c: *Ctx, img: *hexfile.Image, nv: u32) anyerror!?[2]u8 {
    if (nv == 0) return null;
    const mask: u32 = c.dev.page_size - 1;
    if (!img.overlaps(nv & ~mask, (nv + 1) | mask)) return null; // that sector is not erased
    var cur: [2]u8 = undefined;
    c.sess.readMem(.byte, nv, &cur) catch return null; // secured part: nothing readable to keep
    if (cur[0] == 0xFF and cur[1] == 0xFF) return null; // already blank
    var kept = false;
    for (cur, 0..) |b, i| {
        const a = nv + @as(u32, @intCast(i));
        const have = img.byteAt(a);
        // An image that defines the byte (non-erased) wins: that is the user's intent.
        if ((have == null or have.? == 0xFF) and b != 0xFF) {
            try img.setByte(gpa, a, b);
            kept = true;
        }
    }
    return if (kept) cur else null;
}

/// Everything between "connected" and "ready to erase". `image` is modified: the trim
/// bytes are inserted (--trim) or the chip's existing ones are carried over.
pub fn prepare(gpa: std.mem.Allocator, c: *Ctx, req: Request, image: ?*hexfile.Image) anyerror!Result {
    var res = Result{ .bus_hz = 0, .fcdiv = 0 };
    const ty = c.dev.clock;

    // Each step either succeeds or (non-strict) is recorded and skipped.
    // A labeled-block helper keeps the strict path a plain `try`.
    const soft = !req.strict;

    specialReset(c) catch |e| {
        if (!soft) return e;
        res.soft_error = res.soft_error orelse e;
    };
    forceNormalClock(c) catch |e| {
        if (!soft) return e;
        res.soft_error = res.soft_error orelse e;
    };
    if (c.dev.sopt_addr != 0) {
        writeReg(c, c.dev.sopt_addr, SOPT_INIT) catch |e| {
            if (!soft) return e;
            res.soft_error = res.soft_error orelse e;
        };
    }

    const nv = req.nvloc orelse clock.defaultTrimNv(ty);

    if (req.trim_hz) |hz| {
        if (!clock.canTrim(ty)) return error.TrimNotSupported;
        if (nv == 0 or !c.dev.inFlash(nv) or !c.dev.inFlash(nv + 1)) return error.BadTrimLocation;
        _ = try configureIcs(c, clock.lowSpeedRegs(ty).?);
        var p = IcsProbe{ .c = c };
        const target_hz = hz * clock.trimMultiplier(ty).?;
        const r = clock.search(p.probe(), target_hz) catch |e| {
            // Leave a clock the BDM can talk to, not whatever the search stopped on.
            _ = configureIcs(c, clock.flashRegs(ty).?) catch {};
            return e;
        };
        res.trim = r;
        if (image) |img| {
            const tb = clock.trimBytes(nv, r.value, img.byteAt(nv));
            try img.setByte(gpa, tb.ftrim_addr, tb.ftrim);
            try img.setByte(gpa, tb.trim_addr, tb.trim);
            res.trim_bytes = tb;
        }
    } else if (image) |img| {
        if (preserveTrim(gpa, c, img, nv)) |kept| {
            res.preserved = kept;
            res.preserved_at = nv;
        } else |e| {
            if (!soft) return e;
            res.soft_error = res.soft_error orelse e;
        }
    }

    // The programming clock and FCDIV. FCDIV is write-once per reset, so it goes last,
    // after anything that moved the clock.
    var bus_hz: u32 = 0;
    if (clock.flashRegs(ty)) |regs| {
        bus_hz = configureIcs(c, regs) catch |e| blk: {
            if (!soft) return e;
            res.soft_error = res.soft_error orelse e;
            break :blk 0;
        };
    }
    if (bus_hz == 0) {
        bus_hz = measureHz(c) catch |e| blk: {
            if (!soft) return e;
            res.soft_error = res.soft_error orelse e;
            break :blk 0;
        };
    }
    if (req.bus_hz) |b| bus_hz = b;
    res.bus_hz = bus_hz;

    const khz = (bus_hz + 500) / 1000;
    const div = clock.flashDivider(khz) orelse {
        if (!soft) return error.NoFlashDivider;
        res.soft_error = res.soft_error orelse error.NoFlashDivider;
        return res;
    };
    res.fcdiv = div;
    const fcdiv_addr = c.dev.flash_base + FCDIV_OFFSET;
    const loaded: anyerror!void = blk: {
        writeReg(c, fcdiv_addr, div) catch |e| break :blk e;
        const rb = readByte(c, fcdiv_addr) catch |e| break :blk e;
        if (rb != (div | FCDIV_DIVLD)) break :blk error.FcdivNotLoaded;
        writeReg(c, c.dev.flash_base + FPROT_OFFSET, 0xFF) catch |e| break :blk e; // unprotect
    };
    loaded catch |e| {
        if (!soft) return e;
        res.soft_error = res.soft_error orelse e;
    };
    return res;
}

extern "c" fn usleep(us: c_uint) c_int;
pub fn sleepMs(ms: u32) void {
    _ = usleep(ms * 1000);
}
