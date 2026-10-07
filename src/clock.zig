//! HCS08 internal-clock handling for flash programming: the known-good clock a
//! flash operation runs at, the flash clock divider derived from it, and the
//! oscillator trim that PGO (UsbdmFlashProgrammer -trim=) computes by measuring
//! the BDM clock. Pure logic only: the target is reached through `Probe`, so the
//! search is testable without hardware. Reference: USBDM FlashProgrammerCommon.cpp
//! (trimTargetClock/trimICS_Clock/configureTargetClock), DeviceData.cpp (defaults)
//! and HCS08-default-flash-scripts.tcl (calculateFlashDivider).

const std = @import("std");
const types = @import("device_types");

pub const ClockType = types.ClockType;

/// ICS register values in their address order: ICSC1, ICSC2, ICSTRM, ICSSC.
pub const IcsRegs = struct { c1: u8, c2: u8, trim: u8, sc: u8 };

/// Register offsets from the clock base address.
pub const ics_c1: u32 = 0;
pub const ics_c2: u32 = 1;
pub const ics_trm: u32 = 2;
pub const ics_sc: u32 = 3;

/// The clock flash operations run at: internal reference, BDIV /2, nominal trim
/// (reference clock x 512/1024). PGO's "maximum safe speed untrimmed". null for
/// clocks this module does not configure.
pub fn flashRegs(ty: ClockType) ?IcsRegs {
    return switch (ty) {
        .ics_v1, .ics_v2, .ics_v2x512, .ics_v3 => .{ .c1 = 0x04, .c2 = 0x40, .trim = 0x80, .sc = 0x00 }, // BDIV=/2 (2-bit)
        .ics_v4, .ics_v5 => .{ .c1 = 0x04, .c2 = 0x20, .trim = 0x80, .sc = 0x00 }, // BDIV=/2 (3-bit)
        else => null,
    };
}

/// A slow clock to trim at: the BDM sync measurement is more accurate the lower
/// the clock, and the trim is a property of the reference, not of the output.
pub fn lowSpeedRegs(ty: ClockType) ?IcsRegs {
    return switch (ty) {
        .ics_v1, .ics_v2, .ics_v2x512, .ics_v3 => .{ .c1 = 0x04, .c2 = 0xC0, .trim = 0x80, .sc = 0x00 }, // BDIV=/8
        .ics_v4 => .{ .c1 = 0x04, .c2 = 0x60, .trim = 0x80, .sc = 0x00 }, // BDIV=/8
        .ics_v5 => .{ .c1 = 0x04, .c2 = 0x80, .trim = 0x80, .sc = 0x00 }, // BDIV=/16
        else => null,
    };
}

/// Bus clock at the low-speed setting per Hz of reference clock, i.e. the factor
/// that turns the requested trim frequency into the BDM clock to hit.
pub fn trimMultiplier(ty: ClockType) ?u32 {
    return switch (ty) {
        .ics_v1, .ics_v3, .ics_v2x512 => 512 / 8 / 2,
        .ics_v2 => 1024 / 8 / 2,
        .ics_v4 => 512 / 8, // no /2 on the bus clock
        .ics_v5 => 1024 / 16, // no /2 on the bus clock
        else => null,
    };
}

/// Default location of the non-volatile trim bytes (NVFTRIM at the address, NVTRIM
/// at address+1). Hard-coded in USBDM's DeviceData.cpp by clock type; the device
/// XML carries only the clock type and its register address.
pub fn defaultTrimNv(ty: ClockType) u32 {
    return switch (ty) {
        .ics_v1, .ics_v2, .ics_v2x512, .ics_v3 => 0xFFAE,
        .ics_v4, .ics_v5 => 0xFF6E,
        .icg => 0xFFBE,
        .mcg => 0xFFAE,
        else => 0,
    };
}

/// Default (nominal) reference frequency in Hz for the clock type.
pub fn defaultTrimHz(ty: ClockType) u32 {
    return switch (ty) {
        .ics_v1, .ics_v2, .ics_v2x512, .ics_v3, .ics_v4, .ics_v5, .mcg => 31250,
        .icg => 243000,
        else => 0,
    };
}

/// Whether `search` + `trimBytes` can trim this clock. ICS only.
pub fn canTrim(ty: ClockType) bool {
    return ty.isIcs();
}

/// FCDIV for a bus clock in kHz (HCS08 default flash module): flash clock must be
/// 150..200 kHz, PRDIV8 (bit 6) for buses above 12.8 MHz. null if impossible.
/// Integer arithmetic exactly as the reference Tcl (calculateFlashDivider).
pub fn flashDivider(bus_khz: u32) ?u8 {
    if (bus_khz < 150) return null;
    var div: u32 = 0;
    var f = bus_khz;
    if (f > 12800) {
        div = 0x40; // PRDIV8
        f /= 8;
    }
    div |= (f - 1) / 200;
    const fclk = f / ((div & 0x3F) + 1);
    if (fclk < 150 or fclk > 200) return null;
    return @intCast(div);
}

/// FCDIV is a 6-bit divider plus PRDIV8; the flash clock it yields for `bus_khz`.
pub fn flashClockKhz(bus_khz: u32, fcdiv: u8) u32 {
    const f = if (fcdiv & 0x40 != 0) bus_khz / 8 else bus_khz;
    return f / ((fcdiv & 0x3F) + 1);
}

// --- trim search -----------------------------------------------------------

/// How `search` reaches the target. `setTrim` writes ICSTRM when `msb` is non-null and
/// ICSSC's FTRIM bit when `lsb` is non-null (each independently: zeroing the TRIM byte
/// to clear FTRIM would swing the clock 50% and lose the BDM), and verifies what it
/// wrote; `measureHz` re-syncs the BDM and returns its clock in Hz (== bus clock when
/// CLKSW=1).
pub const Probe = struct {
    ctx: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        setTrim: *const fn (ctx: *anyopaque, msb: ?u8, lsb: ?u8) anyerror!void,
        measureHz: *const fn (ctx: *anyopaque) anyerror!u32,
    };

    fn setTrim(self: Probe, msb: ?u8, lsb: ?u8) anyerror!void {
        return self.vtable.setTrim(self.ctx, msb, lsb);
    }
    fn measureHz(self: Probe) anyerror!u32 {
        return self.vtable.measureHz(self.ctx);
    }
};

pub const max_trim: i32 = 505; // largest acceptable 9-bit trim
pub const min_trim: i32 = 5; // smallest
pub const sweep_offset: i32 = 8; // linear sweep is +/- this around the binary-search result
pub const samples_per_step: u32 = 2; // measurements per trim value (9-bit)

pub const TrimError = error{TrimOutOfRange};

pub const TrimResult = struct {
    /// 9-bit trim: TRIM = value >> 1, FTRIM = value & 1.
    value: u16,
    /// BDM clock measured with the trim applied.
    measured_hz: u32,
    /// Measurements taken by the sweep.
    samples: u32,
    /// The regression's slope, Hz per trim step (negative: a larger trim is slower).
    hz_per_step: f64,
};

/// Find the 9-bit trim that makes the BDM clock `target_hz`: an 8-step binary
/// search on the TRIM byte, then a +/-8 linear sweep of the full 9-bit value taking
/// `samples_per_step` measurements per step, then a least-squares line through all
/// of them solved for the target. Leaves the winner written to the target.
/// Larger trim = slower clock on the ICS.
pub fn search(p: Probe, target_hz: u32) anyerror!TrimResult {
    // FTRIM (LSB) to zero before the byte search, as the reference does: the LSB only.
    try p.setTrim(null, 0);

    var msb: u8 = 0;
    var mask: u8 = 0x80;
    while (mask != 0) : (mask >>= 1) {
        msb |= mask;
        try p.setTrim(msb, null);
        const hz = try p.measureHz();
        if (hz < target_hz) msb &= ~mask; // too slow: back the trim off
        msb = std.math.clamp(msb, @as(u8, @intCast(@divTrunc(min_trim, 2))), @as(u8, @intCast(@divTrunc(max_trim, 2))));
    }

    const centre: i32 = @as(i32, msb) << 1;
    const hi = @min(centre + sweep_offset, max_trim);
    const lo = @max(centre - sweep_offset, min_trim);

    var n: f64 = 0;
    var sum_x: f64 = 0;
    var sum_y: f64 = 0;
    var sum_xx: f64 = 0;
    var sum_xy: f64 = 0;
    var samples: u32 = 0;

    // Down then up: each value is measured on both a falling and a rising approach,
    // which averages out the FLL's settling direction.
    var pass: u2 = 0;
    while (pass < 2) : (pass += 1) {
        var t: i32 = if (pass == 0) hi else lo;
        while (if (pass == 0) t >= lo else t <= hi) : (t += if (pass == 0) @as(i32, -1) else 1) {
            try p.setTrim(@intCast(t >> 1), @intCast(t & 1));
            var k: u32 = 0;
            while (k < samples_per_step) : (k += 1) {
                const hz: f64 = @floatFromInt(try p.measureHz());
                const x: f64 = @floatFromInt(t);
                n += 1;
                sum_x += x;
                sum_y += hz;
                sum_xx += x * x;
                sum_xy += x * hz;
                samples += 1;
            }
        }
    }

    const denom = n * sum_xx - sum_x * sum_x;
    if (denom == 0) return error.TrimOutOfRange;
    const beta = (n * sum_xy - sum_x * sum_y) / denom;
    const alpha = (sum_y - beta * sum_x) / n;
    if (beta == 0) return error.TrimOutOfRange;
    const tf = (@as(f64, @floatFromInt(target_hz)) - alpha) / beta;
    // The reference tests a stale loop variable here; the intent is this.
    if (!(tf > @as(f64, @floatFromInt(min_trim)) and tf < @as(f64, @floatFromInt(max_trim)))) return error.TrimOutOfRange;

    const value: u16 = @intFromFloat(@round(tf));
    try p.setTrim(@intCast(value >> 1), @intCast(value & 1));
    const measured = try p.measureHz();
    return .{ .value = value, .measured_hz = measured, .samples = samples, .hz_per_step = beta };
}

// --- trim bytes in flash ---------------------------------------------------

/// The two non-volatile bytes a 9-bit trim lives in.
pub const TrimBytes = struct {
    ftrim_addr: u32,
    ftrim: u8,
    trim_addr: u32,
    trim: u8,
};

/// NVFTRIM carries FTRIM in bit 0 and may share its byte with other options
/// (DMX32, DRS...): `image_ftrim` is what the image already holds there. An
/// absent or erased (0xFF) byte means "nothing to merge with", so the other bits
/// are cleared, as the reference does.
pub fn trimBytes(nv: u32, value: u16, image_ftrim: ?u8) TrimBytes {
    var merge: u8 = image_ftrim orelse 0;
    if (merge == 0xFF) merge = 0;
    return .{
        .ftrim_addr = nv,
        .ftrim = (merge & ~@as(u8, 1)) | @as(u8, @intCast(value & 1)),
        .trim_addr = nv + 1,
        .trim = @intCast(value >> 1),
    };
}

// --- tests -----------------------------------------------------------------

const testing = std.testing;

test "flashDivider matches the reference Tcl" {
    // PGO on the SH16 left FCDIV=0xAB: divider 43 for an 8629 kHz bus.
    try testing.expectEqual(@as(?u8, 43), flashDivider(8629));
    try testing.expectEqual(@as(?u8, 19), flashDivider(4000));
    // Above 12.8 MHz PRDIV8 divides first.
    try testing.expectEqual(@as(?u8, 0x40 | 9), flashDivider(16000)); // 2000 kHz -> 9
    try testing.expectEqual(@as(?u8, null), flashDivider(100)); // too slow
    // 150..200 kHz window for every bus clock the divider accepts.
    var khz: u32 = 150;
    while (khz < 40000) : (khz += 7) {
        const d = flashDivider(khz) orelse continue;
        const fc = flashClockKhz(khz, d);
        try testing.expect(fc >= 150 and fc <= 200);
    }
}

test "trimBytes merges FTRIM and keeps other bits only when the image defines them" {
    const t = trimBytes(0xFFAE, 0x103, null); // 259 = TRIM 0x81, FTRIM 1
    try testing.expectEqual(@as(u8, 0x01), t.ftrim);
    try testing.expectEqual(@as(u8, 0x81), t.trim);
    try testing.expectEqual(@as(u32, 0xFFAF), t.trim_addr);
    // erased byte: other bits cleared
    try testing.expectEqual(@as(u8, 0x00), trimBytes(0xFFAE, 0x100, 0xFF).ftrim);
    // image-defined byte: other bits kept, bit 0 replaced
    try testing.expectEqual(@as(u8, 0x41), trimBytes(0xFFAE, 0x101, 0x40).ftrim);
    try testing.expectEqual(@as(u8, 0x40), trimBytes(0xFFAE, 0x100, 0x41).ftrim);
}

/// A linear ICS: f = f0 * (1 - 0.0019 * (trim - 256)); larger trim is slower.
const ModelProbe = struct {
    msb: u8 = 0,
    lsb: u8 = 0,
    f0_hz: f64,
    jitter: f64 = 0, // +/- fraction, deterministic
    n: u32 = 0,
    set_calls: u32 = 0,

    fn hzNow(self: *const ModelProbe) f64 {
        const t: f64 = @floatFromInt((@as(u32, self.msb) << 1) | self.lsb);
        return self.f0_hz * (1.0 - 0.0019 * (t - 256.0));
    }
    fn setTrim(ctx: *anyopaque, msb: ?u8, lsb: ?u8) anyerror!void {
        const s: *ModelProbe = @ptrCast(@alignCast(ctx));
        if (msb) |m| s.msb = m;
        if (lsb) |l| s.lsb = l;
        s.set_calls += 1;
    }
    fn measureHz(ctx: *anyopaque) anyerror!u32 {
        const s: *ModelProbe = @ptrCast(@alignCast(ctx));
        s.n += 1;
        // saw-tooth noise, mean zero
        const ph: f64 = @floatFromInt(@as(i32, @intCast(s.n % 7)) - 3);
        return @intFromFloat(s.hzNow() * (1.0 + s.jitter * ph / 3.0));
    }
    const vtable = Probe.VTable{ .setTrim = setTrim, .measureHz = measureHz };
    fn probe(self: *ModelProbe) Probe {
        return .{ .ctx = self, .vtable = &vtable };
    }
};

test "search finds the trim whose clock is closest to the target" {
    // 1.075 MHz at nominal trim 256; ask for 1% fast -> about 5 steps lower.
    var m = ModelProbe{ .f0_hz = 1_075_200 };
    const target: u32 = @intFromFloat(1_075_200 * 1.01);
    const r = try search(m.probe(), target);
    // exact answer: 256 - 0.01/0.0019 = 250.7
    try testing.expect(r.value == 251 or r.value == 250);
    try testing.expect(r.hz_per_step < 0);
    const err = @abs(@as(f64, @floatFromInt(r.measured_hz)) - @as(f64, @floatFromInt(target))) / @as(f64, @floatFromInt(target));
    try testing.expect(err < 0.0025); // within ~1 step
    try testing.expectEqual(@as(u32, 34 * samples_per_step), r.samples); // 17 values x 2 passes x 2
    // the winner is left on the target
    try testing.expectEqual(@as(u16, r.value), (@as(u16, m.msb) << 1) | m.lsb);
}

test "search tolerates measurement noise bigger than one trim step" {
    // +/-0.4% saw-tooth noise vs 0.19% per step: regression over ~68 points still lands within 2 steps.
    var m = ModelProbe{ .f0_hz = 2_150_400, .jitter = 0.004 };
    const r = try search(m.probe(), 2_150_400); // target = nominal: ideal trim 256
    try testing.expect(r.value >= 254 and r.value <= 258);
}

test "search never writes a TRIM byte of zero (the clock would swing 50% and lose the BDM)" {
    const Guard = struct {
        inner: ModelProbe,
        zeroed: bool = false,
        fn setTrim(ctx: *anyopaque, msb: ?u8, lsb: ?u8) anyerror!void {
            const g: *@This() = @ptrCast(@alignCast(ctx));
            if (msb) |m| {
                if (m < 2) g.zeroed = true;
            }
            return ModelProbe.setTrim(&g.inner, msb, lsb);
        }
        fn measureHz(ctx: *anyopaque) anyerror!u32 {
            const g: *@This() = @ptrCast(@alignCast(ctx));
            return ModelProbe.measureHz(&g.inner);
        }
        const vt = Probe.VTable{ .setTrim = setTrim, .measureHz = measureHz };
    };
    var g = Guard{ .inner = .{ .f0_hz = 2_150_400, .msb = 0x80 } };
    _ = try search(.{ .ctx = &g, .vtable = &Guard.vt }, 2_300_000);
    try testing.expect(!g.zeroed);
}

test "search reports a target the clock cannot reach" {
    var m = ModelProbe{ .f0_hz = 1_000_000 };
    try testing.expectError(error.TrimOutOfRange, search(m.probe(), 2_000_000)); // 2x too fast
    var m2 = ModelProbe{ .f0_hz = 1_000_000 };
    try testing.expectError(error.TrimOutOfRange, search(m2.probe(), 500_000));
}
