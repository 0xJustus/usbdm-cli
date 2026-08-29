//! Target-level protocol types (target selection, register numbering, status,
//! reset, memory spaces); values mirror Commands.h, tested at file end.

const std = @import("std");

/// Target families (TargetType_t).
pub const TargetType = enum(u8) {
    hcs12 = 0,
    hcs08 = 1,
    rs08 = 2,
    cfv1 = 3,
    cfvx = 4,
    jtag = 5,
    ezflash = 6,
    mc56f80xx = 7,
    arm_jtag = 8,
    arm_swd = 9,
    arm = 10,
    hcs12z = 11,
    illegal = 0xFE,
    off = 0xFF,
    _,

    pub fn fromName(s: []const u8) ?TargetType {
        const map = .{
            .{ "hcs12", TargetType.hcs12 },
            .{ "hc12", TargetType.hcs12 },
            .{ "s12", TargetType.hcs12 },
            .{ "hcs08", TargetType.hcs08 },
            .{ "s08", TargetType.hcs08 },
            .{ "rs08", TargetType.rs08 },
            .{ "cfv1", TargetType.cfv1 },
            .{ "coldfire-v1", TargetType.cfv1 },
            .{ "cfvx", TargetType.cfvx },
            .{ "jtag", TargetType.jtag },
            .{ "arm", TargetType.arm },
            .{ "arm-swd", TargetType.arm_swd },
            .{ "arm-jtag", TargetType.arm_jtag },
            .{ "hcs12z", TargetType.hcs12z },
            .{ "off", TargetType.off },
        };
        inline for (map) |entry| {
            if (std.ascii.eqlIgnoreCase(s, entry[0])) return entry[1];
        }
        return null;
    }

    pub fn name(self: TargetType) []const u8 {
        return switch (self) {
            .hcs12 => "HCS12",
            .hcs08 => "HCS08",
            .rs08 => "RS08",
            .cfv1 => "ColdFire V1",
            .cfvx => "ColdFire V2/3/4",
            .jtag => "JTAG",
            .ezflash => "EzPort Flash",
            .mc56f80xx => "MC56F80xx DSC",
            .arm_jtag => "ARM (JTAG)",
            .arm_swd => "ARM (SWD)",
            .arm => "ARM",
            .hcs12z => "HCS12Z",
            .illegal => "illegal",
            .off => "off",
            _ => "unknown",
        };
    }
};

/// READ_MEM/WRITE_MEM space+size byte (MemorySpace_t): bits 0..2 size (1/2/4),
/// bits 4..6 space, bit 7 fast.
pub const MemSpace = packed struct(u8) {
    size: u3,
    _pad: u1 = 0,
    space: Space = .none,
    fast: bool = false,

    pub const Space = enum(u3) { none = 0, program = 1, data = 2, global = 3, _ };

    pub const byte: MemSpace = .{ .size = 1 };
    pub const word: MemSpace = .{ .size = 2 };
    pub const long: MemSpace = .{ .size = 4 };

    pub fn toByte(self: MemSpace) u8 {
        return @bitCast(self);
    }
};

/// Target Vdd selection (TargetVddSelect_t).
pub const VddSelect = enum(u8) {
    off = 0,
    v3_3 = 1,
    v5 = 2,
    enable = 0x10,
    disable = 0x11,
    _,
};

/// Reset mode byte for CMD_USBDM_TARGET_RESET (TargetMode_t):
/// bits 0..1 mode (special/normal), bits 2..4 method.
pub const ResetMode = enum(u2) { special = 0, normal = 1 };
pub const ResetMethod = enum(u3) { all = 0, hardware = 1, software = 2, power = 3, default = 7 };

pub fn resetByte(mode: ResetMode, method: ResetMethod) u8 {
    return @as(u8, @intFromEnum(mode)) | (@as(u8, @intFromEnum(method)) << 2);
}

/// Decoded CMD_USBDM_GET_BDM_STATUS word (StatusBitMasks_t).
pub const BdmStatus = struct {
    ackn: bool,
    reset_detected: bool,
    reset_pin_inactive: bool,
    connection: Connection,
    halted: bool,
    power: Power,
    vpp: Vpp,

    pub const Connection = enum(u2) { not_connected = 0, sync_done = 1, guess_done = 2, user_done = 3 };
    pub const Power = enum(u2) { none = 0, external = 1, internal = 2, err = 3 };
    pub const Vpp = enum(u2) { off = 0, standby = 1, on = 2, err = 3 };

    pub fn decode(word: u16) BdmStatus {
        return .{
            .ackn = word & (1 << 0) != 0,
            .reset_detected = word & (1 << 1) != 0,
            .reset_pin_inactive = word & (1 << 2) != 0,
            .connection = @enumFromInt(@as(u2, @truncate(word >> 3))),
            .halted = word & (1 << 5) != 0,
            .power = @enumFromInt(@as(u2, @truncate(word >> 6))),
            .vpp = @enumFromInt(@as(u2, @truncate(word >> 8))),
        };
    }
};

/// Capability bits from CMD_USBDM_GET_CAPABILITIES (HardwareCapabilities_t).
/// HCS08/CFV1 bits arrive INVERTED (back-compat); `decode` un-inverts them.
pub const Capabilities = packed struct(u16) {
    hcs12: bool = false,
    rs08_12v: bool = false,
    vdd_control: bool = false,
    vdd_sense: bool = false,
    cfvx: bool = false,
    hcs08: bool = false,
    cfv1: bool = false,
    jtag: bool = false,
    dsc: bool = false,
    arm_jtag: bool = false,
    reset_sense: bool = false,
    pst: bool = false,
    cdc: bool = false,
    arm_swd: bool = false,
    hcs12z: bool = false,
    _pad: u1 = 0,

    pub fn decode(word: u16) Capabilities {
        var caps: Capabilities = @bitCast(word);
        caps.hcs08 = !caps.hcs08;
        caps.cfv1 = !caps.cfv1;
        return caps;
    }
};

/// READ_REG/WRITE_REG numbers per family.
pub const Hcs08Reg = enum(u8) { a = 8, ccr = 9, pc = 0xB, hx = 0xC, sp = 0xF };
pub const Hcs12Reg = enum(u8) { pc = 3, d = 4, x = 5, y = 6, sp = 7 };
/// RS08's distinct model: combined CCR/PC, a shadow PC, and A - NOT the HCS08 set.
pub const Rs08Reg = enum(u8) { a = 8, ccr_pc = 0xB, spc = 0xF };

pub const Cfv1Reg = enum(u8) {
    // zig fmt: off
    d0 = 0, d1 = 1, d2 = 2, d3 = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7,
    a0 = 8, a1 = 9, a2 = 10, a3 = 11, a4 = 12, a5 = 13, a6 = 14, a7 = 15,
    // zig fmt: on
};

/// CFV1 control regs (READ_CREG/WRITE_CREG).
pub const Cfv1CReg = enum(u8) { other_a7 = 0, vbr = 1, cpucr = 2, sr = 14, pc = 15 };

/// CFV1 debug regs (READ_DREG/WRITE_DREG).
pub const Cfv1DReg = enum(u8) { csr = 0, xcsr = 1, csr2 = 2, csr3 = 3 };

/// HCS08 debug reg (READ_DREG): BKPT.
pub const hcs08_dreg_bkpt: u16 = 0;
/// HCS12 debug regs are BD-space addresses (READ_DREG).
pub const hcs12_dreg_bdmsts: u16 = 0xFF01;
pub const hcs12_dreg_ccr: u16 = 0xFF06;

/// Pin levels for CMD_USBDM_CONTROL_PINS (PinLevelMasks_t).
/// Each signal is a 2-bit field: 0=no change, 1=3-state, 2=low, 3=high.
pub const PinLevel = enum(u2) { no_change = 0, tristate = 1, low = 2, high = 3 };

pub fn pinControl(bkgd: PinLevel, reset: PinLevel) u16 {
    return @as(u16, @intFromEnum(bkgd)) | (@as(u16, @intFromEnum(reset)) << 2);
}

pub const pin_release: u16 = 0xFFFF;

fn cValue(comptime T: type, v: anytype) T {
    return switch (@typeInfo(@TypeOf(v))) {
        .@"enum" => @intCast(@intFromEnum(v)),
        else => @intCast(v),
    };
}

test "target types match upstream" {
    const c = @import("usbdm_c");
    const pairs = .{
        .{ TargetType.hcs12, c.T_HC12 },
        .{ TargetType.hcs08, c.T_HCS08 },
        .{ TargetType.rs08, c.T_RS08 },
        .{ TargetType.cfv1, c.T_CFV1 },
        .{ TargetType.cfvx, c.T_CFVx },
        .{ TargetType.jtag, c.T_JTAG },
        .{ TargetType.ezflash, c.T_EZFLASH },
        .{ TargetType.mc56f80xx, c.T_MC56F80xx },
        .{ TargetType.arm_jtag, c.T_ARM_JTAG },
        .{ TargetType.arm_swd, c.T_ARM_SWD },
        .{ TargetType.arm, c.T_ARM },
        .{ TargetType.hcs12z, c.T_HCS12Z },
        .{ TargetType.off, c.T_OFF },
    };
    inline for (pairs) |p| {
        try std.testing.expectEqual(cValue(u8, p[1]), @intFromEnum(p[0]));
    }
}

test "memory space encoding matches upstream" {
    const c = @import("usbdm_c");
    try std.testing.expectEqual(cValue(u8, c.MS_Byte), MemSpace.byte.toByte());
    try std.testing.expectEqual(cValue(u8, c.MS_Word), MemSpace.word.toByte());
    try std.testing.expectEqual(cValue(u8, c.MS_Long), MemSpace.long.toByte());
    try std.testing.expectEqual(
        cValue(u8, c.MS_Global) | cValue(u8, c.MS_Word),
        (MemSpace{ .size = 2, .space = .global }).toByte(),
    );
    try std.testing.expectEqual(
        cValue(u8, c.MS_FAST) | cValue(u8, c.MS_Byte),
        (MemSpace{ .size = 1, .fast = true }).toByte(),
    );
}

test "vdd/reset encodings match upstream" {
    const c = @import("usbdm_c");
    try std.testing.expectEqual(cValue(u8, c.BDM_TARGET_VDD_OFF), @intFromEnum(VddSelect.off));
    try std.testing.expectEqual(cValue(u8, c.BDM_TARGET_VDD_3V3), @intFromEnum(VddSelect.v3_3));
    try std.testing.expectEqual(cValue(u8, c.BDM_TARGET_VDD_5V), @intFromEnum(VddSelect.v5));
    try std.testing.expectEqual(cValue(u8, c.BDM_TARGET_VDD_ENABLE), @intFromEnum(VddSelect.enable));
    try std.testing.expectEqual(cValue(u8, c.BDM_TARGET_VDD_DISABLE), @intFromEnum(VddSelect.disable));

    try std.testing.expectEqual(cValue(u8, c.RESET_SPECIAL) | cValue(u8, c.RESET_ALL), resetByte(.special, .all));
    try std.testing.expectEqual(cValue(u8, c.RESET_NORMAL) | cValue(u8, c.RESET_HARDWARE), resetByte(.normal, .hardware));
    try std.testing.expectEqual(cValue(u8, c.RESET_NORMAL) | cValue(u8, c.RESET_SOFTWARE), resetByte(.normal, .software));
    try std.testing.expectEqual(cValue(u8, c.RESET_SPECIAL) | cValue(u8, c.RESET_POWER), resetByte(.special, .power));
    try std.testing.expectEqual(cValue(u8, c.RESET_NORMAL) | cValue(u8, c.RESET_DEFAULT), resetByte(.normal, .default));
}

test "status word decode matches upstream masks" {
    const c = @import("usbdm_c");
    const word: u16 = cValue(u16, c.S_ACKN) | cValue(u16, c.S_SYNC_DONE) |
        cValue(u16, c.S_POWER_INT) | cValue(u16, c.S_VPP_ON) | cValue(u16, c.S_HALT);
    const st = BdmStatus.decode(word);
    try std.testing.expect(st.ackn);
    try std.testing.expect(!st.reset_detected);
    try std.testing.expectEqual(BdmStatus.Connection.sync_done, st.connection);
    try std.testing.expect(st.halted);
    try std.testing.expectEqual(BdmStatus.Power.internal, st.power);
    try std.testing.expectEqual(BdmStatus.Vpp.on, st.vpp);

    const st2 = BdmStatus.decode(cValue(u16, c.S_USER_DONE) | cValue(u16, c.S_RESET_DETECT) | cValue(u16, c.S_POWER_EXT));
    try std.testing.expectEqual(BdmStatus.Connection.user_done, st2.connection);
    try std.testing.expect(st2.reset_detected);
    try std.testing.expectEqual(BdmStatus.Power.external, st2.power);
}

test "capability bits match upstream" {
    const c = @import("usbdm_c");
    const raw: u16 = cValue(u16, c.BDM_CAP_HCS12) | cValue(u16, c.BDM_CAP_VDDCONTROL) |
        cValue(u16, c.BDM_CAP_RST) | cValue(u16, c.BDM_CAP_CDC);
    const caps = Capabilities.decode(raw);
    try std.testing.expect(caps.hcs12);
    try std.testing.expect(caps.vdd_control);
    try std.testing.expect(caps.reset_sense);
    try std.testing.expect(caps.cdc);
    // HCS08/CFV1 bits arrive inverted: absent bit means supported.
    try std.testing.expect(caps.hcs08);
    try std.testing.expect(caps.cfv1);
    const caps2 = Capabilities.decode(raw | cValue(u16, c.BDM_CAP_HCS08) | cValue(u16, c.BDM_CAP_CFV1));
    try std.testing.expect(!caps2.hcs08);
    try std.testing.expect(!caps2.cfv1);
}

test "register numbers match upstream" {
    const c = @import("usbdm_c");
    try std.testing.expectEqual(cValue(u8, c.HCS08_RegA), @intFromEnum(Hcs08Reg.a));
    try std.testing.expectEqual(cValue(u8, c.HCS08_RegCCR), @intFromEnum(Hcs08Reg.ccr));
    try std.testing.expectEqual(cValue(u8, c.HCS08_RegPC), @intFromEnum(Hcs08Reg.pc));
    try std.testing.expectEqual(cValue(u8, c.HCS08_RegHX), @intFromEnum(Hcs08Reg.hx));
    try std.testing.expectEqual(cValue(u8, c.HCS08_RegSP), @intFromEnum(Hcs08Reg.sp));

    try std.testing.expectEqual(cValue(u8, c.RS08_RegA), @intFromEnum(Rs08Reg.a));
    try std.testing.expectEqual(cValue(u8, c.RS08_RegCCR_PC), @intFromEnum(Rs08Reg.ccr_pc));
    try std.testing.expectEqual(cValue(u8, c.RS08_RegSPC), @intFromEnum(Rs08Reg.spc));

    try std.testing.expectEqual(cValue(u8, c.HCS12_RegPC), @intFromEnum(Hcs12Reg.pc));
    try std.testing.expectEqual(cValue(u8, c.HCS12_RegD), @intFromEnum(Hcs12Reg.d));
    try std.testing.expectEqual(cValue(u8, c.HCS12_RegX), @intFromEnum(Hcs12Reg.x));
    try std.testing.expectEqual(cValue(u8, c.HCS12_RegY), @intFromEnum(Hcs12Reg.y));
    try std.testing.expectEqual(cValue(u8, c.HCS12_RegSP), @intFromEnum(Hcs12Reg.sp));

    try std.testing.expectEqual(cValue(u8, c.CFV1_RegD0), @intFromEnum(Cfv1Reg.d0));
    try std.testing.expectEqual(cValue(u8, c.CFV1_RegA7), @intFromEnum(Cfv1Reg.a7));
    try std.testing.expectEqual(cValue(u8, c.CFV1_CRegVBR), @intFromEnum(Cfv1CReg.vbr));
    try std.testing.expectEqual(cValue(u8, c.CFV1_CRegSR), @intFromEnum(Cfv1CReg.sr));
    try std.testing.expectEqual(cValue(u8, c.CFV1_CRegPC), @intFromEnum(Cfv1CReg.pc));
    try std.testing.expectEqual(cValue(u8, c.CFV1_DRegCSR), @intFromEnum(Cfv1DReg.csr));
    try std.testing.expectEqual(cValue(u8, c.CFV1_DRegXCSR), @intFromEnum(Cfv1DReg.xcsr));

    try std.testing.expectEqual(cValue(u16, c.HCS12_DRegBDMSTS), hcs12_dreg_bdmsts);
    try std.testing.expectEqual(cValue(u16, c.HCS12_DRegCCR), hcs12_dreg_ccr);
}

test "pin control encoding matches upstream" {
    const c = @import("usbdm_c");
    try std.testing.expectEqual(cValue(u16, c.PIN_BKGD_LOW), pinControl(.low, .no_change));
    try std.testing.expectEqual(cValue(u16, c.PIN_BKGD_HIGH), pinControl(.high, .no_change));
    try std.testing.expectEqual(cValue(u16, c.PIN_BKGD_3STATE), pinControl(.tristate, .no_change));
    try std.testing.expectEqual(cValue(u16, c.PIN_RESET_LOW), pinControl(.no_change, .low));
    try std.testing.expectEqual(
        cValue(u16, c.PIN_BKGD_LOW) | cValue(u16, c.PIN_RESET_LOW),
        pinControl(.low, .low),
    );
}

test "target name round trip" {
    try std.testing.expectEqual(TargetType.hcs08, TargetType.fromName("HCS08").?);
    try std.testing.expectEqual(TargetType.hcs08, TargetType.fromName("s08").?);
    try std.testing.expectEqual(TargetType.hcs12, TargetType.fromName("hc12").?);
    try std.testing.expectEqual(TargetType.cfv1, TargetType.fromName("cfv1").?);
    try std.testing.expectEqual(@as(?TargetType, null), TargetType.fromName("z80"));
}
