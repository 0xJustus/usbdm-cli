//! USBDM protocol definitions - command numbers and response codes per the
//! USBDM firmware (Commands.h, protocol v4.x).

const std = @import("std");
const Io = std.Io;
const usb = @import("usb.zig");
const tty = @import("tty.zig");
const hexdump = @import("hexdump.zig");

pub const UsbId = struct { vid: u16, pid: u16, name: []const u8 };

pub const known_ids = [_]UsbId{
    .{ .vid = 0x16d0, .pid = 0x0567, .name = "USBDM" },
    .{ .vid = 0x16d0, .pid = 0x06a5, .name = "USBDM (CDC composite)" },
    .{ .vid = 0x15a2, .pid = 0x0021, .name = "OSBDM (Freescale)" },
    .{ .vid = 0x0425, .pid = 0x1000, .name = "TBDML" },
};

pub fn knownName(vid: u16, pid: u16) ?[]const u8 {
    for (known_ids) |id| {
        if (id.vid == vid and id.pid == pid) return id.name;
    }
    return null;
}

/// USBDM command bytes (BDMCommands, Commands.h). Bulk firmware sends
/// [size, cmd, params...]; get_ver/icp_boot go on EP0 (work in ICP mode).
pub const Command = enum(u8) {
    get_command_response = 0,
    set_target = 1,
    set_vdd = 2,
    debug = 3,
    get_bdm_status = 4,
    get_capabilities = 5,
    set_options = 6,
    // 7 reserved (GET_SETTINGS, commented out upstream)
    control_pins = 8,
    // 9..11 reserved
    get_ver = 12,
    // 13 reserved
    icp_boot = 14,
    connect = 15,
    set_speed = 16,
    get_speed = 17,
    custom_command = 18,
    // 19 reserved
    read_status_reg = 20,
    write_control_reg = 21,
    target_reset = 22,
    target_step = 23,
    target_go = 24,
    target_halt = 25,
    write_reg = 26,
    read_reg = 27,
    write_creg = 28,
    read_creg = 29,
    write_dreg = 30,
    read_dreg = 31,
    write_mem = 32,
    read_mem = 33,
    read_all_regs = 34,
    // 35..37 reserved (deleted RS08 flash commands)
    jtag_gotoreset = 38,
    jtag_gotoshift = 39,
    jtag_write = 40,
    jtag_read = 41,
    set_vpp = 42,
    jtag_read_write = 43,
    jtag_execute_sequence = 44,
    _,
};

/// First response byte (USBDM_ErrorCode, Commands.h). Codes 0..33 from
/// firmware; higher codes are DLL-only, shouldn't appear on the wire.
pub const Rc = enum(u8) {
    ok = 0,
    illegal_params = 1,
    fail = 2,
    busy = 3,
    illegal_command = 4,
    no_connection = 5,
    overrun = 6,
    cf_illegal_command = 7,
    device_open_failed = 8,
    usb_device_busy = 9,
    usb_device_not_installed = 10,
    usb_device_removed = 11,
    usb_retry_ok = 12,
    unexpected_reset = 13,
    cf_not_ready = 14,
    unknown_target = 15,
    no_tx_routine = 16,
    no_rx_routine = 17,
    bdm_en_failed = 18,
    reset_timeout_fall = 19,
    bkgd_timeout = 20,
    sync_timeout = 21,
    unknown_speed = 22,
    wrong_programming_mode = 23,
    flash_programming_busy = 24,
    vdd_not_removed = 25,
    vdd_not_present = 26,
    vdd_wrong_mode = 27,
    cf_bus_error = 28,
    usb_error = 29,
    ack_timeout = 30,
    failed_trim = 31,
    feature_not_supported = 32,
    reset_timeout_rise = 33,
    target_busy = 42,
    secured = 50,
    unexpected_response = 53,
    hcs_access_error = 54,
    cf_data_invalid = 55,
    cf_overrun = 56,
    _,

    pub fn name(self: Rc) []const u8 {
        return switch (self) {
            .ok => "OK",
            .illegal_params => "illegal parameters",
            .fail => "general failure",
            .busy => "busy with last command",
            .illegal_command => "illegal command (wrong target mode?)",
            .no_connection => "no connection to target",
            .overrun => "command overrun",
            .cf_illegal_command => "illegal command (ColdFire)",
            .unknown_target => "target unknown or unsupported by this BDM",
            .sync_timeout => "no response to SYNC sequence",
            .vdd_not_present => "target Vdd not present",
            .target_busy => "target busy (executing?)",
            .secured => "device is secured (operation blocked)",
            .unexpected_response => "unexpected/inconsistent response from BDM",
            .hcs_access_error => "memory access failed (target in stop/wait state)",
            .cf_data_invalid => "ColdFire target returned data-invalid",
            .cf_overrun => "ColdFire target returned overrun",
            else => std.enums.tagName(Rc, self) orelse "unknown error",
        };
    }
};

/// bmRequestType for EP0 vendor-request IN.
pub const vendor_request_in: u8 = 0xC0;

pub const default_timeout_ms: u32 = 1000;

/// CMD_USBDM_GET_VER response. Versions pack major.minor in the high/low
/// nibbles of one byte (0x4C -> 4.12). Wire: [rc, bdm_sw, bdm_hw, icp_sw,
/// icp_hw] (USB.c ICP_GET_VER; Commands.h doxygen order is stale).
pub const Version = struct {
    rc: Rc,
    bdm_software: u8,
    bdm_hardware: u8,
    icp_software: u8,
    icp_hardware: u8,

    pub fn major(byte: u8) u8 {
        return byte >> 4;
    }

    pub fn minor(byte: u8) u8 {
        return byte & 0xF;
    }

    /// bdm_software == 0xFF => running ICP bootloader, not BDM firmware.
    pub fn inIcpMode(self: Version) bool {
        return self.bdm_software == 0xFF;
    }
};

/// Best-effort USB traffic logging for `--verbose`; default is a no-op, never
/// fails the traced command.
pub const Trace = struct {
    out: ?*Io.Writer = null,
    st: tty.Style = .none,

    pub fn note(self: Trace, comptime fmt: []const u8, args: anytype) void {
        const w = self.out orelse return;
        w.print("{s}~ " ++ fmt ++ "{s}\n", .{self.st.dim} ++ args ++ .{self.st.reset}) catch {};
    }

    pub fn packet(self: Trace, label: []const u8, bytes: []const u8) void {
        const w = self.out orelse return;
        w.print("{s}~ {s} ({d} bytes){s}\n", .{ self.st.dim, label, bytes.len, self.st.reset }) catch return;
        hexdump.dump(w, self.st, 0, bytes) catch {};
    }
};

pub const CommandError = usb.Error || error{ShortResponse};

/// Query fw+hw version over EP0; needs no interface claim, so it's the
/// standard "alive?" probe.
pub fn getVersion(handle: usb.DeviceHandle, trace: Trace) CommandError!Version {
    trace.note("EP0 -> vendor request GET_VER (bmRequestType 0x{x:0>2}, bRequest 0x{x:0>2})", .{
        vendor_request_in, @intFromEnum(Command.get_ver),
    });
    var data: [5]u8 = undefined;
    const n = try handle.controlTransfer(
        vendor_request_in,
        @intFromEnum(Command.get_ver),
        0x0100, // wValue per reference host (bdm_usb_getversion)
        0,
        &data,
        default_timeout_ms,
    );
    trace.packet("EP0 <- GET_VER response", data[0..n]);
    if (n < data.len) return error.ShortResponse;
    return .{
        .rc = @enumFromInt(data[0]),
        .bdm_software = data[1],
        .bdm_hardware = data[2],
        .icp_software = data[3],
        .icp_hardware = data[4],
    };
}

/// Max command transfer (MAX_COMMAND_SIZE, Commands.h); sizes transport buffers.
pub const max_command_size = 254;

/// Accepts either shape translate-c emits for C enumerators (int or enum).
fn cValue(v: anytype) u8 {
    return switch (@typeInfo(@TypeOf(v))) {
        .@"enum" => @intCast(@intFromEnum(v)),
        else => @intCast(v),
    };
}

test "command codes match upstream Commands.h" {
    const c = @import("usbdm_c");
    const pairs = .{
        .{ Command.get_command_response, c.CMD_USBDM_GET_COMMAND_RESPONSE },
        .{ Command.set_target, c.CMD_USBDM_SET_TARGET },
        .{ Command.set_vdd, c.CMD_USBDM_SET_VDD },
        .{ Command.debug, c.CMD_USBDM_DEBUG },
        .{ Command.get_bdm_status, c.CMD_USBDM_GET_BDM_STATUS },
        .{ Command.get_capabilities, c.CMD_USBDM_GET_CAPABILITIES },
        .{ Command.set_options, c.CMD_USBDM_SET_OPTIONS },
        .{ Command.control_pins, c.CMD_USBDM_CONTROL_PINS },
        .{ Command.get_ver, c.CMD_USBDM_GET_VER },
        .{ Command.icp_boot, c.CMD_USBDM_ICP_BOOT },
        .{ Command.connect, c.CMD_USBDM_CONNECT },
        .{ Command.set_speed, c.CMD_USBDM_SET_SPEED },
        .{ Command.get_speed, c.CMD_USBDM_GET_SPEED },
        .{ Command.custom_command, c.CMD_CUSTOM_COMMAND },
        .{ Command.read_status_reg, c.CMD_USBDM_READ_STATUS_REG },
        .{ Command.write_control_reg, c.CMD_USBDM_WRITE_CONTROL_REG },
        .{ Command.target_reset, c.CMD_USBDM_TARGET_RESET },
        .{ Command.target_step, c.CMD_USBDM_TARGET_STEP },
        .{ Command.target_go, c.CMD_USBDM_TARGET_GO },
        .{ Command.target_halt, c.CMD_USBDM_TARGET_HALT },
        .{ Command.write_reg, c.CMD_USBDM_WRITE_REG },
        .{ Command.read_reg, c.CMD_USBDM_READ_REG },
        .{ Command.write_creg, c.CMD_USBDM_WRITE_CREG },
        .{ Command.read_creg, c.CMD_USBDM_READ_CREG },
        .{ Command.write_dreg, c.CMD_USBDM_WRITE_DREG },
        .{ Command.read_dreg, c.CMD_USBDM_READ_DREG },
        .{ Command.write_mem, c.CMD_USBDM_WRITE_MEM },
        .{ Command.read_mem, c.CMD_USBDM_READ_MEM },
        .{ Command.read_all_regs, c.CMD_USBDM_READ_ALL_REGS },
        .{ Command.jtag_gotoreset, c.CMD_USBDM_JTAG_GOTORESET },
        .{ Command.jtag_gotoshift, c.CMD_USBDM_JTAG_GOTOSHIFT },
        .{ Command.jtag_write, c.CMD_USBDM_JTAG_WRITE },
        .{ Command.jtag_read, c.CMD_USBDM_JTAG_READ },
        .{ Command.set_vpp, c.CMD_USBDM_SET_VPP },
        .{ Command.jtag_read_write, c.CMD_USBDM_JTAG_READ_WRITE },
        .{ Command.jtag_execute_sequence, c.CMD_USBDM_JTAG_EXECUTE_SEQUENCE },
    };
    inline for (pairs) |p| {
        try std.testing.expectEqual(cValue(p[1]), @intFromEnum(p[0]));
    }
}

test "error codes match upstream Commands.h" {
    const c = @import("usbdm_c");
    const pairs = .{
        .{ Rc.ok, c.BDM_RC_OK },
        .{ Rc.illegal_params, c.BDM_RC_ILLEGAL_PARAMS },
        .{ Rc.fail, c.BDM_RC_FAIL },
        .{ Rc.busy, c.BDM_RC_BUSY },
        .{ Rc.illegal_command, c.BDM_RC_ILLEGAL_COMMAND },
        .{ Rc.no_connection, c.BDM_RC_NO_CONNECTION },
        .{ Rc.overrun, c.BDM_RC_OVERRUN },
        .{ Rc.cf_illegal_command, c.BDM_RC_CF_ILLEGAL_COMMAND },
        .{ Rc.device_open_failed, c.BDM_RC_DEVICE_OPEN_FAILED },
        .{ Rc.usb_device_busy, c.BDM_RC_USB_DEVICE_BUSY },
        .{ Rc.usb_device_not_installed, c.BDM_RC_USB_DEVICE_NOT_INSTALLED },
        .{ Rc.usb_device_removed, c.BDM_RC_USB_DEVICE_REMOVED },
        .{ Rc.usb_retry_ok, c.BDM_RC_USB_RETRY_OK },
        .{ Rc.unexpected_reset, c.BDM_RC_UNEXPECTED_RESET },
        .{ Rc.cf_not_ready, c.BDM_RC_CF_NOT_READY },
        .{ Rc.unknown_target, c.BDM_RC_UNKNOWN_TARGET },
        .{ Rc.no_tx_routine, c.BDM_RC_NO_TX_ROUTINE },
        .{ Rc.no_rx_routine, c.BDM_RC_NO_RX_ROUTINE },
        .{ Rc.bdm_en_failed, c.BDM_RC_BDM_EN_FAILED },
        .{ Rc.reset_timeout_fall, c.BDM_RC_RESET_TIMEOUT_FALL },
        .{ Rc.bkgd_timeout, c.BDM_RC_BKGD_TIMEOUT },
        .{ Rc.sync_timeout, c.BDM_RC_SYNC_TIMEOUT },
        .{ Rc.unknown_speed, c.BDM_RC_UNKNOWN_SPEED },
        .{ Rc.wrong_programming_mode, c.BDM_RC_WRONG_PROGRAMMING_MODE },
        .{ Rc.flash_programming_busy, c.BDM_RC_FLASH_PROGRAMING_BUSY },
        .{ Rc.vdd_not_removed, c.BDM_RC_VDD_NOT_REMOVED },
        .{ Rc.vdd_not_present, c.BDM_RC_VDD_NOT_PRESENT },
        .{ Rc.vdd_wrong_mode, c.BDM_RC_VDD_WRONG_MODE },
        .{ Rc.cf_bus_error, c.BDM_RC_CF_BUS_ERROR },
        .{ Rc.usb_error, c.BDM_RC_USB_ERROR },
        .{ Rc.ack_timeout, c.BDM_RC_ACK_TIMEOUT },
        .{ Rc.failed_trim, c.BDM_RC_FAILED_TRIM },
        .{ Rc.feature_not_supported, c.BDM_RC_FEATURE_NOT_SUPPORTED },
        .{ Rc.reset_timeout_rise, c.BDM_RC_RESET_TIMEOUT_RISE },
        .{ Rc.target_busy, c.BDM_RC_TARGET_BUSY },
        .{ Rc.secured, c.BDM_RC_SECURED },
        .{ Rc.unexpected_response, c.BDM_RC_UNEXPECTED_RESPONSE },
        .{ Rc.hcs_access_error, c.BDM_RC_HCS_ACCESS_ERROR },
        .{ Rc.cf_data_invalid, c.BDM_RC_CF_DATA_INVALID },
        .{ Rc.cf_overrun, c.BDM_RC_CF_OVERRUN },
    };
    inline for (pairs) |p| {
        try std.testing.expectEqual(cValue(p[1]), @intFromEnum(p[0]));
    }
}

test "max command size matches upstream Commands.h" {
    const c = @import("usbdm_c");
    try std.testing.expectEqual(@as(u8, max_command_size), cValue(c.MAX_COMMAND_SIZE));
}

test "version nibble decoding" {
    try std.testing.expectEqual(@as(u8, 4), Version.major(0x4C));
    try std.testing.expectEqual(@as(u8, 12), Version.minor(0x4C));
}

test "known id lookup" {
    try std.testing.expectEqualStrings("USBDM", knownName(0x16d0, 0x0567).?);
    try std.testing.expectEqual(@as(?[]const u8, null), knownName(0xdead, 0xbeef));
}
