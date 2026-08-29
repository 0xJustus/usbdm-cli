//! Minimal hand-written libusb-1.0 bindings (only the surface this tool needs),
//! so the build needs nothing beyond the system library.

const std = @import("std");

pub const raw = struct {
    pub const libusb_context = opaque {};
    pub const libusb_device = opaque {};
    pub const libusb_device_handle = opaque {};

    /// Mirrors `struct libusb_device_descriptor`; multi-byte fields are host-endian.
    pub const DeviceDescriptor = extern struct {
        bLength: u8,
        bDescriptorType: u8,
        bcdUSB: u16,
        bDeviceClass: u8,
        bDeviceSubClass: u8,
        bDeviceProtocol: u8,
        bMaxPacketSize0: u8,
        idVendor: u16,
        idProduct: u16,
        bcdDevice: u16,
        iManufacturer: u8,
        iProduct: u8,
        iSerialNumber: u8,
        bNumConfigurations: u8,
    };

    pub extern fn libusb_init(ctx: *?*libusb_context) c_int;
    pub extern fn libusb_exit(ctx: ?*libusb_context) void;
    pub extern fn libusb_get_device_list(ctx: ?*libusb_context, list: *?[*]*libusb_device) isize;
    pub extern fn libusb_free_device_list(list: [*]*libusb_device, unref_devices: c_int) void;
    pub extern fn libusb_get_device_descriptor(dev: *libusb_device, desc: *DeviceDescriptor) c_int;
    pub extern fn libusb_get_bus_number(dev: *libusb_device) u8;
    pub extern fn libusb_get_device_address(dev: *libusb_device) u8;
    pub extern fn libusb_open(dev: *libusb_device, handle: *?*libusb_device_handle) c_int;
    pub extern fn libusb_close(handle: *libusb_device_handle) void;
    pub extern fn libusb_get_string_descriptor_ascii(handle: *libusb_device_handle, desc_index: u8, data: [*]u8, length: c_int) c_int;
    pub extern fn libusb_claim_interface(handle: *libusb_device_handle, interface_number: c_int) c_int;
    pub extern fn libusb_release_interface(handle: *libusb_device_handle, interface_number: c_int) c_int;
    pub extern fn libusb_control_transfer(handle: *libusb_device_handle, bmRequestType: u8, bRequest: u8, wValue: u16, wIndex: u16, data: [*]u8, wLength: u16, timeout: c_uint) c_int;
    pub extern fn libusb_bulk_transfer(handle: *libusb_device_handle, endpoint: u8, data: [*]u8, length: c_int, transferred: *c_int, timeout: c_uint) c_int;
};

/// LIBUSB_ERROR_* mapped to a Zig error set.
pub const Error = error{
    Io,
    InvalidParam,
    Access,
    NoDevice,
    NotFound,
    Busy,
    Timeout,
    Overflow,
    Pipe,
    Interrupted,
    NoMem,
    NotSupported,
    Other,
};

pub fn check(code: c_int) Error!c_int {
    if (code >= 0) return code;
    return switch (code) {
        -1 => error.Io,
        -2 => error.InvalidParam,
        -3 => error.Access,
        -4 => error.NoDevice,
        -5 => error.NotFound,
        -6 => error.Busy,
        -7 => error.Timeout,
        -8 => error.Overflow,
        -9 => error.Pipe,
        -10 => error.Interrupted,
        -11 => error.NoMem,
        -12 => error.NotSupported,
        else => error.Other,
    };
}

pub const Context = struct {
    ptr: *raw.libusb_context,

    pub fn init() Error!Context {
        var ctx: ?*raw.libusb_context = null;
        _ = try check(raw.libusb_init(&ctx));
        return .{ .ptr = ctx.? };
    }

    pub fn deinit(self: Context) void {
        raw.libusb_exit(self.ptr);
    }

    pub fn deviceList(self: Context) Error!DeviceList {
        var list: ?[*]*raw.libusb_device = null;
        const n = raw.libusb_get_device_list(self.ptr, &list);
        if (n < 0) _ = try check(@intCast(n));
        return .{ .devices = list.?[0..@intCast(n)] };
    }
};

pub const DeviceList = struct {
    devices: []*raw.libusb_device,

    pub fn deinit(self: DeviceList) void {
        raw.libusb_free_device_list(self.devices.ptr, 1);
    }
};

pub fn descriptor(dev: *raw.libusb_device) Error!raw.DeviceDescriptor {
    var desc: raw.DeviceDescriptor = undefined;
    _ = try check(raw.libusb_get_device_descriptor(dev, &desc));
    return desc;
}

pub fn busNumber(dev: *raw.libusb_device) u8 {
    return raw.libusb_get_bus_number(dev);
}

pub fn deviceAddress(dev: *raw.libusb_device) u8 {
    return raw.libusb_get_device_address(dev);
}

pub fn open(dev: *raw.libusb_device) Error!DeviceHandle {
    var handle: ?*raw.libusb_device_handle = null;
    _ = try check(raw.libusb_open(dev, &handle));
    return .{ .ptr = handle.? };
}

pub const DeviceHandle = struct {
    ptr: *raw.libusb_device_handle,

    pub fn close(self: DeviceHandle) void {
        raw.libusb_close(self.ptr);
    }

    pub fn claimInterface(self: DeviceHandle, iface: u8) Error!void {
        _ = try check(raw.libusb_claim_interface(self.ptr, iface));
    }

    pub fn releaseInterface(self: DeviceHandle, iface: u8) void {
        _ = raw.libusb_release_interface(self.ptr, iface);
    }

    pub fn stringDescriptorAscii(self: DeviceHandle, index: u8, buf: []u8) Error![]u8 {
        const n = try check(raw.libusb_get_string_descriptor_ascii(self.ptr, index, buf.ptr, @intCast(buf.len)));
        return buf[0..@intCast(n)];
    }

    pub fn controlTransfer(self: DeviceHandle, bm_request_type: u8, request: u8, value: u16, index: u16, data: []u8, timeout_ms: u32) Error!usize {
        const n = try check(raw.libusb_control_transfer(self.ptr, bm_request_type, request, value, index, data.ptr, @intCast(data.len), timeout_ms));
        return @intCast(n);
    }

    pub fn bulkWrite(self: DeviceHandle, endpoint: u8, data: []const u8, timeout_ms: u32) Error!usize {
        var transferred: c_int = 0;
        // libusb wants a non-const ptr but never writes it on OUT transfers.
        _ = try check(raw.libusb_bulk_transfer(self.ptr, endpoint, @constCast(data.ptr), @intCast(data.len), &transferred, timeout_ms));
        return @intCast(transferred);
    }

    pub fn bulkRead(self: DeviceHandle, endpoint: u8, buf: []u8, timeout_ms: u32) Error!usize {
        var transferred: c_int = 0;
        _ = try check(raw.libusb_bulk_transfer(self.ptr, endpoint, buf.ptr, @intCast(buf.len), &transferred, timeout_ms));
        return @intCast(transferred);
    }
};
