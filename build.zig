const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // Optionally link the system libusb (via pkg-config) instead of building
    // the vendored copy. The default is the self-contained static build so
    // users need nothing installed.
    const system_libusb = b.option(bool, "system-libusb", "Link the system libusb-1.0 instead of the vendored static build") orelse false;

    const libusb = if (system_libusb) null else buildLibusb(b, target, optimize);

    // Upstream USBDM protocol header, translated at build time and imported
    // as `usbdm_c` so protocol constants can be checked against the source of
    // truth (see vendor/usbdm/README.md).
    const usbdm_c = b.addTranslateC(.{
        .root_source_file = b.path("vendor/usbdm/Commands.h"),
        .target = target,
        .optimize = optimize,
    });

    const mod = b.addModule("usbdm", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .link_libc = true,
        .imports = &.{
            .{ .name = "usbdm_c", .module = usbdm_c.createModule() },
        },
    });

    // Device table: generated at build time from the vendored USBDM device XML
    // (the single source of truth). A host tool parses the XML into a Zig table +
    // SDID map, which device.zig re-exports. See vendor/usbdm-devices/.
    const device_types_mod = b.createModule(.{
        .root_source_file = b.path("src/device_types.zig"),
        .target = target,
        .optimize = optimize,
    });
    const gen_devices = b.addExecutable(.{
        .name = "gen_devices",
        .root_module = b.createModule(.{
            .root_source_file = b.path("tools/gen_devices.zig"),
            .target = b.graph.host, // runs on the build host
            .optimize = .Debug,
        }),
    });
    const gen_run = b.addRunArtifact(gen_devices);
    gen_run.addFileArg(b.path("vendor/usbdm-devices/hcs08_devices.xml"));
    gen_run.addFileArg(b.path("vendor/usbdm-devices/hcs12_devices.xml"));
    gen_run.addFileArg(b.path("vendor/usbdm-devices/cfv1_devices.xml"));
    const device_gen_zig = gen_run.addOutputFileArg("device_gen.zig");
    const device_gen_mod = b.createModule(.{
        .root_source_file = device_gen_zig,
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "device_types", .module = device_types_mod }},
    });
    mod.addImport("device_types", device_types_mod);
    mod.addImport("device_gen", device_gen_mod);
    if (libusb) |lib| {
        mod.linkLibrary(lib);
        mod.addIncludePath(b.path("vendor/libusb/libusb"));
    } else {
        mod.linkSystemLibrary("libusb-1.0", .{});
    }

    const exe = b.addExecutable(.{
        .name = "usbdm",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "usbdm", .module = mod },
            },
        }),
    });
    // Embed the vendored flash-routine blobs (outside src/, so exposed as
    // anonymous imports for @embedFile).
    exe.root_module.addAnonymousImport("flash_hcs08_0x80", .{
        .root_source_file = b.path("vendor/flash-routines/HCS08-default-0x80-flash-program.s19"),
    });
    exe.root_module.addAnonymousImport("flash_hcs08_0xB0", .{
        .root_source_file = b.path("vendor/flash-routines/HCS08-default-0xB0-flash-program.s19"),
    });
    exe.root_module.addAnonymousImport("flash_hcs12_fts", .{
        .root_source_file = b.path("vendor/flash-routines/HCS12-MMCV4-FTS-flash-program.s19"),
    });
    exe.root_module.addAnonymousImport("flash_hcs12_gmmc", .{
        .root_source_file = b.path("vendor/flash-routines/HCS12-GMMC-FTMRG-flash-program.s19"),
    });
    exe.root_module.addAnonymousImport("flash_cfv1", .{
        .root_source_file = b.path("vendor/flash-routines/ColdfireV1-default-flash-program.elf.S19"),
    });

    // The `gdb` TCP server uses Winsock2 on Windows.
    if (target.result.os.tag == .windows) exe.root_module.linkSystemLibrary("ws2_32", .{});

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the CLI");
    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);

    const mod_tests = b.addTest(.{ .root_module = mod });
    const run_mod_tests = b.addRunArtifact(mod_tests);
    const exe_tests = b.addTest(.{ .root_module = exe.root_module });
    const run_exe_tests = b.addRunArtifact(exe_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
}

/// Build the vendored libusb as a static library for `target`, selecting the
/// per-OS backend sources, config header, and system libraries. Sources come
/// from vendor/libusb (libusb 1.0.27); configs from vendor/libusb/config.
fn buildLibusb(b: *std.Build, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) *std.Build.Step.Compile {
    const lm = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true });
    const lib = b.addLibrary(.{ .name = "usb-1.0", .linkage = .static, .root_module = lm });
    lm.addIncludePath(b.path("vendor/libusb/libusb"));
    lm.addIncludePath(b.path("vendor/libusb/libusb/os"));

    const os = target.result.os.tag;
    // libusbi.h does `#include <config.h>`, so the per-OS config.h dir goes on
    // the include path (each config selects the HAVE_*/PLATFORM_* macros).
    const cfg_dir = switch (os) {
        .macos => "vendor/libusb/config/darwin",
        .windows => "vendor/libusb/config/windows",
        else => "vendor/libusb/config/linux",
    };
    lm.addIncludePath(b.path(cfg_dir));

    const flags = [_][]const u8{ "-std=gnu11", "-fno-sanitize=undefined" };

    const core = [_][]const u8{
        "libusb/core.c", "libusb/descriptor.c", "libusb/hotplug.c",
        "libusb/io.c",   "libusb/strerror.c",   "libusb/sync.c",
    };
    lm.addCSourceFiles(.{ .root = b.path("vendor/libusb"), .files = &core, .flags = &flags });

    const backend: []const []const u8 = switch (os) {
        .macos => &.{
            "libusb/os/darwin_usb.c",
            "libusb/os/threads_posix.c",
            "libusb/os/events_posix.c",
        },
        .windows => &.{
            "libusb/os/windows_common.c",
            "libusb/os/windows_winusb.c",
            "libusb/os/windows_usbdk.c",
            "libusb/os/threads_windows.c",
            "libusb/os/events_windows.c",
        },
        else => &.{
            "libusb/os/linux_usbfs.c",
            "libusb/os/linux_netlink.c",
            "libusb/os/threads_posix.c",
            "libusb/os/events_posix.c",
        },
    };
    lm.addCSourceFiles(.{ .root = b.path("vendor/libusb"), .files = backend, .flags = &flags });

    switch (os) {
        // macOS links Apple frameworks, which live in the Apple SDK. Building
        // for macOS therefore requires that SDK: it works natively on a Mac,
        // but cross-compiling to macOS from Linux/Windows needs the SDK synced
        // (e.g. `zig libc`/`--sysroot`) and is not exercised in CI.
        .macos => {
            lm.linkFramework("IOKit", .{});
            lm.linkFramework("CoreFoundation", .{});
            lm.linkFramework("Security", .{});
        },
        .windows => {
            lm.linkSystemLibrary("setupapi", .{});
            lm.linkSystemLibrary("ole32", .{});
            lm.linkSystemLibrary("winmm", .{});
        },
        else => {}, // Linux: netlink backend needs no extra libs
    }

    return lib;
}
