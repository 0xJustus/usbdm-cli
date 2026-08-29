const std = @import("std");
const Io = std.Io;

const lib = @import("usbdm");
const usb = lib.usb;
const protocol = lib.protocol;
const target = lib.target;
const transport = lib.transport;
const session = lib.session;
const tty = lib.tty;
const hexdump = lib.hexdump;
const hexfile = lib.hexfile;
const gdb = lib.gdb;
const gdbstub = lib.gdbstub;

var global_io: Io = undefined;
/// Buffered stdout; error/exit paths flush it.
var global_out: ?*Io.Writer = null;

/// Keep in sync with build.zig.zon .version.
const tool_version = "0.1.0";

const usage_fmt =
    \\{s}usbdm{s} - CLI for FZ0622C (USBDM/OSBDM) programmer-debugger interfaces
    \\
    \\{s}Usage:{s} usbdm <command> [options]
    \\
    \\{s}Device commands:{s}
    \\  list             List connected BDM interfaces (--all: every USB device)
    \\  parts            List known --part devices (name, family, flash geometry)
    \\  version          Firmware/hardware version (EP0; works in ICP mode too)
    \\  probe            Version, capabilities and status of the BDM
    \\  status           BDM status: connection, target power, speed
    \\
    \\{s}Target commands{s} (use --target, default hcs08):
    \\  connect          Bring up the target connection and measure its speed
    \\  reset            Reset the target (--normal/--special, --hardware/--software/--power)
    \\  halt | go | step Execution control
    \\  regs             Show all core registers
    \\  reg <name> [val] Read or write one register (e.g. pc, sp, a; d0..a7 on cfv1)
    \\  read <addr> <n>  Read n bytes of memory, hexdump to stdout
    \\  write <addr> <byte...>  Write bytes to memory
    \\  fill <addr> <n> <byte>  Fill n bytes of memory with a byte value
    \\  dump <addr> <n> -o <file>  Save memory to file (--format bin|srec|ihex)
    \\  load <file>      Write an image file (srec/ihex/bin) to target RAM
    \\  vdd off|3v3|5v   Control BDM-supplied target power
    \\  pins [sig=lvl]   Pin control: bkgd/reset = low|high|3state, or 'release'
    \\
    \\{s}Flash commands{s} (HCS08/HCS12/CFV1 via a downloaded RAM routine; --part <name>):
    \\  program <file>   Erase affected pages, program, and verify an image
    \\  verify <file>    Read back and compare flash against an image
    \\  erase [<addr>]   Mass-erase, or erase one 512-byte page at <addr>
    \\  blank-check      Check the flash array is fully erased
    \\  unsecure         Mass-erase and clear flash security (HCS08/HCS12/CFV1)
    \\  secure           Lock debug access (program the security byte)
    \\  eeprom <file>    Program EEPROM/data-flash (S12G GMMC parts; addr = offset)
    \\  gdb              Serve a GDB remote stub over TCP (--port, default 1234)
    \\
    \\{s}Options:{s}
    \\  --target <t>     hcs08 | hcs12 | rs08 | cfv1 | cfvx (default hcs08)
    \\  --vdd <v>        Power target from BDM: off | 3v3 | 5v
    \\  --speed <hz>     Force communication speed (e.g. 4000000, 0x3D0900)
    \\  --device <n>     Pick the n-th BDM when several are connected (0-based)
    \\  --serial <s>     Pick the BDM whose USB serial number is <s>
    \\  --word, --long   Element size for memory access (default byte)
    \\  --addr <a>       Load address override for binary files
    \\  --format <f>     dump output format: bin | srec | ihex (default from -o suffix)
    \\  -o <file>        Output file for dump
    \\  -f, --force      Skip confirmation for destructive actions (overwrite, erase)
    \\  --part <name>    Target device for flash (e.g. mc9s08jm60); --flash-base overrides
    \\  --bus-hz <n>     Target bus clock for flash timing (else measured)
    \\  --no-verify      Skip the post-program verify pass
    \\  --sim            Run against a software virtual target (no hardware needed)
    \\  --json           Machine-readable output for list/parts/status/regs/read
    \\  -v, --verbose    Hex-dump USB traffic as it happens
    \\  --color, --no-color  Force colors (auto-detects a TTY, honors NO_COLOR)
    \\
;

const Flags = struct {
    verbose: bool = false,
    color: ?bool = null,
    all: bool = false,
    target: target.TargetType = .hcs08,
    vdd: ?target.VddSelect = null,
    device: ?usize = null,
    serial: ?[]const u8 = null,
    speed_hz: ?u32 = null,
    out_path: ?[]const u8 = null,
    format: ?hexfile.Format = null,
    load_addr: ?u32 = null,
    reset_mode: target.ResetMode = .special,
    reset_method: target.ResetMethod = .all,
    elem: target.MemSpace = .byte,
    force: bool = false,
    part: ?[]const u8 = null,
    bus_hz: ?u32 = null,
    flash_base: ?u16 = null,
    no_verify: bool = false,
    port: u16 = 1234,
    sim: bool = false,
    json: bool = false,
    show_version: bool = false,
    help: bool = false,
    positionals: []const []const u8 = &.{},
};

fn eq(a: []const u8, b: []const u8) bool {
    return std.mem.eql(u8, a, b);
}

/// JSON string literal (quoted, escaped) for --json output.
fn jsonStr(out: *Io.Writer, s: []const u8) !void {
    try out.writeByte('"');
    for (s) |c| switch (c) {
        '"' => try out.writeAll("\\\""),
        '\\' => try out.writeAll("\\\\"),
        '\n' => try out.writeAll("\\n"),
        '\r' => try out.writeAll("\\r"),
        '\t' => try out.writeAll("\\t"),
        0...8, 11, 12, 14...31 => try out.print("\\u{x:0>4}", .{c}),
        else => try out.writeByte(c),
    };
    try out.writeByte('"');
}

/// Distinct exit codes so scripts/CI/GDB can classify failures.
const ExitCode = enum(u8) {
    ok = 0,
    general = 1,
    usage = 2, // bad arguments / unknown command
    no_device = 3, // no BDM found / cannot open
    bdm_error = 4, // BDM/target protocol error
    flash_error = 5, // flash routine failed (erase/program/blank)
    verify_mismatch = 6, // programmed data didn't read back
    io_error = 7, // host file read/parse/write
};

fn fatalCode(code: ExitCode, comptime fmt: []const u8, args: anytype) noreturn {
    if (global_out) |o| o.flush() catch {};
    std.debug.print("error: " ++ fmt ++ "\n", args);
    std.process.exit(@intFromEnum(code));
}

fn fatal(comptime fmt: []const u8, args: anytype) noreturn {
    fatalCode(.general, fmt, args);
}

extern "c" fn read(fd: c_int, buf: [*]u8, count: usize) isize;

/// Gate a destructive action. --force proceeds; a TTY prompts; non-TTY
/// (scripts/CI) refuses rather than block on input that never comes.
fn confirm(out: *Io.Writer, st: tty.Style, force: bool, comptime fmt: []const u8, args: anytype) bool {
    if (force) return true;
    if (!tty.stdinIsTty()) {
        fatal("refusing without confirmation - re-run with --force (stdin is not a terminal)", .{});
    }
    out.print("{s}" ++ fmt ++ "{s} [y/N] ", .{st.yellow} ++ args ++ .{st.reset}) catch {};
    out.flush() catch {};
    var buf: [8]u8 = undefined;
    const n = read(0, &buf, buf.len);
    if (n <= 0) return false;
    return buf[0] == 'y' or buf[0] == 'Y';
}

fn fileExists(path: []const u8) bool {
    Io.Dir.cwd().access(global_io, path, .{}) catch return false;
    return true;
}

fn parseInt(comptime T: type, s: []const u8, what: []const u8) T {
    return std.fmt.parseInt(T, s, 0) catch
        fatalCode(.usage, "invalid {s}: '{s}' (use decimal or 0x-prefixed hex)", .{ what, s });
}

fn nextValue(args: []const [:0]const u8, i: *usize, flag: []const u8) []const u8 {
    i.* += 1;
    if (i.* >= args.len) fatalCode(.usage, "{s} requires a value", .{flag});
    return args[i.*];
}

fn parseFlags(arena: std.mem.Allocator, args: []const [:0]const u8) !Flags {
    var f: Flags = .{};
    var positionals: std.ArrayList([]const u8) = .empty;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg: []const u8 = args[i];
        if (eq(arg, "-v") or eq(arg, "--verbose")) {
            f.verbose = true;
        } else if (eq(arg, "--no-color")) {
            f.color = false;
        } else if (eq(arg, "--color")) {
            f.color = true;
        } else if (eq(arg, "--all")) {
            f.all = true;
        } else if (eq(arg, "-f") or eq(arg, "--force") or eq(arg, "-y") or eq(arg, "--yes")) {
            f.force = true;
        } else if (eq(arg, "--word")) {
            f.elem = .word;
        } else if (eq(arg, "--long")) {
            f.elem = .long;
        } else if (eq(arg, "--normal")) {
            f.reset_mode = .normal;
        } else if (eq(arg, "--special")) {
            f.reset_mode = .special;
        } else if (eq(arg, "--hardware")) {
            f.reset_method = .hardware;
        } else if (eq(arg, "--software")) {
            f.reset_method = .software;
        } else if (eq(arg, "--power")) {
            f.reset_method = .power;
        } else if (eq(arg, "--target")) {
            const v = nextValue(args, &i, "--target");
            f.target = target.TargetType.fromName(v) orelse
                fatalCode(.usage, "unknown target '{s}' (try hcs08, hcs12, rs08, cfv1, cfvx)", .{v});
        } else if (eq(arg, "--vdd")) {
            const v = nextValue(args, &i, "--vdd");
            f.vdd = if (eq(v, "off"))
                .off
            else if (eq(v, "3v3") or eq(v, "3.3") or eq(v, "3300"))
                .v3_3
            else if (eq(v, "5v") or eq(v, "5") or eq(v, "5000"))
                .v5
            else
                fatalCode(.usage, "invalid --vdd '{s}' (off, 3v3, 5v)", .{v});
        } else if (eq(arg, "--device")) {
            f.device = parseInt(usize, nextValue(args, &i, "--device"), "device index");
        } else if (eq(arg, "--serial")) {
            f.serial = nextValue(args, &i, "--serial");
        } else if (eq(arg, "--part")) {
            f.part = nextValue(args, &i, "--part");
        } else if (eq(arg, "--bus-hz")) {
            f.bus_hz = parseInt(u32, nextValue(args, &i, "--bus-hz"), "bus frequency");
        } else if (eq(arg, "--flash-base")) {
            f.flash_base = parseInt(u16, nextValue(args, &i, "--flash-base"), "flash base");
        } else if (eq(arg, "--sim")) {
            f.sim = true;
        } else if (eq(arg, "--json")) {
            f.json = true;
        } else if (eq(arg, "--version") or eq(arg, "-V")) {
            f.show_version = true;
        } else if (eq(arg, "--help") or eq(arg, "-h")) {
            // `<cmd> --help` -> per-command help; bare `--help` -> full usage.
            f.help = true;
        } else if (eq(arg, "--no-verify")) {
            f.no_verify = true;
        } else if (eq(arg, "--port")) {
            f.port = parseInt(u16, nextValue(args, &i, "--port"), "port");
        } else if (eq(arg, "--speed")) {
            f.speed_hz = parseInt(u32, nextValue(args, &i, "--speed"), "speed");
        } else if (eq(arg, "--addr")) {
            f.load_addr = parseInt(u32, nextValue(args, &i, "--addr"), "address");
        } else if (eq(arg, "-o") or eq(arg, "--out")) {
            f.out_path = nextValue(args, &i, "-o");
        } else if (eq(arg, "--format")) {
            const v = nextValue(args, &i, "--format");
            f.format = if (eq(v, "bin") or eq(v, "binary"))
                .binary
            else if (eq(v, "srec") or eq(v, "s19"))
                .srec
            else if (eq(v, "ihex") or eq(v, "hex"))
                .ihex
            else
                fatalCode(.usage, "invalid --format '{s}' (bin, srec, ihex)", .{v});
        } else if (arg.len > 2 and std.mem.startsWith(u8, arg, "--")) {
            fatalCode(.usage, "unknown option '{s}' (see 'usbdm help')", .{arg});
        } else {
            try positionals.append(arena, arg);
        }
    }
    f.positionals = positionals.items;
    return f;
}

pub fn main(init: std.process.Init) !void {
    const arena = init.arena.allocator();
    global_io = init.io;

    var stdout_buffer: [8192]u8 = undefined;
    var stdout_file_writer: Io.File.Writer = .init(.stdout(), global_io, &stdout_buffer);
    const out = &stdout_file_writer.interface;
    global_out = out;

    const args = try init.minimal.args.toSlice(arena);
    const flags = try parseFlags(arena, if (args.len > 1) args[1..] else args[0..0]);

    const st: tty.Style = if (flags.color orelse tty.stdoutSupportsColor()) .ansi else .none;
    const trace: protocol.Trace = if (flags.verbose) .{ .out = out, .st = st } else .{};
    const cmd: []const u8 = if (flags.positionals.len > 0) flags.positionals[0] else "help";

    if (flags.show_version) {
        if (flags.json)
            try out.print("{{\"tool\":\"usbdm-cli\",\"version\":\"{s}\"}}\n", .{tool_version})
        else
            try out.print("usbdm-cli {s}\n", .{tool_version});
        try out.flush();
        return;
    }

    if (eq(cmd, "help") or eq(cmd, "--help") or eq(cmd, "-h") or (flags.help and flags.positionals.len == 0)) {
        try printUsage(out, st);
    } else if (eq(cmd, "list")) {
        try cmdList(out, st, flags.all, flags.json);
    } else if (eq(cmd, "parts")) {
        try cmdParts(out, st, flags.json);
    } else if (eq(cmd, "version")) {
        try cmdVersion(out, st, flags);
    } else {
        try runDeviceCommand(arena, cmd, &flags, out, st, trace);
    }

    try out.flush();
}

fn printUsage(out: *Io.Writer, st: tty.Style) !void {
    try out.print(usage_fmt, .{
        st.bold, st.reset, st.bold, st.reset, st.bold, st.reset,
        st.bold, st.reset, st.bold, st.reset, st.bold, st.reset,
    });
}

const Bdm = struct {
    ctx: usb.Context,
    devices: usb.DeviceList,
    handle: usb.DeviceHandle,
    bulk: transport.BulkUsb,
    sess: session.Session,
    kind_name: []const u8,
    desc: usb.raw.DeviceDescriptor,
    bus: u8,
    address: u8,
    /// Set in `--sim` mode; the USB fields above are then unused.
    sim: ?*lib.sim.Sim = null,

    fn close(self: *Bdm) void {
        if (self.sim != null) return; // no USB resources to release
        self.handle.releaseInterface(0);
        self.handle.close();
        self.devices.deinit();
        self.ctx.deinit();
    }
};

fn openBdm(flags: *const Flags, trace: protocol.Trace) !Bdm {
    const ctx = try usb.Context.init();
    errdefer ctx.deinit();
    const list = try ctx.deviceList();
    errdefer list.deinit();

    var index: usize = 0;
    for (list.devices) |dev| {
        const desc = usb.descriptor(dev) catch continue;
        const name = protocol.knownName(desc.idVendor, desc.idProduct) orelse continue;
        if (flags.device) |want| {
            if (index != want) {
                index += 1;
                continue;
            }
        }
        const handle = usb.open(dev) catch |err| switch (err) {
            error.Access => fatalCode(.no_device, "cannot open {s} ({x:0>4}:{x:0>4}): permission denied", .{
                name, desc.idVendor, desc.idProduct,
            }),
            else => return err,
        };
        // --serial match needs the open handle.
        if (flags.serial) |want| {
            var sbuf: [128]u8 = undefined;
            const got = if (desc.iSerialNumber != 0) handle.stringDescriptorAscii(desc.iSerialNumber, &sbuf) catch "" else "";
            if (!eq(got, want)) {
                handle.close();
                index += 1;
                continue;
            }
        }
        errdefer handle.close();
        handle.claimInterface(0) catch |err| switch (err) {
            error.Busy, error.Access => fatalCode(
                .no_device,
                "{s} is in use by another program (claim interface failed: {s})",
                .{ name, @errorName(err) },
            ),
            else => return err,
        };
        var bdm: Bdm = .{
            .ctx = ctx,
            .devices = list,
            .handle = handle,
            .bulk = .{ .handle = handle },
            .sess = undefined,
            .kind_name = name,
            .desc = desc,
            .bus = usb.busNumber(dev),
            .address = usb.deviceAddress(dev),
        };
        _ = trace;
        _ = &bdm;
        return bdm;
    }
    if (flags.serial) |want| {
        fatalCode(.no_device, "no BDM with serial '{s}' found ({d} present)", .{ want, index });
    }
    if (flags.device != null) {
        fatalCode(.no_device, "BDM device index {d} not found ({d} present)", .{ flags.device.?, index });
    }
    fatalCode(.no_device, "no BDM interface found (try 'usbdm list --all')", .{});
}

/// Must run after Bdm is at its final address: transport captures &bdm.bulk.
fn attachSession(bdm: *Bdm, trace: protocol.Trace) void {
    bdm.sess = session.Session.init(bdm.bulk.transport(trace));
}

fn runDeviceCommand(
    arena: std.mem.Allocator,
    cmd: []const u8,
    flags: *const Flags,
    out: *Io.Writer,
    st: tty.Style,
    trace: protocol.Trace,
) !void {
    const Handler = struct {
        name: []const u8,
        f: *const fn (std.mem.Allocator, *Bdm, *const Flags, *Io.Writer, tty.Style) anyerror!void,
        /// Min positional args, including the command word.
        min_args: usize = 1,
        usage: []const u8 = "",
    };
    const handlers = [_]Handler{
        .{ .name = "probe", .f = cmdProbe },
        .{ .name = "status", .f = cmdStatus },
        .{ .name = "connect", .f = cmdConnect },
        .{ .name = "identify", .f = cmdIdentify },
        .{ .name = "reset", .f = cmdReset },
        .{ .name = "halt", .f = cmdHalt },
        .{ .name = "go", .f = cmdGo },
        .{ .name = "step", .f = cmdStep },
        .{ .name = "regs", .f = cmdRegs },
        .{ .name = "reg", .f = cmdReg, .min_args = 2, .usage = "usbdm reg <name> [value]" },
        .{ .name = "read", .f = cmdRead, .min_args = 3, .usage = "usbdm read <addr> <len>" },
        .{ .name = "write", .f = cmdWrite, .min_args = 3, .usage = "usbdm write <addr> <byte...>" },
        .{ .name = "fill", .f = cmdFill, .min_args = 4, .usage = "usbdm fill <addr> <len> <byte>" },
        .{ .name = "dump", .f = cmdDump, .min_args = 3, .usage = "usbdm dump <addr> <len> -o <file>" },
        .{ .name = "load", .f = cmdLoad, .min_args = 2, .usage = "usbdm load <file>" },
        .{ .name = "vdd", .f = cmdVdd, .min_args = 2, .usage = "usbdm vdd off|3v3|5v" },
        .{ .name = "pins", .f = cmdPins },
        .{ .name = "program", .f = cmdProgram, .min_args = 2, .usage = "usbdm program <file> [--part <p>] [--no-verify]" },
        .{ .name = "verify", .f = cmdVerify, .min_args = 2, .usage = "usbdm verify <file> [--part <p>]" },
        .{ .name = "erase", .f = cmdErase, .usage = "usbdm erase [<page-addr>] [--part <p>]" },
        .{ .name = "blank-check", .f = cmdBlankCheck },
        .{ .name = "unsecure", .f = cmdUnsecure },
        .{ .name = "secure", .f = cmdSecure },
        .{ .name = "eeprom", .f = cmdEeprom, .min_args = 2, .usage = "usbdm eeprom <file> --part <p>" },
        .{ .name = "gdb", .f = cmdGdb },
    };
    const handler = for (handlers) |h| {
        if (eq(cmd, h.name)) break h;
    } else {
        std.debug.print("unknown command: {s}\n\n", .{cmd});
        try printUsage(out, st);
        try out.flush();
        std.process.exit(@intFromEnum(ExitCode.usage));
    };

    if (flags.help) {
        const usage = if (handler.usage.len > 0) handler.usage else handler.name;
        try out.print("usage: {s}\n", .{usage});
        try out.flush();
        return;
    }

    // Validate arg count before opening hardware (reported with no BDM).
    if (flags.positionals.len < handler.min_args) fatalCode(.usage, "usage: {s}", .{handler.usage});

    // Overwrite check before opening the BDM (works with no dongle).
    if (eq(cmd, "dump")) {
        if (flags.out_path) |path| {
            if (fileExists(path) and !confirm(out, st, flags.force, "{s} already exists - overwrite?", .{path}))
                fatal("aborted: {s} exists (pass --force to overwrite)", .{path});
        }
    }

    // Validate target + --part before opening (reported with no dongle).
    const is_flash = eq(cmd, "program") or eq(cmd, "verify") or eq(cmd, "erase") or
        eq(cmd, "blank-check") or eq(cmd, "unsecure") or eq(cmd, "secure") or eq(cmd, "eeprom");
    if (is_flash) {
        requireFlashTarget(flags);
        _ = resolveDevice(flags);
    }

    if (flags.sim) return runSim(arena, cmd, handler.f, flags, out, st, trace);

    var bdm = try openBdm(flags, trace);
    defer bdm.close();
    attachSession(&bdm, trace);

    handler.f(arena, &bdm, flags, out, st) catch |err| switch (err) {
        error.BdmError => {
            try out.flush();
            std.debug.print("BDM error: {s} ({d})\n", .{
                bdm.sess.last_rc.name(), @intFromEnum(bdm.sess.last_rc),
            });
            std.process.exit(@intFromEnum(ExitCode.bdm_error));
        },
        error.BdmBusyTimeout => fatalCode(.bdm_error, "BDM stayed busy - command timed out", .{}),
        error.ToggleMismatch => fatalCode(.bdm_error, "USB response out of sequence (stale data); retry the command", .{}),
        else => {
            out.flush() catch {};
            return err;
        },
    };
}

/// --sim: run against the src/sim.zig virtual target (no hardware). Models the
/// BDM/target side, not USB enumeration/EP0 or the GDB socket - those declined.
fn runSim(
    arena: std.mem.Allocator,
    cmd: []const u8,
    f: *const fn (std.mem.Allocator, *Bdm, *const Flags, *Io.Writer, tty.Style) anyerror!void,
    flags: *const Flags,
    out: *Io.Writer,
    st: tty.Style,
    trace: protocol.Trace,
) !void {
    if (eq(cmd, "probe") or eq(cmd, "version") or eq(cmd, "list") or eq(cmd, "vdd") or eq(cmd, "pins"))
        fatal("--sim can't run '{s}' - it models the target (BDM) side, not USB enumeration/EP0", .{cmd});
    if (eq(cmd, "gdb"))
        out.print("{s}[--sim] note: memory/registers/execution are served by the emulated CPU running the real routine; single-step and continue-to-BGND work{s}\n", .{ st.dim, st.reset }) catch {};

    const d = resolveDevice(flags);
    var sim = lib.sim.Sim.init(arena, d);
    defer sim.deinit();

    // Real CPU core so --sim runs the ACTUAL vendored routine (single-steppable);
    // `go` runs to BGND. Flash spans both fixed windows. PPAGE-banked (>16-bit)
    // windows aren't covered (cores are 16-bit).
    const flash_lo = d.flash_start;
    const flash_hi: u32 = if (d.flash_end2 != 0) d.flash_end2 else d.flash_end;
    var hcs08_cpu: lib.cpu08.Cpu = undefined;
    var hcs12_cpu: lib.cpu12.Cpu = undefined;
    var cfv1_cpu: lib.cfv1_emu.Cpu = undefined;
    switch (familyOfTarget(flags.target)) {
        .hcs08 => {
            hcs08_cpu = lib.cpu08.Cpu.init(arena, d.flash_base, flash_lo, flash_hi) catch fatal("out of memory building the emulator", .{});
            hcs08_cpu.halted = true; // target starts halted (special mode)
            sim.cpu = .{ .hcs08 = &hcs08_cpu };
        },
        .hcs12 => {
            hcs12_cpu = if (d.hcs12_routine == .gmmc_ftmrg)
                lib.cpu12.Cpu.initGmmc(arena, d.flash_base) catch fatal("out of memory building the emulator", .{})
            else
                lib.cpu12.Cpu.init(arena, d.flash_base, flash_lo, flash_hi) catch fatal("out of memory building the emulator", .{});
            hcs12_cpu.halted = true;
            sim.cpu = .{ .hcs12 = &hcs12_cpu };
        },
        .cfv1 => {
            cfv1_cpu = lib.cfv1_emu.Cpu.init(arena, d.flash_base, flash_lo, flash_hi) catch fatal("out of memory building the emulator", .{});
            cfv1_cpu.halted = true;
            sim.cpu = .{ .cfv1 = &cfv1_cpu };
        },
    }

    var bdm: Bdm = undefined;
    bdm.sim = &sim;
    bdm.kind_name = "virtual target (--sim)";
    bdm.desc = std.mem.zeroes(usb.raw.DeviceDescriptor);
    bdm.bus = 0;
    bdm.address = 0;
    bdm.sess = session.Session.init(sim.asTransport(trace));

    try out.print("{s}[--sim] virtual {s} target: {s}{s}\n", .{ st.dim, @tagName(d.family), d.name, st.reset });
    f(arena, &bdm, flags, out, st) catch |err| switch (err) {
        error.BdmError => {
            try out.flush();
            std.debug.print("BDM error: {s} ({d})\n", .{ bdm.sess.last_rc.name(), @intFromEnum(bdm.sess.last_rc) });
            std.process.exit(@intFromEnum(ExitCode.bdm_error));
        },
        else => {
            out.flush() catch {};
            return err;
        },
    };
    try out.flush();
}

/// Mirrors the reference host's adaptRequiredBdmOptions: RESET signal for
/// HCS12-family, speed-guess only for HC12, auto-reconnect = always.
fn optionsForTarget(flags: *const Flags) session.Options {
    var opts: session.Options = .{ .auto_reconnect = 2 };
    switch (flags.target) {
        .hcs12, .hcs12z => opts.use_reset_signal = true,
        else => opts.guess_speed = false,
    }
    if (flags.vdd) |v| {
        opts.target_vdd = v;
        opts.leave_target_powered = true;
    }
    return opts;
}

fn bringUp(bdm: *Bdm, flags: *const Flags) !void {
    // Learn cmd-buffer size (JS16=145 vs JMxx=254) for chunk sizing; ok if it fails.
    _ = bdm.sess.capabilities() catch {};
    try bdm.sess.setOptions(optionsForTarget(flags));
    try bdm.sess.setTarget(flags.target);
    if (flags.speed_hz) |hz| {
        const sync = session.hzToSync(hz) catch fatal("--speed {d} out of range", .{hz});
        try bdm.sess.setSpeedSync(sync);
    }
    const allow_reset_retry = switch (flags.target) {
        .hcs08, .rs08, .cfv1 => true,
        else => false,
    };
    try bdm.sess.connectWithRetry(allow_reset_retry);
}

fn cmdParts(out: *Io.Writer, st: tty.Style, json: bool) !void {
    if (json) {
        try out.writeAll("[");
        for (lib_device.table, 0..) |d, i| {
            if (i != 0) try out.writeAll(",");
            try out.writeAll("{\"name\":");
            try jsonStr(out, d.name);
            try out.print(",\"family\":\"{s}\",\"ram_start\":{d},\"ram_end\":{d},\"flash_start\":{d},\"flash_end\":{d},\"page_size\":{d},\"write_align\":{d},\"flash_bytes\":{d}}}", .{
                @tagName(d.family), d.ram_start, d.ram_end, d.flash_start, d.flash_end, d.page_size, d.write_align, d.flashLen(),
            });
        }
        try out.writeAll("]\n");
        return;
    }
    try out.print("{s}known --part values{s} ({d} devices):\n", .{ st.bold, st.reset, lib_device.table.len });
    for (lib_device.table) |d| {
        try out.print("  {s}{s: <14}{s} {s}{s: <5}{s}  ram {x:0>4}-{x:0>4}  flash {x:0>4}-{x:0>4}", .{
            st.green,    d.name,             st.reset,
            st.dim,      @tagName(d.family), st.reset,
            d.ram_start, d.ram_end,          d.flash_start,
            d.flash_end,
        });
        if (d.flash_start2) |s2| try out.print(",{x:0>4}-{x:0>4}", .{ s2, d.flash_end2 });
        if (d.paged) |p| try out.print(" +paged({d} pages)", .{@as(u16, p.page_last - p.page_first) + 1});
        try out.print("  {s}{d}K{s}\n", .{ st.dim, d.flashLen() / 1024, st.reset });
    }
    try out.print("\n  {s}use with --part, e.g. `usbdm program app.s19 --part mc9s08jm60`{s}\n", .{ st.dim, st.reset });
    try out.print("  {s}not listed? override with --flash-base, or try it under `--sim` first{s}\n", .{ st.dim, st.reset });
}

fn cmdList(out: *Io.Writer, st: tty.Style, all: bool, json: bool) !void {
    const ctx = try usb.Context.init();
    defer ctx.deinit();
    const list = try ctx.deviceList();
    defer list.deinit();

    if (json) {
        try out.writeAll("[");
        var n: usize = 0;
        for (list.devices) |dev| {
            const desc = usb.descriptor(dev) catch continue;
            const name = protocol.knownName(desc.idVendor, desc.idProduct);
            if (name == null and !all) continue;
            if (n != 0) try out.writeAll(",");
            n += 1;
            try out.print("{{\"bus\":{d},\"addr\":{d},\"vid\":{d},\"pid\":{d},\"name\":", .{
                usb.busNumber(dev), usb.deviceAddress(dev), desc.idVendor, desc.idProduct,
            });
            if (name) |nm| try jsonStr(out, nm) else try out.writeAll("null");
            try out.writeAll("}");
        }
        try out.writeAll("]\n");
        return;
    }

    var found: usize = 0;
    for (list.devices) |dev| {
        const desc = usb.descriptor(dev) catch continue;
        const name = protocol.knownName(desc.idVendor, desc.idProduct);
        if (name == null and !all) continue;
        found += 1;

        try out.print("{s}bus {d:0>3} addr {d:0>3}{s}  {s}{x:0>4}:{x:0>4}{s}", .{
            st.dim,    usb.busNumber(dev), usb.deviceAddress(dev), st.reset,
            st.yellow, desc.idVendor,      desc.idProduct,         st.reset,
        });
        if (name) |n| try out.print("  {s}{s}[{s}]{s}", .{ st.bold, st.green, n, st.reset });
        printStrings(out, st, dev, desc) catch {};
        try out.writeAll("\n");
    }

    if (found == 0) {
        if (all) {
            try out.print("{s}no USB devices visible{s}\n", .{ st.dim, st.reset });
        } else {
            try out.print("{s}no BDM interfaces found{s} (try {s}usbdm list --all{s})\n", .{
                st.yellow, st.reset, st.bold, st.reset,
            });
        }
    }
}

fn printStrings(out: *Io.Writer, st: tty.Style, dev: *usb.raw.libusb_device, desc: usb.raw.DeviceDescriptor) !void {
    if (desc.iManufacturer == 0 and desc.iProduct == 0) return;
    const handle = usb.open(dev) catch return;
    defer handle.close();

    var buf: [128]u8 = undefined;
    if (desc.iManufacturer != 0) {
        if (handle.stringDescriptorAscii(desc.iManufacturer, &buf)) |s| {
            try out.print("  {s}{s}{s}", .{ st.dim, s, st.reset });
        } else |_| {}
    }
    if (desc.iProduct != 0) {
        if (handle.stringDescriptorAscii(desc.iProduct, &buf)) |s| {
            try out.print("  {s}\"{s}\"{s}", .{ st.cyan, s, st.reset });
        } else |_| {}
    }
}

fn printVersion(out: *Io.Writer, st: tty.Style, ver: protocol.Version) !void {
    const V = protocol.Version;
    if (ver.inIcpMode()) {
        try out.print("  {s}device is in ICP (bootloader) mode{s}\n", .{ st.yellow, st.reset });
    }
    try out.print("  {s}BDM firmware{s}  {s}{s}{d}.{d}{s} {s}(0x{x:0>2}){s}   {s}hardware{s} 0x{x:0>2}\n", .{
        st.dim,                    st.reset,
        st.bold,                   st.green,
        V.major(ver.bdm_software), V.minor(ver.bdm_software),
        st.reset,                  st.dim,
        ver.bdm_software,          st.reset,
        st.dim,                    st.reset,
        ver.bdm_hardware,
    });
    try out.print("  {s}ICP firmware{s}  {s}{s}{d}.{d}{s} {s}(0x{x:0>2}){s}   {s}hardware{s} 0x{x:0>2}\n", .{
        st.dim,                    st.reset,
        st.bold,                   st.green,
        V.major(ver.icp_software), V.minor(ver.icp_software),
        st.reset,                  st.dim,
        ver.icp_software,          st.reset,
        st.dim,                    st.reset,
        ver.icp_hardware,
    });
}

fn printDeviceHeading(bdm: *Bdm, out: *Io.Writer, st: tty.Style) !void {
    try out.print("{s}{s}{s} at {s}bus {d:0>3} addr {d:0>3}{s} ({s}{x:0>4}:{x:0>4}{s})\n", .{
        st.bold,            bdm.kind_name, st.reset,
        st.dim,             bdm.bus,       bdm.address,
        st.reset,           st.yellow,     bdm.desc.idVendor,
        bdm.desc.idProduct, st.reset,
    });
    try out.flush();
}

fn cmdVersion(out: *Io.Writer, st: tty.Style, flags: Flags) !void {
    var f = flags;
    f.verbose = false; // EP0-only path has no bulk trace
    var bdm = try openBdm(&f, .{});
    defer bdm.close();
    try printDeviceHeading(&bdm, out, st);
    const ver = try protocol.getVersion(bdm.handle, .{});
    if (ver.rc != .ok) {
        std.debug.print("device returned error: {s} ({d})\n", .{ ver.rc.name(), @intFromEnum(ver.rc) });
        std.process.exit(@intFromEnum(ExitCode.bdm_error));
    }
    try printVersion(out, st, ver);
}

fn cmdProbe(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    _ = flags;
    try printDeviceHeading(bdm, out, st);

    const ver = try protocol.getVersion(bdm.handle, .{});
    if (ver.rc == .ok) try printVersion(out, st, ver);

    const info = try bdm.sess.capabilities();
    try out.print("  {s}extended ver{s}  {s}{s}{d}.{d}.{d}{s}   {s}command buffer{s} {d} bytes\n", .{
        st.dim,          st.reset,              st.bold,
        st.green,        info.version[0],       info.version[1],
        info.version[2], st.reset,              st.dim,
        st.reset,        info.max_command_size,
    });

    try out.print("  {s}capabilities{s}  ", .{ st.dim, st.reset });
    const caps = info.caps;
    const cap_list = .{
        .{ caps.hcs12, "HCS12" },             .{ caps.hcs08, "HCS08" },
        .{ caps.rs08_12v, "RS08" },           .{ caps.cfv1, "CFV1" },
        .{ caps.cfvx, "CFVx" },               .{ caps.jtag, "JTAG" },
        .{ caps.dsc, "DSC" },                 .{ caps.arm_jtag, "ARM-JTAG" },
        .{ caps.arm_swd, "ARM-SWD" },         .{ caps.hcs12z, "HCS12Z" },
        .{ caps.vdd_control, "Vdd-control" }, .{ caps.vdd_sense, "Vdd-sense" },
        .{ caps.reset_sense, "RESET" },       .{ caps.cdc, "CDC" },
    };
    var first = true;
    inline for (cap_list) |entry| {
        if (entry[0]) {
            if (!first) try out.writeAll(", ");
            try out.print("{s}{s}{s}", .{ st.green, entry[1], st.reset });
            first = false;
        }
    }
    if (first) try out.print("{s}none{s}", .{ st.dim, st.reset });
    try out.writeAll("\n");

    try printBdmStatus(bdm, out, st);
}

fn printBdmStatus(bdm: *Bdm, out: *Io.Writer, st: tty.Style) !void {
    const s = try bdm.sess.bdmStatus();
    const conn: []const u8 = switch (s.connection) {
        .not_connected => "not connected",
        .sync_done => "connected (SYNC)",
        .guess_done => "connected (guessed)",
        .user_done => "connected (user speed)",
    };
    const power: []const u8 = switch (s.power) {
        .none => "none",
        .external => "external",
        .internal => "BDM-supplied",
        .err => "ERROR (overcurrent?)",
    };
    try out.print("  {s}target{s}        {s}{s}{s}{s}   {s}power{s} {s}{s}{s}{s}", .{
        st.dim,                                                      st.reset,
        if (s.connection == .not_connected) st.yellow else st.green, st.bold,
        conn,                                                        st.reset,
        st.dim,                                                      st.reset,
        if (s.power == .none) st.yellow else st.green,               st.bold,
        power,                                                       st.reset,
    });
    if (s.ackn) try out.print("   {s}ACKN{s}", .{ st.green, st.reset });
    if (s.halted) try out.print("   {s}halted{s}", .{ st.yellow, st.reset });
    if (s.reset_detected) try out.print("   {s}reset detected{s}", .{ st.yellow, st.reset });
    if (!s.reset_pin_inactive) try out.print("   {s}RESET asserted{s}", .{ st.yellow, st.reset });
    if (s.vpp != .off) try out.print("   {s}Vpp {t}{s}", .{ st.dim, s.vpp, st.reset });
    try out.writeAll("\n");
}

fn cmdStatus(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    if (flags.json) {
        const s = try bdm.sess.bdmStatus();
        const sync = bdm.sess.getSpeedSync() catch 0;
        const hz = if (sync > 1) session.syncToHz(sync) else 0;
        try out.print("{{\"connection\":\"{s}\",\"power\":\"{s}\",\"vpp\":\"{s}\",\"halted\":{},\"ackn\":{},\"reset_detected\":{},\"reset_pin_inactive\":{},\"speed_hz\":{d}}}\n", .{
            @tagName(s.connection), @tagName(s.power), @tagName(s.vpp), s.halted, s.ackn, s.reset_detected, s.reset_pin_inactive, hz,
        });
        return;
    }
    try printDeviceHeading(bdm, out, st);
    try printBdmStatus(bdm, out, st);
    const sync = try bdm.sess.getSpeedSync();
    if (sync != 0) {
        const hz = session.syncToHz(sync);
        try out.print("  {s}speed{s}         {s}{s}{d}.{d:0>3} MHz{s} {s}(sync {d}){s}\n", .{
            st.dim,   st.reset,       st.bold,
            st.green, hz / 1_000_000, (hz % 1_000_000) / 1000,
            st.reset, st.dim,         sync,
            st.reset,
        });
    }
}

fn cmdConnect(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try printDeviceHeading(bdm, out, st);
    try bringUp(bdm, flags);
    try out.print("  {s}connected to {s}{s}{s} target\n", .{ st.green, st.bold, flags.target.name(), st.reset });
    const sync = try bdm.sess.getSpeedSync();
    if (sync != 0) {
        const hz = session.syncToHz(sync);
        try out.print("  {s}bus speed{s}     {s}{s}{d}.{d:0>3} MHz{s}\n", .{
            st.dim, st.reset, st.bold, st.green, hz / 1_000_000, (hz % 1_000_000) / 1000, st.reset,
        });
    }
    reportDetected(bdm, flags, out, st);
    try printBdmStatus(bdm, out, st);
}

/// Read the family SDID register (SDIDH:SDIDL, big-endian u16). null on failure.
fn readSdid(bdm: *Bdm, family: lib_device.Family) ?u16 {
    var b: [2]u8 = undefined;
    bdm.sess.readMem(.byte, lib_device.familySdidAddr(family), &b) catch return null;
    return (@as(u16, b[0]) << 8) | b[1];
}

/// Count SDID matches for a family, returning the first. Many variants share an
/// SDID (they differ only in flash size), so a single match can be ambiguous.
fn sdidMatches(fam: lib_device.Family, sdid: u16) struct { first: ?lib_device.SdidEntry, count: usize } {
    var first: ?lib_device.SdidEntry = null;
    var count: usize = 0;
    for (lib_device.sdid_table) |e| {
        if (e.family != fam) continue;
        if (sdid & e.mask != e.sdid) continue;
        if (first == null) first = e;
        count += 1;
    }
    return .{ .first = first, .count = count };
}

/// Print the SDID and the part(s) it identifies (informational; silent on failure).
fn reportDetected(bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) void {
    const fam = familyOfTarget(flags.target);
    const sdid = readSdid(bdm, fam) orelse return;
    const m = sdidMatches(fam, sdid);
    if (m.first) |e| {
        var extra: [24]u8 = undefined;
        const more = if (m.count > 1) (std.fmt.bufPrint(&extra, " +{d} sharing SDID", .{m.count - 1}) catch "") else "";
        out.print("  {s}device{s}        {s}{s}{s} {s}(SDID 0x{X:0>4}{s}){s}\n", .{ st.dim, st.reset, st.bold, e.name, st.reset, st.dim, sdid, more, st.reset }) catch {};
    } else {
        out.print("  {s}device{s}        {s}unrecognized (SDID 0x{X:0>4}){s}\n", .{ st.dim, st.reset, st.dim, sdid, st.reset }) catch {};
    }
}

/// identify: read the target's SDID and list every part that matches. SDIDs are
/// often shared across flash-size variants (js8/js16, qe32/64/128, g192/g240),
/// so programming still needs an explicit --part to pin the exact geometry.
fn cmdIdentify(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try bringUp(bdm, flags);
    const fam = familyOfTarget(flags.target);
    const addr = lib_device.familySdidAddr(fam);
    const sdid = readSdid(bdm, fam) orelse {
        try out.print("could not read the SDID register at 0x{X}\n", .{addr});
        return;
    };
    try out.print("SDID 0x{X:0>4} @ 0x{X}\n", .{ sdid, addr });
    var n: usize = 0;
    for (lib_device.sdid_table) |e| {
        if (e.family != fam or sdid & e.mask != e.sdid) continue;
        const geo = if (lib_device.lookup(e.name) != null) "" else "  (no built-in geometry)";
        try out.print("  {s}{s}{s}{s}\n", .{ st.bold, e.name, st.reset, geo });
        n += 1;
    }
    if (n == 0) {
        try out.print("unrecognized device id (pass --part or --flash-base)\n", .{});
    } else if (n > 1) {
        try out.print("{s}SDID shared by {d} parts (flash-size variants); pass --part to program{s}\n", .{ st.dim, n, st.reset });
    }
}

fn cmdReset(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    var opts: session.Options = .{};
    if (flags.vdd) |v| {
        opts.target_vdd = v;
        opts.leave_target_powered = true;
    }
    try bdm.sess.setOptions(opts);
    try bdm.sess.setTarget(flags.target);
    try bdm.sess.reset(flags.reset_mode, flags.reset_method);
    try out.print("{s}reset{s} ({t} mode, {t} method)\n", .{
        st.green, st.reset, flags.reset_mode, flags.reset_method,
    });
    if (flags.reset_mode == .special) {
        // Reset invalidates measured speed; reconnect to leave target debuggable.
        try bdm.sess.connect();
        try printBdmStatus(bdm, out, st);
    }
}

fn cmdHalt(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try bringUp(bdm, flags);
    try bdm.sess.halt();
    try out.print("{s}halted{s}\n", .{ st.yellow, st.reset });
    printPcIfKnown(bdm, flags, out, st) catch {};
}

fn cmdGo(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try bringUp(bdm, flags);
    try bdm.sess.go();
    try out.print("{s}running{s}\n", .{ st.green, st.reset });
}

fn cmdStep(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try bringUp(bdm, flags);
    try bdm.sess.step();
    try out.print("{s}stepped{s}\n", .{ st.green, st.reset });
    printPcIfKnown(bdm, flags, out, st) catch {};
}

fn printPcIfKnown(bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    const pc: u32 = switch (flags.target) {
        .hcs08 => try bdm.sess.readReg(@intFromEnum(target.Hcs08Reg.pc)),
        .hcs12 => try bdm.sess.readReg(@intFromEnum(target.Hcs12Reg.pc)),
        .cfv1 => try bdm.sess.readCReg(@intFromEnum(target.Cfv1CReg.pc)),
        else => return,
    };
    try out.print("  {s}pc{s} = {s}0x{x:0>4}{s}\n", .{ st.dim, st.reset, st.yellow, pc, st.reset });
}

fn printRegRow(out: *Io.Writer, st: tty.Style, name: []const u8, value: u32, bits: u8) !void {
    switch (bits) {
        8 => try out.print("  {s}{s: <7}{s} {s}0x{x:0>2}{s}\n", .{ st.dim, name, st.reset, st.yellow, @as(u8, @truncate(value)), st.reset }),
        16 => try out.print("  {s}{s: <7}{s} {s}0x{x:0>4}{s}\n", .{ st.dim, name, st.reset, st.yellow, @as(u16, @truncate(value)), st.reset }),
        else => try out.print("  {s}{s: <7}{s} {s}0x{x:0>8}{s}\n", .{ st.dim, name, st.reset, st.yellow, value, st.reset }),
    }
}

fn cmdRegs(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    try bringUp(bdm, flags);
    const s = &bdm.sess;
    if (flags.json) return regsJson(s, flags.target, out);
    switch (flags.target) {
        .hcs08 => {
            try printRegRow(out, st, "pc", try s.readReg(@intFromEnum(target.Hcs08Reg.pc)), 16);
            try printRegRow(out, st, "sp", try s.readReg(@intFromEnum(target.Hcs08Reg.sp)), 16);
            try printRegRow(out, st, "hx", try s.readReg(@intFromEnum(target.Hcs08Reg.hx)), 16);
            try printRegRow(out, st, "a", try s.readReg(@intFromEnum(target.Hcs08Reg.a)), 8);
            try printRegRow(out, st, "ccr", try s.readReg(@intFromEnum(target.Hcs08Reg.ccr)), 8);
            try printRegRow(out, st, "bdcscr", try s.readStatusReg(), 8);
        },
        .rs08 => {
            try printRegRow(out, st, "ccr_pc", try s.readReg(@intFromEnum(target.Rs08Reg.ccr_pc)), 16);
            try printRegRow(out, st, "spc", try s.readReg(@intFromEnum(target.Rs08Reg.spc)), 16);
            try printRegRow(out, st, "a", try s.readReg(@intFromEnum(target.Rs08Reg.a)), 8);
            try printRegRow(out, st, "bdcscr", try s.readStatusReg(), 8);
        },
        .hcs12 => {
            try printRegRow(out, st, "pc", try s.readReg(@intFromEnum(target.Hcs12Reg.pc)), 16);
            try printRegRow(out, st, "sp", try s.readReg(@intFromEnum(target.Hcs12Reg.sp)), 16);
            try printRegRow(out, st, "x", try s.readReg(@intFromEnum(target.Hcs12Reg.x)), 16);
            try printRegRow(out, st, "y", try s.readReg(@intFromEnum(target.Hcs12Reg.y)), 16);
            try printRegRow(out, st, "d", try s.readReg(@intFromEnum(target.Hcs12Reg.d)), 16);
            try printRegRow(out, st, "ccr", try s.readDReg(target.hcs12_dreg_ccr), 8);
        },
        .cfv1 => {
            inline for (0..8) |i| {
                var nbuf: [4]u8 = undefined;
                const n = std.fmt.bufPrint(&nbuf, "d{d}", .{i}) catch unreachable;
                try printRegRow(out, st, n, try s.readReg(@intCast(i)), 32);
            }
            inline for (0..8) |i| {
                var nbuf: [4]u8 = undefined;
                const n = std.fmt.bufPrint(&nbuf, "a{d}", .{i}) catch unreachable;
                try printRegRow(out, st, n, try s.readReg(@intCast(8 + i)), 32);
            }
            try printRegRow(out, st, "pc", try s.readCReg(@intFromEnum(target.Cfv1CReg.pc)), 32);
            try printRegRow(out, st, "sr", try s.readCReg(@intFromEnum(target.Cfv1CReg.sr)), 32);
            try printRegRow(out, st, "vbr", try s.readCReg(@intFromEnum(target.Cfv1CReg.vbr)), 32);
        },
        else => fatal("'regs' not implemented for target {s}", .{flags.target.name()}),
    }
}

/// Core registers as flat JSON {name: value}; mirrors cmdRegs.
fn regsJson(s: *session.Session, t: target.TargetType, out: *Io.Writer) !void {
    try out.writeAll("{");
    var first = true;
    const P = struct {
        fn kv(o: *Io.Writer, f: *bool, name: []const u8, v: u32) !void {
            if (!f.*) try o.writeAll(",");
            f.* = false;
            try o.print("\"{s}\":{d}", .{ name, v });
        }
    };
    switch (t) {
        .hcs08 => {
            try P.kv(out, &first, "pc", try s.readReg(@intFromEnum(target.Hcs08Reg.pc)));
            try P.kv(out, &first, "sp", try s.readReg(@intFromEnum(target.Hcs08Reg.sp)));
            try P.kv(out, &first, "hx", try s.readReg(@intFromEnum(target.Hcs08Reg.hx)));
            try P.kv(out, &first, "a", try s.readReg(@intFromEnum(target.Hcs08Reg.a)));
            try P.kv(out, &first, "ccr", try s.readReg(@intFromEnum(target.Hcs08Reg.ccr)));
        },
        .rs08 => {
            try P.kv(out, &first, "ccr_pc", try s.readReg(@intFromEnum(target.Rs08Reg.ccr_pc)));
            try P.kv(out, &first, "spc", try s.readReg(@intFromEnum(target.Rs08Reg.spc)));
            try P.kv(out, &first, "a", try s.readReg(@intFromEnum(target.Rs08Reg.a)));
        },
        .hcs12 => {
            try P.kv(out, &first, "pc", try s.readReg(@intFromEnum(target.Hcs12Reg.pc)));
            try P.kv(out, &first, "sp", try s.readReg(@intFromEnum(target.Hcs12Reg.sp)));
            try P.kv(out, &first, "x", try s.readReg(@intFromEnum(target.Hcs12Reg.x)));
            try P.kv(out, &first, "y", try s.readReg(@intFromEnum(target.Hcs12Reg.y)));
            try P.kv(out, &first, "d", try s.readReg(@intFromEnum(target.Hcs12Reg.d)));
            try P.kv(out, &first, "ccr", try s.readDReg(target.hcs12_dreg_ccr));
        },
        .cfv1 => {
            var nbuf: [4]u8 = undefined;
            for (0..8) |i| try P.kv(out, &first, std.fmt.bufPrint(&nbuf, "d{d}", .{i}) catch unreachable, try s.readReg(@intCast(i)));
            for (0..8) |i| try P.kv(out, &first, std.fmt.bufPrint(&nbuf, "a{d}", .{i}) catch unreachable, try s.readReg(@intCast(8 + i)));
            try P.kv(out, &first, "pc", try s.readCReg(@intFromEnum(target.Cfv1CReg.pc)));
            try P.kv(out, &first, "sr", try s.readCReg(@intFromEnum(target.Cfv1CReg.sr)));
            try P.kv(out, &first, "vbr", try s.readCReg(@intFromEnum(target.Cfv1CReg.vbr)));
        },
        else => fatalCode(.usage, "'regs' not implemented for target {s}", .{t.name()}),
    }
    try out.writeAll("}\n");
}

const RegKind = enum { core, creg, dreg, status };
const RegSpec = struct { kind: RegKind, no: u16, bits: u8 };

fn lookupReg(t: target.TargetType, name: []const u8) ?RegSpec {
    const H = struct {
        fn spec(kind: RegKind, no: u16, bits: u8) RegSpec {
            return .{ .kind = kind, .no = no, .bits = bits };
        }
    };
    switch (t) {
        .hcs08 => {
            if (eq(name, "pc")) return H.spec(.core, @intFromEnum(target.Hcs08Reg.pc), 16);
            if (eq(name, "sp")) return H.spec(.core, @intFromEnum(target.Hcs08Reg.sp), 16);
            if (eq(name, "hx")) return H.spec(.core, @intFromEnum(target.Hcs08Reg.hx), 16);
            if (eq(name, "a")) return H.spec(.core, @intFromEnum(target.Hcs08Reg.a), 8);
            if (eq(name, "ccr")) return H.spec(.core, @intFromEnum(target.Hcs08Reg.ccr), 8);
            if (eq(name, "bkpt")) return H.spec(.dreg, target.hcs08_dreg_bkpt, 16);
            if (eq(name, "bdcscr")) return H.spec(.status, 0, 8);
        },
        .rs08 => {
            // RS08: combined CCR/PC + shadow PC + A. pc=CCR/PC; spc/sp=shadow PC.
            if (eq(name, "pc") or eq(name, "ccr_pc")) return H.spec(.core, @intFromEnum(target.Rs08Reg.ccr_pc), 16);
            if (eq(name, "spc") or eq(name, "sp")) return H.spec(.core, @intFromEnum(target.Rs08Reg.spc), 16);
            if (eq(name, "a")) return H.spec(.core, @intFromEnum(target.Rs08Reg.a), 8);
            if (eq(name, "bkpt")) return H.spec(.dreg, target.hcs08_dreg_bkpt, 16);
            if (eq(name, "bdcscr")) return H.spec(.status, 0, 8);
        },
        .hcs12 => {
            if (eq(name, "pc")) return H.spec(.core, @intFromEnum(target.Hcs12Reg.pc), 16);
            if (eq(name, "sp")) return H.spec(.core, @intFromEnum(target.Hcs12Reg.sp), 16);
            if (eq(name, "x")) return H.spec(.core, @intFromEnum(target.Hcs12Reg.x), 16);
            if (eq(name, "y")) return H.spec(.core, @intFromEnum(target.Hcs12Reg.y), 16);
            if (eq(name, "d")) return H.spec(.core, @intFromEnum(target.Hcs12Reg.d), 16);
            if (eq(name, "ccr")) return H.spec(.dreg, target.hcs12_dreg_ccr, 8);
            if (eq(name, "bdmsts")) return H.spec(.dreg, target.hcs12_dreg_bdmsts, 8);
        },
        .cfv1 => {
            if (name.len == 2 and (name[0] == 'd' or name[0] == 'a')) {
                const idx = std.fmt.charToDigit(name[1], 10) catch return null;
                if (idx > 7) return null;
                const base: u16 = if (name[0] == 'a') 8 else 0;
                return H.spec(.core, base + idx, 32);
            }
            if (eq(name, "pc")) return H.spec(.creg, @intFromEnum(target.Cfv1CReg.pc), 32);
            if (eq(name, "sr")) return H.spec(.creg, @intFromEnum(target.Cfv1CReg.sr), 32);
            if (eq(name, "vbr")) return H.spec(.creg, @intFromEnum(target.Cfv1CReg.vbr), 32);
            if (eq(name, "cpucr")) return H.spec(.creg, @intFromEnum(target.Cfv1CReg.cpucr), 32);
            if (eq(name, "csr")) return H.spec(.dreg, @intFromEnum(target.Cfv1DReg.csr), 32);
            if (eq(name, "xcsr")) return H.spec(.dreg, @intFromEnum(target.Cfv1DReg.xcsr), 32);
        },
        else => {},
    }
    return null;
}

fn cmdReg(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    if (flags.positionals.len < 2) fatalCode(.usage, "usage: usbdm reg <name> [value]", .{});
    const name = flags.positionals[1];
    const spec = lookupReg(flags.target, name) orelse
        fatalCode(.usage, "unknown register '{s}' for target {s}", .{ name, flags.target.name() });

    try bringUp(bdm, flags);
    const s = &bdm.sess;

    if (flags.positionals.len >= 3) {
        const value = parseInt(u32, flags.positionals[2], "register value");
        switch (spec.kind) {
            .core => try s.writeReg(spec.no, value),
            .creg => try s.writeCReg(spec.no, value),
            .dreg => try s.writeDReg(spec.no, value),
            .status => try s.writeControlReg(value),
        }
        try out.print("{s}{s}{s} <- ", .{ st.dim, name, st.reset });
        try printRegRow(out, st, name, value, spec.bits);
    } else {
        const value = switch (spec.kind) {
            .core => try s.readReg(spec.no),
            .creg => try s.readCReg(spec.no),
            .dreg => try s.readDReg(spec.no),
            .status => try s.readStatusReg(),
        };
        try printRegRow(out, st, name, value, spec.bits);
    }
}

fn cmdRead(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    if (flags.positionals.len < 3) fatalCode(.usage, "usage: usbdm read <addr> <len>", .{});
    const addr = parseInt(u32, flags.positionals[1], "address");
    const len = parseInt(u32, flags.positionals[2], "length");
    if (len == 0) return;

    const buf = try arena.alloc(u8, len);
    try bringUp(bdm, flags);
    try bdm.sess.readMem(flags.elem, addr, buf);
    if (flags.json) {
        try out.print("{{\"addr\":{d},\"len\":{d},\"data\":\"", .{ addr, buf.len });
        for (buf) |b| try out.print("{x:0>2}", .{b});
        try out.writeAll("\"}\n");
        return;
    }
    try hexdump.dump(out, st, addr, buf);
}

fn cmdFill(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    if (flags.positionals.len < 4) fatalCode(.usage, "usage: usbdm fill <addr> <len> <byte>", .{});
    const addr = parseInt(u32, flags.positionals[1], "address");
    const len = parseInt(u32, flags.positionals[2], "length");
    const byte = parseInt(u8, flags.positionals[3], "fill byte");
    if (len == 0) return;
    const data = try arena.alloc(u8, len);
    @memset(data, byte);
    try bringUp(bdm, flags);
    try bdm.sess.writeMem(flags.elem, addr, data);
    try out.print("{s}filled {d} bytes at 0x{x:0>4} with 0x{x:0>2}{s}\n", .{ st.green, len, addr, byte, st.reset });
}

fn cmdWrite(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    if (flags.positionals.len < 3) fatalCode(.usage, "usage: usbdm write <addr> <byte...>", .{});
    const addr = parseInt(u32, flags.positionals[1], "address");
    const byte_args = flags.positionals[2..];
    const data = try arena.alloc(u8, byte_args.len);
    for (byte_args, data) |s, *d| {
        d.* = std.fmt.parseInt(u8, s, 16) catch
            std.fmt.parseInt(u8, s, 0) catch fatalCode(.usage, "invalid byte '{s}'", .{s});
    }
    try bringUp(bdm, flags);
    try bdm.sess.writeMem(flags.elem, addr, data);
    try out.print("{s}wrote {d} bytes at 0x{x:0>4}{s}\n", .{ st.green, data.len, addr, st.reset });
}

fn cmdDump(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    if (flags.positionals.len < 3) fatalCode(.usage, "usage: usbdm dump <addr> <len> -o <file>", .{});
    const addr = parseInt(u32, flags.positionals[1], "address");
    const len = parseInt(u32, flags.positionals[2], "length");
    const path = flags.out_path orelse fatal("dump requires -o <file>", .{});

    // The overwrite guard runs pre-open in runDeviceCommand.
    const format: hexfile.Format = flags.format orelse detectByExt(path);
    const buf = try arena.alloc(u8, len);
    try bringUp(bdm, flags);
    try bdm.sess.readMem(flags.elem, addr, buf);

    var file_bytes: []const u8 = undefined;
    switch (format) {
        .binary => file_bytes = buf,
        .srec, .ihex => {
            var segments = [_]hexfile.Segment{.{ .addr = addr, .data = buf }};
            const image: hexfile.Image = .{ .segments = &segments };
            var aw: Io.Writer.Allocating = .init(arena);
            try hexfile.emit(&aw.writer, image, format);
            file_bytes = aw.written();
        },
    }
    try Io.Dir.cwd().writeFile(global_io, .{ .sub_path = path, .data = file_bytes });
    try out.print("{s}dumped {d} bytes at 0x{x:0>4} to {s} ({t}){s}\n", .{
        st.green, len, addr, path, format, st.reset,
    });
}

fn detectByExt(path: []const u8) hexfile.Format {
    const ext = std.fs.path.extension(path);
    if (eq(ext, ".s19") or eq(ext, ".srec") or eq(ext, ".s28") or eq(ext, ".s37")) return .srec;
    if (eq(ext, ".hex") or eq(ext, ".ihex")) return .ihex;
    return .binary;
}

fn cmdLoad(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    if (flags.positionals.len < 2) fatalCode(.usage, "usage: usbdm load <file> [--addr <a>]", .{});
    const path = flags.positionals[1];
    const bytes = Io.Dir.cwd().readFileAlloc(global_io, path, arena, .limited(64 * 1024 * 1024)) catch |err|
        fatalCode(.io_error, "cannot read '{s}': {s}", .{ path, @errorName(err) });

    var image = hexfile.parse(arena, bytes) catch |err|
        fatalCode(.io_error, "cannot parse '{s}': {s}", .{ path, @errorName(err) });
    if (flags.load_addr) |a| {
        if (image.segments.len == 1) {
            image.segments[0].addr = a;
        } else fatal("--addr only applies to single-segment images", .{});
    }

    try bringUp(bdm, flags);
    for (image.segments) |seg| {
        try bdm.sess.writeMem(flags.elem, seg.addr, seg.data);
        try out.print("{s}wrote {d} bytes at 0x{x:0>4}{s}\n", .{ st.green, seg.data.len, seg.addr, st.reset });
    }
    try out.print("{s}note: load writes memory over BDM; it does not program flash{s}\n", .{ st.dim, st.reset });
}

fn cmdVdd(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    if (flags.positionals.len < 2) fatalCode(.usage, "usage: usbdm vdd off|3v3|5v", .{});
    const v = flags.positionals[1];
    const sel: target.VddSelect = if (eq(v, "off"))
        .off
    else if (eq(v, "3v3"))
        .v3_3
    else if (eq(v, "5v"))
        .v5
    else
        fatalCode(.usage, "invalid vdd '{s}' (off, 3v3, 5v)", .{v});
    try bdm.sess.setVdd(sel);
    try out.print("{s}target Vdd: {s}{s}\n", .{ st.green, v, st.reset });
}

fn cmdPins(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    _ = arena;
    var control: u16 = 0;
    if (flags.positionals.len >= 2 and eq(flags.positionals[1], "release")) {
        control = target.pin_release;
    } else {
        for (flags.positionals[1..]) |spec| {
            const eq_pos = std.mem.indexOfScalar(u8, spec, '=') orelse
                fatal("pin spec '{s}' should be signal=level (e.g. bkgd=low)", .{spec});
            const sig = spec[0..eq_pos];
            const lvl_s = spec[eq_pos + 1 ..];
            const lvl: target.PinLevel = if (eq(lvl_s, "low"))
                .low
            else if (eq(lvl_s, "high"))
                .high
            else if (eq(lvl_s, "3state") or eq(lvl_s, "3s") or eq(lvl_s, "z"))
                .tristate
            else
                fatalCode(.usage, "invalid level '{s}' (low, high, 3state)", .{lvl_s});
            if (eq(sig, "bkgd")) {
                control |= @as(u16, @intFromEnum(lvl));
            } else if (eq(sig, "reset")) {
                control |= @as(u16, @intFromEnum(lvl)) << 2;
            } else fatalCode(.usage, "unknown signal '{s}' (bkgd, reset)", .{sig});
        }
    }
    try bdm.sess.setTarget(flags.target);
    const status = try bdm.sess.controlPins(control);
    try out.print("{s}pin state{s} {s}0x{x:0>4}{s}\n", .{ st.dim, st.reset, st.yellow, status, st.reset });
}

const ramflash = lib.ramflash;
const lib_device = lib.device;

// Vendored HCS08 small-code routines (embedded). 0xB0 variant: RAM above 0x80.
const flash_blob_80 = @embedFile("flash_hcs08_0x80");
const flash_blob_b0 = @embedFile("flash_hcs08_0xB0");
// Vendored HCS12 large-code routines. FTS = S12 MMCV4; GMMC = S12G FTMRG.
const flash_blob_hcs12_fts = @embedFile("flash_hcs12_fts");
const flash_blob_hcs12_gmmc = @embedFile("flash_hcs12_gmmc");
// Vendored CFV1 (ColdFire V1) large-code routine (32-bit ABI, relocatable-off).
const flash_blob_cfv1 = @embedFile("flash_cfv1");

fn requireFlashTarget(flags: *const Flags) void {
    switch (flags.target) {
        .hcs08, .rs08, .hcs12, .hcs12z, .cfv1 => {},
        else => fatalCode(.usage, "flash supports HCS08/RS08 (small-code), HCS12 and CFV1 (large-code) RAM routines (got {s})", .{flags.target.name()}),
    }
}

/// CLI target → flash device family (rs08→hcs08, hcs12z→hcs12).
fn familyOfTarget(t: target.TargetType) lib_device.Family {
    return switch (t) {
        .hcs12, .hcs12z => .hcs12,
        .cfv1 => .cfv1,
        else => .hcs08, // hcs08, rs08
    };
}

fn resolveDevice(flags: *const Flags) lib_device.Device {
    var d = if (flags.part) |name| blk: {
        const found = lib_device.lookup(name) orelse
            fatalCode(.usage, "unknown --part '{s}' (see 'usbdm parts', or use --flash-base)", .{name});
        // Guard --part family vs --target: routine comes from the part's family
        // but connect/PC use --target, so a mismatch would silently break.
        if (found.family != familyOfTarget(flags.target)) {
            fatalCode(.usage, "--part '{s}' is a {s} device but --target is {s}; pass --target {s}", .{
                name, @tagName(found.family), flags.target.name(), @tagName(found.family),
            });
        }
        break :blk found;
    } else lib_device.genericFor(familyOfTarget(flags.target));
    if (flags.flash_base) |b| d.flash_base = b;
    return d;
}

/// Session-backed RAM flash target. pc_creg: CFV1's PC is a control register;
/// mem_space: byte for HCS08/HCS12, word for CFV1.
const RamTarget = struct {
    sess: *session.Session,
    pc_reg: u8 = @intFromEnum(target.Hcs08Reg.pc),
    pc_creg: bool = false,
    mem_space: target.MemSpace = .byte,
    fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return s.sess.writeMem(s.mem_space, addr, data);
    }
    fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return s.sess.readMem(s.mem_space, addr, buf);
    }
    fn writePc(ctx: *anyopaque, pc: u32) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return if (s.pc_creg) s.sess.writeCReg(s.pc_reg, pc) else s.sess.writeReg(s.pc_reg, pc);
    }
    fn go(ctx: *anyopaque) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return s.sess.go();
    }
    fn halt(ctx: *anyopaque) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return s.sess.halt();
    }
    fn readStatus(ctx: *anyopaque) anyerror!u8 {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return @truncate(try s.sess.readStatusReg());
    }
    extern "c" fn usleep(us: c_uint) c_int;
    fn sleepMs(ctx: *anyopaque, ms: u32) void {
        _ = ctx;
        _ = usleep(ms * 1000);
    }
    fn reconnect(ctx: *anyopaque) anyerror!void {
        const s: *RamTarget = @ptrCast(@alignCast(ctx));
        return s.sess.connect();
    }
    const vtable = ramflash.Target.VTable{
        .writeMem = writeMem,
        .readMem = readMem,
        .writePc = writePc,
        .go = go,
        .halt = halt,
        .readStatus = readStatus,
        .sleepMs = sleepMs,
        .reconnect = reconnect,
    };
    fn iface(self: *RamTarget) ramflash.Target {
        return .{ .ctx = self, .vtable = &vtable };
    }
};

/// Flash-driver Target on a CPU core (test-only; --sim serves the CLI): mem hits
/// emulator memory, `go` runs to BGND. halted_status: BDMACT (HCS08/HCS12) or
/// XCSR RUNSTATE (CFV1).
fn EmuTargetOf(comptime Cpu: type, comptime halted_status: u8) type {
    return struct {
        cpu: *Cpu,
        const Self = @This();
        fn cpuOf(ctx: *anyopaque) *Cpu {
            return @as(*Self, @ptrCast(@alignCast(ctx))).cpu;
        }
        fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
            cpuOf(ctx).hostWrite(@intCast(addr), data);
        }
        fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!void {
            cpuOf(ctx).hostRead(@intCast(addr), buf);
        }
        fn writePc(ctx: *anyopaque, pc: u32) anyerror!void {
            cpuOf(ctx).pc = @intCast(pc);
        }
        fn go(ctx: *anyopaque) anyerror!void {
            try cpuOf(ctx).run(8_000_000);
        }
        fn halt(_: *anyopaque) anyerror!void {}
        fn readStatus(ctx: *anyopaque) anyerror!u8 {
            return if (cpuOf(ctx).halted) halted_status else 0;
        }
        fn sleepMs(_: *anyopaque, _: u32) void {}
        fn reconnect(_: *anyopaque) anyerror!void {}
        const vtable = ramflash.Target.VTable{
            .writeMem = writeMem,
            .readMem = readMem,
            .writePc = writePc,
            .go = go,
            .halt = halt,
            .readStatus = readStatus,
            .sleepMs = sleepMs,
            .reconnect = reconnect,
        };
        fn iface(self: *Self) ramflash.Target {
            return .{ .ctx = self, .vtable = &vtable };
        }
    };
}
const EmuTarget = EmuTargetOf(lib.cpu08.Cpu, lib.cpu08.BDMACT);
const Cfv1EmuTarget = EmuTargetOf(lib.cfv1_emu.Cpu, lib.cfv1_emu.XCSR_RUNSTATE);
const Cpu12EmuTarget = EmuTargetOf(lib.cpu12.Cpu, lib.cpu12.BDMACT);

fn mapRoutineError(last: ramflash.DriverError, err: anyerror) noreturn {
    switch (err) {
        error.RoutineError => {
            // verify_mismatch is its own exit class: readback differed vs other failures.
            const code: ExitCode = if (last == .verify_failed) .verify_mismatch else .flash_error;
            fatalCode(code, "flash routine reported: {s}", .{last.name()});
        },
        error.Timeout => fatalCode(.bdm_error, "flash routine timed out (target never halted)", .{}),
        error.NoRamForData => fatalCode(.flash_error, "not enough target RAM for the flash-routine data buffer", .{}),
        error.NotSmallCode, error.BadImage => fatalCode(.flash_error, "flash-routine image is malformed", .{}),
        else => fatalCode(.flash_error, "flash operation failed: {s}", .{@errorName(err)}),
    }
}

/// One interface over the small/large/CFV1 flash drivers.
const AnyDriver = union(enum) {
    small: ramflash.Driver,
    large: ramflash.LargeDriver,
    cfv1: ramflash.Cfv1Driver,

    fn lastError(self: *const AnyDriver) ramflash.DriverError {
        return switch (self.*) {
            inline else => |*d| d.last_error,
        };
    }
    fn program(self: *AnyDriver, addr: u32, data: []const u8) ramflash.Error!void {
        return switch (self.*) {
            inline else => |*d| d.program(addr, data),
        };
    }
    fn verify(self: *AnyDriver, addr: u32, data: []const u8) ramflash.Error!void {
        return switch (self.*) {
            inline else => |*d| d.verify(addr, data),
        };
    }
    fn eraseRange(self: *AnyDriver, addr: u32, size: u32) ramflash.Error!void {
        return switch (self.*) {
            inline else => |*d| d.eraseRange(addr, size),
        };
    }
    /// `flash_addr` seeds the large-code block-erase address; small code ignores it.
    fn massErase(self: *AnyDriver, flash_addr: u32) ramflash.Error!void {
        return switch (self.*) {
            inline else => |*d| d.massErase(flash_addr),
        };
    }
    fn blankCheck(self: *AnyDriver, addr: u32, size: u32) ramflash.Error!void {
        return switch (self.*) {
            inline else => |*d| d.blankCheck(addr, size),
        };
    }
};

const FlashPrep = struct {
    routine: union(enum) { small: ramflash.Routine, large: ramflash.LargeRoutine, cfv1: ramflash.Cfv1Routine },
    params: ramflash.Params,
    pc_reg: u8,
    pc_creg: bool = false,
    mem_space: target.MemSpace = .byte,

    /// Build the concrete driver bound to `iface` (kept on the caller's stack).
    fn driver(self: FlashPrep, iface: ramflash.Target) AnyDriver {
        return switch (self.routine) {
            .small => |r| .{ .small = .{ .target = iface, .routine = r, .params = self.params } },
            .large => |r| .{ .large = .{ .target = iface, .routine = r, .params = self.params } },
            .cfv1 => |r| .{ .cfv1 = .{ .target = iface, .routine = r, .params = self.params } },
        };
    }
    fn ramTarget(self: FlashPrep, sess: *session.Session) RamTarget {
        return .{ .sess = sess, .pc_reg = self.pc_reg, .pc_creg = self.pc_creg, .mem_space = self.mem_space };
    }
};

/// Connect, measure bus clock, parse the routine blob. Caller builds the
/// driver locally (target-pointer lifetime).
fn prepareFlash(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) FlashPrep {
    const d = resolveDevice(flags);
    // HCS08/HCS12 controller base is a u16; CFV1 is u32 (sits high at 0xFF9820).
    if (d.family != .cfv1 and d.flash_base > 0xFFFF) {
        fatal("--part '{s}' has a flash controller base 0x{x} that doesn't fit the {s} routine's 16-bit field", .{ d.name, d.flash_base, @tagName(d.family) });
    }
    bringUp(bdm, flags) catch |err| fatal("connect failed: {s}", .{@errorName(err)});
    const bus_hz = flags.bus_hz orelse blk: {
        // sync==0: no measurement; sync==1: firmware "unknown" sentinel (syncToHz
        // clamps to ~4.3 GHz). Treat both as unknown, demand --bus-hz.
        const sync = bdm.sess.getSpeedSync() catch 0;
        const hz = if (sync > 1) session.syncToHz(sync) else 0;
        if (hz == 0) fatal("cannot determine target bus clock - pass --bus-hz <hz>", .{});
        out.print("{s}using measured bus clock ~{d} kHz (override with --bus-hz){s}\n", .{ st.dim, hz / 1000, st.reset }) catch {};
        break :blk hz;
    };
    return flashPrepFor(arena, d, bus_hz);
}

/// FlashPrep for a device at a known bus clock, no I/O (shared by prepareFlash + gdb).
fn flashPrepFor(arena: std.mem.Allocator, d: lib_device.Device, bus_hz: u32) FlashPrep {
    // frequency_khz is u16 (max ~65.5 MHz); reject rather than overflow the cast.
    if (bus_hz / 1000 > std.math.maxInt(u16)) {
        fatal("target bus clock {d} Hz is out of range (max ~65.5 MHz); check --bus-hz", .{bus_hz});
    }
    const params = ramflash.Params{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = @intCast(bus_hz / 1000),
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    };
    switch (d.family) {
        .hcs08 => {
            const blob = if (d.routineLoadAddress() == 0xB0) flash_blob_b0 else flash_blob_80;
            const parsed = ramflash.parseSmall(arena, blob) catch |err|
                fatal("cannot load flash routine: {s}", .{@errorName(err)});
            return .{ .routine = .{ .small = parsed.routine }, .params = params, .pc_reg = @intFromEnum(target.Hcs08Reg.pc) };
        },
        .hcs12 => {
            const blob = switch (d.hcs12_routine) {
                .mmcv4_fts => flash_blob_hcs12_fts,
                .gmmc_ftmrg => flash_blob_hcs12_gmmc,
            };
            const routine = ramflash.parseLarge(arena, blob) catch |err|
                fatal("cannot load HCS12 flash routine: {s}", .{@errorName(err)});
            return .{ .routine = .{ .large = routine }, .params = params, .pc_reg = @intFromEnum(target.Hcs12Reg.pc) };
        },
        .cfv1 => {
            const routine = ramflash.parseCfv1(arena, flash_blob_cfv1) catch |err|
                fatal("cannot load CFV1 flash routine: {s}", .{@errorName(err)});
            // CFV1: PC is control register 15, header/data via 16-bit access.
            return .{
                .routine = .{ .cfv1 = routine },
                .params = params,
                .pc_reg = @intFromEnum(target.Cfv1CReg.pc),
                .pc_creg = true,
                .mem_space = .word,
            };
        },
    }
}

/// GDB memory-map XML (RAM + flash blocksize) so `load` uses vFlash*.
fn memoryMapXml(arena: std.mem.Allocator, d: lib_device.Device) ![]const u8 {
    var aw: Io.Writer.Allocating = .init(arena);
    const w = &aw.writer;
    try w.writeAll(
        \\<?xml version="1.0"?>
        \\<!DOCTYPE memory-map PUBLIC "+//IDN gnu.org//DTD GDB Memory Map V1.0//EN" "http://sourceware.org/gdb/gdb-memory-map.dtd">
        \\<memory-map>
        \\
    );
    const ram_len = d.ram_end - d.ram_start + 1;
    try w.print("  <memory type=\"ram\" start=\"0x{x}\" length=\"0x{x}\"/>\n", .{ d.ram_start, ram_len });
    var range_buf: [258]lib_device.Device.Range = undefined;
    for (d.flashRanges(&range_buf)) |rng| {
        try w.print("  <memory type=\"flash\" start=\"0x{x}\" length=\"0x{x}\"><property name=\"blocksize\">0x{x}</property></memory>\n", .{ rng.start, rng.size, d.page_size });
    }
    try w.writeAll("</memory-map>");
    return aw.written();
}

fn cmdProgram(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    requireFlashTarget(flags);
    const path = flags.positionals[1];
    const bytes = Io.Dir.cwd().readFileAlloc(global_io, path, arena, .limited(16 * 1024 * 1024)) catch |err|
        fatalCode(.io_error, "cannot read '{s}': {s}", .{ path, @errorName(err) });
    const image = hexfile.parse(arena, bytes) catch |err|
        fatalCode(.io_error, "cannot parse '{s}': {s}", .{ path, @errorName(err) });

    const d = resolveDevice(flags);
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        const seg_end = seg.addr + @as(u32, @intCast(seg.data.len - 1));
        // Whole span must be in one range - endpoint checks would pass a segment
        // straddling the gap between ranges (e.g. I/O window on split-flash parts).
        if (!d.spanInFlash(seg.addr, seg_end)) {
            fatal("image segment 0x{x:0>4}..0x{x:0>4} is outside {s} flash (or straddles a gap between flash ranges)", .{ seg.addr, seg_end, d.name });
        }
    }
    if (image.totalBytes() == 0) fatal("'{s}' contains no data to program", .{path});
    if (!confirm(out, st, flags.force, "program {d} bytes to {s} flash (erases affected sectors)?", .{ image.totalBytes(), d.name }))
        fatal("aborted", .{});

    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());

    // Erase all sectors first, then program (a shared sector mustn't wipe a neighbour).
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        drv.eraseRange(seg.addr, @intCast(seg.data.len)) catch |err| mapRoutineError(drv.lastError(), err);
    }
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        drv.program(seg.addr, seg.data) catch |err| mapRoutineError(drv.lastError(), err);
        try out.print("{s}programmed {d} bytes at 0x{x:0>4}{s}\n", .{ st.green, seg.data.len, seg.addr, st.reset });
        if (!flags.no_verify) {
            drv.verify(seg.addr, seg.data) catch |err| mapRoutineError(drv.lastError(), err);
            try out.print("  {s}verified{s}\n", .{ st.dim, st.reset });
        }
    }
}

/// eeprom <file>: program EEPROM/data-flash; addresses are 0-based offsets.
/// S12G GMMC/FTMRG only - routine routes a bit-31 "linear" global address to
/// the D-flash commands. Erase/program/verify like `program`, EEPROM geometry.
fn cmdEeprom(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    const d = resolveDevice(flags);
    const ee = d.eeprom orelse
        fatalCode(.usage, "--part '{s}' has no EEPROM programming support (only S12G GMMC parts, e.g. mc9s12g240)", .{d.name});
    const path = flags.positionals[1];
    const bytes = Io.Dir.cwd().readFileAlloc(global_io, path, arena, .limited(1 * 1024 * 1024)) catch |err|
        fatalCode(.io_error, "cannot read '{s}': {s}", .{ path, @errorName(err) });
    const image = hexfile.parse(arena, bytes) catch |err|
        fatalCode(.io_error, "cannot parse '{s}': {s}", .{ path, @errorName(err) });

    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        const seg_end = seg.addr + @as(u32, @intCast(seg.data.len - 1));
        if (seg_end >= ee.size)
            fatalCode(.usage, "EEPROM segment 0x{x}..0x{x} exceeds the {d}-byte EEPROM (address as a 0-based offset)", .{ seg.addr, seg_end, ee.size });
    }
    if (image.totalBytes() == 0) fatalCode(.io_error, "'{s}' contains no data to program", .{path});
    if (!confirm(out, st, flags.force, "program {d} bytes to {s} EEPROM (erases affected sectors)?", .{ image.totalBytes(), d.name }))
        fatal("aborted", .{});

    // Flash routine/params, but EEPROM align+sector; bit-31 global addr → D-flash.
    var prep = prepareFlash(arena, bdm, flags, out, st);
    prep.params.alignment = ee.align_bytes;
    prep.params.sector_size = @intCast(ee.sector);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());

    const lin = struct {
        fn addr(e: lib_device.Eeprom, off: u32) u32 {
            return lib_device.eeprom_linear_flag | (e.base_global + off);
        }
    };
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        drv.eraseRange(lin.addr(ee, seg.addr), @intCast(seg.data.len)) catch |err| mapRoutineError(drv.lastError(), err);
    }
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        drv.program(lin.addr(ee, seg.addr), seg.data) catch |err| mapRoutineError(drv.lastError(), err);
        try out.print("{s}programmed {d} bytes at EEPROM+0x{x}{s}\n", .{ st.green, seg.data.len, seg.addr, st.reset });
        if (!flags.no_verify) {
            drv.verify(lin.addr(ee, seg.addr), seg.data) catch |err| mapRoutineError(drv.lastError(), err);
            try out.print("  {s}verified{s}\n", .{ st.dim, st.reset });
        }
    }
}

fn cmdVerify(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    requireFlashTarget(flags);
    const path = flags.positionals[1];
    const bytes = Io.Dir.cwd().readFileAlloc(global_io, path, arena, .limited(16 * 1024 * 1024)) catch |err|
        fatalCode(.io_error, "cannot read '{s}': {s}", .{ path, @errorName(err) });
    const image = hexfile.parse(arena, bytes) catch |err|
        fatalCode(.io_error, "cannot parse '{s}': {s}", .{ path, @errorName(err) });

    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());
    for (image.segments) |seg| {
        drv.verify(seg.addr, seg.data) catch |err| mapRoutineError(drv.lastError(), err);
        try out.print("{s}verified {d} bytes at 0x{x:0>4}{s}\n", .{ st.green, seg.data.len, seg.addr, st.reset });
    }
}

fn cmdErase(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    requireFlashTarget(flags);
    const d = resolveDevice(flags);
    // Align down to the sector boundary: an unaligned addr would wipe two sectors.
    const page_raw: ?u32 = if (flags.positionals.len >= 2) parseInt(u32, flags.positionals[1], "page address") else null;
    const page: ?u32 = if (page_raw) |a| a & ~(d.page_size - 1) else null;

    if (page) |addr| {
        if (!confirm(out, st, flags.force, "erase the sector at 0x{x:0>4}?", .{addr})) fatal("aborted", .{});
    } else {
        if (!confirm(out, st, flags.force, "MASS ERASE all flash on {s}? this wipes the entire chip", .{d.name})) fatal("aborted", .{});
    }
    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());
    if (page) |addr| {
        if (page_raw.? != addr)
            try out.print("{s}note: 0x{x:0>4} rounded down to sector base 0x{x:0>4}{s}\n", .{ st.dim, page_raw.?, addr, st.reset });
        drv.eraseRange(addr, d.page_size) catch |err| mapRoutineError(drv.lastError(), err);
        try out.print("{s}erased sector at 0x{x:0>4}{s}\n", .{ st.green, addr, st.reset });
    } else {
        drv.massErase(d.flash_start) catch |err| mapRoutineError(drv.lastError(), err);
        try out.print("{s}mass erased{s}\n", .{ st.green, st.reset });
    }
}

fn cmdBlankCheck(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    requireFlashTarget(flags);
    const d = resolveDevice(flags);
    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());
    // 258 = up to 256 banked pages + two fixed windows.
    var range_buf: [258]lib_device.Device.Range = undefined;
    const ranges = d.flashRanges(&range_buf);
    for (ranges) |rng| {
        drv.blankCheck(rng.start, rng.size) catch |err| switch (err) {
            error.RoutineError => {
                try out.print("{s}not blank{s} - flash 0x{x:0>4}..0x{x:0>4} contains data (routine: {s})\n", .{ st.yellow, st.reset, rng.start, rng.start + rng.size - 1, drv.lastError().name() });
                try out.flush();
                std.process.exit(@intFromEnum(ExitCode.flash_error));
            },
            else => mapRoutineError(drv.lastError(), err),
        };
    }
    try out.print("{s}blank{s} - flash is fully erased\n", .{ st.green, st.reset });
}

fn cmdUnsecure(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    const d = resolveDevice(flags);
    const val = lib_device.unsecuredSecurityByte(d.family);
    if (!confirm(out, st, flags.force, "UNSECURE {s}? this mass-erases the entire chip", .{d.name})) fatal("aborted", .{});
    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());
    // Mass-erase leaves the security byte 0xFF (secured); program the unsecured
    // value (SEC=0b10) - its sector is now blank so program() can write it.
    drv.massErase(d.flash_start) catch |err| mapRoutineError(drv.lastError(), err);
    drv.program(d.security_addr, &.{val}) catch |err| mapRoutineError(drv.lastError(), err);
    try out.print("{s}unsecured{s} (mass-erased, security@0x{x:0>4} = 0x{x:0>2}); power-cycle the target\n", .{
        st.green, st.reset, d.security_addr, val,
    });
}

/// secure: program the security byte to SECURED (SEC=0b00) - locks debug access.
/// Erases its sector first (lives in flash).
fn cmdSecure(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    const d = resolveDevice(flags);
    const val = lib_device.securedSecurityByte(d.family);
    if (!confirm(out, st, flags.force, "SECURE {s}? this LOCKS debug access (erases the security sector); recovery needs 'unsecure'", .{d.name})) fatal("aborted", .{});
    const prep = prepareFlash(arena, bdm, flags, out, st);
    var rt = prep.ramTarget(&bdm.sess);
    var drv = prep.driver(rt.iface());
    // Erase the security byte's sector so it can be reprogrammed.
    const sector = d.security_addr & ~(d.page_size - 1);
    drv.eraseRange(sector, d.page_size) catch |err| mapRoutineError(drv.lastError(), err);
    drv.program(d.security_addr, &.{val}) catch |err| mapRoutineError(drv.lastError(), err);
    try out.print("{s}secured{s} (security@0x{x:0>4} = 0x{x:0>2}); power-cycle the target\n", .{
        st.yellow, st.reset, d.security_addr, val,
    });
}

/// `usbdm gdb` entry; net = POSIX sockets / Winsock2.
fn cmdGdb(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
    try Gdb.run(arena, bdm, flags, out, st);
}

const Gdb = struct {
    // Cross-platform TCP: gdb.zig/gdbstub.zig are socket-agnostic, so platform
    // difference is confined here. sockaddr_in is identical on-wire; only ABI differs.
    const is_windows = @import("builtin").os.tag == .windows;
    const RecvResult = union(enum) { got: usize, closed, again };

    const net = if (is_windows) struct {
        pub const Socket = usize; // SOCKET is UINT_PTR
        const INVALID_SOCKET: Socket = ~@as(Socket, 0);
        const AF_INET: c_int = 2;
        const SOCK_STREAM: c_int = 1;
        const SOL_SOCKET: c_int = 0xFFFF;
        const SO_REUSEADDR: c_int = 0x0004;
        const FIONBIO: c_long = @bitCast(@as(c_ulong, 0x8004667E));
        const WSAEWOULDBLOCK: c_int = 10035;
        const SockAddrIn = extern struct { family: u16 = AF_INET, port: u16, addr: u32, zero: [8]u8 = [_]u8{0} ** 8 };

        extern "ws2_32" fn WSAStartup(v: u16, data: *anyopaque) callconv(.winapi) c_int;
        extern "ws2_32" fn WSAGetLastError() callconv(.winapi) c_int;
        extern "ws2_32" fn socket(af: c_int, t: c_int, p: c_int) callconv(.winapi) Socket;
        extern "ws2_32" fn setsockopt(s: Socket, level: c_int, opt: c_int, val: [*]const u8, len: c_int) callconv(.winapi) c_int;
        extern "ws2_32" fn bind(s: Socket, name: *const SockAddrIn, len: c_int) callconv(.winapi) c_int;
        extern "ws2_32" fn listen(s: Socket, backlog: c_int) callconv(.winapi) c_int;
        extern "ws2_32" fn accept(s: Socket, name: ?*anyopaque, len: ?*c_int) callconv(.winapi) Socket;
        extern "ws2_32" fn closesocket(s: Socket) callconv(.winapi) c_int;
        extern "ws2_32" fn recv(s: Socket, buf: [*]u8, len: c_int, flags: c_int) callconv(.winapi) c_int;
        extern "ws2_32" fn send(s: Socket, buf: [*]const u8, len: c_int, flags: c_int) callconv(.winapi) c_int;
        extern "ws2_32" fn ioctlsocket(s: Socket, cmd: c_long, argp: *c_ulong) callconv(.winapi) c_int;

        fn startup() void {
            var data: [512]u8 = undefined; // WSADATA (we never read it back)
            _ = WSAStartup(0x0202, @ptrCast(&data));
        }
        fn listenOn(port: u16) ?Socket {
            const s = socket(AF_INET, SOCK_STREAM, 0);
            if (s == INVALID_SOCKET) return null;
            const one: c_int = 1;
            _ = setsockopt(s, SOL_SOCKET, SO_REUSEADDR, @ptrCast(&one), @sizeOf(c_int));
            const sa = SockAddrIn{ .port = std.mem.nativeToBig(u16, port), .addr = 0 };
            if (bind(s, &sa, @sizeOf(SockAddrIn)) != 0) return null;
            if (listen(s, 1) != 0) return null;
            return s;
        }
        fn acceptOne(s: Socket) ?Socket {
            const c = accept(s, null, null);
            return if (c == INVALID_SOCKET) null else c;
        }
        fn close(s: Socket) void {
            _ = closesocket(s);
        }
        fn sendAll(s: Socket, buf: []const u8) void {
            _ = send(s, buf.ptr, @intCast(buf.len), 0);
        }
        fn recvBlocking(s: Socket, buf: []u8) isize {
            return recv(s, buf.ptr, @intCast(buf.len), 0);
        }
        fn setNonblock(s: Socket, nb: bool) void {
            var mode: c_ulong = if (nb) 1 else 0;
            _ = ioctlsocket(s, FIONBIO, &mode);
        }
        fn recvNonblock(s: Socket, buf: []u8) RecvResult {
            const n = recv(s, buf.ptr, @intCast(buf.len), 0);
            if (n > 0) return .{ .got = @intCast(n) };
            if (n == 0) return .closed;
            return if (WSAGetLastError() == WSAEWOULDBLOCK) .again else .closed;
        }
    } else struct {
        pub const Socket = c_int;
        const SockAddrIn = std.posix.sockaddr.in;
        const MSG_DONTWAIT: c_int = std.posix.MSG.DONTWAIT;
        extern "c" fn socket(domain: c_int, sock_type: c_int, protocol: c_int) c_int;
        extern "c" fn setsockopt(fd: c_int, level: c_int, optname: c_int, optval: *const anyopaque, optlen: u32) c_int;
        extern "c" fn bind(fd: c_int, addr: *const SockAddrIn, len: u32) c_int;
        extern "c" fn listen(fd: c_int, backlog: c_int) c_int;
        extern "c" fn accept(fd: c_int, addr: ?*anyopaque, len: ?*u32) c_int;
        extern "c" fn recv(fd: c_int, buf: [*]u8, len: usize, flags: c_int) isize;
        extern "c" fn send(fd: c_int, buf: [*]const u8, len: usize, flags: c_int) isize;
        // `close` clashes with the wrapper name, so bind the libc symbol directly.
        const c_close = @extern(*const fn (c_int) callconv(.c) c_int, .{ .name = "close" });

        fn startup() void {}
        fn listenOn(port: u16) ?Socket {
            const s = socket(std.posix.AF.INET, std.posix.SOCK.STREAM, 0);
            if (s < 0) return null;
            const one: c_int = 1;
            _ = setsockopt(s, std.posix.SOL.SOCKET, std.posix.SO.REUSEADDR, &one, @sizeOf(c_int));
            const sa = SockAddrIn{ .port = std.mem.nativeToBig(u16, port), .addr = 0 };
            if (bind(s, &sa, @sizeOf(SockAddrIn)) != 0) return null;
            if (listen(s, 1) != 0) return null;
            return s;
        }
        fn acceptOne(s: Socket) ?Socket {
            const c = accept(s, null, null);
            return if (c < 0) null else c;
        }
        fn close(s: Socket) void {
            _ = c_close(s);
        }
        fn sendAll(s: Socket, buf: []const u8) void {
            _ = send(s, buf.ptr, buf.len, 0);
        }
        fn recvBlocking(s: Socket, buf: []u8) isize {
            return recv(s, buf.ptr, buf.len, 0);
        }
        fn setNonblock(_: Socket, _: bool) void {} // MSG_DONTWAIT is used per-call instead
        fn recvNonblock(s: Socket, buf: []u8) RecvResult {
            const n = recv(s, buf.ptr, buf.len, MSG_DONTWAIT);
            if (n > 0) return .{ .got = @intCast(n) };
            if (n == 0) return .closed;
            return .again; // EAGAIN/EWOULDBLOCK: no data yet
        }
    };

    /// GDB target: maps the stub register set to BDM registers over the session.
    const Arch = enum { hcs08, hcs12, cfv1 };

    const GdbTarget = struct {
        sess: *session.Session,
        conn: net.Socket,
        arch: Arch = .hcs08,
        breakpoints: lib.hwbreak.hcs08.Set = .{}, // HCS08 DBG comparators
        hcs12_breakpoints: lib.hwbreak.hcs12.Set = .{}, // classic-S12 DBG comparators
        cfv1_breakpoints: lib.hwbreak.cfv1.BreakpointSet = .{}, // CFV1 PBR0..3
        /// Classic-S12 (DBGV1) HW breakpoints available: HCS12 part on the FTS
        /// routine. S12G (GMMC) uses a different DBG module -> software bps.
        hcs12_hwbp: bool = false,
        /// vFlash* load support: parsed routine + params. null when unavailable
        /// (e.g. no bus clock), so `load` won't touch flash.
        flash: ?FlashPrep = null,
        memory_map: ?[]const u8 = null,
        /// HCS12 PPAGE register addr so the GDB PAGE reg maps the real bank;
        /// null on non-paged parts.
        ppage_addr: ?u16 = null,
        // vFlashWrite accumulator: GDB splits a block at arbitrary (non-align)
        // offsets; coalesce contiguous writes and program whole runs - a sub-align
        // chunk would 0xFF-pad and clobber a just-programmed cell (word/longword).
        gpa: std.mem.Allocator = undefined,
        flash_buf: std.ArrayList(u8) = .empty,
        flash_base: u32 = 0,
        flash_have: bool = false,

        /// BDCSCR.BDMACT: set in active background mode (halted) - HCS08 halt
        /// detect; GET_BDM_STATUS S_HALT is CFVx-only.
        const bdcscr_bdmact: u32 = 0x40;

        fn mem(self: *GdbTarget) lib.flash.Mem {
            return lib.flash.sessionMem(self.sess);
        }

        /// Debug-register write sink for CFV1 breakpoint programming.
        fn dregWrite(ctx: *anyopaque, reg: u16, value: u32) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            return self.sess.writeDReg(reg, value);
        }
        fn dregWriter(self: *GdbTarget) lib.hwbreak.cfv1.DRegWriter {
            return .{ .ctx = self, .writeFn = dregWrite };
        }

        /// GDB register (g-block index) → BDM register number. Layouts:
        /// gdb.hcs08_regs / gdb.hcs12_regs (m68hc12) / gdb.cfv1_regs.
        fn readReg(ctx: *anyopaque, index: usize) anyerror!u32 {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            const s = self.sess;
            return switch (self.arch) {
                .hcs08 => s.readReg(@intFromEnum(@as(target.Hcs08Reg, switch (index) {
                    0 => .pc,
                    1 => .sp,
                    2 => .hx,
                    3 => .a,
                    4 => .ccr,
                    else => return error.BadReg,
                }))),
                .hcs12 => switch (index) {
                    0 => s.readReg(@intFromEnum(target.Hcs12Reg.x)),
                    1 => s.readReg(@intFromEnum(target.Hcs12Reg.d)),
                    2 => s.readReg(@intFromEnum(target.Hcs12Reg.y)),
                    3 => s.readReg(@intFromEnum(target.Hcs12Reg.sp)),
                    4 => s.readReg(@intFromEnum(target.Hcs12Reg.pc)),
                    5 => (try s.readReg(@intFromEnum(target.Hcs12Reg.d))) >> 8, // A = D.hi
                    6 => (try s.readReg(@intFromEnum(target.Hcs12Reg.d))) & 0xFF, // B = D.lo
                    7 => s.readDReg(target.hcs12_dreg_ccr),
                    8 => if (self.ppage_addr) |pa| blk: {
                        var b: [1]u8 = undefined;
                        s.readMem(.byte, pa, &b) catch break :blk 0;
                        break :blk b[0];
                    } else 0, // PAGE/PPAGE (0 when the part isn't banked)
                    else => error.BadReg,
                },
                .cfv1 => switch (index) {
                    0...15 => s.readReg(@intCast(index)), // d0-d7, a0-a7
                    16 => s.readCReg(@intFromEnum(target.Cfv1CReg.sr)),
                    17 => s.readCReg(@intFromEnum(target.Cfv1CReg.pc)),
                    else => error.BadReg,
                },
            };
        }
        fn writeReg(ctx: *anyopaque, index: usize, value: u32) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            const s = self.sess;
            switch (self.arch) {
                .hcs08 => try s.writeReg(@intFromEnum(@as(target.Hcs08Reg, switch (index) {
                    0 => .pc,
                    1 => .sp,
                    2 => .hx,
                    3 => .a,
                    4 => .ccr,
                    else => return error.BadReg,
                })), value),
                .hcs12 => switch (index) {
                    0 => try s.writeReg(@intFromEnum(target.Hcs12Reg.x), value),
                    1 => try s.writeReg(@intFromEnum(target.Hcs12Reg.d), value),
                    2 => try s.writeReg(@intFromEnum(target.Hcs12Reg.y), value),
                    3 => try s.writeReg(@intFromEnum(target.Hcs12Reg.sp), value),
                    4 => try s.writeReg(@intFromEnum(target.Hcs12Reg.pc), value),
                    5, 6 => { // A/B: read-modify-write the D accumulator
                        const d = try s.readReg(@intFromEnum(target.Hcs12Reg.d));
                        const nd = if (index == 5) (d & 0x00FF) | ((value & 0xFF) << 8) else (d & 0xFF00) | (value & 0xFF);
                        try s.writeReg(@intFromEnum(target.Hcs12Reg.d), nd);
                    },
                    7 => try s.writeDReg(target.hcs12_dreg_ccr, value),
                    8 => if (self.ppage_addr) |pa| try s.writeMem(.byte, pa, &.{@truncate(value)}), // PAGE
                    else => return error.BadReg,
                },
                .cfv1 => switch (index) {
                    0...15 => try s.writeReg(@intCast(index), value),
                    16 => try s.writeCReg(@intFromEnum(target.Cfv1CReg.sr), value),
                    17 => try s.writeCReg(@intFromEnum(target.Cfv1CReg.pc), value),
                    else => return error.BadReg,
                },
            }
        }
        fn readMem(ctx: *anyopaque, addr: u32, buf: []u8) anyerror!usize {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            try self.sess.readMem(.byte, addr, buf);
            return buf.len;
        }
        fn writeMem(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            return self.sess.writeMem(.byte, addr, data);
        }
        fn resume_(ctx: *anyopaque, step: bool) anyerror!u8 {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            if (step) {
                try self.sess.step();
                return gdb.sig_trap;
            }
            // Continue: go, poll for halt, watch for GDB Ctrl-C (0x03). HCS08/HCS12
            // halt = BDCSCR/BDMSTS.BDMACT (0x40) via READ_STATUS_REG; CFV1 =
            // GET_BDM_STATUS S_HALT (CFVx-only).
            try self.sess.go();
            var probe: [1]u8 = undefined;
            // Windows non-blocking recv needs FIONBIO toggled (POSIX no-op:
            // MSG_DONTWAIT per-call). Restore blocking on exit for serveGdb's recv.
            net.setNonblock(self.conn, true);
            defer net.setNonblock(self.conn, false);
            while (true) {
                const halted = switch (self.arch) {
                    .cfv1 => (self.sess.bdmStatus() catch break).halted,
                    else => ((self.sess.readStatusReg() catch break) & bdcscr_bdmact) != 0,
                };
                if (halted) break;
                // Poll ~1 kHz, not flat-out (spinning pegged a core, hammered the BDM).
                RamTarget.sleepMs(undefined, 1);
                switch (net.recvNonblock(self.conn, &probe)) {
                    // Peer closed mid-run: halt and unwind so a new gdb can attach.
                    .closed => {
                        self.sess.halt() catch {};
                        return gdb.sig_int;
                    },
                    .got => if (probe[0] == 0x03) {
                        self.sess.halt() catch {};
                        return gdb.sig_int;
                    },
                    .again => {}, // no data yet - keep polling
                }
            }
            return gdb.sig_trap;
        }

        fn setBp(ctx: *anyopaque, addr: u32) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            if (self.arch == .cfv1) {
                self.cfv1_breakpoints.add(addr) catch return error.NoBreakpointSlot;
                return self.cfv1_breakpoints.apply(self.dregWriter());
            }
            // HCS08/HCS12 DBG comparators are 16-bit; reject >0xFFFF before the cast.
            if (addr > 0xFFFF) return error.NoBreakpointSlot;
            if (self.arch == .hcs12) {
                self.hcs12_breakpoints.add(@intCast(addr)) catch return error.NoBreakpointSlot;
                return self.hcs12_breakpoints.apply(self.mem());
            }
            self.breakpoints.add(@intCast(addr)) catch return error.NoBreakpointSlot;
            try self.breakpoints.apply(self.mem());
        }
        fn clearBp(ctx: *anyopaque, addr: u32) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            if (self.arch == .cfv1) {
                self.cfv1_breakpoints.remove(addr);
                return self.cfv1_breakpoints.apply(self.dregWriter());
            }
            if (addr > 0xFFFF) return; // never set; nothing to clear
            if (self.arch == .hcs12) {
                self.hcs12_breakpoints.remove(@intCast(addr));
                return self.hcs12_breakpoints.apply(self.mem());
            }
            self.breakpoints.remove(@intCast(addr));
            try self.breakpoints.apply(self.mem());
        }

        // vFlash* hooks: each builds a driver on the fly from the stored FlashPrep
        // (op completes in-call, so the stack RamTarget stays valid).
        /// One contiguous run per driver call, so runPadded's 0xFF pad only
        /// touches the run's ends (already blanked by vFlashErase).
        fn flashProgramRun(self: *GdbTarget, base: u32, data: []const u8) anyerror!void {
            var fp = self.flash orelse return error.NoFlash;
            var rt = fp.ramTarget(self.sess);
            var drv = fp.driver(rt.iface());
            return drv.program(base, data);
        }
        fn flashFlush(self: *GdbTarget) anyerror!void {
            if (!self.flash_have) return;
            self.flash_have = false;
            if (self.flash_buf.items.len == 0) return;
            try self.flashProgramRun(self.flash_base, self.flash_buf.items);
            self.flash_buf.clearRetainingCapacity();
        }
        fn flashErase(ctx: *anyopaque, addr: u32, len: u32) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            // Drop any stale accumulator from a prior/aborted load.
            self.flash_have = false;
            self.flash_buf.clearRetainingCapacity();
            var fp = self.flash orelse return error.NoFlash;
            var rt = fp.ramTarget(self.sess);
            var drv = fp.driver(rt.iface());
            return drv.eraseRange(addr, len);
        }
        fn flashWrite(ctx: *anyopaque, addr: u32, data: []const u8) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            // Append if contiguous, else flush and start a new run. GDB streams a
            // block ascending, so this coalesces it into one aligned program() pass.
            if (self.flash_have and addr == self.flash_base +% @as(u32, @intCast(self.flash_buf.items.len))) {
                return self.flash_buf.appendSlice(self.gpa, data);
            }
            try self.flashFlush();
            self.flash_base = addr;
            self.flash_buf.clearRetainingCapacity();
            try self.flash_buf.appendSlice(self.gpa, data);
            self.flash_have = true;
        }
        fn flashDone(ctx: *anyopaque) anyerror!void {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            try self.flashFlush(); // commit the final run
        }

        /// `monitor <cmd>`: a few target-control conveniences over the session.
        fn monitor(ctx: *anyopaque, cmd: []const u8, w: *Io.Writer) anyerror!bool {
            const self: *GdbTarget = @ptrCast(@alignCast(ctx));
            const s = self.sess;
            if (eq(cmd, "reset")) {
                try s.reset(.special, .all);
                s.connect() catch {};
                try w.writeAll("target reset\n");
                return true;
            }
            if (eq(cmd, "halt")) {
                try s.halt();
                try w.writeAll("target halted\n");
                return true;
            }
            if (eq(cmd, "help")) {
                try w.writeAll("monitor commands: reset, halt, help\n");
                return true;
            }
            return false; // unknown -> GDB prints "not supported"
        }

        // HW breakpoints: HCS08 (DBG), CFV1 (PBR), and classic-S12 HCS12 (DBGV1).
        // S12G and other archs leave the hooks null so the stub replies empty to
        // Z1/z1 (GDB falls back to software bps) rather than E02 "insertion failed".
        const vtable_common = gdb.Target.VTable{
            .readReg = readReg,
            .writeReg = writeReg,
            .readMem = readMem,
            .writeMem = writeMem,
            .resume_ = resume_,
            .monitor = monitor, // `monitor reset`/`halt` available on all archs
        };
        fn withHwbp(v: gdb.Target.VTable) gdb.Target.VTable {
            var out = v;
            out.setHwBreakpoint = setBp;
            out.clearHwBreakpoint = clearBp;
            return out;
        }
        fn withFlash(v: gdb.Target.VTable) gdb.Target.VTable {
            var out = v;
            out.flashErase = flashErase;
            out.flashWrite = flashWrite;
            out.flashDone = flashDone;
            return out;
        }
        const vtable_hwbp = withHwbp(vtable_common);
        const vtable_flash = withFlash(vtable_common);
        const vtable_hwbp_flash = withFlash(withHwbp(vtable_common));
        fn iface(self: *GdbTarget) gdb.Target {
            const regs: []const gdb.RegDef, const xml: []const u8 = switch (self.arch) {
                .hcs08 => .{ &gdb.hcs08_regs, gdb.target_xml },
                .hcs12 => .{ &gdb.hcs12_regs, gdb.hcs12_target_xml },
                .cfv1 => .{ &gdb.cfv1_regs, gdb.cfv1_target_xml },
            };
            // hwbp on HCS08, CFV1, and classic-S12 HCS12; flash hooks only when
            // routine+clock exist.
            const hwbp = self.arch == .hcs08 or self.arch == .cfv1 or (self.arch == .hcs12 and self.hcs12_hwbp);
            const has_flash = self.flash != null;
            const vt: *const gdb.Target.VTable = if (hwbp and has_flash)
                &vtable_hwbp_flash
            else if (hwbp)
                &vtable_hwbp
            else if (has_flash)
                &vtable_flash
            else
                &vtable_common;
            return .{ .ctx = self, .vtable = vt, .regs = regs, .xml = xml, .memory_map = self.memory_map };
        }
    };

    fn run(arena: std.mem.Allocator, bdm: *Bdm, flags: *const Flags, out: *Io.Writer, st: tty.Style) !void {
        const arch: Arch = switch (flags.target) {
            .hcs08, .rs08 => .hcs08,
            .hcs12, .hcs12z => .hcs12,
            .cfv1 => .cfv1,
            else => fatal("the gdb stub supports HCS08/RS08, HCS12, and CFV1 targets (got {s})", .{flags.target.name()}),
        };
        try bringUp(bdm, flags);
        // Set the BDM ENBDM bit (0x80) so a DBG match halts to the debugger, not
        // SWI: BDCSCR on HCS08, BDMSTS on HCS12 (both via WRITE_CONTROL_REG).
        if (arch == .hcs08 or arch == .hcs12) bdm.sess.writeControlReg(0x80) catch {};

        var gt = GdbTarget{ .sess = &bdm.sess, .conn = undefined, .arch = arch, .gpa = arena };
        // Classic S12 (FTS routine) has the DBGV1 module; S12G (GMMC) does not.
        if (arch == .hcs12) gt.hcs12_hwbp = resolveDevice(flags).hcs12_routine == .mmcv4_fts;
        // Enable load-to-flash when geometry + bus clock are known: build prep +
        // memory-map so GDB drives flash via vFlash*.
        const flash_on = flash_blk: {
            const d = resolveDevice(flags);
            if (d.paged) |p| gt.ppage_addr = p.ppage_addr; // GDB PAGE reg -> real PPAGE
            const bus_hz = flags.bus_hz orelse hz: {
                const sync = bdm.sess.getSpeedSync() catch 0;
                break :hz if (sync > 1) session.syncToHz(sync) else 0;
            };
            if (bus_hz == 0) break :flash_blk false;
            gt.flash = flashPrepFor(arena, d, bus_hz);
            gt.memory_map = memoryMapXml(arena, d) catch {
                gt.flash = null; // keep flash hooks off if the map build failed
                break :flash_blk false;
            };
            break :flash_blk true;
        };

        net.startup(); // Winsock init (no-op on POSIX)
        const fd = net.listenOn(flags.port) orelse fatal("bind/listen on port {d} failed (in use?)", .{flags.port});
        defer net.close(fd);

        const hwbp: []const u8 = switch (arch) {
            .hcs08 => "up to 2 hardware breakpoints (DBG module)",
            .cfv1 => "up to 4 hardware breakpoints (PBR debug registers)",
            .hcs12 => "software breakpoints only (no hardware breakpoints)",
        };
        try out.print("{s}gdb stub{s} listening on {s}localhost:{d}{s}\n", .{ st.bold, st.reset, st.green, flags.port, st.reset });
        try out.print("  {s}connect with:{s} (gdb) target remote localhost:{d}\n", .{ st.dim, st.reset, flags.port });
        try out.print("  {s}single-step; {s}{s}\n", .{ st.dim, hwbp, st.reset });
        if (flash_on)
            try out.print("  {s}`load` programs flash (vFlash) via {s}{s}{s}\n", .{ st.dim, resolveDevice(flags).name, st.dim, st.reset })
        else
            try out.print("  {s}`load` writes RAM only (no flash geometry/clock - pass --part and --bus-hz){s}\n", .{ st.dim, st.reset });
        try out.flush();
        while (true) {
            const conn = net.acceptOne(fd) orelse continue;
            gt.conn = conn;
            try out.print("{s}gdb connected{s}\n", .{ st.green, st.reset });
            try out.flush();
            serveGdb(gt.iface(), conn) catch {};
            net.close(conn);
            try out.print("{s}gdb disconnected{s}\n", .{ st.dim, st.reset });
            try out.flush();
        }
    }

    /// One GDB connection: read, ack, dispatch, respond (ack mode stays on).
    fn serveGdb(t: gdb.Target, conn: net.Socket) !void {
        var inbuf: [2 * gdb.max_packet]u8 = undefined;
        var len: usize = 0;
        var respbuf: [gdb.max_packet]u8 = undefined;
        var pktbuf: [gdb.max_packet + 8]u8 = undefined;

        while (true) {
            const decoded = gdbstub.decodePacket(inbuf[0..len], &respbuf) catch |e| switch (e) {
                error.NoPacketStart, error.Truncated => {
                    if (len == inbuf.len) len = 0; // overflow guard: drop garbage
                    const n = net.recvBlocking(conn, inbuf[len..]);
                    if (n <= 0) return; // connection closed
                    len += @intCast(n);
                    continue;
                },
                error.BadChecksum, error.BadEscape, error.Overflow => {
                    net.sendAll(conn, "-"); // request retransmit
                    len = 0;
                    continue;
                },
            };

            // Copy payload out before reusing respbuf.
            net.sendAll(conn, "+");
            var payload_buf: [gdb.max_packet]u8 = undefined;
            const payload = payload_buf[0..decoded.payload.len];
            @memcpy(payload, decoded.payload);

            const consumed = decoded.consumed;
            std.mem.copyForwards(u8, inbuf[0 .. len - consumed], inbuf[consumed..len]);
            len -= consumed;

            var rw = Io.Writer.fixed(&respbuf);
            const action = gdb.dispatch(t, payload, &rw) catch {
                var ew = Io.Writer.fixed(&respbuf);
                ew.writeAll("E01") catch {};
                var pw = Io.Writer.fixed(&pktbuf);
                gdbstub.writePacket(&pw, ew.buffered()) catch {};
                net.sendAll(conn, pw.buffered());
                continue;
            };

            if (action != .kill) {
                var pw = Io.Writer.fixed(&pktbuf);
                try gdbstub.writePacket(&pw, rw.buffered());
                net.sendAll(conn, pw.buffered());
            }
            if (action == .detach or action == .kill) return;
        }
    }
};

const Sim = lib.sim.Sim;

test "e2e HCS08: program -> readback -> blank-check(not blank) -> erase -> blank" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s08sh8").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    const parsed = try ramflash.parseSmall(gpa, flash_blob_80);
    defer gpa.free(parsed.image_owned);
    sim.small_routine = parsed.routine;
    sim.flash_armed = true;

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {}; // sync command-buffer size (chunking)
    var rt = RamTarget{ .sess = &s, .pc_reg = @intFromEnum(target.Hcs08Reg.pc) };
    var drv = ramflash.Driver{ .target = rt.iface(), .routine = parsed.routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
    } };

    const image = [_]u8{ 0x11, 0x22, 0x33, 0x44 };
    const addr = d.flash_start;
    try drv.program(addr, &image);
    var back: [4]u8 = undefined;
    try s.readMem(.byte, addr, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image); // routine-side verify agrees

    try std.testing.expectError(error.RoutineError, drv.blankCheck(addr, 0x100));
    try drv.eraseRange(addr, d.page_size);
    try drv.blankCheck(addr, d.page_size);
    try s.readMem(.byte, addr, &back);
    try std.testing.expectEqualSlices(u8, &.{ 0xFF, 0xFF, 0xFF, 0xFF }, &back);
}

test "e2e HCS12: large-code program + verify + mass-erase via the sim" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s12c32").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    sim.flash_armed = true;
    const routine = try ramflash.parseLarge(gpa, flash_blob_hcs12_fts);
    defer gpa.free(routine.image);

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    var rt = RamTarget{ .sess = &s, .pc_reg = @intFromEnum(target.Hcs12Reg.pc) };
    var drv = ramflash.LargeDriver{ .target = rt.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };

    // >244 bytes so the header+data write splits into multiple frames - exercises
    // writeMem-call-boundary tracking (a lone tail-chunk mustn't look like header).
    var image: [260]u8 = undefined;
    for (&image, 0..) |*b, i| b.* = @truncate(i * 3 + 1);
    const addr: u32 = 0x4000; // fixed flash window
    try drv.program(addr, &image); // blank-check + program + verify in one pass
    var back: [260]u8 = undefined;
    try s.readMem(.byte, addr, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);

    try drv.massErase(d.flash_start);
    try drv.blankCheck(addr, 0x100);
}

test "e2e S12G (MC9S12G240, GMMC/FTMRG blob): program + verify via the sim" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s12g240").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    sim.flash_armed = true;
    const routine = try ramflash.parseLarge(gpa, flash_blob_hcs12_gmmc);
    defer gpa.free(routine.image);

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    var rt = RamTarget{ .sess = &s, .pc_reg = @intFromEnum(target.Hcs12Reg.pc) };
    var drv = ramflash.LargeDriver{ .target = rt.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };
    // Odd length exercises the 8-byte write-align padding.
    const image = [_]u8{ 0x5A, 0xA5, 0x12, 0x34, 0x56 };
    const addr: u32 = 0xC000;
    try drv.program(addr, &image);
    var back: [5]u8 = undefined;
    try s.readMem(.byte, addr, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image);
}

test "e2e CFV1: 32-bit large-code program + verify via the sim" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mcf51qe128").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    sim.flash_armed = true;
    const routine = try ramflash.parseCfv1(gpa, flash_blob_cfv1);
    defer gpa.free(routine.image);

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    var rt = RamTarget{ .sess = &s, .pc_reg = @intFromEnum(target.Cfv1CReg.pc), .pc_creg = true, .mem_space = .word };
    var drv = ramflash.Cfv1Driver{ .target = rt.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 8000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };

    const image = [_]u8{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    const addr: u32 = 0x0000;
    try drv.program(addr, &image);
    var back: [8]u8 = undefined;
    try s.readMem(.byte, addr, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image);
}

test "e2e gdb: memory + register access maps through a sim session" {
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mc9s08qg8").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs08);
    try s.connect();
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .hcs08 };
    const t = gt.iface();
    var buf: [256]u8 = undefined;

    // M writes, m reads back - the gdb→GdbTarget→Session→sim path.
    var w1 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "M1000,4:deadbeef", &w1);
    try std.testing.expectEqualStrings("OK", w1.buffered());
    var w2 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "m1000,4", &w2);
    try std.testing.expectEqualStrings("deadbeef", w2.buffered());

    // p0 = pc (index 0, 2 bytes).
    sim.reg[@intFromEnum(target.Hcs08Reg.pc)] = 0x1234;
    var w3 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "p0", &w3);
    try std.testing.expectEqualStrings("1234", w3.buffered());

    // >0xFFFF breakpoint must E-reply, not panic the u32→u16 cast.
    var w4 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "Z1,10000,1", &w4); // 0x10000 > 0xFFFF
    try std.testing.expect(w4.buffered().len > 0 and w4.buffered()[0] == 'E');
    var w5 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "Z1,e000,1", &w5); // 0xE000 in range
    try std.testing.expectEqualStrings("OK", w5.buffered());
}

test "e2e gdb CFV1: Z1 programs the PBR/TDR debug registers; z1 disarms" {
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mcf51qe128").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.cfv1);
    try s.connect();
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .cfv1 };
    const t = gt.iface();
    var buf: [128]u8 = undefined;

    var w1 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "Z1,1234,1", &w1); // hardware breakpoint at 0x1234
    try std.testing.expectEqualStrings("OK", w1.buffered());
    try std.testing.expectEqual(@as(?u32, 0x1234), sim.dreg.get(lib.hwbreak.cfv1.DRegPBR0));
    try std.testing.expectEqual(@as(?u32, lib.hwbreak.cfv1.tdr_pc_halt), sim.dreg.get(lib.hwbreak.cfv1.DRegTDR));

    var w2 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "z1,1234,1", &w2); // remove -> TDR disarmed
    try std.testing.expectEqualStrings("OK", w2.buffered());
    try std.testing.expectEqual(@as(?u32, 0), sim.dreg.get(lib.hwbreak.cfv1.DRegTDR));
}

test "e2e gdb HCS12: Z1 programs the classic-S12 DBG comparator; z1 disarms" {
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mc9s12c32").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs12);
    try s.connect();
    // Classic S12 (FTS routine) has the DBGV1 module -> hardware breakpoints.
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .hcs12, .hcs12_hwbp = true };
    const t = gt.iface();
    var buf: [128]u8 = undefined;

    var w1 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "Z1,4321,1", &w1); // HW breakpoint at 0x4321
    try std.testing.expectEqualStrings("OK", w1.buffered());
    try std.testing.expectEqual(@as(?u8, 0x43), sim.mem.get(lib.hwbreak.hcs12.DBGCAH));
    try std.testing.expectEqual(@as(?u8, 0x21), sim.mem.get(lib.hwbreak.hcs12.DBGCAL));
    try std.testing.expectEqual(@as(?u8, 0x20), sim.mem.get(lib.hwbreak.hcs12.DBGC2)); // BDM route
    try std.testing.expectEqual(@as(?u8, 0xE8), sim.mem.get(lib.hwbreak.hcs12.DBGC1)); // armed, tagged

    var w2 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "z1,4321,1", &w2); // remove -> DBGC1 disarmed
    try std.testing.expectEqualStrings("OK", w2.buffered());
    try std.testing.expectEqual(@as(?u8, 0x00), sim.mem.get(lib.hwbreak.hcs12.DBGC1));
}

test "e2e gdb HCS12: S12G (GMMC) declines hardware breakpoints (software fallback)" {
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mc9s12g240").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs12);
    try s.connect();
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .hcs12, .hcs12_hwbp = false };
    const t = gt.iface();
    var buf: [64]u8 = undefined;
    var w = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "Z1,4321,1", &w); // no HW bp -> empty reply, GDB uses software
    try std.testing.expectEqualStrings("", w.buffered());
}

test "e2e SDID: --sim serves the device's SDID for identify/auto-detect" {
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mc9s12c32").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs12);
    try s.connect();
    // c32's SDID from the generated map.
    var expected: ?u16 = null;
    for (lib_device.sdid_table) |e| {
        if (std.mem.eql(u8, e.name, "mc9s12c32")) {
            expected = e.sdid; // sim serves the first entry for the device name
            break;
        }
    }
    try std.testing.expect(expected != null);
    // Read it back over the (virtual) BDM at the family SDID address.
    var b: [2]u8 = undefined;
    try s.readMem(.byte, lib_device.familySdidAddr(.hcs12), &b);
    const sdid = (@as(u16, b[0]) << 8) | b[1];
    try std.testing.expectEqual(expected.?, sdid);
    try std.testing.expect(lib_device.identify(.hcs12, sdid) != null);
}

test "e2e gdb: vFlash load programs flash through the real dispatch + sim" {
    const gpa = std.testing.allocator;
    var arena_inst = std.heap.ArenaAllocator.init(gpa);
    defer arena_inst.deinit();
    const arena = arena_inst.allocator();
    const d = lib_device.lookup("mc9s08sh8").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    sim.flash_armed = true;
    if (ramflash.parseSmall(arena, flash_blob_80)) |parsed| {
        sim.small_routine = parsed.routine; // backing image lives in the arena
    } else |_| {}
    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    try s.setTarget(.hcs08);
    try s.connect();

    const prep = flashPrepFor(arena, d, 4_000_000);
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .hcs08, .flash = prep, .gpa = arena };
    const t = gt.iface();
    var buf: [256]u8 = undefined;

    // The vFlash sequence GDB `load` sends: erase the region, stream data, done.
    var we = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashErase:e000,2000", &we);
    try std.testing.expectEqualStrings("OK", we.buffered());
    var ww = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashWrite:e000:\xAA\xBB\xCC\xDD", &ww);
    try std.testing.expectEqualStrings("OK", ww.buffered());
    var wd = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashDone", &wd);
    try std.testing.expectEqualStrings("OK", wd.buffered());

    var back: [4]u8 = undefined;
    try s.readMem(.byte, 0xE000, &back);
    try std.testing.expectEqualSlices(u8, &.{ 0xAA, 0xBB, 0xCC, 0xDD }, &back);
}

test "e2e gdb: vFlash coalesces split writes on a wide-alignment family (CFV1)" {
    // CFV1 programs longwords (align 4). GDB splits a block at arbitrary offsets;
    // programming each sub-4 chunk alone would 0xFF-pad and clobber the previous
    // (blank-check/verify fail). The accumulator coalesces into one aligned pass.
    const gpa = std.testing.allocator;
    var arena_inst = std.heap.ArenaAllocator.init(gpa);
    defer arena_inst.deinit();
    const arena = arena_inst.allocator();
    const d = lib_device.lookup("mcf51qe128").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    sim.flash_armed = true;
    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    try s.setTarget(.cfv1);
    try s.connect();

    const prep = flashPrepFor(arena, d, 8_000_000);
    var gt = Gdb.GdbTarget{ .sess = &s, .conn = undefined, .arch = .cfv1, .flash = prep, .gpa = arena };
    const t = gt.iface();
    var buf: [64]u8 = undefined;

    var we = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashErase:0,1000", &we);
    try std.testing.expectEqualStrings("OK", we.buffered());
    // Two contiguous writes split at offset 5 (NOT a multiple of 4).
    var w1 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashWrite:0:\x01\x02\x03\x04\x05", &w1);
    try std.testing.expectEqualStrings("OK", w1.buffered());
    var w2 = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashWrite:5:\x06\x07\x08\x09\x0A", &w2);
    try std.testing.expectEqualStrings("OK", w2.buffered());
    var wd = Io.Writer.fixed(&buf);
    _ = try gdb.dispatch(t, "vFlashDone", &wd); // flushes -> one program(0, 10 bytes)
    try std.testing.expectEqualStrings("OK", wd.buffered());

    var back: [10]u8 = undefined;
    try s.readMem(.byte, 0x0000, &back);
    try std.testing.expectEqualSlices(u8, &.{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A }, &back);
}

test "e2e HCS08: the REAL flash routine executes on the CPU08 emulator" {
    // The real vendored routine runs on the CPU08+FLASH model, fed the param
    // block the driver marshals; a wrong ABI mis-programs.
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s08sh8").?;
    var cpu = try lib.cpu08.Cpu.init(gpa, d.flash_base, d.flash_start, d.flash_end);
    defer cpu.deinit();
    const parsed = try ramflash.parseSmall(gpa, flash_blob_80);
    defer gpa.free(parsed.image_owned);

    var et = EmuTarget{ .cpu = &cpu };
    var drv = ramflash.Driver{ .target = et.iface(), .routine = parsed.routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
    } };

    const image = [_]u8{ 0x11, 0x22, 0x33, 0x44 };
    const addr = d.flash_start; // 0xE000

    // program() downloads + runs the real routine; it writes flash via the model.
    try drv.program(addr, &image);
    try std.testing.expectEqual(@as(u8, 0x11), cpu.mem[addr]);
    try std.testing.expectEqual(@as(u8, 0x44), cpu.mem[addr + 3]);
    try std.testing.expectEqual(@as(u16, 0), std.mem.readInt(u16, cpu.mem[0x0080..0x0082], .big)); // DriverError OK
    try drv.verify(addr, &image); // host readback-compare (blob has no verify routine)

    try drv.eraseRange(addr, d.page_size);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.mem[addr]);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.mem[addr + 3]);
    try drv.blankCheck(addr, d.page_size);

    try drv.program(addr, &image);
    try std.testing.expectError(error.RoutineError, drv.blankCheck(addr, d.page_size));
    try std.testing.expectEqual(ramflash.DriverError.erase_failed, drv.last_error);
}

test "e2e CFV1: the REAL ColdFire flash routine executes on the cfv1 emulator" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mcf51qe128").?;
    var cpu = try lib.cfv1_emu.Cpu.init(gpa, d.flash_base, d.flash_start, d.flash_end);
    defer cpu.deinit();
    const routine = try ramflash.parseCfv1(gpa, flash_blob_cfv1);
    defer gpa.free(routine.image);

    var et = Cfv1EmuTarget{ .cpu = &cpu };
    var drv = ramflash.Cfv1Driver{ .target = et.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 6000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };

    const image = [_]u8{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    const addr: u32 = 0x0000;
    try drv.program(addr, &image); // init + blank-check + program + verify, real routine
    var back: [8]u8 = undefined;
    cpu.hostRead(addr, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image);
    try drv.massErase(d.flash_start);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.hostReadByte(addr));
    try drv.blankCheck(addr, 0x400);
}

test "e2e HCS12: the REAL flash routine executes on the cpu12 emulator" {
    // The real HCS12 FTS routine runs on the CPU12+FTS model, fed the 18-byte
    // big-endian param block LargeDriver marshals; a marshalling mistake reads
    // wrong header fields and mis-programs.
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s12c32").?;
    // Test flash window: the 0x4000-0x7FFF page.
    var cpu = try lib.cpu12.Cpu.init(gpa, d.flash_base, 0x4000, 0x7FFF);
    defer cpu.deinit();
    const routine = try ramflash.parseLarge(gpa, flash_blob_hcs12_fts);
    defer gpa.free(routine.image);

    var et = Cpu12EmuTarget{ .cpu = &cpu };
    var drv = ramflash.LargeDriver{ .target = et.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };

    const image = [_]u8{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    const addr: u32 = 0x4000;
    try drv.program(addr, &image); // init + blank-check + program + verify, real routine
    var back: [8]u8 = undefined;
    cpu.hostRead(@intCast(addr), &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image);

    try drv.massErase(d.flash_start);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.hostReadByte(@intCast(addr)));
    try drv.blankCheck(addr, 0x400);
}

test "e2e S12G: the REAL GMMC/FTMRG flash routine executes on the cpu12 emulator" {
    // S12G sibling of the FTS test: GMMC uses a different controller (FCCOB objects,
    // FSTAT@0x06/CCIF=0x80) and checks P-flash through the PPAGE-banked window -
    // exercises the FTMRG model + paged global addressing.
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s12g240").?;
    var cpu = try lib.cpu12.Cpu.initGmmc(gpa, d.flash_base);
    defer cpu.deinit();
    const routine = try ramflash.parseLarge(gpa, flash_blob_hcs12_gmmc);
    defer gpa.free(routine.image);

    var et = Cpu12EmuTarget{ .cpu = &cpu };
    var drv = ramflash.LargeDriver{ .target = et.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(d.page_size),
        .frequency_khz = 4000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = d.write_align,
    } };

    const image = [_]u8{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 };
    const addr: u32 = 0x4000; // CPU window -> global 0x03_4000 (fixed PPAGE 0x0D)
    try drv.program(addr, &image); // blank-check + program (one phrase) + verify
    var back: [8]u8 = undefined;
    cpu.hostRead(@intCast(addr), &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.verify(addr, &image);

    try drv.massErase(d.flash_start);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.hostReadByte(@intCast(addr)));
    try drv.blankCheck(addr, 0x400);
}

test "e2e S12G EEPROM: the REAL GMMC routine programs D-flash (bit-31 linear addr)" {
    // The FLASH blob routes a bit-31 "linear" global address to the D-flash
    // commands (0x11/0x12), verified empirically. Drives the whole EEPROM path:
    // erase + blank-check + program + verify on the FTMRG model.
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s12g240").?;
    const ee = d.eeprom.?;
    var cpu = try lib.cpu12.Cpu.initGmmc(gpa, d.flash_base);
    defer cpu.deinit();
    const routine = try ramflash.parseLarge(gpa, flash_blob_hcs12_gmmc);
    defer gpa.free(routine.image);

    var et = Cpu12EmuTarget{ .cpu = &cpu };
    var drv = ramflash.LargeDriver{ .target = et.iface(), .routine = routine, .params = .{
        .ram_start = d.ram_start,
        .ram_end = d.ram_end,
        .controller = d.flash_base,
        .sector_size = @intCast(ee.sector),
        .frequency_khz = 4000,
        .watchdog_addr = d.watchdog_addr,
        .alignment = ee.align_bytes,
    } };

    const lin = lib_device.eeprom_linear_flag | ee.base_global; // 0x80000400
    const image = [_]u8{ 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE };
    try drv.program(lin, &image); // blank-check + program + verify D-flash
    // D-flash array directly (global 0x000400 → dflash[0]).
    try std.testing.expectEqual(@as(u8, 0xDE), cpu.dflash[0]);
    try std.testing.expectEqual(@as(u8, 0xFE), cpu.dflash[5]);
    try drv.verify(lin, &image);
    // P-flash is untouched by the EEPROM program.
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.pflash[0x8000]);
    try drv.eraseRange(lin, 8);
    try std.testing.expectEqual(@as(u8, 0xFF), cpu.dflash[0]);
    try drv.blankCheck(lin, 8);
}

test "jsonStr escapes the JSON-significant characters" {
    var buf: [128]u8 = undefined;
    var w = Io.Writer.fixed(&buf);
    try jsonStr(&w, "a\"b\\c\n\t");
    try std.testing.expectEqualStrings("\"a\\\"b\\\\c\\n\\t\"", w.buffered());
    var w2 = Io.Writer.fixed(&buf);
    try jsonStr(&w2, "\x01"); // control char -> \u0001
    try std.testing.expectEqualStrings("\"\\u0001\"", w2.buffered());
}

test "parseFlags: positionals, valued flags, and toggles" {
    var arena_inst = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena_inst.deinit();
    const f = try parseFlags(arena_inst.allocator(), &.{ "read", "0x80", "4", "--target", "hcs12", "--json", "-v", "--part", "mc9s12c32" });
    try std.testing.expectEqual(@as(usize, 3), f.positionals.len);
    try std.testing.expectEqualStrings("read", f.positionals[0]);
    try std.testing.expectEqual(target.TargetType.hcs12, f.target);
    try std.testing.expect(f.json and f.verbose);
    try std.testing.expectEqualStrings("mc9s12c32", f.part.?);
}

test "lookupReg maps names per family (and rejects unknowns)" {
    try std.testing.expectEqual(@as(u16, @intFromEnum(target.Hcs08Reg.pc)), lookupReg(.hcs08, "pc").?.no);
    try std.testing.expectEqual(@as(u16, 15), lookupReg(.cfv1, "a7").?.no);
    try std.testing.expectEqual(@as(u16, @intFromEnum(target.Rs08Reg.spc)), lookupReg(.rs08, "spc").?.no);
    try std.testing.expectEqual(@as(?RegSpec, null), lookupReg(.hcs08, "nope"));
    try std.testing.expectEqual(@as(?RegSpec, null), lookupReg(.rs08, "hx")); // RS08 has no HX
}

test "detectByExt and familyOfTarget" {
    try std.testing.expectEqual(hexfile.Format.srec, detectByExt("app.s19"));
    try std.testing.expectEqual(hexfile.Format.ihex, detectByExt("app.hex"));
    try std.testing.expectEqual(hexfile.Format.binary, detectByExt("app.bin"));
    try std.testing.expectEqual(lib_device.Family.hcs08, familyOfTarget(.rs08));
    try std.testing.expectEqual(lib_device.Family.hcs12, familyOfTarget(.hcs12z));
    try std.testing.expectEqual(lib_device.Family.cfv1, familyOfTarget(.cfv1));
}

test "serveGdb: full framing loop over a socketpair (qSupported, then detach)" {
    if (@import("builtin").os.tag == .windows) return error.SkipZigTest;
    const gpa = std.testing.allocator;
    var sim = Sim.init(gpa, lib_device.lookup("mc9s08sh8").?);
    defer sim.deinit();
    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs08);
    try s.connect();

    const lc = struct {
        extern "c" fn socketpair(domain: c_int, ty: c_int, protocol: c_int, sv: *[2]c_int) c_int;
        extern "c" fn read(fd: c_int, buf: [*]u8, n: usize) isize;
        extern "c" fn write(fd: c_int, buf: [*]const u8, n: usize) isize;
        extern "c" fn close(fd: c_int) c_int;
    };
    var fds: [2]c_int = undefined;
    if (lc.socketpair(std.posix.AF.UNIX, std.posix.SOCK.STREAM, 0, &fds) != 0) return error.SocketpairFailed;
    const server: c_int = fds[0];
    const client: c_int = fds[1];
    defer _ = lc.close(client);

    var gt = Gdb.GdbTarget{ .sess = &s, .conn = server, .arch = .hcs08 };
    const Th = struct {
        fn run(t: gdb.Target, conn: c_int) void {
            Gdb.serveGdb(t, conn) catch {};
            _ = lc.close(conn); // let the client's reads unblock on EOF
        }
    };
    const th = try std.Thread.spawn(.{}, Th.run, .{ gt.iface(), server });

    // Send $qSupported, expect ack '+' then a framed reply.
    var pbuf: [64]u8 = undefined;
    var pw = Io.Writer.fixed(&pbuf);
    try gdbstub.writePacket(&pw, "qSupported");
    _ = lc.write(client, pw.buffered().ptr, pw.buffered().len);

    var acc: [512]u8 = undefined;
    var got: usize = 0;
    // Read until we have a complete '$...#hh' frame (after the leading '+').
    while (std.mem.indexOfScalar(u8, acc[0..got], '#') == null or got < 4) {
        const n = lc.read(client, acc[got..].ptr, acc.len - got);
        if (n <= 0) break;
        got += @intCast(n);
    }
    var dbuf: [512]u8 = undefined;
    const decoded = try gdbstub.decodePacket(acc[0..got], &dbuf);
    try std.testing.expect(std.mem.indexOf(u8, decoded.payload, "PacketSize=") != null);

    // Detach so serveGdb returns and the thread joins.
    var dw = Io.Writer.fixed(&pbuf);
    try gdbstub.writePacket(&dw, "D");
    _ = lc.write(client, dw.buffered().ptr, dw.buffered().len);
    th.join();
}

test "e2e --sim+cpu backend: the REAL HCS08 routine programs through the sim stack" {
    const gpa = std.testing.allocator;
    var arena_inst = std.heap.ArenaAllocator.init(gpa);
    defer arena_inst.deinit();
    const arena = arena_inst.allocator();
    const d = lib_device.lookup("mc9s08sh8").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    var cpu = try lib.cpu08.Cpu.init(gpa, d.flash_base, d.flash_start, d.flash_end);
    defer cpu.deinit();
    sim.cpu = .{ .hcs08 = &cpu };

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    try s.setTarget(.hcs08);
    try s.connect();

    const prep = flashPrepFor(arena, d, 4_000_000);
    var rt = prep.ramTarget(&s);
    var drv = prep.driver(rt.iface());
    const image = [_]u8{ 0x11, 0x22, 0x33, 0x44 };
    try drv.program(0xE000, &image); // real routine runs on cpu08 via the sim
    try drv.verify(0xE000, &image);
    var back: [4]u8 = undefined;
    try s.readMem(.byte, 0xE000, &back); // sim -> backend memory
    try std.testing.expectEqualSlices(u8, &image, &back);
    try drv.eraseRange(0xE000, d.page_size);
    try drv.blankCheck(0xE000, d.page_size);
}

test "e2e --sim+cpu backend: single-step executes a real instruction" {
    const gpa = std.testing.allocator;
    const d = lib_device.lookup("mc9s08sh8").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    var cpu = try lib.cpu08.Cpu.init(gpa, d.flash_base, d.flash_start, d.flash_end);
    defer cpu.deinit();
    sim.cpu = .{ .hcs08 = &cpu };

    var s = session.Session.init(sim.asTransport(.{}));
    try s.setTarget(.hcs08);
    try s.connect();
    // LDA #$5A (0xA6 0x5A) at 0x0200; step once -> PC=0x0202, A=0x5A.
    try s.writeMem(.byte, 0x0200, &.{ 0xA6, 0x5A });
    try s.writeReg(@intFromEnum(target.Hcs08Reg.pc), 0x0200);
    try s.step();
    try std.testing.expectEqual(@as(u32, 0x0202), try s.readReg(@intFromEnum(target.Hcs08Reg.pc)));
    try std.testing.expectEqual(@as(u32, 0x5A), try s.readReg(@intFromEnum(target.Hcs08Reg.a)));
}

test "e2e --sim+cpu backend: the REAL CFV1 routine programs through the sim stack" {
    // Exercises the CFV1 arm of CpuBackend (creg PC routing + XCSR status).
    const gpa = std.testing.allocator;
    var arena_inst = std.heap.ArenaAllocator.init(gpa);
    defer arena_inst.deinit();
    const arena = arena_inst.allocator();
    const d = lib_device.lookup("mcf51qe128").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    var cpu = try lib.cfv1_emu.Cpu.init(gpa, d.flash_base, d.flash_start, d.flash_end);
    defer cpu.deinit();
    sim.cpu = .{ .cfv1 = &cpu };

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    try s.setTarget(.cfv1);
    try s.connect();
    const prep = flashPrepFor(arena, d, 8_000_000);
    var rt = prep.ramTarget(&s);
    var drv = prep.driver(rt.iface());
    const image = [_]u8{ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
    try drv.program(0x0000, &image);
    try drv.verify(0x0000, &image);
    var back: [8]u8 = undefined;
    try s.readMem(.byte, 0x0000, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
}

test "e2e --sim+cpu backend: the REAL HCS12 FTS routine programs through the sim stack" {
    // Exercises the HCS12 arm of CpuBackend (core-reg PC routing + BDMACT).
    const gpa = std.testing.allocator;
    var arena_inst = std.heap.ArenaAllocator.init(gpa);
    defer arena_inst.deinit();
    const arena = arena_inst.allocator();
    const d = lib_device.lookup("mc9s12c32").?;
    var sim = Sim.init(gpa, d);
    defer sim.deinit();
    const flash_hi: u32 = if (d.flash_end2 != 0) d.flash_end2 else d.flash_end;
    var cpu = try lib.cpu12.Cpu.init(gpa, d.flash_base, d.flash_start, flash_hi);
    defer cpu.deinit();
    sim.cpu = .{ .hcs12 = &cpu };

    var s = session.Session.init(sim.asTransport(.{}));
    _ = s.capabilities() catch {};
    try s.setTarget(.hcs12);
    try s.connect();
    const prep = flashPrepFor(arena, d, 4_000_000);
    var rt = prep.ramTarget(&s);
    var drv = prep.driver(rt.iface());
    const image = [_]u8{ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x01, 0x02 };
    try drv.program(0x4000, &image);
    try drv.verify(0x4000, &image);
    var back: [8]u8 = undefined;
    try s.readMem(.byte, 0x4000, &back);
    try std.testing.expectEqualSlices(u8, &image, &back);
}
