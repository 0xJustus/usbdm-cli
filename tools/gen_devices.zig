//! Parses vendored USBDM device-XML into Zig `table` + `sdid_table`.
//! Geometry that can't be faithfully represented is dropped and reported, never guessed.

const std = @import("std");

const Family = enum {
    hcs08,
    hcs12,
    cfv1,
    fn tag(self: Family) []const u8 {
        return switch (self) {
            .hcs08 => "HCS08",
            .hcs12 => "HCS12",
            .cfv1 => "CFV1",
        };
    }
    fn enumName(self: Family) []const u8 {
        return @tagName(self);
    }
};

fn fatal(comptime fmt: []const u8, args: anytype) noreturn {
    std.debug.print("gen_devices: " ++ fmt ++ "\n", args);
    std.process.exit(1);
}

// Vendored XML is simple, well-formed, attribute-only: no full parser needed.

fn isIdent(c: u8) bool {
    return (c >= 'a' and c <= 'z') or (c >= 'A' and c <= 'Z') or
        (c >= '0' and c <= '9') or c == '_' or c == ':' or c == '-';
}

/// Attr name = identifier run before `="`, so `subfamily="X"` isn't matched as `family="X"`.
fn attr(tag: []const u8, name: []const u8) ?[]const u8 {
    var i: usize = 0;
    while (i + 1 < tag.len) {
        if (tag[i] == '=' and tag[i + 1] == '"') {
            var j = i;
            while (j > 0 and isIdent(tag[j - 1])) j -= 1;
            const aname = tag[j..i];
            const vstart = i + 2;
            var k = vstart;
            while (k < tag.len and tag[k] != '"') k += 1;
            const val = tag[vstart..k];
            if (std.mem.eql(u8, aname, name)) return val;
            i = k + 1;
        } else i += 1;
    }
    return null;
}

fn tagBoundary(c: u8) bool {
    return c == ' ' or c == '\t' or c == '\n' or c == '\r' or c == '>' or c == '/';
}

const Elem = struct {
    attrs: []const u8, // "<name" .. ">", trailing "/" trimmed
    body: []const u8, // ">" .. "</name>", "" if self-closing
    end: usize, // index just past the element
    self_closing: bool,
};

/// Boundary-char match: "memory" won't match "memoryRange"/"memoryRef", "sdid" not "sdidAddress".
fn scanElem(src: []const u8, from: usize, name: []const u8) ?Elem {
    var p = from;
    while (p < src.len) {
        if (src[p] == '<' and p + 1 + name.len < src.len and
            std.mem.eql(u8, src[p + 1 .. p + 1 + name.len], name) and
            tagBoundary(src[p + 1 + name.len]))
        {
            const astart = p + 1 + name.len;
            // honour quoted values: '>' can appear inside them
            var q = astart;
            var in_q = false;
            while (q < src.len) : (q += 1) {
                if (src[q] == '"') in_q = !in_q;
                if (src[q] == '>' and !in_q) break;
            }
            if (q >= src.len) return null;
            const self_closing = src[q - 1] == '/';
            const attrs_end = if (self_closing) q - 1 else q;
            const attrs = src[astart..attrs_end];
            if (self_closing) {
                return .{ .attrs = attrs, .body = "", .end = q + 1, .self_closing = true };
            }
            const close = std.fmt.allocPrint(std.heap.page_allocator, "</{s}>", .{name}) catch unreachable;
            defer std.heap.page_allocator.free(close);
            const cidx = std.mem.indexOfPos(u8, src, q + 1, close) orelse return null;
            return .{ .attrs = attrs, .body = src[q + 1 .. cidx], .end = cidx + close.len, .self_closing = false };
        }
        p += 1;
    }
    return null;
}

fn elemAttr(body: []const u8, name: []const u8, a: []const u8) ?[]const u8 {
    const e = scanElem(body, 0, name) orelse return null;
    return attr(e.attrs, a);
}

/// Strip <!-- --> so commented-out devices/ranges are ignored.
fn stripComments(alloc: std.mem.Allocator, src: []const u8) []u8 {
    const out = alloc.alloc(u8, src.len) catch fatal("oom", .{});
    var n: usize = 0;
    var i: usize = 0;
    while (i < src.len) {
        if (i + 3 < src.len and std.mem.eql(u8, src[i .. i + 4], "<!--")) {
            const close = std.mem.indexOfPos(u8, src, i + 4, "-->") orelse src.len;
            i = if (close == src.len) src.len else close + 3;
        } else {
            out[n] = src[i];
            n += 1;
            i += 1;
        }
    }
    return out[0..n];
}

fn parseNum(s0: []const u8) u64 {
    var s = std.mem.trim(u8, s0, " \t\r\n");
    if (s.len == 0) return 0;
    var mul: u64 = 1;
    const last = s[s.len - 1];
    if (last == 'K' or last == 'k') {
        mul = 1024;
        s = s[0 .. s.len - 1];
    } else if (last == 'M' or last == 'm') {
        mul = 1024 * 1024;
        s = s[0 .. s.len - 1];
    }
    s = std.mem.trim(u8, s, " \t");
    const v = if (s.len > 2 and s[0] == '0' and (s[1] == 'x' or s[1] == 'X'))
        (std.fmt.parseInt(u64, s[2..], 16) catch 0)
    else
        (std.fmt.parseInt(u64, s, 10) catch 0);
    return v * mul;
}

fn lower(alloc: std.mem.Allocator, s: []const u8) []u8 {
    const out = alloc.alloc(u8, s.len) catch fatal("oom", .{});
    for (s, 0..) |c, i| out[i] = std.ascii.toLower(c);
    return out;
}

const RawDev = struct {
    name: []const u8,
    attrs: []const u8,
    body: []const u8,
    family: ?[]const u8,
    alias: ?[]const u8,
    is_default: bool,
};

const Defaults = struct {
    sdid_addr: u64,
    sdid_mask: u64,
    flash_reg: u64,
    flash_sec: u64,
    sector: u64,
    alignv: u64,
    copctl: u64,
    sopt: u64,
    routine: ?[]const u8,
};

const Win = struct { start: u64, end: u64 };

const SUP_HCS08 = [_][]const u8{ "HCS08-default-flash-program", "HCS08-small-flash-program", "HCS08-alt-load-flash-program" };
const MMCV4 = [_][]const u8{ "HCS12-MMCV4-FTS-flash-program", "HCS12-MMCV4-FTS_2-flash-program", "HCS12-MMCV4-FTS_T-flash-program" };
const GMMC = "HCS12-GMMC-FTMRG-flash-program";
const GMMC_EE = "HCS12-GMMC-FTMRG-eeprom-program";
const SUP_CFV1 = [_][]const u8{ "CFV1-default-FlashProgram", "CFV1-Watchdog-FlashProgram" };

fn inList(list: []const []const u8, s: []const u8) bool {
    for (list) |x| if (std.mem.eql(u8, x, s)) return true;
    return false;
}

const Shared = struct { id: []const u8, type_: []const u8, attrs: []const u8, body: []const u8 };

const Stats = struct {
    table_hcs08: usize = 0,
    table_hcs12: usize = 0,
    table_cfv1: usize = 0,
    sdid_hcs08: usize = 0,
    sdid_hcs12: usize = 0,
    sdid_cfv1: usize = 0,
    skip_default: usize = 0,
    skip_family: usize = 0,
    skip_routine: usize = 0,
    skip_noflash: usize = 0,
    reports: usize = 0,
    fn report(self: *Stats, comptime fmt: []const u8, args: anytype) void {
        self.reports += 1;
        std.debug.print("  REPORT: " ++ fmt ++ "\n", args);
    }
};

const Ctx = struct {
    alloc: std.mem.Allocator,
    devs: []RawDev,
    shared: []Shared,
    fam: Family,
    def: Defaults,

    fn find(self: *const Ctx, name: []const u8) ?*RawDev {
        for (self.devs) |*d| if (std.mem.eql(u8, d.name, name)) return d;
        return null;
    }
    fn sharedById(self: *const Ctx, id: []const u8) ?*const Shared {
        for (self.shared) |*s| if (std.mem.eql(u8, s.id, id)) return s;
        return null;
    }
};

/// Collect type==mtype blocks: inline <memory type=..> and <memoryRef ref=..> to shared blocks.
const Block = struct { attrs: []const u8, body: []const u8 };
fn memBlocks(ctx: *const Ctx, body: []const u8, mtype: []const u8, out: *std.ArrayList(Block)) void {
    var pos: usize = 0;
    while (scanElem(body, pos, "memory")) |e| {
        pos = e.end;
        if (attr(e.attrs, "type")) |ty| {
            if (std.mem.eql(u8, ty, mtype)) out.append(ctx.alloc, .{ .attrs = e.attrs, .body = e.body }) catch fatal("oom", .{});
        }
    }
    pos = 0;
    while (scanElem(body, pos, "memoryRef")) |e| {
        pos = e.end;
        if (attr(e.attrs, "ref")) |ref| {
            if (ctx.sharedById(ref)) |s| {
                if (std.mem.eql(u8, s.type_, mtype)) out.append(ctx.alloc, .{ .attrs = s.attrs, .body = s.body }) catch fatal("oom", .{});
            }
        }
    }
}

/// First flashProgramRef NOT inside a <memory> block (those carry their own eeprom refs).
fn deviceRoutine(ctx: *const Ctx, body: []const u8) ?[]const u8 {
    var spans: std.ArrayList([2]usize) = .empty;
    var pos: usize = 0;
    while (scanElem(body, pos, "memory")) |e| {
        // e.body span suffices: a memory block's flashProgramRef lives inside its body
        const bstart = @intFromPtr(e.body.ptr) - @intFromPtr(body.ptr);
        spans.append(ctx.alloc, .{ bstart, bstart + e.body.len }) catch fatal("oom", .{});
        pos = e.end;
    }
    pos = 0;
    while (scanElem(body, pos, "flashProgramRef")) |e| {
        const idx = @intFromPtr(e.attrs.ptr) - @intFromPtr(body.ptr);
        var inside = false;
        for (spans.items) |sp| if (idx >= sp[0] and idx < sp[1]) {
            inside = true;
        };
        if (!inside) return attr(e.attrs, "ref");
        pos = e.end;
    }
    return null;
}

fn hasSdid(body: []const u8) bool {
    return scanElem(body, 0, "sdid") != null;
}

fn rangeStartEnd(r: []const u8) ?Win {
    const has_s = attr(r, "start");
    const has_e = attr(r, "end");
    const has_z = attr(r, "size");
    var st: ?u64 = if (has_s) |x| parseNum(x) else null;
    var en: ?u64 = if (has_e) |x| parseNum(x) else null;
    const sz: ?u64 = if (has_z) |x| parseNum(x) else null;
    if (st != null and en != null) {} else if (st != null and sz != null) {
        en = st.? + sz.? - 1;
    } else if (sz != null and en != null) {
        st = en.? - sz.? + 1;
    } else return null;
    return .{ .start = st.?, .end = en.? };
}

fn rangeBanked(r: []const u8) bool {
    return attr(r, "pageReset") != null or attr(r, "pageStart") != null or
        attr(r, "pages") != null or attr(r, "pageEnd") != null;
}

fn parseDefaults(ctx: *Ctx) void {
    for (ctx.devs) |*d| {
        if (!d.is_default) continue;
        var fl: std.ArrayList(Block) = .empty;
        memBlocks(ctx, d.body, "flash", &fl);
        const fa = if (fl.items.len > 0) fl.items[0].attrs else "";
        ctx.def = .{
            .sdid_addr = if (elemAttr(d.body, "sdidAddress", "value")) |x| parseNum(x) else 0,
            .sdid_mask = if (elemAttr(d.body, "sdidMask", "value")) |x| parseNum(x) else 0,
            .flash_reg = if (attr(fa, "registerAddress")) |x| parseNum(x) else 0,
            .flash_sec = if (attr(fa, "securityAddress")) |x| parseNum(x) else 0,
            .sector = if (attr(fa, "sectorSize")) |x| parseNum(x) else 512,
            .alignv = if (attr(fa, "alignment")) |x| parseNum(x) else 1,
            .copctl = if (elemAttr(d.body, "copctlAddress", "value")) |x| parseNum(x) else 0,
            .sopt = if (elemAttr(d.body, "soptAddress", "value")) |x| parseNum(x) else 0,
            .routine = deviceRoutine(ctx, d.body),
        };
        return;
    }
    fatal("no isDefault device in {s}", .{ctx.fam.tag()});
}

fn processFile(
    alloc: std.mem.Allocator,
    src_raw: []const u8,
    fam: Family,
    table_w: *std.Io.Writer,
    sdid_w: *std.Io.Writer,
    stats: *Stats,
) !void {
    const src = stripComments(alloc, src_raw);

    const dl = std.mem.indexOf(u8, src, "<deviceList>") orelse fatal("no <deviceList> in {s}", .{fam.tag()});
    const shared_part = src[0..dl];
    const dev_part = src[dl..];

    var shared: std.ArrayList(Shared) = .empty;
    {
        var pos: usize = 0;
        while (scanElem(shared_part, pos, "memory")) |e| {
            pos = e.end;
            const id = attr(e.attrs, "id") orelse continue;
            const ty = attr(e.attrs, "type") orelse "";
            try shared.append(alloc, .{ .id = id, .type_ = ty, .attrs = e.attrs, .body = e.body });
        }
    }

    var devs: std.ArrayList(RawDev) = .empty;
    {
        var pos: usize = 0;
        while (scanElem(dev_part, pos, "device")) |e| {
            pos = e.end;
            const name = attr(e.attrs, "name") orelse continue;
            try devs.append(alloc, .{
                .name = name,
                .attrs = e.attrs,
                .body = e.body,
                .family = attr(e.attrs, "family"),
                .alias = attr(e.attrs, "alias"),
                .is_default = if (attr(e.attrs, "isDefault")) |v| std.mem.eql(u8, v, "true") else false,
            });
        }
    }

    var ctx = Ctx{ .alloc = alloc, .devs = devs.items, .shared = shared.items, .fam = fam, .def = undefined };
    parseDefaults(&ctx);

    for (ctx.devs) |*d| {
        if (d.is_default) {
            stats.skip_default += 1;
            continue;
        }

        // alias resolved one level only
        var target: ?*RawDev = null;
        if (d.alias) |al| target = ctx.find(al);

        // alias inherits target's family
        const eff_family = if (target) |t| (t.family orelse d.family) else d.family;
        if (eff_family == null or !std.mem.eql(u8, eff_family.?, fam.tag())) {
            stats.skip_family += 1;
            continue;
        }

        const lname = lower(alloc, d.name);

        // tbody: target's body, for inherited fields
        const tbody: []const u8 = if (target) |t| t.body else "";

        var routine = deviceRoutine(&ctx, d.body);
        if (routine == null and target != null) routine = deviceRoutine(&ctx, tbody);
        if (routine == null) routine = ctx.def.routine;
        const rt = routine orelse "";

        var sdid_addr: u64 = ctx.def.sdid_addr;
        if (elemAttr(d.body, "sdidAddress", "value")) |x| {
            sdid_addr = parseNum(x);
        } else if (target != null) {
            if (elemAttr(tbody, "sdidAddress", "value")) |x| sdid_addr = parseNum(x);
        }
        var dev_mask: u64 = ctx.def.sdid_mask;
        if (elemAttr(d.body, "sdidMask", "value")) |x| {
            dev_mask = parseNum(x);
        } else if (target != null) {
            if (elemAttr(tbody, "sdidMask", "value")) |x| dev_mask = parseNum(x);
        }

        const sbody: []const u8 = if (hasSdid(d.body)) d.body else if (target != null and hasSdid(tbody)) tbody else d.body;
        {
            var pos: usize = 0;
            while (scanElem(sbody, pos, "sdid")) |e| {
                pos = e.end;
                const val = if (attr(e.attrs, "value")) |x| parseNum(x) else continue;
                const m = if (attr(e.attrs, "mask")) |x| parseNum(x) else dev_mask;
                if (val == 0 and m == 0) continue; // default placeholder
                try sdid_w.print("    .{{ .name = \"{s}\", .family = .{s}, .sdid_addr = 0x{X}, .mask = 0x{X}, .sdid = 0x{X} }},\n", .{ lname, fam.enumName(), sdid_addr, m, val });
                switch (fam) {
                    .hcs08 => stats.sdid_hcs08 += 1,
                    .hcs12 => stats.sdid_hcs12 += 1,
                    .cfv1 => stats.sdid_cfv1 += 1,
                }
            }
        }

        const supported = switch (fam) {
            .hcs08 => inList(&SUP_HCS08, rt),
            .hcs12 => inList(&MMCV4, rt) or std.mem.eql(u8, rt, GMMC),
            .cfv1 => inList(&SUP_CFV1, rt),
        };
        if (!supported) {
            stats.skip_routine += 1;
            continue;
        }

        // alias with no own flash block uses target's geometry
        var gbody = d.body;
        {
            var fl: std.ArrayList(Block) = .empty;
            memBlocks(&ctx, d.body, "flash", &fl);
            if (fl.items.len == 0 and target != null) gbody = tbody;
        }

        var flashes: std.ArrayList(Block) = .empty;
        memBlocks(&ctx, gbody, "flash", &flashes);
        if (flashes.items.len == 0) {
            stats.skip_noflash += 1;
            stats.report("{s}: supported routine but no flash block; skipped geometry", .{lname});
            continue;
        }
        const fblk = flashes.items[0].attrs;
        const flash_base = if (attr(fblk, "registerAddress")) |x| parseNum(x) else ctx.def.flash_reg;
        // multiple flash controllers (differing registerAddress) can't share one flash_base -> excluded
        var multi_ctrl = false;
        for (flashes.items[1..]) |blk| {
            const r2 = if (attr(blk.attrs, "registerAddress")) |x| parseNum(x) else ctx.def.flash_reg;
            if (r2 != flash_base) {
                multi_ctrl = true;
                break;
            }
        }
        const sec = (if (attr(fblk, "securityAddress")) |x| parseNum(x) else ctx.def.flash_sec) + 0x0F;
        const page_size = if (attr(fblk, "sectorSize")) |x| parseNum(x) else ctx.def.sector;
        const write_align = if (attr(fblk, "alignment")) |x| parseNum(x) else ctx.def.alignv;
        const page_addr = if (attr(fblk, "pageAddress")) |x| parseNum(x) else 0;

        var wins_buf: [64]Win = undefined;
        var nwin: usize = 0;
        var banked_buf: [16][]const u8 = undefined;
        var nbank: usize = 0;
        var dropped_ppage: usize = 0;
        for (flashes.items) |blk| {
            var pos: usize = 0;
            while (scanElem(blk.body, pos, "memoryRange")) |e| {
                pos = e.end;
                if (rangeBanked(e.attrs)) {
                    if (nbank < banked_buf.len) {
                        banked_buf[nbank] = e.attrs;
                        nbank += 1;
                    }
                    continue;
                }
                const w = rangeStartEnd(e.attrs) orelse continue;
                // flat 16-bit map: PPAGE global windows (>=0x10000) unreachable by flat routines
                if ((fam == .hcs08 or fam == .hcs12) and w.start >= 0x10000) {
                    dropped_ppage += 1;
                    continue;
                }
                if (nwin < wins_buf.len) {
                    wins_buf[nwin] = w;
                    nwin += 1;
                }
            }
        }
        if (dropped_ppage > 0)
            stats.report("{s}: dropped {d} PPAGE global window(s) >=0x10000 (flat routine addresses only low 64K)", .{ lname, dropped_ppage });
        if (nbank > 0 and fam == .hcs08)
            stats.report("{s}: HCS08 banked (PPAGE) window not represented", .{lname});

        const wins = wins_buf[0..nwin];
        std.mem.sort(Win, wins, {}, struct {
            fn lt(_: void, a: Win, b: Win) bool {
                return a.start < b.start;
            }
        }.lt);
        var merged: [64]Win = undefined;
        var nm: usize = 0;
        for (wins) |w| {
            if (nm > 0 and w.start <= merged[nm - 1].end + 1) {
                if (w.end > merged[nm - 1].end) merged[nm - 1].end = w.end;
            } else {
                merged[nm] = w;
                nm += 1;
            }
        }
        if (nm == 0) {
            stats.skip_noflash += 1;
            stats.report("{s}: no addressable flash window; skipped geometry", .{lname});
            continue;
        }
        const too_many_windows = nm > 2;

        const flash_start = merged[0].start;
        const flash_end = merged[0].end;
        const has_w2 = nm >= 2;
        const flash_start2 = if (has_w2) merged[1].start else 0;
        const flash_end2 = if (has_w2) merged[1].end else 0;

        const is_gmmc = fam == .hcs12 and std.mem.eql(u8, rt, GMMC);
        const hcs12_routine: []const u8 = if (is_gmmc) "gmmc_ftmrg" else "mmcv4_fts";
        var have_paged = false;
        var pg_first: u64 = 0;
        var pg_last: u64 = 0;
        if (fam == .hcs12 and !is_gmmc and nbank > 0) {
            var saw_reset = false;
            for (banked_buf[0..nbank]) |b| {
                const pr = attr(b, "pageReset");
                const pe = attr(b, "pageEnd");
                if (pr != null and pe != null) {
                    saw_reset = true;
                    const prv = parseNum(pr.?);
                    const pev = parseNum(pe.?);
                    // paged only if pages exist below the top two fixed windows; else they cover all -> null
                    if (prv + 2 <= pev) {
                        have_paged = true;
                        pg_first = prv;
                        pg_last = pev;
                    }
                    break;
                }
            }
            if (!saw_reset)
                stats.report("{s}: mmcv4 banked window has no pageReset -> paged null (only the two fixed windows)", .{lname});
        }

        var rams: std.ArrayList(Block) = .empty;
        memBlocks(&ctx, gbody, "ram", &rams);
        var ram_start: u64 = 0x80;
        var ram_end: u64 = 0x27F;
        if (rams.items.len > 0) {
            if (scanElem(rams.items[0].body, 0, "memoryRange")) |e| {
                if (rangeStartEnd(e.attrs)) |w| {
                    ram_start = w.start;
                    ram_end = w.end;
                }
            }
            var total_ranges: usize = 0;
            for (rams.items) |rb| {
                var pos: usize = 0;
                while (scanElem(rb.body, pos, "memoryRange")) |e| {
                    total_ranges += 1;
                    pos = e.end;
                }
            }
            if (rams.items.len > 1 or total_ranges > 1)
                stats.report("{s}: multiple RAM ranges -> used the first", .{lname});
        }

        var watchdog: u64 = 0;
        if (fam == .hcs12) {
            watchdog = ctx.def.copctl;
            if (elemAttr(d.body, "copctlAddress", "value")) |x| {
                watchdog = parseNum(x);
            } else if (target != null) {
                if (elemAttr(tbody, "copctlAddress", "value")) |x| watchdog = parseNum(x);
            }
        } else if (fam == .cfv1) {
            watchdog = ctx.def.sopt;
            if (elemAttr(d.body, "soptAddress", "value")) |x| {
                watchdog = parseNum(x);
            } else if (target != null) {
                if (elemAttr(tbody, "soptAddress", "value")) |x| watchdog = parseNum(x);
            }
        }

        var have_ee = false;
        var ee_base: u64 = 0;
        var ee_size: u64 = 0;
        var ee_sector: u64 = 0;
        var ee_align: u64 = 0;
        if (is_gmmc) {
            var ees: std.ArrayList(Block) = .empty;
            memBlocks(&ctx, gbody, "eeprom", &ees);
            for (ees.items) |eb| {
                const rr = elemAttr(eb.body, "flashProgramRef", "ref");
                if (rr != null and std.mem.eql(u8, rr.?, GMMC_EE)) {
                    if (scanElem(eb.body, 0, "memoryRange")) |e| {
                        if (rangeStartEnd(e.attrs)) |w| {
                            ee_base = w.start;
                            ee_size = w.end - w.start + 1;
                            // XML sectorSize on S12G D-flash = program phrase (4B), NOT erase sector.
                            // True erase sector = 256B (MC9S12GRMV1; cmd 0x12 erases one).
                            ee_sector = 0x100;
                            ee_align = if (attr(eb.attrs, "alignment")) |x| parseNum(x) else 0;
                            have_ee = true;
                        }
                    }
                    break;
                }
            }
        }

        // Excluded from geometry (SDID entry kept): flash not faithfully representable would risk
        // touching gaps/missing flash. User can still identify the part and pass --flash-base.
        const exclude_geometry =
            multi_ctrl or too_many_windows or dropped_ppage > 0 or (nbank > 0 and fam == .hcs08);
        if (exclude_geometry) {
            stats.report("{s}: geometry not faithfully representable -> excluded from table (kept in sdid)", .{lname});
            continue;
        }

        try table_w.print("    .{{ .name = \"{s}\", .family = .{s}", .{ lname, fam.enumName() });
        try table_w.print(", .flash_base = 0x{X}", .{flash_base});
        try table_w.print(", .ram_start = 0x{X}, .ram_end = 0x{X}", .{ ram_start, ram_end });
        try table_w.print(", .flash_start = 0x{X}, .flash_end = 0x{X}", .{ flash_start, flash_end });
        if (has_w2) {
            try table_w.print(", .flash_start2 = 0x{X}, .flash_end2 = 0x{X}", .{ flash_start2, flash_end2 });
        } else {
            try table_w.print(", .flash_start2 = null, .flash_end2 = 0", .{});
        }
        try table_w.print(", .page_size = {d}, .write_align = {d}", .{ page_size, write_align });
        try table_w.print(", .security_addr = 0x{X}, .watchdog_addr = 0x{X}", .{ sec, watchdog });
        try table_w.print(", .hcs12_routine = .{s}", .{hcs12_routine});
        if (have_paged) {
            try table_w.print(", .paged = .{{ .ppage_addr = 0x{X}, .page_first = 0x{X}, .page_last = 0x{X} }}", .{ page_addr, pg_first, pg_last });
        } else {
            try table_w.print(", .paged = null", .{});
        }
        if (have_ee) {
            try table_w.print(", .eeprom = .{{ .base_global = 0x{X}, .size = 0x{X}, .sector = {d}, .align_bytes = {d} }}", .{ ee_base, ee_size, ee_sector, ee_align });
        } else {
            try table_w.print(", .eeprom = null", .{});
        }
        try table_w.print(" }},\n", .{});

        switch (fam) {
            .hcs08 => stats.table_hcs08 += 1,
            .hcs12 => stats.table_hcs12 += 1,
            .cfv1 => stats.table_cfv1 += 1,
        }
    }
}

pub fn main(init: std.process.Init) !void {
    const arena = init.arena.allocator();
    const io = init.io;
    const args = try init.minimal.args.toSlice(arena);
    if (args.len < 5) fatal("usage: gen_devices <hcs08.xml> <hcs12.xml> <cfv1.xml> <out.zig>", .{});

    const inputs = [_]struct { path: []const u8, fam: Family }{
        .{ .path = args[1], .fam = .hcs08 },
        .{ .path = args[2], .fam = .hcs12 },
        .{ .path = args[3], .fam = .cfv1 },
    };
    const out_path = args[4];

    var table_buf: std.Io.Writer.Allocating = .init(arena);
    var sdid_buf: std.Io.Writer.Allocating = .init(arena);
    var stats: Stats = .{};

    for (inputs) |inp| {
        std.debug.print("== {s} ({s}) ==\n", .{ inp.path, inp.fam.tag() });
        const bytes = std.Io.Dir.cwd().readFileAlloc(io, inp.path, arena, .limited(32 * 1024 * 1024)) catch |err|
            fatal("cannot read {s}: {s}", .{ inp.path, @errorName(err) });
        try processFile(arena, bytes, inp.fam, &table_buf.writer, &sdid_buf.writer, &stats);
    }

    var out: std.Io.Writer.Allocating = .init(arena);
    const w = &out.writer;
    try w.writeAll("// AUTO-GENERATED from vendor/usbdm-devices/*.xml - do not edit.\n");
    try w.writeAll("const t = @import(\"device_types\");\n\n");
    try w.writeAll("pub const table = [_]t.Device{\n");
    try w.writeAll(table_buf.written());
    try w.writeAll("};\n\n");
    try w.writeAll("pub const sdid_table = [_]t.SdidEntry{\n");
    try w.writeAll(sdid_buf.written());
    try w.writeAll("};\n");

    std.Io.Dir.cwd().writeFile(io, .{ .sub_path = out_path, .data = out.written() }) catch |err|
        fatal("cannot write {s}: {s}", .{ out_path, @errorName(err) });

    std.debug.print(
        \\
        \\== SUMMARY ==
        \\table: hcs08={d} hcs12={d} cfv1={d} total={d}
        \\sdid:  hcs08={d} hcs12={d} cfv1={d} total={d}
        \\skipped: default={d} family={d} routine={d} noflash={d}
        \\reports: {d}
        \\wrote {s}
        \\
    , .{
        stats.table_hcs08,                                        stats.table_hcs12,                                     stats.table_cfv1,
        stats.table_hcs08 + stats.table_hcs12 + stats.table_cfv1, stats.sdid_hcs08,                                      stats.sdid_hcs12,
        stats.sdid_cfv1,                                          stats.sdid_hcs08 + stats.sdid_hcs12 + stats.sdid_cfv1, stats.skip_default,
        stats.skip_family,                                        stats.skip_routine,                                    stats.skip_noflash,
        stats.reports,                                            out_path,
    });
}
