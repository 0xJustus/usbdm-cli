//! Device geometry table + SDID map, generated at build time from the vendored
//! USBDM device XML (tools/gen_devices.zig). Family generics are hand-defined here.

const std = @import("std");
const types = @import("device_types");
const gen = @import("device_gen");

pub const Family = types.Family;
pub const Hcs12Routine = types.Hcs12Routine;
pub const Paged = types.Paged;
pub const Eeprom = types.Eeprom;
pub const Device = types.Device;
pub const SdidEntry = types.SdidEntry;
pub const eeprom_linear_flag = types.eeprom_linear_flag;
pub const unsecuredSecurityByte = types.unsecuredSecurityByte;
pub const securedSecurityByte = types.securedSecurityByte;

/// Geometry for parts whose flash is faithfully representable (generated).
pub const table = gen.table;
/// SDID map for auto-detect: every parseable part, including some paged/dual-
/// controller ones that carry no geometry entry (identify-only).
pub const sdid_table = gen.sdid_table;

/// Family generics (not in the XML): used when no --part is named, so a
/// non-HCS08 target isn't handed HCS08 geometry.
pub const generics = [_]Device{
    .{ .name = "hcs08-generic", .ram_start = 0x80, .ram_end = 0x27F, .flash_start = 0x8000, .flash_end = 0xFFFF },
    .{ .name = "hcs12-generic", .family = .hcs12, .flash_base = 0x0100, .ram_start = 0x0800, .ram_end = 0x0FFF, .flash_start = 0x4000, .flash_end = 0x7FFF, .flash_start2 = 0xC000, .flash_end2 = 0xFFFF, .page_size = 512, .write_align = 2, .security_addr = 0xFF0F, .watchdog_addr = 0x003C, .hcs12_routine = .mmcv4_fts },
    .{ .name = "cfv1-generic", .family = .cfv1, .flash_base = 0xFF9820, .ram_start = 0x00800000, .ram_end = 0x00801FFF, .flash_start = 0x0, .flash_end = 0x1FFFF, .page_size = 1024, .write_align = 4, .security_addr = 0x40F, .watchdog_addr = 0xFF9802 },
};

pub fn lookup(name: []const u8) ?Device {
    for (table) |d| if (std.ascii.eqlIgnoreCase(name, d.name)) return d;
    for (generics) |d| if (std.ascii.eqlIgnoreCase(name, d.name)) return d;
    return null;
}

/// Best-effort generic device for a family.
pub fn genericFor(family: Family) Device {
    return switch (family) {
        .hcs08 => generics[0],
        .hcs12 => generics[1],
        .cfv1 => generics[2],
    };
}

/// SDID (System Device ID) register address probed for a family - the address
/// the overwhelming majority of that family's parts share.
pub fn familySdidAddr(family: Family) u32 {
    return switch (family) {
        .hcs08 => 0x1806,
        .hcs12 => 0x001A,
        .cfv1 => 0xFF9806,
    };
}

/// Match a raw SDID register read to a known part: the first entry (for this
/// family) whose (value & mask) == sdid. null if unrecognized.
pub fn identify(family: Family, value: u32) ?SdidEntry {
    const v: u16 = @truncate(value);
    for (sdid_table) |e| {
        if (e.family != family) continue;
        if (v & e.mask == e.sdid) return e;
    }
    return null;
}

test "lookup finds known parts and the generic fallback" {
    try std.testing.expectEqual(@as(u32, 0x1820), lookup("MC9S08JM60").?.flash_base);
    try std.testing.expectEqual(@as(u32, 0xFFBF), lookup("mc9s08qg8").?.security_addr);
    try std.testing.expectEqual(@as(?Device, null), lookup("stm32f4"));
    try std.testing.expectEqual(Family.hcs12, lookup("hcs12-generic").?.family);
}

test "genericFor gives family-correct geometry (not HCS08 for HCS12/CFV1)" {
    try std.testing.expectEqual(Family.hcs08, genericFor(.hcs08).family);
    const h12 = genericFor(.hcs12);
    try std.testing.expectEqual(Family.hcs12, h12.family);
    try std.testing.expectEqual(@as(u32, 0x0100), h12.flash_base); // S12 controller, not 0x1820
    try std.testing.expectEqual(@as(u32, 2), h12.write_align);
    const cfv1 = genericFor(.cfv1);
    try std.testing.expectEqual(Family.cfv1, cfv1.family);
    try std.testing.expectEqual(@as(u32, 0xFF9820), cfv1.flash_base);
    try std.testing.expectEqual(@as(u32, 0x00800000), cfv1.ram_start);
}

test "generated HCS08 table: geometry and routine-blob selection" {
    // JM parts start RAM at 0xB0 -> 0xB0 blob; the rest -> 0x80 blob.
    try std.testing.expectEqual(@as(u16, 0xB0), lookup("mc9s08jm8").?.routineLoadAddress());
    try std.testing.expectEqual(@as(u16, 0x80), lookup("mc9s08qe16").?.routineLoadAddress());
    try std.testing.expectEqual(@as(u32, 0xC000), lookup("mc9s08qe16").?.flash_start);
    try std.testing.expectEqual(@as(u32, 0x8000), lookup("mc9s08sh32").?.flash_start);
    try std.testing.expectEqual(@as(u32, 0x1820), lookup("mc9s08sh4").?.flash_base);
    // GC32 mirrors C32.
    const gc32 = lookup("mc9s12gc32").?;
    try std.testing.expectEqual(Family.hcs12, gc32.family);
    try std.testing.expectEqual(@as(u32, 0x003C), gc32.watchdog_addr);
    try std.testing.expectEqual(@as(u32, 0xFF0F), gc32.security_addr);
    const g240 = lookup("mc9s12g240").?;
    try std.testing.expectEqual(Hcs12Routine.gmmc_ftmrg, g240.hcs12_routine);
    try std.testing.expectEqual(@as(u32, 8), g240.write_align);
    try std.testing.expectEqual(@as(u32, 0x1400), g240.ram_start);
    try std.testing.expectEqual(@as(u32, 0x100), g240.eeprom.?.sector); // erase sector, not the XML phrase size
}

test "generated device table: known parts resolve with sane geometry" {
    const js16 = lookup("mc9s08js16").?; // the FZ0622C's own MCU
    try std.testing.expectEqual(Family.hcs08, js16.family);
    try std.testing.expectEqual(@as(u32, 0x4000), js16.flashLen()); // 16 KB
    try std.testing.expectEqual(@as(u16, 0x80), js16.routineLoadAddress());
    // AC48: single 48 KB window; AC60: split (low sliver + main).
    try std.testing.expectEqual(@as(u32, 0xC000), lookup("mc9s08ac48").?.flashLen());
    const ac60 = lookup("mc9s08ac60").?;
    try std.testing.expect(ac60.inFlash(0x870) and ac60.inFlash(0xFFFF));
    try std.testing.expect(!ac60.inFlash(0x1800)); // register/gap window
    // Classic CFV1 parts carry the 0xFF9820 controller + 0x00800000 RAM.
    const jm64 = lookup("mcf51jm64").?;
    try std.testing.expectEqual(Family.cfv1, jm64.family);
    try std.testing.expectEqual(@as(u32, 0xFF9820), jm64.flash_base);
    try std.testing.expectEqual(@as(u32, 0x00800000), jm64.ram_start);
    try std.testing.expectEqual(@as(u32, 0x10000), jm64.flashLen()); // 64 KB
    // Every generated entry's flash/RAM extent must be non-inverted and, for a
    // split part, a valid 2nd range. Catches any parser regression across the table.
    for (table) |d| {
        try std.testing.expect(d.flash_end >= d.flash_start);
        try std.testing.expect(d.ram_end >= d.ram_start);
        if (d.flash_start2 != null) try std.testing.expect(d.flash_end2 >= d.flash_start2.?);
    }
}

test "flash length and range helpers" {
    const d = lookup("mc9s08jm16").?;
    try std.testing.expectEqual(@as(u32, 0x4000), d.flashLen());
    try std.testing.expect(d.inFlash(0xC000));
    try std.testing.expect(d.inFlash(0xFFFF));
    try std.testing.expect(!d.inFlash(0xBFFF));
}

test "split-flash parts cover both ranges but not the gap" {
    const jm60 = lookup("mc9s08jm60").?;
    try std.testing.expect(jm60.inFlash(0x10B0)); // low sliver start
    try std.testing.expect(jm60.inFlash(0x17FF)); // low sliver end
    try std.testing.expect(!jm60.inFlash(0x1800)); // I/O + register window (gap)
    try std.testing.expect(!jm60.inFlash(0x195F)); // gap
    try std.testing.expect(jm60.inFlash(0x1960)); // main block start
    try std.testing.expect(jm60.inFlash(0xFFFF)); // main block end
    try std.testing.expectEqual(@as(u32, 0x750 + 0xE6A0), jm60.flashLen());
}

test "cfv1 parts carry the 32-bit controller/watchdog and flat flash range" {
    const qe = lookup("mcf51qe128").?;
    try std.testing.expectEqual(Family.cfv1, qe.family);
    try std.testing.expectEqual(@as(u32, 0xFF9820), qe.flash_base);
    try std.testing.expectEqual(@as(u32, 0xFF9802), qe.watchdog_addr);
    try std.testing.expectEqual(@as(u32, 0x00800000), qe.ram_start);
    try std.testing.expectEqual(@as(u32, 4), qe.write_align);
    try std.testing.expect(qe.inFlash(0x0000) and qe.inFlash(0x1FFFF));
    try std.testing.expectEqual(@as(u32, 0x40000), lookup("mcf51ac256b").?.flashLen()); // 256K
}

test "flashRanges enumerates fixed windows, banked pages, and never overflows" {
    var buf: [258]Device.Range = undefined;
    const jm60 = lookup("mc9s08jm60").?;
    const r1 = jm60.flashRanges(&buf);
    try std.testing.expectEqual(@as(usize, 2), r1.len);
    try std.testing.expectEqual(@as(u32, 0x10B0), r1[0].start);
    // Paged part: one window per page (0x30..0x3F = 16), each 16 KB at 0x?8000.
    const dp = lookup("mc9s12dp256b").?;
    const r2 = dp.flashRanges(&buf);
    try std.testing.expectEqual(@as(usize, 16), r2.len);
    try std.testing.expectEqual(@as(u32, 0x308000), r2[0].start);
    try std.testing.expectEqual(@as(u32, 0x3F8000), r2[15].start);
    try std.testing.expectEqual(@as(u32, 0x4000), r2[15].size);
    const big = Device{ .name = "x", .family = .hcs12, .flash_start = 0x4000, .paged = .{ .ppage_addr = 0x30, .page_first = 0x00, .page_last = 0xFF } };
    var small: [8]Device.Range = undefined;
    try std.testing.expectEqual(@as(usize, 8), big.flashRanges(&small).len);
}

test "paged HCS12 part: banked-window addresses and page-bounded spans" {
    const dp = lookup("mc9s12dp256b").?;
    try std.testing.expect(dp.paged != null);
    try std.testing.expect(dp.inFlash(0x4000) and dp.inFlash(0xFFFF));
    try std.testing.expect(dp.inFlash(0x308000)); // page 0x30, window start
    try std.testing.expect(dp.inFlash(0x3FBFFF)); // page 0x3F, window end
    try std.testing.expect(!dp.inFlash(0x30C000)); // above the banked window
    try std.testing.expect(!dp.inFlash(0x2F8000)); // page below page_first
    try std.testing.expect(dp.spanInFlash(0x308000, 0x30BFFF));
    try std.testing.expect(!dp.spanInFlash(0x30BFFF, 0x318000));
    try std.testing.expectEqual(@as(u32, 16 * 0x4000), dp.flashLen());
}

test "SDID auto-detect: identify maps a device-id read to a part" {
    // js16 (the FZ0622C MCU) is in the SDID map at 0x1806; identify resolves it.
    var found_js16 = false;
    for (sdid_table) |e| {
        if (std.mem.eql(u8, e.name, "mc9s08js16")) {
            found_js16 = true;
            try std.testing.expectEqual(@as(u32, 0x1806), e.sdid_addr);
            const m = identify(.hcs08, e.sdid) orelse return error.NotFound;
            try std.testing.expectEqual(Family.hcs08, m.family);
            try std.testing.expect(e.sdid & m.mask == m.sdid);
        }
    }
    try std.testing.expect(found_js16);
    try std.testing.expectEqual(@as(u32, 0x1806), familySdidAddr(.hcs08));
    try std.testing.expectEqual(@as(u32, 0x001A), familySdidAddr(.hcs12));
    try std.testing.expectEqual(@as(u32, 0xFF9806), familySdidAddr(.cfv1));
    // A masked S12G id resolves to a G-family part (g192/g240 share 0xF08x, so
    // SDID alone can't tell them apart - first match wins, both are "mc9s12g").
    const g = identify(.hcs12, 0xF085) orelse return error.NotFound;
    try std.testing.expect(0xF085 & g.mask == g.sdid);
    try std.testing.expect(std.mem.startsWith(u8, g.name, "mc9s12g"));
    try std.testing.expectEqual(@as(?SdidEntry, null), identify(.hcs08, 0x7FF));
}
