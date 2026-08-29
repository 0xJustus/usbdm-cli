//! Device-table types, shared by device.zig and the build-time generator
//! (tools/gen_devices.zig -> device_gen.zig) so both agree on the layout.

const std = @import("std");

/// Flash family - selects the RAM routine + ABI (HCS08 small-code, HCS12 large-code).
pub const Family = enum { hcs08, hcs12, cfv1 };

/// HCS12 routine: classic S12 = MMCV4/FTS, S12G = GMMC/FTMRG.
pub const Hcs12Routine = enum { mmcv4_fts, gmmc_ftmrg };

/// PPAGE-banked HCS12 flash. Window always 0x8000-0xBFFF; pages
/// page_first..page_last map globally as (page << 16) | offset (routine sets
/// PPAGE).
pub const Paged = struct { ppage_addr: u16, page_first: u8, page_last: u8 };

/// EEPROM/D-flash geometry (only the S12G GMMC/FTMRG family). `base_global` =
/// global D-flash address (driver sets bit 31 to route to D-flash commands);
/// `sector` = erase granularity, `align_bytes` = program alignment.
pub const Eeprom = struct { base_global: u32, size: u32, sector: u32, align_bytes: u32 };

/// OR'd into an EEPROM address so GMMC treats it as a global "linear" address
/// (-> D-flash commands 0x11/0x12, not PPAGE/P-flash).
pub const eeprom_linear_flag: u32 = 0x8000_0000;

/// One SDID (System Device ID) match: read `sdid_addr` over BDM, and the part is
/// `name` when (read & mask) == sdid.
pub const SdidEntry = struct {
    name: []const u8,
    family: Family,
    sdid_addr: u32,
    mask: u16,
    sdid: u16,
};

pub const Device = struct {
    name: []const u8,
    family: Family = .hcs08,
    /// FLASH controller base (FCDIV = base+0); u32 for CFV1's high map.
    flash_base: u32 = 0x1820,
    /// Inclusive RAM extent (small-code routine: code at top, params/buffer at bottom).
    ram_start: u32 = 0x0080,
    ram_end: u32 = 0x027F,
    /// Inclusive programmable flash range.
    flash_start: u32,
    flash_end: u32 = 0xFFFF,
    /// Optional 2nd flash range: split-flash parts straddle the I/O + register
    /// window (e.g. JM60's low sliver below 0x1800); a single range would accept
    /// the gap. null when contiguous.
    flash_start2: ?u32 = null,
    flash_end2: u32 = 0,
    /// Erase page size in bytes.
    page_size: u32 = 512,
    /// Program/verify write alignment (1 = HCS08 byte, 2 = S12 MMCV4/FTS word);
    /// driver rounds ranges to this and 0xFF-pads the remainder.
    write_align: u32 = 1,
    /// NVOPT/FSEC security byte address.
    security_addr: u32 = 0xFFBF,
    /// Watchdog/COP control register (routine disables the watchdog). u32 for
    /// CFV1 SOPT 0xFF9802; HCS12 copctl 0x003C; 0 = none.
    watchdog_addr: u32 = 0,
    /// HCS12 routine blob (ignored for HCS08).
    hcs12_routine: Hcs12Routine = .mmcv4_fts,
    /// PPAGE-banked window (HCS12 parts beyond the two fixed windows); null = none.
    paged: ?Paged = null,
    /// EEPROM/D-flash geometry (when the routine can program it).
    eeprom: ?Eeprom = null,

    pub fn flashLen(self: Device) u32 {
        if (self.paged) |p| return (@as(u32, p.page_last - p.page_first) + 1) * 0x4000;
        var n = self.flash_end - self.flash_start + 1;
        if (self.flash_start2) |s2| n += self.flash_end2 - s2 + 1;
        return n;
    }
    pub fn inFlash(self: Device, addr: u32) bool {
        if (addr >= self.flash_start and addr <= self.flash_end) return true;
        if (self.flash_start2) |s2| {
            if (addr >= s2 and addr <= self.flash_end2) return true;
        }
        if (self.paged) |p| {
            const pg = addr >> 16;
            const off = addr & 0xFFFF;
            if (off >= 0x8000 and off <= 0xBFFF and pg >= p.page_first and pg <= p.page_last) return true;
        }
        return false;
    }

    pub const Range = struct { start: u32, size: u32 };

    /// Enumerate programmable flash ranges into `buf`: fixed window(s), or on a
    /// paged part each page's 0x8000-0xBFFF window (subsuming the fixed windows).
    /// Truncates rather than overflowing `buf`.
    pub fn flashRanges(self: Device, buf: []Range) []Range {
        var n: usize = 0;
        if (self.paged) |p| {
            var pg: u32 = p.page_first;
            while (pg <= p.page_last and n < buf.len) : (pg += 1) {
                buf[n] = .{ .start = (pg << 16) | 0x8000, .size = 0x4000 };
                n += 1;
            }
        } else {
            if (n < buf.len) {
                buf[n] = .{ .start = self.flash_start, .size = self.flash_end - self.flash_start + 1 };
                n += 1;
            }
            if (self.flash_start2) |s2| {
                if (n < buf.len) {
                    buf[n] = .{ .start = s2, .size = self.flash_end2 - s2 + 1 };
                    n += 1;
                }
            }
        }
        return buf[0..n];
    }

    /// True only if the whole span [addr, end] lies in a SINGLE range. Rejects a
    /// span straddling the inter-range gap or crossing a page boundary.
    pub fn spanInFlash(self: Device, addr: u32, end: u32) bool {
        if (addr > end) return false;
        if (addr >= self.flash_start and end <= self.flash_end) return true;
        if (self.flash_start2) |s2| {
            if (addr >= s2 and end <= self.flash_end2) return true;
        }
        if (self.paged) |p| {
            const pg = addr >> 16;
            if (pg == end >> 16 and pg >= p.page_first and pg <= p.page_last and
                (addr & 0xFFFF) >= 0x8000 and (end & 0xFFFF) <= 0xBFFF) return true;
        }
        return false;
    }

    /// Small-code blob load address: 0x80, or 0xB0 when RAM starts above 0x80.
    pub fn routineLoadAddress(self: Device) u16 {
        return if (self.ram_start > 0x80) 0xB0 else 0x80;
    }
};

/// Security-byte value leaving the part UNSECURED. All families: SEC = 0b10 in
/// the low two bits (NVOPT/FSEC/CFV1 config byte), other bits erased (1); 0xFF
/// = SECURED.
pub fn unsecuredSecurityByte(family: Family) u8 {
    return switch (family) {
        .hcs08, .hcs12, .cfv1 => 0xFE,
    };
}

/// SECURED security-byte value (SEC field = 0b00).
pub fn securedSecurityByte(family: Family) u8 {
    return switch (family) {
        .hcs08, .hcs12, .cfv1 => 0xFC,
    };
}
