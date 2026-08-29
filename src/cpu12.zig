//! Functional CPU12 (HCS12, big-endian) emulator + S12 FTS/FTMRG FLASH model:
//! runs the vendored flash-routine binary to cross-check LargeDriver's ABI. Refs: CPU12RM, S12 FTS block guide.

const std = @import("std");

// CCR masks - CPU12 layout (CPU12RM Fig 2-1), not HCS08's.
const C: u8 = 0x01;
const V: u8 = 0x02;
const Z: u8 = 0x04;
const N: u8 = 0x08;
const I: u8 = 0x10;
const H: u8 = 0x20;
const Xm: u8 = 0x40;
const S: u8 = 0x80;

// S12 FTS (MMCV4) register offsets from base.
const FCLKDIV = 0x00;
const FSEC = 0x01;
const FPROT = 0x04;
const FSTAT = 0x05;
const FCMD = 0x06;
const CBEIF: u8 = 0x80; // command buffer empty
const CCIF: u8 = 0x40; // command complete
const PVIOL: u8 = 0x20; // protection violation
const ACCERR: u8 = 0x10; // access error
const BLANK: u8 = 0x04; // array verified erased (erase-verify result)
const CMD_ERASE_VERIFY: u8 = 0x05;
const CMD_WORD_PROG: u8 = 0x20;
const CMD_SECTOR_ERASE: u8 = 0x40;
const CMD_MASS_ERASE: u8 = 0x41;

// S12 FTMRG (GMMC) registers (MC9S12GRMV1 ch.21).
// Unlike FTS: 6-word FCCOB interface (not FADDR/FDATA+FCMD), FSTAT@0x06, CCIF@0x80.
const G_FCLKDIV = 0x00;
const G_FSEC = 0x01;
const G_FCCOBIX = 0x02;
const G_FSTAT = 0x06;
const G_FPROT = 0x08;
const G_DFPROT = 0x09;
const G_FCCOBHI = 0x0A;
const G_FCCOBLO = 0x0B;
const G_CCIF: u8 = 0x80; // command complete; write 1 to launch
const G_ACCERR: u8 = 0x20;
const G_FPVIOL: u8 = 0x10;
// FTMRG command codes (FCCOB0 hi byte).
const G_CMD_PROGRAM: u8 = 0x06; // Program P-Flash (up to 4 data words / 8-byte phrase)
const G_CMD_ERASE_ALL: u8 = 0x08; // Erase All Blocks (P-flash + D-flash)
const G_CMD_ERASE_SECTOR: u8 = 0x0A; // Erase P-Flash Sector (512 bytes)
const G_CMD_PROGRAM_D: u8 = 0x11; // Program D-Flash / EEPROM (1-4 data words, word-aligned)
const G_CMD_ERASE_SECTOR_D: u8 = 0x12;
// D-flash (EEPROM) global 0x000400..0x0013FF (4 KiB); routine routes any
// FCCOB global < 0x008000 to D-flash. HW erase sector 256 B.
const G_DFLASH_BASE: u32 = 0x000400;
const G_DFLASH_SIZE: u32 = 0x1000;
const G_DFLASH_SECTOR: u32 = 0x100;
// PPAGE: standalone MMC reg (not in flash block); global = (PPAGE<<14)|(cpuAddr&0x3FFF)
// for the 0x8000-0xBFFF window.
const G_PPAGE_ADDR: u16 = 0x0015;
const G_PFLASH_SIZE: u32 = 0x40000; // 18-bit P-flash space (0x00000..0x3FFFF)

const FlashKind = enum { fts, ftmrg };

/// BDMSTS.BDMACT: set when self-halted (BGND); 0x40 matches ramflash's poll mask.
pub const BDMACT: u8 = 0x40;

pub const Error = error{ UnimplementedOpcode, UnimplementedPostbyte, Runaway } || std.mem.Allocator.Error;

const M = enum { imm, dir, ext, idx };

// Classic S12 DBG module (DBGV1) - modeled just enough to honor a tagged PC
// breakpoint armed via hwbreak.hcs12. Registers live in the normal register map,
// so they land in self.mem; step() reads them to decide whether to break.
const DBG_DBGC1 = 0x20;
const DBG_DBGSC = 0x21;
const DBG_DBGC2 = 0x28;
const DBG_DBGCAH = 0x2B;
const DBG_DBGCAL = 0x2C;
const DBG_DBGCBH = 0x2E;
const DBG_DBGCBL = 0x2F;
const DBGC1_ARMED = 0x80 | 0x40 | 0x08; // DBGEN | ARM | DBGBRK
const DBGC2_BDM = 0x20; // break routes to background (not SWI)

pub const Cpu = struct {
    gpa: std.mem.Allocator,
    mem: []u8,
    a: u8 = 0,
    b: u8 = 0,
    x: u16 = 0,
    y: u16 = 0,
    sp: u16 = 0x0FFF, // overwritten by routine's LEAS,PCR
    pc: u16 = 0,
    ccr: u8 = S | Xm | I,
    halted: bool = false,

    ctrl_base: u32,
    flash_lo: u32,
    flash_hi: u32,
    fstat: u8 = CBEIF | CCIF,
    fcmd: u8 = 0,
    fclkdiv: u8 = 0,
    fclkdiv_written: bool = false,
    cmd_armed: bool = false, // FCMD buffered, awaiting CBEIF launch
    latched: bool = false, // addr/data latched via flash-array write
    latch_addr: u16 = 0,
    latch_data: u16 = 0,
    // routine also erases via FADDR/FDATA with FTSTMOD test bit set (not a flash-array write)
    ftstmod: u8 = 0,
    faddr: u16 = 0,
    fdata: u16 = 0,

    // FTMRG (GMMC) mode: a second, incompatible controller. Flash lives in `pflash`
    // (18-bit global) via paged windows; commands go through the FCCOB array.
    kind: FlashKind = .fts,
    pflash: []u8 = &[_]u8{},
    dflash: []u8 = &[_]u8{}, // D-flash/EEPROM (ftmrg), global 0x000400..0x0013FF
    ppage: u8 = 0,
    ccobix: u8 = 0,
    ccob_max: u8 = 0, // highest FCCOBIX since last launch (word count)
    ccob: [6]u16 = [_]u16{0} ** 6,

    bad_opcode: u8 = 0,
    bad_pc: u16 = 0,

    pub fn init(gpa: std.mem.Allocator, ctrl_base: u32, flash_lo: u32, flash_hi: u32) Error!Cpu {
        const mem = try gpa.alloc(u8, 0x10000);
        @memset(mem, 0);
        const lo: usize = flash_lo;
        const end: usize = @min(@as(usize, flash_hi) + 1, mem.len);
        if (lo < end) @memset(mem[lo..end], 0xFF); // erased flash reads 0xFF
        return .{ .gpa = gpa, .mem = mem, .ctrl_base = ctrl_base, .flash_lo = flash_lo, .flash_hi = flash_hi };
    }
    /// FTMRG (GMMC) CPU (S12G, e.g. mc9s12g240): flash is a 256 KiB global
    /// P-flash array via paged windows, no flat range.
    pub fn initGmmc(gpa: std.mem.Allocator, ctrl_base: u32) Error!Cpu {
        const mem = try gpa.alloc(u8, 0x10000);
        @memset(mem, 0);
        const pflash = try gpa.alloc(u8, G_PFLASH_SIZE);
        @memset(pflash, 0xFF);
        const dflash = try gpa.alloc(u8, G_DFLASH_SIZE);
        @memset(dflash, 0xFF);
        return .{
            .gpa = gpa,
            .mem = mem,
            .ctrl_base = ctrl_base,
            .flash_lo = 0, // unused in ftmrg (flash in pflash)
            .flash_hi = 0,
            .kind = .ftmrg,
            .pflash = pflash,
            .dflash = dflash,
        };
    }
    pub fn deinit(self: *Cpu) void {
        self.gpa.free(self.mem);
        if (self.pflash.len != 0) self.gpa.free(self.pflash);
        if (self.dflash.len != 0) self.gpa.free(self.dflash);
    }

    fn d(self: *const Cpu) u16 {
        return (@as(u16, self.a) << 8) | self.b;
    }
    fn setD(self: *Cpu, v: u16) void {
        self.a = @truncate(v >> 8);
        self.b = @truncate(v);
    }
    fn setFlag(self: *Cpu, mask: u8, on: bool) void {
        if (on) self.ccr |= mask else self.ccr &= ~mask;
    }
    fn setNZ8V0(self: *Cpu, r: u8) void {
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }
    fn setNZ16V0(self: *Cpu, r: u16) void {
        self.setFlag(N, r & 0x8000 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }

    fn inFlash(self: *const Cpu, a: u32) bool {
        return a >= self.flash_lo and a <= self.flash_hi;
    }
    fn inCtrl(self: *const Cpu, a: u32) bool {
        return a >= self.ctrl_base and a <= self.ctrl_base + 0x0F;
    }

    /// True when a classic-S12 DBG breakpoint is armed and PC matches comparator
    /// A (or B, when TRG selects A-or-B). Tagged, so checked before executing PC.
    fn dbgBreakMatch(self: *const Cpu) bool {
        if (self.kind != .fts) return false; // S12G/S12X DBG modeled elsewhere / not at all
        if (self.mem[DBG_DBGC1] & DBGC1_ARMED != DBGC1_ARMED) return false;
        if (self.mem[DBG_DBGC2] & DBGC2_BDM == 0) return false; // would SWI, not halt
        const a = (@as(u16, self.mem[DBG_DBGCAH]) << 8) | self.mem[DBG_DBGCAL];
        if (self.pc == a) return true;
        if (self.mem[DBG_DBGSC] & 0x0F == 0x01) { // TRG = A or B
            const b = (@as(u16, self.mem[DBG_DBGCBH]) << 8) | self.mem[DBG_DBGCBL];
            if (self.pc == b) return true;
        }
        return false;
    }

    /// Global P-flash addr for a paged-window CPU addr (FTMRG): (page<<14)|(addr&0x3FFF).
    /// Fixed windows hardwire PPAGE (0x0D@0x4000, 0x0F@0xC000); 0x8000-0xBFFF uses live PPAGE.
    fn globalOf(self: *const Cpu, addr: u16) u32 {
        const off: u32 = addr & 0x3FFF;
        const page: u32 = switch (addr & 0xC000) {
            0x4000 => 0x0D,
            0x8000 => self.ppage,
            0xC000 => 0x0F,
            else => 0x0C, // 0x0000-0x3FFF (registers/RAM below flash; unused)
        };
        return ((page << 14) | off) & (G_PFLASH_SIZE - 1);
    }
    fn isFlashWindowG(addr: u16) bool {
        return addr >= 0x4000; // 0x4000-0xFFFF are flash windows on the S12G
    }

    fn read8Gmmc(self: *Cpu, addr: u16) u8 {
        if (addr == G_PPAGE_ADDR) return self.ppage;
        const a: u32 = addr;
        if (self.inCtrl(a)) {
            return switch (a - self.ctrl_base) {
                G_FCLKDIV => 0x80, // FDIVLD set (host configured the clock)
                G_FSTAT => self.fstat,
                G_FCCOBIX => self.ccobix,
                G_FCCOBHI => @truncate(self.ccob[self.ccobix & 7] >> 8),
                G_FCCOBLO => @truncate(self.ccob[self.ccobix & 7]),
                G_FPROT, G_DFPROT => 0xFF, // fully unprotected
                G_FSEC => 0xFE, // unsecured
                else => 0,
            };
        }
        if (isFlashWindowG(addr)) {
            // D-flash (global < 0x8000) comes in via PPAGE=0 -> low 0x0000..0x3FFF;
            // resolve that back to the D-flash array.
            const g = self.globalOf(addr);
            if (inDflash(g)) return self.dflash[g - G_DFLASH_BASE];
            return self.pflash[g];
        }
        return self.mem[addr];
    }
    fn inDflash(g: u32) bool {
        return g >= G_DFLASH_BASE and g < G_DFLASH_BASE + G_DFLASH_SIZE;
    }
    fn write8Gmmc(self: *Cpu, addr: u16, val: u8) void {
        if (addr == G_PPAGE_ADDR) {
            self.ppage = val;
            return;
        }
        const a: u32 = addr;
        if (self.inCtrl(a)) {
            switch (a - self.ctrl_base) {
                G_FCLKDIV => self.fclkdiv = val,
                G_FCCOBIX => {
                    self.ccobix = val & 7;
                    if (self.ccobix > self.ccob_max) self.ccob_max = self.ccobix;
                },
                G_FCCOBHI => {
                    const i = self.ccobix & 7;
                    self.ccob[i] = (self.ccob[i] & 0x00FF) | (@as(u16, val) << 8);
                },
                G_FCCOBLO => {
                    const i = self.ccobix & 7;
                    self.ccob[i] = (self.ccob[i] & 0xFF00) | val;
                },
                G_FSTAT => {
                    if (val & G_ACCERR != 0) self.fstat &= ~G_ACCERR;
                    if (val & G_FPVIOL != 0) self.fstat &= ~G_FPVIOL;
                    if (val & G_CCIF != 0) self.launchGmmc(); // write 1 to CCIF = launch
                },
                else => {}, // FPROT/DFPROT/FSEC etc: inert
            }
            return;
        }
        if (isFlashWindowG(addr)) return; // flash isn't byte-writable; programs go via FCCOB
        self.mem[addr] = val;
    }
    /// Program `words` (FCCOB2..) into `arr` at `off`; AND semantics (clears 1->0). P- and D-flash.
    fn programWords(self: *Cpu, arr: []u8, off: u32, words: u32) void {
        var i: u32 = 0;
        while (i < words and 2 + i < self.ccob.len) : (i += 1) { // guard the FCCOB source
            const w = self.ccob[2 + i];
            const p = off + i * 2;
            if (p + 1 >= arr.len) break;
            arr[p] &= @truncate(w >> 8);
            arr[p + 1] &= @truncate(w);
        }
    }
    fn launchGmmc(self: *Cpu) void {
        const cmd: u8 = @truncate(self.ccob[0] >> 8);
        const raw_global: u32 = (@as(u32, self.ccob[0] & 0x03) << 16) | self.ccob[1];
        // word count = FCCOB 2..ccob_max; clamp to 4 data slots so a bare high
        // FCCOBIX write can't read past ccob[5].
        const words: u32 = if (self.ccob_max >= 2) @min(self.ccob_max - 1, 4) else 0;
        defer {
            self.ccob_max = 0;
        }
        switch (cmd) {
            G_CMD_PROGRAM => {
                const global = raw_global & (G_PFLASH_SIZE - 1);
                self.programWords(self.pflash, global, if (words == 0) 4 else words);
            },
            G_CMD_ERASE_SECTOR => {
                const s = (raw_global & (G_PFLASH_SIZE - 1)) & ~@as(u32, 0x1FF); // 512-byte sector
                var p = s;
                while (p < s + 0x200 and p < G_PFLASH_SIZE) : (p += 1) self.pflash[p] = 0xFF;
            },
            G_CMD_PROGRAM_D => {
                if (!inDflash(raw_global)) {
                    self.fstat = G_CCIF | G_ACCERR;
                    return;
                }
                self.programWords(self.dflash, raw_global - G_DFLASH_BASE, if (words == 0) 1 else words);
            },
            G_CMD_ERASE_SECTOR_D => {
                if (!inDflash(raw_global)) {
                    self.fstat = G_CCIF | G_ACCERR;
                    return;
                }
                const s = (raw_global - G_DFLASH_BASE) & ~(G_DFLASH_SECTOR - 1);
                var p = s;
                while (p < s + G_DFLASH_SECTOR and p < G_DFLASH_SIZE) : (p += 1) self.dflash[p] = 0xFF;
            },
            G_CMD_ERASE_ALL => {
                @memset(self.pflash, 0xFF); // Erase All Blocks erases P- and D-flash
                @memset(self.dflash, 0xFF);
            },
            else => {
                self.fstat = G_CCIF | G_ACCERR;
                return;
            },
        }
        self.fstat = G_CCIF; // complete, MGSTAT=00 (success)
    }

    fn read8(self: *Cpu, addr: u16) u8 {
        if (self.kind == .ftmrg) return self.read8Gmmc(addr);
        const a: u32 = addr;
        if (self.inCtrl(a)) {
            return switch (a - self.ctrl_base) {
                FCLKDIV => 0x80 | (self.fclkdiv & 0x7F), // FDIVLD set
                0x02 => self.ftstmod,
                FSTAT => self.fstat,
                FCMD => self.fcmd,
                FPROT => 0xFF, // fully unprotected (FPOPEN set)
                FSEC => 0xFE, // unsecured (SEC = 0b10)
                0x08 => @truncate(self.faddr >> 8),
                0x09 => @truncate(self.faddr),
                0x0A => @truncate(self.fdata >> 8),
                0x0B => @truncate(self.fdata),
                else => 0,
            };
        }
        return self.mem[addr];
    }
    fn read16(self: *Cpu, addr: u16) u16 {
        const hi = self.read8(addr);
        const lo = self.read8(addr +% 1);
        return (@as(u16, hi) << 8) | lo;
    }
    fn write8(self: *Cpu, addr: u16, val: u8) void {
        if (self.kind == .ftmrg) {
            self.write8Gmmc(addr, val);
            return;
        }
        const a: u32 = addr;
        if (self.inCtrl(a)) {
            self.flashRegWrite(@intCast(a - self.ctrl_base), val);
            return;
        }
        if (self.inFlash(a)) {
            // single-byte flash write illegal; program unit is a word
            self.fstat |= ACCERR;
            return;
        }
        self.mem[addr] = val;
    }
    /// 16-bit store; word program latches in flash space.
    fn writeW(self: *Cpu, addr: u16, val: u16) void {
        const a: u32 = addr;
        if (self.inFlash(a)) {
            if (self.fstat & CBEIF == 0 or addr & 1 != 0) {
                self.fstat |= ACCERR;
                return;
            }
            self.latch_addr = addr;
            self.latch_data = val;
            self.latched = true;
            return;
        }
        self.write8(addr, @truncate(val >> 8));
        self.write8(addr +% 1, @truncate(val));
    }
    fn flashRegWrite(self: *Cpu, off: u8, val: u8) void {
        switch (off) {
            FCLKDIV => {
                self.fclkdiv = val;
                self.fclkdiv_written = true;
            },
            0x02 => self.ftstmod = val, // FTSTMOD (test/stress mode select)
            FCMD => {
                self.fcmd = val;
                self.cmd_armed = true;
            },
            0x08 => self.faddr = (self.faddr & 0x00FF) | (@as(u16, val) << 8),
            0x09 => self.faddr = (self.faddr & 0xFF00) | val,
            0x0A => self.fdata = (self.fdata & 0x00FF) | (@as(u16, val) << 8),
            0x0B => self.fdata = (self.fdata & 0xFF00) | val,
            FSTAT => {
                if (val & ACCERR != 0) self.fstat &= ~ACCERR;
                if (val & PVIOL != 0) self.fstat &= ~PVIOL;
                // write 1 to CBEIF launches the buffered command
                if (val & CBEIF != 0 and self.cmd_armed) self.launch();
            },
            else => {}, // FCNFG/FPROT/FSEC etc: functionally inert here
        }
    }
    fn launch(self: *Cpu) void {
        self.cmd_armed = false;
        // addr/data from the flash-array write (word program), else FADDR/FDATA
        // (FTSTMOD test-mode erase path). Not gated on FCLKDIV: host set it at connect.
        const eff_addr: u16 = if (self.latched) self.latch_addr else self.faddr;
        const eff_data: u16 = if (self.latched) self.latch_data else self.fdata;
        self.latched = false;
        var newstat: u8 = CBEIF | CCIF;
        switch (self.fcmd) {
            CMD_WORD_PROG => {
                // program only clears 1->0
                self.mem[eff_addr] &= @truncate(eff_data >> 8);
                self.mem[eff_addr +% 1] &= @truncate(eff_data);
            },
            CMD_SECTOR_ERASE => {
                const sect: u32 = eff_addr & ~@as(u32, 0x1FF); // 512-byte sector
                var p = sect;
                while (p < sect + 0x200) : (p += 1) {
                    if (self.inFlash(p)) self.mem[@intCast(p)] = 0xFF;
                }
            },
            CMD_MASS_ERASE => {
                var p = self.flash_lo;
                while (p <= self.flash_hi and p <= 0xFFFF) : (p += 1) self.mem[@intCast(p)] = 0xFF;
            },
            CMD_ERASE_VERIFY => {
                var all_blank = true;
                var p = self.flash_lo;
                while (p <= self.flash_hi and p <= 0xFFFF) : (p += 1) {
                    if (self.mem[@intCast(p)] != 0xFF) {
                        all_blank = false;
                        break;
                    }
                }
                if (all_blank) newstat |= BLANK;
            },
            else => {
                self.fstat = CBEIF | CCIF | ACCERR;
                return;
            },
        }
        self.fstat = newstat;
    }

    // BDM host access: direct, bypasses the controller. In FTMRG a flash-window
    // addr resolves to the global P-flash array, so host readback matches.
    pub fn hostWrite(self: *Cpu, addr: u16, data: []const u8) void {
        for (data, 0..) |b, i| {
            const at = addr +% @as(u16, @intCast(i));
            if (self.kind == .ftmrg and isFlashWindowG(at)) {
                self.pflash[self.globalOf(at)] = b;
            } else self.mem[at] = b;
        }
    }
    pub fn hostRead(self: *Cpu, addr: u16, out: []u8) void {
        for (out, 0..) |*b, i| b.* = self.hostReadByte(addr +% @as(u16, @intCast(i)));
    }
    pub fn hostReadByte(self: *Cpu, addr: u16) u8 {
        if (self.kind == .ftmrg and isFlashWindowG(addr)) return self.pflash[self.globalOf(addr)];
        return self.mem[addr];
    }

    fn fetch8(self: *Cpu) u8 {
        const b = self.mem[self.pc];
        self.pc +%= 1;
        return b;
    }
    fn fetch16(self: *Cpu) u16 {
        const hi = self.fetch8();
        const lo = self.fetch8();
        return (@as(u16, hi) << 8) | lo;
    }

    fn idxReg(self: *const Cpu, rr: u2) u16 {
        return switch (rr) {
            0 => self.x,
            1 => self.y,
            2 => self.sp,
            3 => self.pc,
        };
    }
    fn setIdxReg(self: *Cpu, rr: u2, v: u16) void {
        switch (rr) {
            0 => self.x = v,
            1 => self.y = v,
            2 => self.sp = v,
            3 => self.pc = v,
        }
    }
    fn signext5(v: u8) u16 {
        const s: i16 = if (v & 0x10 != 0) @as(i16, v & 0x1F) - 32 else @as(i16, v & 0x1F);
        return @bitCast(s);
    }
    fn signext9(sbit: u8, ff: u8) u16 {
        const mag: i16 = @intCast((@as(u16, sbit) << 8) | ff);
        const val: i16 = if (sbit != 0) mag - 512 else mag;
        return @bitCast(val);
    }

    /// Indexed postbyte -> effective address (applies auto inc/dec, 16-bit indirect).
    fn idxEA(self: *Cpu, xb: u8) u16 {
        if (xb & 0xE0 == 0xE0) {
            const rr: u2 = @truncate((xb >> 3) & 3);
            if (xb & 0x04 == 0) {
                // constant offset / 16-bit indirect
                const z = (xb >> 1) & 1;
                const sbit = xb & 1;
                if (z == 0) {
                    const ff = self.fetch8(); // 9-bit (1 ext byte)
                    return self.idxReg(rr) +% signext9(sbit, ff);
                } else if (sbit == 0) {
                    const off = self.fetch16(); // 16-bit constant (2 ext bytes)
                    return self.idxReg(rr) +% off;
                } else {
                    const off = self.fetch16(); // 16-bit indirect (2 ext bytes)
                    const ptr = self.idxReg(rr) +% off;
                    return self.read16(ptr);
                }
            } else {
                // accumulator offset / D-indirect
                const aa = xb & 3;
                return switch (aa) {
                    0 => self.idxReg(rr) +% self.a, // A,r (zero-extended)
                    1 => self.idxReg(rr) +% self.b, // B,r
                    2 => self.idxReg(rr) +% self.d(), // D,r
                    3 => self.read16(self.idxReg(rr) +% self.d()), // [D,r]
                    else => unreachable,
                };
            }
        } else if (xb & 0x20 == 0) {
            // 5-bit constant offset
            const rr: u2 = @truncate((xb >> 6) & 3);
            return self.idxReg(rr) +% signext5(xb & 0x1F);
        } else {
            // auto pre/post inc/dec (base X/Y/SP only)
            const rr: u2 = @truncate((xb >> 6) & 3);
            const post = (xb >> 4) & 1;
            const nnnn = xb & 0xF;
            const delta: i16 = if (nnnn < 8) @as(i16, nnnn) + 1 else @as(i16, nnnn) - 16;
            var reg = self.idxReg(rr);
            var ea: u16 = undefined;
            if (post != 0) {
                ea = reg; // post-modify: use, then adjust
                reg +%= @bitCast(delta);
            } else {
                reg +%= @bitCast(delta); // pre-modify: adjust, then use
                ea = reg;
            }
            self.setIdxReg(rr, reg);
            return ea;
        }
    }

    fn opAddr(self: *Cpu, m: M) u16 {
        return switch (m) {
            .dir => self.fetch8(),
            .ext => self.fetch16(),
            .idx => self.idxEA(self.fetch8()),
            .imm => unreachable,
        };
    }
    fn rd8(self: *Cpu, m: M) u8 {
        return if (m == .imm) self.fetch8() else self.read8(self.opAddr(m));
    }
    fn rd16(self: *Cpu, m: M) u16 {
        return if (m == .imm) self.fetch16() else self.read16(self.opAddr(m));
    }

    fn add8(self: *Cpu, x: u8, m: u8, cin: u8) u8 {
        const r: u16 = @as(u16, x) + m + cin;
        const r8: u8 = @truncate(r);
        self.setFlag(H, ((x & 0xF) + (m & 0xF) + cin) > 0xF);
        self.setFlag(C, r > 0xFF);
        self.setFlag(V, (x ^ r8) & (m ^ r8) & 0x80 != 0);
        self.setFlag(N, r8 & 0x80 != 0);
        self.setFlag(Z, r8 == 0);
        return r8;
    }
    fn sub8(self: *Cpu, x: u8, m: u8, borrow: u8) u8 {
        const diff: i16 = @as(i16, x) - m - borrow;
        const r8: u8 = @truncate(@as(u16, @bitCast(diff)));
        self.setFlag(C, @as(u16, m) + borrow > x); // borrow out
        self.setFlag(V, (x ^ m) & (x ^ r8) & 0x80 != 0);
        self.setFlag(N, r8 & 0x80 != 0);
        self.setFlag(Z, r8 == 0);
        return r8;
    }
    fn add16(self: *Cpu, x: u16, m: u16) u16 {
        const r: u32 = @as(u32, x) + m;
        const r16: u16 = @truncate(r);
        self.setFlag(C, r > 0xFFFF);
        self.setFlag(V, (x ^ r16) & (m ^ r16) & 0x8000 != 0);
        self.setFlag(N, r16 & 0x8000 != 0);
        self.setFlag(Z, r16 == 0);
        return r16;
    }
    fn sub16(self: *Cpu, x: u16, m: u16) u16 {
        const diff: i32 = @as(i32, x) - m;
        const r16: u16 = @truncate(@as(u32, @bitCast(diff)));
        self.setFlag(C, m > x);
        self.setFlag(V, (x ^ m) & (x ^ r16) & 0x8000 != 0);
        self.setFlag(N, r16 & 0x8000 != 0);
        self.setFlag(Z, r16 == 0);
        return r16;
    }
    fn inc8(self: *Cpu, v: u8) u8 {
        const r = v +% 1;
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, v == 0x7F);
        return r;
    }
    fn dec8(self: *Cpu, v: u8) u8 {
        const r = v -% 1;
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, v == 0x80);
        return r;
    }
    fn neg8(self: *Cpu, v: u8) u8 {
        const r = 0 -% v;
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, v == 0x80);
        self.setFlag(C, v != 0);
        return r;
    }
    fn com8(self: *Cpu, v: u8) u8 {
        const r = ~v;
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
        self.setFlag(C, true);
        return r;
    }
    fn lsl8(self: *Cpu, v: u8) u8 {
        const cout = v & 0x80 != 0;
        const r = v << 1;
        self.setFlag(C, cout);
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, (r & 0x80 != 0) != cout);
        return r;
    }
    fn lsr8(self: *Cpu, v: u8) u8 {
        const cout = v & 1 != 0;
        const r = v >> 1;
        self.setFlag(C, cout);
        self.setFlag(N, false);
        self.setFlag(Z, r == 0);
        self.setFlag(V, cout);
        return r;
    }
    fn asr8(self: *Cpu, v: u8) u8 {
        const cout = v & 1 != 0;
        const r = (v >> 1) | (v & 0x80);
        self.setFlag(C, cout);
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, (r & 0x80 != 0) != cout);
        return r;
    }
    fn rol8(self: *Cpu, v: u8) u8 {
        const cin: u8 = if (self.ccr & C != 0) 1 else 0;
        const cout = v & 0x80 != 0;
        const r = (v << 1) | cin;
        self.setFlag(C, cout);
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, (r & 0x80 != 0) != cout);
        return r;
    }
    fn ror8(self: *Cpu, v: u8) u8 {
        const cin: u8 = if (self.ccr & C != 0) 0x80 else 0;
        const cout = v & 1 != 0;
        const r = (v >> 1) | cin;
        self.setFlag(C, cout);
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, (r & 0x80 != 0) != cout);
        return r;
    }

    fn ldA(self: *Cpu, m: M) void {
        self.a = self.rd8(m);
        self.setNZ8V0(self.a);
    }
    fn ldB(self: *Cpu, m: M) void {
        self.b = self.rd8(m);
        self.setNZ8V0(self.b);
    }
    // AND/ORA/EOR into A/B: N,Z from result, V=0, C unaffected.
    fn andA(self: *Cpu, m: M) void {
        self.a &= self.rd8(m);
        self.setNZ8V0(self.a);
    }
    fn andB(self: *Cpu, m: M) void {
        self.b &= self.rd8(m);
        self.setNZ8V0(self.b);
    }
    fn orA(self: *Cpu, m: M) void {
        self.a |= self.rd8(m);
        self.setNZ8V0(self.a);
    }
    fn orB(self: *Cpu, m: M) void {
        self.b |= self.rd8(m);
        self.setNZ8V0(self.b);
    }
    fn eorA(self: *Cpu, m: M) void {
        self.a ^= self.rd8(m);
        self.setNZ8V0(self.a);
    }
    fn eorB(self: *Cpu, m: M) void {
        self.b ^= self.rd8(m);
        self.setNZ8V0(self.b);
    }
    fn ldD(self: *Cpu, m: M) void {
        self.setD(self.rd16(m));
        self.setNZ16V0(self.d());
    }
    fn ldX(self: *Cpu, m: M) void {
        self.x = self.rd16(m);
        self.setNZ16V0(self.x);
    }
    fn ldY(self: *Cpu, m: M) void {
        self.y = self.rd16(m);
        self.setNZ16V0(self.y);
    }
    fn ldS(self: *Cpu, m: M) void {
        self.sp = self.rd16(m);
        self.setNZ16V0(self.sp);
    }
    fn stA(self: *Cpu, m: M) void {
        self.write8(self.opAddr(m), self.a);
        self.setNZ8V0(self.a);
    }
    fn stB(self: *Cpu, m: M) void {
        self.write8(self.opAddr(m), self.b);
        self.setNZ8V0(self.b);
    }
    fn stD(self: *Cpu, m: M) void {
        self.writeW(self.opAddr(m), self.d());
        self.setNZ16V0(self.d());
    }
    fn stX(self: *Cpu, m: M) void {
        self.writeW(self.opAddr(m), self.x);
        self.setNZ16V0(self.x);
    }
    fn stY(self: *Cpu, m: M) void {
        self.writeW(self.opAddr(m), self.y);
        self.setNZ16V0(self.y);
    }
    fn stS(self: *Cpu, m: M) void {
        self.writeW(self.opAddr(m), self.sp);
        self.setNZ16V0(self.sp);
    }

    const Rmw = enum { neg, com, inc, dec, lsr, ror, asr, lsl, rol };
    fn memRmw(self: *Cpu, m: M, which: Rmw) void {
        const ea = self.opAddr(m);
        const v = self.read8(ea);
        const r = switch (which) {
            .neg => self.neg8(v),
            .com => self.com8(v),
            .inc => self.inc8(v),
            .dec => self.dec8(v),
            .lsr => self.lsr8(v),
            .ror => self.ror8(v),
            .asr => self.asr8(v),
            .lsl => self.lsl8(v),
            .rol => self.rol8(v),
        };
        self.write8(ea, r);
    }
    fn memClr(self: *Cpu, m: M) void {
        self.write8(self.opAddr(m), 0);
        self.setFlag(N, false);
        self.setFlag(Z, true);
        self.setFlag(V, false);
        self.setFlag(C, false);
    }
    fn memTst(self: *Cpu, m: M) void {
        const v = self.read8(self.opAddr(m));
        self.setFlag(N, v & 0x80 != 0);
        self.setFlag(Z, v == 0);
        self.setFlag(V, false);
        self.setFlag(C, false);
    }
    fn accTst(self: *Cpu, v: u8) void {
        self.setFlag(N, v & 0x80 != 0);
        self.setFlag(Z, v == 0);
        self.setFlag(V, false);
        self.setFlag(C, false);
    }

    fn bset(self: *Cpu, m: M) void {
        const ea = self.opAddr(m);
        const mask = self.fetch8();
        const r = self.read8(ea) | mask;
        self.write8(ea, r);
        self.setNZ8V0(r);
    }
    fn bclr(self: *Cpu, m: M) void {
        const ea = self.opAddr(m);
        const mask = self.fetch8();
        const r = self.read8(ea) & ~mask;
        self.write8(ea, r);
        self.setNZ8V0(r);
    }
    fn brset(self: *Cpu, m: M) void {
        const ea = self.opAddr(m);
        const mask = self.fetch8();
        const rel: i8 = @bitCast(self.fetch8());
        if (self.read8(ea) & mask == mask) self.pc +%= @bitCast(@as(i16, rel));
    }
    fn brclr(self: *Cpu, m: M) void {
        const ea = self.opAddr(m);
        const mask = self.fetch8();
        const rel: i8 = @bitCast(self.fetch8());
        if (self.read8(ea) & mask == 0) self.pc +%= @bitCast(@as(i16, rel));
    }

    fn push16(self: *Cpu, v: u16) void {
        self.sp -%= 2;
        self.mem[self.sp] = @truncate(v >> 8);
        self.mem[self.sp +% 1] = @truncate(v);
    }
    fn pull16(self: *Cpu) u16 {
        const hi = self.mem[self.sp];
        const lo = self.mem[self.sp +% 1];
        self.sp +%= 2;
        return (@as(u16, hi) << 8) | lo;
    }
    fn push8(self: *Cpu, v: u8) void {
        self.sp -%= 1;
        self.mem[self.sp] = v;
    }
    fn pull8(self: *Cpu) u8 {
        const v = self.mem[self.sp];
        self.sp +%= 1;
        return v;
    }

    fn cond(self: *const Cpu, cc: u8) bool {
        const c = self.ccr & C != 0;
        const z = self.ccr & Z != 0;
        const n = self.ccr & N != 0;
        const v = self.ccr & V != 0;
        return switch (cc) {
            0x0 => true, // BRA
            0x1 => false, // BRN
            0x2 => !c and !z, // BHI
            0x3 => c or z, // BLS
            0x4 => !c, // BCC/BHS
            0x5 => c, // BCS/BLO
            0x6 => !z, // BNE
            0x7 => z, // BEQ
            0x8 => !v, // BVC
            0x9 => v, // BVS
            0xA => !n, // BPL
            0xB => n, // BMI
            0xC => n == v, // BGE (N^V == 0)
            0xD => n != v, // BLT
            0xE => !z and (n == v), // BGT
            0xF => z or (n != v), // BLE
            else => false,
        };
    }

    fn regIs16(code: u8) bool {
        return code >= 4 and code <= 7; // D,X,Y,SP
    }
    fn getReg(self: *const Cpu, code: u8) u16 {
        return switch (code) {
            0 => self.a,
            1 => self.b,
            2 => self.ccr,
            4 => self.d(),
            5 => self.x,
            6 => self.y,
            7 => self.sp,
            else => 0,
        };
    }
    fn setReg(self: *Cpu, code: u8, v: u16) void {
        switch (code) {
            0 => self.a = @truncate(v),
            1 => self.b = @truncate(v),
            2 => self.ccr = @truncate(v),
            4 => self.setD(v),
            5 => self.x = v,
            6 => self.y = v,
            7 => self.sp = v,
            else => {},
        }
    }
    fn tfrExg(self: *Cpu) void {
        const eb = self.fetch8();
        const exg = eb & 0x80 != 0;
        const src: u8 = (eb >> 4) & 7;
        const dst: u8 = eb & 7;
        if (!exg) {
            if (regIs16(dst) and !regIs16(src)) {
                // 8 -> 16: sign-extend (SEX)
                const b8: u8 = @truncate(self.getReg(src));
                const se: i16 = @as(i8, @bitCast(b8));
                self.setReg(dst, @bitCast(se));
            } else {
                self.setReg(dst, self.getReg(src)); // setReg truncates for 8-bit dst
            }
        } else {
            const sv = self.getReg(src);
            const dv = self.getReg(dst);
            self.setReg(src, dv);
            self.setReg(dst, sv);
        }
    }

    fn loopPrim(self: *Cpu) void {
        const lb = self.fetch8();
        const rr = self.fetch8();
        const mode = (lb >> 5) & 7; // 0 DBEQ,1 DBNE,2 TBEQ,3 TBNE,4 IBEQ,5 IBNE
        const sbit = (lb >> 4) & 1;
        const ctr = lb & 7; // 0=A,1=B,4=D,5=X,6=Y,7=SP
        const is16 = regIs16(ctr);
        var val = self.getReg(ctr);
        switch (mode) {
            0, 1 => val = if (is16) val -% 1 else @as(u8, @truncate(val)) -% 1,
            4, 5 => val = if (is16) val +% 1 else @as(u8, @truncate(val)) +% 1,
            else => {}, // TBEQ/TBNE: test only
        }
        if (mode != 2 and mode != 3) self.setReg(ctr, val);
        const zero = if (is16) val == 0 else (val & 0xFF) == 0;
        const branch_if_eq = (mode & 1) == 0; // *EQ branch when counter == 0
        const take = if (branch_if_eq) zero else !zero;
        if (take) self.pc +%= signext9(sbit, rr);
    }

    fn page2(self: *Cpu) Error!void {
        const op = self.fetch8();
        switch (op) {
            // MOVW
            0x00 => { // IMM -> IDX
                const ea = self.idxEA(self.fetch8());
                self.writeW(ea, self.fetch16());
            },
            0x01 => { // EXT -> IDX
                const ea = self.idxEA(self.fetch8());
                const src = self.fetch16();
                self.writeW(ea, self.read16(src));
            },
            0x02 => { // IDX -> IDX
                const sea = self.idxEA(self.fetch8());
                const dea = self.idxEA(self.fetch8());
                self.writeW(dea, self.read16(sea));
            },
            0x03 => { // IMM -> EXT
                const val = self.fetch16();
                self.writeW(self.fetch16(), val);
            },
            0x04 => { // EXT -> EXT
                const src = self.fetch16();
                const dst = self.fetch16();
                self.writeW(dst, self.read16(src));
            },
            0x05 => { // IDX -> EXT
                const sea = self.idxEA(self.fetch8());
                self.writeW(self.fetch16(), self.read16(sea));
            },
            0x06 => self.a = self.add8(self.a, self.b, 0), // ABA
            // MOVB
            0x08 => { // IMM -> IDX
                const ea = self.idxEA(self.fetch8());
                self.write8(ea, self.fetch8());
            },
            0x09 => { // EXT -> IDX
                const ea = self.idxEA(self.fetch8());
                const src = self.fetch16();
                self.write8(ea, self.read8(src));
            },
            0x0A => { // IDX -> IDX
                const sea = self.idxEA(self.fetch8());
                const dea = self.idxEA(self.fetch8());
                self.write8(dea, self.read8(sea));
            },
            0x0B => { // IMM -> EXT
                const val = self.fetch8();
                self.write8(self.fetch16(), val);
            },
            0x0C => { // EXT -> EXT
                const src = self.fetch16();
                const dst = self.fetch16();
                self.write8(dst, self.read8(src));
            },
            0x0D => { // IDX -> EXT
                const sea = self.idxEA(self.fetch8());
                self.write8(self.fetch16(), self.read8(sea));
            },
            0x0E => { // TAB (B = A)
                self.b = self.a;
                self.setNZ8V0(self.b);
            },
            0x0F => { // TBA (A = B)
                self.a = self.b;
                self.setNZ8V0(self.a);
            },
            0x10 => { // IDIV: X = D / X, D = D % X
                const num = self.d();
                const den = self.x;
                if (den == 0) {
                    self.setFlag(C, true);
                } else {
                    const q = num / den;
                    self.x = q;
                    self.setD(num % den);
                    self.setFlag(C, false);
                    self.setFlag(Z, q == 0);
                    self.setFlag(V, false);
                    self.setFlag(N, false);
                }
            },
            0x13 => { // EMUL: Y:D = D * Y
                const p: u32 = @as(u32, self.d()) * self.y;
                self.y = @truncate(p >> 16);
                self.setD(@truncate(p));
                self.setFlag(N, p & 0x80000000 != 0);
                self.setFlag(Z, p == 0);
                self.setFlag(C, p & 0x8000 != 0);
            },
            0x16 => self.a = self.sub8(self.a, self.b, 0), // SBA
            0x17 => _ = self.sub8(self.a, self.b, 0), // CBA (flags only)
            0x20...0x2F => { // long branches LBcc (rel16)
                const rel: i16 = @bitCast(self.fetch16());
                if (self.cond(op & 0x0F)) self.pc +%= @bitCast(rel);
            },
            else => {
                self.bad_opcode = op;
                self.bad_pc = self.pc -% 2;
                return error.UnimplementedPostbyte;
            },
        }
    }

    /// One instruction; false once self-halted (BGND).
    pub fn step(self: *Cpu) Error!bool {
        if (self.halted) return false;
        if (self.dbgBreakMatch()) { // tagged HW breakpoint: halt before this opcode
            self.halted = true;
            return false;
        }
        const op = self.fetch8();
        switch (op) {
            0x00 => { // BGND - self-halt
                self.halted = true;
                return false;
            },
            0xA7 => {}, // NOP
            0x18 => try self.page2(),

            // INX/DEX/INY/DEY affect ONLY Z (not N/V).
            0x02 => {
                self.y +%= 1;
                self.setFlag(Z, self.y == 0);
            }, // INY
            0x03 => {
                self.y -%= 1;
                self.setFlag(Z, self.y == 0);
            }, // DEY
            0x08 => {
                self.x +%= 1;
                self.setFlag(Z, self.x == 0);
            }, // INX
            0x09 => {
                self.x -%= 1;
                self.setFlag(Z, self.x == 0);
            }, // DEX
            0x87 => {
                self.a = 0;
                self.setFlag(N, false);
                self.setFlag(Z, true);
                self.setFlag(V, false);
                self.setFlag(C, false);
            }, // CLRA
            0xC7 => {
                self.b = 0;
                self.setFlag(N, false);
                self.setFlag(Z, true);
                self.setFlag(V, false);
                self.setFlag(C, false);
            }, // CLRB
            0x40 => self.a = self.neg8(self.a), // NEGA
            0x50 => self.b = self.neg8(self.b), // NEGB
            0x41 => self.a = self.com8(self.a), // COMA
            0x51 => self.b = self.com8(self.b), // COMB
            0x42 => self.a = self.inc8(self.a), // INCA
            0x52 => self.b = self.inc8(self.b), // INCB
            0x43 => self.a = self.dec8(self.a), // DECA
            0x53 => self.b = self.dec8(self.b), // DECB
            0x44 => self.a = self.lsr8(self.a), // LSRA
            0x54 => self.b = self.lsr8(self.b), // LSRB
            0x45 => self.a = self.rol8(self.a), // ROLA
            0x55 => self.b = self.rol8(self.b), // ROLB
            0x46 => self.a = self.ror8(self.a), // RORA
            0x56 => self.b = self.ror8(self.b), // RORB
            0x47 => self.a = self.asr8(self.a), // ASRA
            0x57 => self.b = self.asr8(self.b), // ASRB
            0x48 => self.a = self.lsl8(self.a), // ASLA/LSLA
            0x58 => self.b = self.lsl8(self.b), // ASLB/LSLB
            0x97 => self.accTst(self.a), // TSTA
            0xD7 => self.accTst(self.b), // TSTB

            0x49 => { // LSRD
                const cout = self.d() & 1 != 0;
                const r = self.d() >> 1;
                self.setD(r);
                self.setFlag(C, cout);
                self.setFlag(N, false);
                self.setFlag(Z, r == 0);
                self.setFlag(V, cout);
            },
            0x59 => { // ASLD/LSLD
                const cout = self.d() & 0x8000 != 0;
                const r = self.d() << 1;
                self.setD(r);
                self.setFlag(C, cout);
                self.setFlag(N, r & 0x8000 != 0);
                self.setFlag(Z, r == 0);
                self.setFlag(V, (r & 0x8000 != 0) != cout);
            },

            0x12 => { // MUL: D = A * B (C = bit7 of result)
                const p: u16 = @as(u16, self.a) * self.b;
                self.setFlag(C, p & 0x80 != 0);
                self.setD(p);
            },
            0x11 => { // EDIV: Y = (Y:D) / X, D = remainder
                const dividend: u32 = (@as(u32, self.y) << 16) | self.d();
                const divisor: u16 = self.x;
                if (divisor == 0) {
                    self.setFlag(C, true);
                } else {
                    const q = dividend / divisor;
                    self.y = @truncate(q);
                    self.setD(@truncate(dividend % divisor));
                    self.setFlag(C, false);
                    self.setFlag(V, q > 0xFFFF);
                    self.setFlag(N, self.y & 0x8000 != 0);
                    self.setFlag(Z, self.y == 0);
                }
            },

            0x10 => self.ccr &= self.fetch8(), // ANDCC (CLI = ANDCC #$EF)
            0x14 => self.ccr |= self.fetch8(), // ORCC  (SEI = ORCC #$10)

            0xB7 => self.tfrExg(),
            0x19 => self.y = self.idxEA(self.fetch8()), // LEAY
            0x1A => self.x = self.idxEA(self.fetch8()), // LEAX
            0x1B => self.sp = self.idxEA(self.fetch8()), // LEAS

            0x36 => self.push8(self.a), // PSHA
            0x37 => self.push8(self.b), // PSHB
            0x32 => {
                self.a = self.pull8();
            }, // PULA
            0x33 => {
                self.b = self.pull8();
            }, // PULB
            0x34 => self.push16(self.x), // PSHX
            0x35 => self.push16(self.y), // PSHY
            0x30 => self.x = self.pull16(), // PULX
            0x31 => self.y = self.pull16(), // PULY
            0x3B => self.push16(self.d()), // PSHD
            0x3A => self.setD(self.pull16()), // PULD
            0x39 => self.push8(self.ccr), // PSHC
            0x38 => self.ccr = self.pull8(), // PULC

            0x07 => { // BSR
                const rel: i8 = @bitCast(self.fetch8());
                self.push16(self.pc);
                self.pc +%= @bitCast(@as(i16, rel));
            },
            0x16 => { // JSR EXT
                const ea = self.fetch16();
                self.push16(self.pc);
                self.pc = ea;
            },
            0x17 => { // JSR DIR
                const ea = self.fetch8();
                self.push16(self.pc);
                self.pc = ea;
            },
            0x15 => { // JSR IDX
                const ea = self.idxEA(self.fetch8());
                self.push16(self.pc);
                self.pc = ea;
            },
            0x06 => self.pc = self.fetch16(), // JMP EXT
            0x05 => self.pc = self.idxEA(self.fetch8()), // JMP IDX
            0x3D => self.pc = self.pull16(), // RTS

            0x04 => self.loopPrim(),

            0x20...0x2F => {
                const rel: i8 = @bitCast(self.fetch8());
                if (self.cond(op & 0x0F)) self.pc +%= @bitCast(@as(i16, rel));
            },

            0x4C => self.bset(.dir),
            0x1C => self.bset(.ext),
            0x0C => self.bset(.idx),
            0x4D => self.bclr(.dir),
            0x1D => self.bclr(.ext),
            0x0D => self.bclr(.idx),
            0x4E => self.brset(.dir),
            0x1E => self.brset(.ext),
            0x0E => self.brset(.idx),
            0x4F => self.brclr(.dir),
            0x1F => self.brclr(.ext),
            0x0F => self.brclr(.idx),

            0x86 => self.ldA(.imm),
            0x96 => self.ldA(.dir),
            0xB6 => self.ldA(.ext),
            0xA6 => self.ldA(.idx),
            0xC6 => self.ldB(.imm),
            0xD6 => self.ldB(.dir),
            0xF6 => self.ldB(.ext),
            0xE6 => self.ldB(.idx),
            0xCC => self.ldD(.imm),
            0xDC => self.ldD(.dir),
            0xFC => self.ldD(.ext),
            0xEC => self.ldD(.idx),
            0xCE => self.ldX(.imm),
            0xDE => self.ldX(.dir),
            0xFE => self.ldX(.ext),
            0xEE => self.ldX(.idx),
            0xCD => self.ldY(.imm),
            0xDD => self.ldY(.dir),
            0xFD => self.ldY(.ext),
            0xED => self.ldY(.idx),
            0xCF => self.ldS(.imm),
            0xDF => self.ldS(.dir),
            0xFF => self.ldS(.ext),
            0xEF => self.ldS(.idx),
            0x5A => self.stA(.dir),
            0x7A => self.stA(.ext),
            0x6A => self.stA(.idx),
            0x5B => self.stB(.dir),
            0x7B => self.stB(.ext),
            0x6B => self.stB(.idx),
            0x5C => self.stD(.dir),
            0x7C => self.stD(.ext),
            0x6C => self.stD(.idx),
            0x5D => self.stY(.dir),
            0x7D => self.stY(.ext),
            0x6D => self.stY(.idx),
            0x5E => self.stX(.dir),
            0x7E => self.stX(.ext),
            0x6E => self.stX(.idx),
            0x5F => self.stS(.dir),
            0x7F => self.stS(.ext),
            0x6F => self.stS(.idx),

            0x8B => self.a = self.add8(self.a, self.rd8(.imm), 0), // ADDA
            0x9B => self.a = self.add8(self.a, self.rd8(.dir), 0),
            0xBB => self.a = self.add8(self.a, self.rd8(.ext), 0),
            0xAB => self.a = self.add8(self.a, self.rd8(.idx), 0),
            0xCB => self.b = self.add8(self.b, self.rd8(.imm), 0), // ADDB
            0xDB => self.b = self.add8(self.b, self.rd8(.dir), 0),
            0xFB => self.b = self.add8(self.b, self.rd8(.ext), 0),
            0xEB => self.b = self.add8(self.b, self.rd8(.idx), 0),
            0x89 => self.a = self.add8(self.a, self.rd8(.imm), self.carry()), // ADCA
            0x99 => self.a = self.add8(self.a, self.rd8(.dir), self.carry()),
            0xB9 => self.a = self.add8(self.a, self.rd8(.ext), self.carry()),
            0xA9 => self.a = self.add8(self.a, self.rd8(.idx), self.carry()),
            0xC9 => self.b = self.add8(self.b, self.rd8(.imm), self.carry()), // ADCB
            0xD9 => self.b = self.add8(self.b, self.rd8(.dir), self.carry()),
            0xF9 => self.b = self.add8(self.b, self.rd8(.ext), self.carry()),
            0xE9 => self.b = self.add8(self.b, self.rd8(.idx), self.carry()),

            0x80 => self.a = self.sub8(self.a, self.rd8(.imm), 0), // SUBA
            0x90 => self.a = self.sub8(self.a, self.rd8(.dir), 0),
            0xB0 => self.a = self.sub8(self.a, self.rd8(.ext), 0),
            0xA0 => self.a = self.sub8(self.a, self.rd8(.idx), 0),
            0xC0 => self.b = self.sub8(self.b, self.rd8(.imm), 0), // SUBB
            0xD0 => self.b = self.sub8(self.b, self.rd8(.dir), 0),
            0xF0 => self.b = self.sub8(self.b, self.rd8(.ext), 0),
            0xE0 => self.b = self.sub8(self.b, self.rd8(.idx), 0),
            0x82 => self.a = self.sub8(self.a, self.rd8(.imm), self.carry()), // SBCA
            0x92 => self.a = self.sub8(self.a, self.rd8(.dir), self.carry()),
            0xB2 => self.a = self.sub8(self.a, self.rd8(.ext), self.carry()),
            0xA2 => self.a = self.sub8(self.a, self.rd8(.idx), self.carry()),
            0xC2 => self.b = self.sub8(self.b, self.rd8(.imm), self.carry()), // SBCB
            0xD2 => self.b = self.sub8(self.b, self.rd8(.dir), self.carry()),
            0xF2 => self.b = self.sub8(self.b, self.rd8(.ext), self.carry()),
            0xE2 => self.b = self.sub8(self.b, self.rd8(.idx), self.carry()),

            0x81 => _ = self.sub8(self.a, self.rd8(.imm), 0), // CMPA
            0x91 => _ = self.sub8(self.a, self.rd8(.dir), 0),
            0xB1 => _ = self.sub8(self.a, self.rd8(.ext), 0),
            0xA1 => _ = self.sub8(self.a, self.rd8(.idx), 0),
            0xC1 => _ = self.sub8(self.b, self.rd8(.imm), 0), // CMPB
            0xD1 => _ = self.sub8(self.b, self.rd8(.dir), 0),
            0xF1 => _ = self.sub8(self.b, self.rd8(.ext), 0),
            0xE1 => _ = self.sub8(self.b, self.rd8(.idx), 0),

            0xC3 => self.setD(self.add16(self.d(), self.rd16(.imm))), // ADDD
            0xD3 => self.setD(self.add16(self.d(), self.rd16(.dir))),
            0xF3 => self.setD(self.add16(self.d(), self.rd16(.ext))),
            0xE3 => self.setD(self.add16(self.d(), self.rd16(.idx))),
            0x83 => self.setD(self.sub16(self.d(), self.rd16(.imm))), // SUBD
            0x93 => self.setD(self.sub16(self.d(), self.rd16(.dir))),
            0xB3 => self.setD(self.sub16(self.d(), self.rd16(.ext))),
            0xA3 => self.setD(self.sub16(self.d(), self.rd16(.idx))),
            0x8C => _ = self.sub16(self.d(), self.rd16(.imm)), // CPD
            0x9C => _ = self.sub16(self.d(), self.rd16(.dir)),
            0xBC => _ = self.sub16(self.d(), self.rd16(.ext)),
            0xAC => _ = self.sub16(self.d(), self.rd16(.idx)),
            0x8E => _ = self.sub16(self.x, self.rd16(.imm)), // CPX
            0x9E => _ = self.sub16(self.x, self.rd16(.dir)),
            0xBE => _ = self.sub16(self.x, self.rd16(.ext)),
            0xAE => _ = self.sub16(self.x, self.rd16(.idx)),
            0x8D => _ = self.sub16(self.y, self.rd16(.imm)), // CPY
            0x9D => _ = self.sub16(self.y, self.rd16(.dir)),
            0xBD => _ = self.sub16(self.y, self.rd16(.ext)),
            0xAD => _ = self.sub16(self.y, self.rd16(.idx)),
            0x8F => _ = self.sub16(self.sp, self.rd16(.imm)), // CPS
            0x9F => _ = self.sub16(self.sp, self.rd16(.dir)),
            0xBF => _ = self.sub16(self.sp, self.rd16(.ext)),
            0xAF => _ = self.sub16(self.sp, self.rd16(.idx)),

            0x84 => self.andA(.imm),
            0x94 => self.andA(.dir),
            0xB4 => self.andA(.ext),
            0xA4 => self.andA(.idx),
            0xC4 => self.andB(.imm),
            0xD4 => self.andB(.dir),
            0xF4 => self.andB(.ext),
            0xE4 => self.andB(.idx),
            0x8A => self.orA(.imm),
            0x9A => self.orA(.dir),
            0xBA => self.orA(.ext),
            0xAA => self.orA(.idx),
            0xCA => self.orB(.imm),
            0xDA => self.orB(.dir),
            0xFA => self.orB(.ext),
            0xEA => self.orB(.idx),
            0x88 => self.eorA(.imm),
            0x98 => self.eorA(.dir),
            0xB8 => self.eorA(.ext),
            0xA8 => self.eorA(.idx),
            0xC8 => self.eorB(.imm),
            0xD8 => self.eorB(.dir),
            0xF8 => self.eorB(.ext),
            0xE8 => self.eorB(.idx),
            0x85 => self.setNZ8V0(self.a & self.rd8(.imm)), // BITA
            0x95 => self.setNZ8V0(self.a & self.rd8(.dir)),
            0xB5 => self.setNZ8V0(self.a & self.rd8(.ext)),
            0xA5 => self.setNZ8V0(self.a & self.rd8(.idx)),
            0xC5 => self.setNZ8V0(self.b & self.rd8(.imm)), // BITB
            0xD5 => self.setNZ8V0(self.b & self.rd8(.dir)),
            0xF5 => self.setNZ8V0(self.b & self.rd8(.ext)),
            0xE5 => self.setNZ8V0(self.b & self.rd8(.idx)),

            0x70 => self.memRmw(.ext, .neg),
            0x60 => self.memRmw(.idx, .neg),
            0x71 => self.memRmw(.ext, .com),
            0x61 => self.memRmw(.idx, .com),
            0x72 => self.memRmw(.ext, .inc),
            0x62 => self.memRmw(.idx, .inc),
            0x73 => self.memRmw(.ext, .dec),
            0x63 => self.memRmw(.idx, .dec),
            0x74 => self.memRmw(.ext, .lsr),
            0x64 => self.memRmw(.idx, .lsr),
            0x75 => self.memRmw(.ext, .rol),
            0x65 => self.memRmw(.idx, .rol),
            0x76 => self.memRmw(.ext, .ror),
            0x66 => self.memRmw(.idx, .ror),
            0x77 => self.memRmw(.ext, .asr),
            0x67 => self.memRmw(.idx, .asr),
            0x78 => self.memRmw(.ext, .lsl),
            0x68 => self.memRmw(.idx, .lsl),
            0x79 => self.memClr(.ext),
            0x69 => self.memClr(.idx),
            0xF7 => self.memTst(.ext),
            0xE7 => self.memTst(.idx),

            else => {
                self.bad_opcode = op;
                self.bad_pc = self.pc -% 1;
                return error.UnimplementedOpcode;
            },
        }
        return true;
    }

    fn carry(self: *const Cpu) u8 {
        return if (self.ccr & C != 0) 1 else 0;
    }

    /// Run until BGND (self-halt) or step cap.
    pub fn run(self: *Cpu, max_steps: usize) Error!void {
        self.halted = false;
        var n: usize = 0;
        while (n < max_steps) : (n += 1) {
            if (!try self.step()) return;
        }
        return error.Runaway;
    }
};

const testing = std.testing;

fn testCpu() !Cpu {
    return Cpu.init(testing.allocator, 0x0100, 0x4000, 0x7FFF);
}

test "core: LDD#/STD-ext/LDX#/branch and self-halt on BGND" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDX #$0900 ; LDD #$1234 ; STD $0,X ; INX ; INX ; BGND
    const prog = [_]u8{ 0xCE, 0x09, 0x00, 0xCC, 0x12, 0x34, 0x6C, 0x00, 0x08, 0x08, 0x00 };
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u16, 0x1234), std.mem.readInt(u16, cpu.mem[0x0900..0x0902], .big));
    try testing.expectEqual(@as(u16, 0x0902), cpu.x);
    try testing.expectEqual(@as(u8, BDMACT), if (cpu.halted) BDMACT else 0);
}

test "DBG hardware breakpoint: tagged comparator A halts before the tagged opcode" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // 6x NOP then BGND at 0x0A06.
    const prog = [_]u8{ 0xA7, 0xA7, 0xA7, 0xA7, 0xA7, 0xA7, 0x00 };
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    // Arm DBG comparator A at 0x0A03 (as hwbreak.hcs12.program would over BDM).
    cpu.mem[DBG_DBGC1] = 0xE8; // DBGEN|ARM|TRGSEL|DBGBRK
    cpu.mem[DBG_DBGC2] = 0x20; // BDM route
    cpu.mem[DBG_DBGSC] = 0x00; // TRG = A only
    cpu.mem[DBG_DBGCAH] = 0x0A;
    cpu.mem[DBG_DBGCAL] = 0x03;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u16, 0x0A03), cpu.pc); // halted before executing 0x0A03, not at BGND
}

test "DBG hardware breakpoint: disarmed comparator does not halt" {
    var cpu = try testCpu();
    defer cpu.deinit();
    const prog = [_]u8{ 0xA7, 0xA7, 0x00 }; // NOP NOP BGND
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    cpu.mem[DBG_DBGC1] = 0x00; // not armed
    cpu.mem[DBG_DBGC2] = 0x20;
    cpu.mem[DBG_DBGCAH] = 0x0A;
    cpu.mem[DBG_DBGCAL] = 0x01;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u16, 0x0A03), cpu.pc); // ran to the BGND, breakpoint ignored
}

test "core: DEX sets Z so DEX/BNE loops terminate" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDX #3 ; (loop) INY ; DEX ; BNE loop ; BGND  - Y must end at exactly 3.
    // relies on DEX setting Z at X==0 (past bug)
    // BNE rel from next instr (0x0A07) back to INY (0x0A03) = -4 -> 0xFC.
    const prog = [_]u8{ 0xCE, 0x00, 0x03, 0x02, 0x09, 0x26, 0xFC, 0x00 };
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u16, 3), cpu.y);
    try testing.expectEqual(@as(u16, 0), cpu.x);
    try testing.expect(cpu.ccr & Z != 0); // DEX left Z set at exit
}

test "core: DBNE loop primitive counts a register down to zero" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDX #3 ; (loop) INY ; DBNE X,loop ; BGND  - Y should end at 3.
    // DBNE X, negative offset: opcode 04, lb = 001 (DBNE)<<5 | sign(1)<<4 | 5 (X)
    // = 0x35 (the 9-bit offset's sign bit lives in lb bit4, not rr's bit7).
    // 9-bit offset from next instr (0x0A07) back to INY (0x0A03) = -4 -> rr=0xFC.
    const prog = [_]u8{ 0xCE, 0x00, 0x03, 0x02, 0x04, 0x35, 0xFC, 0x00 };
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u16, 3), cpu.y);
    try testing.expectEqual(@as(u16, 0), cpu.x);
}

test "core: BSR/RTS use the stack" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.sp = 0x0A00;
    // LDS #$0A00 ; BSR sub ; BGND ; (sub) LDAB #$7 ; RTS
    // BSR rel from next instr (0x0905) to sub (0x0906) = +1.
    const prog = [_]u8{ 0xCF, 0x0A, 0x00, 0x07, 0x01, 0x00, 0xC6, 0x07, 0x3D };
    cpu.hostWrite(0x0900, &prog);
    cpu.pc = 0x0900;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u8, 7), cpu.b);
    try testing.expectEqual(@as(u16, 0x0A00), cpu.sp);
}

test "flash controller: word program latches then commits on CBEIF launch" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.fclkdiv_written = true;
    cpu.mem[0x4000] = 0xFF;
    cpu.mem[0x4001] = 0xFF;
    cpu.writeW(0x4000, 0x5AA5); // latch aligned word
    cpu.flashRegWrite(FCMD, CMD_WORD_PROG);
    cpu.flashRegWrite(FSTAT, CBEIF); // launch
    try testing.expectEqual(@as(u8, 0x5A), cpu.mem[0x4000]);
    try testing.expectEqual(@as(u8, 0xA5), cpu.mem[0x4001]);
    try testing.expectEqual(@as(u8, CBEIF | CCIF), cpu.fstat);
}

test "flash controller: sector and mass erase set 0xFF; erase-verify sets BLANK" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.fclkdiv_written = true;
    cpu.mem[0x4010] = 0x00;
    cpu.mem[0x5000] = 0x00;
    // sector-erase the 512-B sector at 0x4010
    cpu.writeW(0x4010, 0);
    cpu.flashRegWrite(FCMD, CMD_SECTOR_ERASE);
    cpu.flashRegWrite(FSTAT, CBEIF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0x4010]);
    try testing.expectEqual(@as(u8, 0x00), cpu.mem[0x5000]); // different sector untouched
    cpu.writeW(0x5000, 0);
    cpu.flashRegWrite(FCMD, CMD_MASS_ERASE);
    cpu.flashRegWrite(FSTAT, CBEIF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0x5000]);
    cpu.writeW(0x4000, 0);
    cpu.flashRegWrite(FCMD, CMD_ERASE_VERIFY);
    cpu.flashRegWrite(FSTAT, CBEIF);
    try testing.expect(cpu.fstat & BLANK != 0);
}

test "ftmrg D-flash: program (0x11) and erase-sector (0x12) via FCCOB" {
    var cpu = try Cpu.initGmmc(testing.allocator, 0x0100);
    defer cpu.deinit();
    // Program two words at D-flash global 0x000400 (dflash[0..4]).
    // FCCOB0 = 0x11<<8 | addr[23:16]=0; FCCOB1 = 0x0400; FCCOB2/3 = data words.
    const base = cpu.ctrl_base;
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 0);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), G_CMD_PROGRAM_D);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 1);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0x04);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 2);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0xAB);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0xCD);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 3);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0x12);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x34);
    cpu.write8Gmmc(@intCast(base + G_FSTAT), G_CCIF); // launch
    try testing.expectEqual(@as(u8, 0xAB), cpu.dflash[0]);
    try testing.expectEqual(@as(u8, 0xCD), cpu.dflash[1]);
    try testing.expectEqual(@as(u8, 0x12), cpu.dflash[2]);
    try testing.expectEqual(@as(u8, 0x34), cpu.dflash[3]);
    try testing.expect(cpu.fstat & G_ACCERR == 0);

    // erase D-flash sector at 0x000400 -> 0xFF
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 0);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), G_CMD_ERASE_SECTOR_D);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 1);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0x04);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FSTAT), G_CCIF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.dflash[0]);
    try testing.expectEqual(@as(u8, 0xFF), cpu.dflash[3]);
    // P-flash untouched by D-flash commands.
    try testing.expectEqual(@as(u8, 0xFF), cpu.pflash[0x8000]);
}

test "ftmrg D-flash: a bare high FCCOBIX before a program launch doesn't read ccob OOB" {
    var cpu = try Cpu.initGmmc(testing.allocator, 0x0100);
    defer cpu.deinit();
    const base = cpu.ctrl_base;
    // bare FCCOBIX=7 raises high-water mark without writing ccob[6/7]; a normal
    // program at 0/1 must not derive words>4 or read past ccob[5].
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 7);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 0);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), G_CMD_PROGRAM_D);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 1);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0x04);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0x00);
    cpu.write8Gmmc(@intCast(base + G_FCCOBIX), 2);
    cpu.write8Gmmc(@intCast(base + G_FCCOBHI), 0x5A);
    cpu.write8Gmmc(@intCast(base + G_FCCOBLO), 0xA5);
    cpu.write8Gmmc(@intCast(base + G_FSTAT), G_CCIF); // launch - must not panic
    try testing.expectEqual(@as(u8, 0x5A), cpu.dflash[0]); // the one word we wrote
    try testing.expectEqual(@as(u8, 0xA5), cpu.dflash[1]);
}

test "ftmrg D-flash: readGlobalWord path resolves to the D-flash array" {
    var cpu = try Cpu.initGmmc(testing.allocator, 0x0100);
    defer cpu.deinit();
    cpu.dflash[0] = 0x5A; // global 0x000400
    cpu.dflash[1] = 0xA5;
    // global 0x400 via PPAGE=0 + 0x8000 window -> 0x8400.
    cpu.ppage = 0;
    try testing.expectEqual(@as(u8, 0x5A), cpu.read8(0x8400));
    try testing.expectEqual(@as(u8, 0xA5), cpu.read8(0x8401));
}

test "core: indexed 5-bit, auto-inc and TFR D<->X" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDX #$0900 ; LDAA #$AB ; STAA 1,X+ ; TFR X,D ; BGND
    const prog = [_]u8{ 0xCE, 0x09, 0x00, 0x86, 0xAB, 0x6A, 0x30, 0xB7, 0x54, 0x00 };
    cpu.hostWrite(0x0A00, &prog);
    cpu.pc = 0x0A00;
    try cpu.run(1000);
    try testing.expectEqual(@as(u8, 0xAB), cpu.mem[0x0900]);
    try testing.expectEqual(@as(u16, 0x0901), cpu.x); // post-incremented
    try testing.expectEqual(@as(u16, 0x0901), cpu.d()); // TFR X,D
}
