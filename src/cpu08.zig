//! Functional CPU08 (HCS08) emulator + MC9S08 FLASH model: runs the vendored
//! flash-routine binary to cross-check the driver's small-code ABI. Refs: CPU08RM, MC9S08 RM.

const std = @import("std");

// CCR masks (bits 6,5 read as 1, not modeled).
const C: u8 = 0x01;
const Z: u8 = 0x02;
const N: u8 = 0x04;
const I: u8 = 0x08;
const H: u8 = 0x10;
const V: u8 = 0x80;

const FCDIV = 0;
const FSTAT = 5;
const FCMD = 6;
const FCBEF: u8 = 0x80;
const FCCF: u8 = 0x40;
const FPVIOL: u8 = 0x20;
const FACCERR: u8 = 0x10;
const CMD_BLANK: u8 = 0x05;
const CMD_BYTE_PROG: u8 = 0x20;
const CMD_BURST_PROG: u8 = 0x25;
const CMD_PAGE_ERASE: u8 = 0x40;
const CMD_MASS_ERASE: u8 = 0x41;

/// BDCSCR.BDMACT: set when self-halted (BGND).
pub const BDMACT: u8 = 0x40;

pub const Error = error{ UnimplementedOpcode, Runaway } || std.mem.Allocator.Error;

pub const Cpu = struct {
    gpa: std.mem.Allocator,
    mem: []u8,
    a: u8 = 0,
    h: u8 = 0,
    x: u8 = 0,
    sp: u16 = 0x00FF,
    pc: u16 = 0,
    ccr: u8 = I,
    halted: bool = false,

    ctrl_base: u32,
    flash_lo: u32,
    flash_hi: u32,
    fstat: u8 = FCBEF | FCCF,
    fcmd: u8 = 0,
    latched: bool = false,
    latch_addr: u16 = 0,
    latch_data: u8 = 0,

    bad_opcode: u8 = 0,
    bad_pc: u16 = 0,

    pub fn init(gpa: std.mem.Allocator, ctrl_base: u32, flash_lo: u32, flash_hi: u32) Error!Cpu {
        const mem = try gpa.alloc(u8, 0x10000);
        @memset(mem, 0);
        // erased flash reads 0xFF
        const lo: usize = flash_lo;
        const end: usize = @min(@as(usize, flash_hi) + 1, mem.len);
        if (lo < end) @memset(mem[lo..end], 0xFF);
        return .{ .gpa = gpa, .mem = mem, .ctrl_base = ctrl_base, .flash_lo = flash_lo, .flash_hi = flash_hi };
    }
    pub fn deinit(self: *Cpu) void {
        self.gpa.free(self.mem);
    }

    fn hx(self: *const Cpu) u16 {
        return (@as(u16, self.h) << 8) | self.x;
    }
    fn setHx(self: *Cpu, v: u16) void {
        self.h = @truncate(v >> 8);
        self.x = @truncate(v);
    }
    fn setFlag(self: *Cpu, mask: u8, on: bool) void {
        if (on) self.ccr |= mask else self.ccr &= ~mask;
    }
    fn nz8(self: *Cpu, r: u8) void {
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }
    fn nz16(self: *Cpu, r: u16) void {
        self.setFlag(N, r & 0x8000 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }

    fn read(self: *Cpu, addr: u16) u8 {
        const a: u32 = addr;
        if (a >= self.ctrl_base and a <= self.ctrl_base + 6) {
            return switch (a - self.ctrl_base) {
                FCDIV => 0x80, // DIVLD set
                FSTAT => self.fstat,
                FCMD => self.fcmd,
                else => 0,
            };
        }
        return self.mem[addr];
    }
    fn write(self: *Cpu, addr: u16, val: u8) void {
        const a: u32 = addr;
        if (a >= self.ctrl_base and a <= self.ctrl_base + 6) {
            self.flashRegWrite(@intCast(a - self.ctrl_base), val);
            return;
        }
        if (a >= self.flash_lo and a <= self.flash_hi) {
            // latch only; array commits at launch
            self.latch_addr = addr;
            self.latch_data = val;
            self.latched = true;
            return;
        }
        self.mem[addr] = val;
    }
    fn flashRegWrite(self: *Cpu, off: u8, val: u8) void {
        switch (off) {
            FCMD => self.fcmd = val,
            FSTAT => {
                if (val & FACCERR != 0) self.fstat &= ~FACCERR;
                if (val & FPVIOL != 0) self.fstat &= ~FPVIOL;
                if (val & FCBEF != 0 and self.latched) self.launch();
            },
            else => {}, // FCDIV/FOPT/FCNFG/FPROT: inert
        }
    }
    fn launch(self: *Cpu) void {
        switch (self.fcmd) {
            CMD_BYTE_PROG, CMD_BURST_PROG => {
                // program only clears bits; on erased flash a store
                self.mem[self.latch_addr] &= self.latch_data;
            },
            CMD_PAGE_ERASE => {
                const page: u32 = self.latch_addr & ~@as(u32, 0x1FF); // 512-byte page
                var p = page;
                while (p < page + 0x200 and p <= self.flash_hi) : (p += 1) {
                    if (p >= self.flash_lo) self.mem[@intCast(p)] = 0xFF;
                }
            },
            CMD_MASS_ERASE => {
                var p = self.flash_lo;
                while (p <= self.flash_hi and p <= 0xFFFF) : (p += 1) self.mem[@intCast(p)] = 0xFF;
            },
            CMD_BLANK => {}, // routine blank-checks via direct reads
            else => {},
        }
        self.fstat = FCBEF | FCCF; // success
        self.latched = false;
    }

    // BDM host access: direct, bypasses the controller
    pub fn hostWrite(self: *Cpu, addr: u16, data: []const u8) void {
        for (data, 0..) |b, i| self.mem[addr + i] = b;
    }
    pub fn hostRead(self: *Cpu, addr: u16, out: []u8) void {
        for (out, 0..) |*b, i| b.* = self.mem[addr + i];
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
    fn branch(self: *Cpu, taken: bool) void {
        const rel: i8 = @bitCast(self.fetch8());
        if (taken) self.pc = @intCast(@as(i32, self.pc) + rel);
    }

    /// One instruction; false once self-halted (BGND).
    pub fn step(self: *Cpu) Error!bool {
        if (self.halted) return false;
        const op = self.fetch8();
        switch (op) {
            0x82 => { // BGND (self-halt)
                self.halted = true;
                return false;
            },
            0x9F => self.a = self.x, // TXA (no flags)
            0x5F => { // CLRX
                self.x = 0;
                self.nz8(0);
            },
            0xA6 => { // LDA #ii
                self.a = self.fetch8();
                self.nz8(self.a);
            },
            0xA5 => { // BIT #ii
                self.nz8(self.a & self.fetch8());
            },
            0xA1 => self.cmp(self.fetch8()), // CMP #ii
            0xAF => { // AIX #ii (signed, no flags)
                const ii: i8 = @bitCast(self.fetch8());
                self.setHx(@bitCast(@as(i16, @bitCast(self.hx())) +% ii));
            },
            0x45 => { // LDHX #ii16
                self.setHx(self.fetch16());
                self.nz16(self.hx());
            },
            0x55 => { // LDHX dd (direct)
                const dd = self.fetch8();
                self.h = self.read(dd);
                self.x = self.read(dd +% 1);
                self.nz16(self.hx());
            },
            0x35 => { // STHX dd (direct)
                const dd = self.fetch8();
                self.write(dd, self.h);
                self.write(dd +% 1, self.x);
                self.nz16(self.hx());
            },
            0xB6 => { // LDA dd
                self.a = self.read(self.fetch8());
                self.nz8(self.a);
            },
            0xB7 => { // STA dd
                self.write(self.fetch8(), self.a);
                self.nz8(self.a);
            },
            0xBB => self.add(self.read(self.fetch8()), false), // ADD dd
            0xB9 => self.add(self.read(self.fetch8()), true), // ADC dd
            0xF6 => { // LDA ,X
                self.a = self.read(self.hx());
                self.nz8(self.a);
            },
            0xF7 => { // STA ,X
                self.write(self.hx(), self.a);
                self.nz8(self.a);
            },
            0xE7 => { // STA ff,X
                const ff = self.fetch8();
                self.write(self.hx() +% ff, self.a);
                self.nz8(self.a);
            },
            0xE4 => { // AND ff,X
                const ff = self.fetch8();
                self.a &= self.read(self.hx() +% ff);
                self.nz8(self.a);
            },
            0xE5 => { // BIT ff,X
                const ff = self.fetch8();
                self.nz8(self.a & self.read(self.hx() +% ff));
            },
            0x27 => self.branch(self.ccr & Z != 0), // BEQ
            0x26 => self.branch(self.ccr & Z == 0), // BNE
            else => {
                self.bad_opcode = op;
                self.bad_pc = self.pc -% 1;
                return error.UnimplementedOpcode;
            },
        }
        return true;
    }

    fn cmp(self: *Cpu, m: u8) void {
        const r: u8 = self.a -% m;
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, self.a == m);
        self.setFlag(C, self.a < m); // borrow
        self.setFlag(V, (self.a ^ m) & (self.a ^ r) & 0x80 != 0);
    }
    fn add(self: *Cpu, m: u8, carry: bool) void {
        const cin: u16 = if (carry and self.ccr & C != 0) 1 else 0;
        const r: u16 = @as(u16, self.a) + m + cin;
        const r8: u8 = @truncate(r);
        self.setFlag(H, ((self.a & 0xF) + (m & 0xF) + @as(u8, @intCast(cin))) > 0xF);
        self.setFlag(C, r > 0xFF);
        self.setFlag(V, (self.a ^ r8) & (m ^ r8) & 0x80 != 0);
        self.a = r8;
        self.setFlag(N, r8 & 0x80 != 0);
        self.setFlag(Z, r8 == 0);
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
    return Cpu.init(testing.allocator, 0x1820, 0xE000, 0xFFFF);
}

test "core: LDA#/STA-dir/LDHX#/AIX/BEQ and self-halt on BGND" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDHX #$0200 ; LDA #$AB ; STA $10 ; AIX #4 ; BGND
    const prog = [_]u8{ 0x45, 0x02, 0x00, 0xA6, 0xAB, 0xB7, 0x10, 0xAF, 0x04, 0x82 };
    cpu.hostWrite(0x0300, &prog);
    cpu.pc = 0x0300;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u8, 0xAB), cpu.mem[0x10]);
    try testing.expectEqual(@as(u16, 0x0204), cpu.hx());
    try testing.expectEqual(@as(u8, BDMACT), if (cpu.halted) BDMACT else 0);
}

test "flash controller: program latches then commits on FCBEF launch" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDHX #$E000 ; LDA #$5A ; STA ,X ; LDA #$25 ; STA $1826(FCMD) ; LDA #$80 ; STA $1825(FSTAT) ; BGND
    const prog = [_]u8{
        0x45, 0xE0, 0x00, // LDHX #$E000
        0xA6, 0x5A, // LDA #$5A
        0xF7, // STA ,X   (latch addr=0xE000 data=0x5A)
        0xA6, 0x25, 0xC7, 0x18, 0x26, // LDA #$25 ; STA $1826 (FCMD)  -- 0xC7 = STA ext
        0xA6, 0x80, 0xC7, 0x18, 0x25, // LDA #$80 ; STA $1825 (FSTAT launch)
        0x82,
    };
    // 0xC7 (STA ext) isn't in the routine's 21; drive the controller via host writes below.
    _ = prog;
    cpu.mem[0xE000] = 0xFF; // erased
    cpu.write(0xE000, 0x5A); // latch
    cpu.fcmd = CMD_BURST_PROG;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF); // launch
    try testing.expectEqual(@as(u8, 0x5A), cpu.mem[0xE000]);
    try testing.expectEqual(@as(u8, FCBEF | FCCF), cpu.fstat);
}

test "flash controller: page and mass erase set 0xFF" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.mem[0xE010] = 0x00;
    cpu.mem[0xF000] = 0x00;
    cpu.write(0xE010, 0); // latch
    cpu.fcmd = CMD_PAGE_ERASE;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xE010]);
    try testing.expectEqual(@as(u8, 0x00), cpu.mem[0xF000]); // different page untouched
    cpu.write(0xF000, 0);
    cpu.fcmd = CMD_MASS_ERASE;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xF000]);
}

test "core: ADD/ADC carry chain and CMP#" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.mem[0x20] = 0xFF;
    cpu.mem[0x21] = 0x01;
    cpu.a = 0x01;
    cpu.add(cpu.mem[0x20], false); // 0x01 + 0xFF = 0x00, carry out
    try testing.expectEqual(@as(u8, 0x00), cpu.a);
    try testing.expect(cpu.ccr & C != 0);
    cpu.a = 0x00;
    cpu.add(cpu.mem[0x21], true); // 0x00 + 0x01 + C(1) = 0x02
    try testing.expectEqual(@as(u8, 0x02), cpu.a);
    cpu.a = 0xFF;
    cpu.cmp(0xFF);
    try testing.expect(cpu.ccr & Z != 0); // equal
    cpu.cmp(0x00);
    try testing.expect(cpu.ccr & Z == 0);
}
