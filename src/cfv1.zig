//! Functional ColdFire V1 (big-endian) emulator + MCF51 FLASH model: runs the
//! vendored CFV1 flash routine to cross-check the driver's ABI. Sparse mem (wide address space). Refs: ColdFire PRM, MCF51 RM.

const std = @import("std");

// CCR bits: X=4 N=3 Z=2 V=1 C=0.
const C: u8 = 0x01;
const V: u8 = 0x02;
const Z: u8 = 0x04;
const N: u8 = 0x08;
const X: u8 = 0x10;

// MCF51 FLASH regs (MC9S08-compatible), offsets from base.
const FCDIV = 0;
const FSTAT = 5;
const FCMD = 6;
const FCBEF: u8 = 0x80;
const FCCF: u8 = 0x40;
const FPVIOL: u8 = 0x20;
const FACCERR: u8 = 0x10;
const CMD_WORD_PROG: u8 = 0x20; // 32-bit longword program
const CMD_BURST_PROG: u8 = 0x25;
const CMD_PAGE_ERASE: u8 = 0x40;
const CMD_MASS_ERASE: u8 = 0x41;

/// XCSR run-state (HALT|STOP) the host polls after HALT.
pub const XCSR_RUNSTATE: u8 = 0xC0;

pub const Error = error{ UnimplementedOpcode, Runaway, BadEa } || std.mem.Allocator.Error;

pub const Cpu = struct {
    gpa: std.mem.Allocator,
    mem: std.AutoHashMap(u32, u8),
    d: [8]u32 = [_]u32{0} ** 8,
    a: [8]u32 = [_]u32{0} ** 8, // a[7] = SP
    pc: u32 = 0,
    ccr: u8 = 0,
    sr_hi: u8 = 0x27, // supervisor, IPL7
    vbr: u32 = 0,
    halted: bool = false,

    ctrl_base: u32,
    flash_lo: u32,
    flash_hi: u32,
    fstat: u8 = FCBEF | FCCF,
    fcmd: u8 = 0,
    fcdiv: u8 = 0,
    latched: bool = false,
    latch_addr: u32 = 0,
    latch_data: u32 = 0,

    bad_opcode: u16 = 0,
    bad_pc: u32 = 0,

    pub fn init(gpa: std.mem.Allocator, ctrl_base: u32, flash_lo: u32, flash_hi: u32) Error!Cpu {
        return .{ .gpa = gpa, .mem = std.AutoHashMap(u32, u8).init(gpa), .ctrl_base = ctrl_base, .flash_lo = flash_lo, .flash_hi = flash_hi };
    }
    pub fn deinit(self: *Cpu) void {
        self.mem.deinit();
    }

    fn inFlash(self: *const Cpu, addr: u32) bool {
        return addr >= self.flash_lo and addr <= self.flash_hi;
    }
    fn inCtrl(self: *const Cpu, addr: u32) bool {
        return addr >= self.ctrl_base and addr <= self.ctrl_base + 6;
    }
    fn rawGet(self: *Cpu, addr: u32) u8 {
        return self.mem.get(addr) orelse (if (self.inFlash(addr)) @as(u8, 0xFF) else 0);
    }
    fn rawPut(self: *Cpu, addr: u32, val: u8) void {
        self.mem.put(addr, val) catch {};
    }

    fn read8(self: *Cpu, addr: u32) u8 {
        if (self.inCtrl(addr)) {
            return switch (addr - self.ctrl_base) {
                FCDIV => self.fcdiv, // read-back must match write
                FSTAT => self.fstat,
                FCMD => self.fcmd,
                else => 0,
            };
        }
        return self.rawGet(addr);
    }
    fn write8(self: *Cpu, addr: u32, val: u8) void {
        if (self.inCtrl(addr)) {
            self.ctrlWrite(@intCast(addr - self.ctrl_base), val);
            return;
        }
        if (self.inFlash(addr)) { // flash write latches
            self.latch_addr = addr;
            self.latch_data = (self.latch_data << 8) | val; // accumulate longword bytes
            self.latched = true;
            return;
        }
        self.rawPut(addr, val);
    }
    fn read16(self: *Cpu, addr: u32) u16 {
        return (@as(u16, self.read8(addr)) << 8) | self.read8(addr + 1);
    }
    fn read32(self: *Cpu, addr: u32) u32 {
        return (@as(u32, self.read16(addr)) << 16) | self.read16(addr + 2);
    }
    fn write16(self: *Cpu, addr: u32, val: u16) void {
        self.write8(addr, @truncate(val >> 8));
        self.write8(addr + 1, @truncate(val));
    }
    fn write32(self: *Cpu, addr: u32, val: u32) void {
        // latch the whole 32-bit longword as one datum
        if (self.inFlash(addr) and !self.inCtrl(addr)) {
            self.latch_addr = addr;
            self.latch_data = val;
            self.latched = true;
            return;
        }
        self.write16(addr, @truncate(val >> 16));
        self.write16(addr + 2, @truncate(val));
    }

    fn ctrlWrite(self: *Cpu, off: u8, val: u8) void {
        switch (off) {
            FCDIV => self.fcdiv = val, // echo back for read-back check
            FCMD => self.fcmd = val,
            FSTAT => {
                if (val & FACCERR != 0) self.fstat &= ~FACCERR;
                if (val & FPVIOL != 0) self.fstat &= ~FPVIOL;
                if (val & FCBEF != 0 and self.latched) self.launch();
            },
            else => {},
        }
    }
    fn launch(self: *Cpu) void {
        switch (self.fcmd) {
            CMD_WORD_PROG, CMD_BURST_PROG => {
                // program clears bits; erased flash => 32-bit BE store
                const cur = self.read32flash(self.latch_addr);
                self.putFlash32(self.latch_addr, cur & self.latch_data);
            },
            CMD_PAGE_ERASE => {
                const page = self.latch_addr & ~@as(u32, 0x3FF); // 1 KB sector
                var p = page;
                while (p < page + 0x400) : (p += 1) {
                    if (p >= self.flash_lo and p <= self.flash_hi) _ = self.mem.remove(p);
                }
            },
            CMD_MASS_ERASE => {
                var it = self.mem.keyIterator();
                var kill: std.ArrayList(u32) = .empty;
                defer kill.deinit(self.gpa);
                while (it.next()) |k| {
                    if (self.inFlash(k.*)) kill.append(self.gpa, k.*) catch {};
                }
                for (kill.items) |k| _ = self.mem.remove(k);
            },
            else => {},
        }
        self.fstat = FCBEF | FCCF;
        self.latched = false;
        self.latch_data = 0;
    }
    fn read32flash(self: *Cpu, addr: u32) u32 {
        return (@as(u32, self.rawGet(addr)) << 24) | (@as(u32, self.rawGet(addr + 1)) << 16) |
            (@as(u32, self.rawGet(addr + 2)) << 8) | self.rawGet(addr + 3);
    }
    fn putFlash32(self: *Cpu, addr: u32, val: u32) void {
        self.rawPut(addr, @truncate(val >> 24));
        self.rawPut(addr + 1, @truncate(val >> 16));
        self.rawPut(addr + 2, @truncate(val >> 8));
        self.rawPut(addr + 3, @truncate(val));
    }

    // BDM host access: direct, bypasses the controller
    pub fn hostWrite(self: *Cpu, addr: u32, data: []const u8) void {
        for (data, 0..) |b, i| self.rawPut(addr + @as(u32, @intCast(i)), b);
    }
    pub fn hostRead(self: *Cpu, addr: u32, out: []u8) void {
        for (out, 0..) |*b, i| b.* = self.rawGet(addr + @as(u32, @intCast(i)));
    }
    pub fn hostReadByte(self: *Cpu, addr: u32) u8 {
        return self.rawGet(addr);
    }

    fn setf(self: *Cpu, mask: u8, on: bool) void {
        if (on) self.ccr |= mask else self.ccr &= ~mask;
    }
    fn msb(size: u3) u32 {
        return switch (size) {
            1 => 0x80,
            2 => 0x8000,
            else => 0x8000_0000,
        };
    }
    fn maskOf(size: u3) u32 {
        return switch (size) {
            1 => 0xFF,
            2 => 0xFFFF,
            else => 0xFFFF_FFFF,
        };
    }
    fn setNZ(self: *Cpu, val: u32, size: u3) void {
        self.setf(N, val & msb(size) != 0);
        self.setf(Z, val & maskOf(size) == 0);
    }
    fn logicFlags(self: *Cpu, val: u32, size: u3) void {
        self.setNZ(val, size);
        self.setf(V, false);
        self.setf(C, false);
    }

    const Ea = union(enum) {
        d: u3,
        a: u3,
        mem: u32,
        imm: u32, // already fetched
    };

    fn signExt(v: u32, size: u3) u32 {
        return switch (size) {
            1 => @bitCast(@as(i32, @as(i8, @bitCast(@as(u8, @truncate(v)))))),
            2 => @bitCast(@as(i32, @as(i16, @bitCast(@as(u16, @truncate(v)))))),
            else => v,
        };
    }

    fn decodeEa(self: *Cpu, mode: u3, reg: u3, size: u3) Error!Ea {
        switch (mode) {
            0 => return .{ .d = reg },
            1 => return .{ .a = reg },
            2 => return .{ .mem = self.a[reg] },
            3 => { // (An)+
                const addr = self.a[reg];
                self.a[reg] +%= size;
                return .{ .mem = addr };
            },
            4 => { // -(An)
                self.a[reg] -%= size;
                return .{ .mem = self.a[reg] };
            },
            5 => { // (d16,An)
                const d16 = signExt(self.fetch16(), 2);
                return .{ .mem = self.a[reg] +% d16 };
            },
            6 => return .{ .mem = try self.decodeIndexed(self.a[reg]) },
            7 => switch (reg) {
                0 => return .{ .mem = signExt(self.fetch16(), 2) }, // (xxx).W
                1 => { // (xxx).L
                    const hi = self.fetch16();
                    const lo = self.fetch16();
                    return .{ .mem = (@as(u32, hi) << 16) | lo };
                },
                2 => { // (d16,PC)
                    const base = self.pc;
                    const d16 = signExt(self.fetch16(), 2);
                    return .{ .mem = base +% d16 };
                },
                3 => { // (d8,PC,Xn)
                    const base = self.pc;
                    return .{ .mem = try self.decodeIndexed(base) };
                },
                4 => switch (size) { // #imm
                    1, 2 => return .{ .imm = self.fetch16() & maskOf(size) },
                    else => {
                        const hi = self.fetch16();
                        const lo = self.fetch16();
                        return .{ .imm = (@as(u32, hi) << 16) | lo };
                    },
                },
                else => return error.BadEa,
            },
        }
    }
    fn decodeIndexed(self: *Cpu, base: u32) Error!u32 {
        const ext = self.fetch16();
        const da = ext & 0x8000 != 0; // 1 = An index
        const ireg: u3 = @truncate(ext >> 12);
        const scale: u5 = @as(u5, 1) << @truncate((ext >> 9) & 3);
        const disp = signExt(@as(u8, @truncate(ext)), 1);
        const idx: u32 = if (da) self.a[ireg] else self.d[ireg]; // longword index (W/L=1 on CFV1)
        return base +% (idx *% scale) +% disp;
    }

    /// Decode mode/reg fields (bits 5:3 / 2:0).
    fn eaOf(self: *Cpu, w: u16, size: u3) Error!Ea {
        return self.decodeEa(@truncate((w >> 3) & 7), @truncate(w & 7), size);
    }
    /// Address of a memory EA (LEA/JSR/JMP).
    fn eaAddr(ea: Ea) Error!u32 {
        return switch (ea) {
            .mem => |addr| addr,
            else => error.BadEa,
        };
    }

    fn readEa(self: *Cpu, ea: Ea, size: u3) u32 {
        return switch (ea) {
            .d => |r| self.d[r] & maskOf(size),
            .a => |r| self.a[r] & maskOf(size),
            .imm => |v| v,
            .mem => |addr| switch (size) {
                1 => self.read8(addr),
                2 => self.read16(addr),
                else => self.read32(addr),
            },
        };
    }
    fn writeEa(self: *Cpu, ea: Ea, size: u3, val: u32) void {
        switch (ea) {
            .d => |r| self.d[r] = (self.d[r] & ~maskOf(size)) | (val & maskOf(size)),
            .a => |r| self.a[r] = signExt(val, size), // address writes are full 32-bit
            .imm => {},
            .mem => |addr| switch (size) {
                1 => self.write8(addr, @truncate(val)),
                2 => self.write16(addr, @truncate(val)),
                else => self.write32(addr, val),
            },
        }
    }

    fn fetch16(self: *Cpu) u16 {
        const w = self.read16(self.pc);
        self.pc +%= 2;
        return w;
    }

    /// MOVE size field: 01=byte, 11=word, 10=long.
    fn moveSize(bits: u2) u3 {
        return switch (bits) {
            1 => 1,
            3 => 2,
            2 => 4,
            else => 4,
        };
    }
    /// Standard size field: 00=byte, 01=word, 10=long.
    fn stdSize(bits: u2) u3 {
        return switch (bits) {
            0 => 1,
            1 => 2,
            else => 4,
        };
    }

    /// One instruction; false once HALT self-halts.
    pub fn step(self: *Cpu) Error!bool {
        if (self.halted) return false;
        const op_pc = self.pc;
        const w = self.fetch16();
        switch (w) {
            0x4E71 => {}, // NOP
            0x4E75 => { // RTS
                self.pc = self.pop32();
            },
            0x4AC8 => { // HALT
                self.halted = true;
                return false;
            },
            0x4AFC => { // ILLEGAL
                self.bad_opcode = w;
                self.bad_pc = op_pc;
                return error.UnimplementedOpcode;
            },
            0x4E7B => { // MOVEC Ry,Rc
                const ext = self.fetch16();
                const da = ext & 0x8000 != 0;
                const rn: u3 = @truncate(ext >> 12);
                const rc = ext & 0x0FFF;
                const val = if (da) self.a[rn] else self.d[rn];
                if (rc == 0x801) self.vbr = val; // VBR; others accepted/ignored
            },
            else => try self.decode(w, op_pc),
        }
        return true;
    }

    fn decode(self: *Cpu, w: u16, op_pc: u32) Error!void {
        const top: u4 = @truncate(w >> 12);
        switch (top) {
            0x1, 0x2, 0x3 => try self.opMove(w, top),
            0x0 => try self.opBitAndImm(w, op_pc),
            0x4 => try self.op4(w, op_pc),
            0x5 => try self.opAddqSubq(w),
            0x6 => self.opBranch(w),
            0x7 => try self.opMoveqMvsz(w),
            0x8, 0x9, 0xC, 0xD => try self.opAlu(w, top),
            0xA => try self.opMov3q(w, op_pc),
            0xB => try self.opCmpEor(w),
            0xE => try self.opShift(w),
            else => self.unimpl(w, op_pc),
        }
    }
    fn unimpl(self: *Cpu, w: u16, op_pc: u32) void {
        self.bad_opcode = w;
        self.bad_pc = op_pc;
    }

    fn pop32(self: *Cpu) u32 {
        const v = self.read32(self.a[7]);
        self.a[7] +%= 4;
        return v;
    }
    fn push32(self: *Cpu, v: u32) void {
        self.a[7] -%= 4;
        self.write32(self.a[7], v);
    }

    fn opMove(self: *Cpu, w: u16, top: u4) Error!void {
        const size: u3 = moveSize(@truncate(top)); // 1->byte,2->long,3->word
        const src = try self.eaOf(w, size);
        const dmode: u3 = @truncate((w >> 6) & 7);
        const dreg: u3 = @truncate((w >> 9) & 7);
        const val = self.readEa(src, size);
        const dst = try self.decodeEa(dmode, dreg, size);
        if (dmode == 1) { // MOVEA: sign-extend to 32, no flags
            self.a[dreg] = signExt(val, size);
        } else {
            self.writeEa(dst, size, val);
            self.logicFlags(val, size);
        }
    }

    fn opMoveqMvsz(self: *Cpu, w: u16) Error!void {
        const dreg: u3 = @truncate((w >> 9) & 7);
        if (w & 0x0100 == 0) { // MOVEQ
            self.d[dreg] = signExt(@as(u8, @truncate(w)), 1);
            self.logicFlags(self.d[dreg], 4);
        } else { // MVS/MVZ
            const size: u3 = if (w & 0x40 != 0) 2 else 1;
            const src = try self.eaOf(w, size);
            const val = self.readEa(src, size);
            self.d[dreg] = if (w & 0x80 == 0) signExt(val, size) else (val & maskOf(size)); // MVS sign / MVZ zero
            self.logicFlags(self.d[dreg], 4);
        }
    }

    fn opMov3q(self: *Cpu, w: u16, op_pc: u32) Error!void {
        if (w & 0x0138 == 0x0140 or (w & 0xF1C0) == 0xA140) {
            const imm3: u3 = @truncate((w >> 9) & 7);
            const val: u32 = if (imm3 == 0) 0xFFFF_FFFF else imm3;
            const dst = try self.eaOf(w, 4);
            self.writeEa(dst, 4, val);
            self.logicFlags(val, 4);
        } else self.unimpl(w, op_pc);
    }

    fn opAddqSubq(self: *Cpu, w: u16) Error!void {
        const size = stdSize(@truncate((w >> 6) & 3));
        var data: u32 = (w >> 9) & 7;
        if (data == 0) data = 8;
        const isSub = w & 0x0100 != 0;
        const mode: u3 = @truncate((w >> 3) & 7);
        const reg: u3 = @truncate(w & 7);
        const dst = try self.decodeEa(mode, reg, size);
        if (mode == 1) { // An: full 32-bit, no flags
            if (isSub) self.a[reg] -%= data else self.a[reg] +%= data;
            return;
        }
        const cur = self.readEa(dst, size);
        const r = if (isSub) self.sub(cur, data, size) else self.add(cur, data, size);
        self.writeEa(dst, size, r);
    }

    fn opAlu(self: *Cpu, w: u16, top: u4) Error!void {
        // ADD=D, SUB=9, AND=C, OR=8. Opmode 010: ea op Dn -> Dn (long). 110: Dn op ea -> ea.
        const dreg: u3 = @truncate((w >> 9) & 7);
        const opmode: u3 = @truncate((w >> 6) & 7);
        const mode: u3 = @truncate((w >> 3) & 7);
        const reg: u3 = @truncate(w & 7);
        if (opmode == 7 or opmode == 3) { // ADDA/SUBA (long/word) -> An
            const size: u3 = if (opmode == 7) 4 else 2;
            const ea = try self.decodeEa(mode, reg, size);
            const v = signExt(self.readEa(ea, size), size);
            if (top == 0xD) self.a[dreg] +%= v else self.a[dreg] -%= v;
            return;
        }
        const size: u3 = 4; // CFV1 register/EA ALU is longword-only
        const toEa = opmode == 6;
        const ea = try self.decodeEa(mode, reg, size);
        const a_val = self.d[dreg];
        const b_val = self.readEa(ea, size);
        const r = switch (top) {
            0xD => self.add(if (toEa) b_val else a_val, if (toEa) a_val else b_val, size),
            0x9 => self.sub(if (toEa) b_val else a_val, if (toEa) a_val else b_val, size),
            0xC => blk: {
                const r = a_val & b_val;
                self.logicFlags(r, size);
                break :blk r;
            },
            0x8 => blk: {
                const r = a_val | b_val;
                self.logicFlags(r, size);
                break :blk r;
            },
            else => unreachable,
        };
        if (toEa) self.writeEa(ea, size, r) else self.d[dreg] = r;
    }

    fn opCmpEor(self: *Cpu, w: u16) Error!void {
        const dreg: u3 = @truncate((w >> 9) & 7);
        const opmode: u3 = @truncate((w >> 6) & 7);
        const mode: u3 = @truncate((w >> 3) & 7);
        const reg: u3 = @truncate(w & 7);
        if (opmode == 0b110) { // EOR Dy,<ea>
            const ea = try self.decodeEa(mode, reg, 4);
            const r = self.readEa(ea, 4) ^ self.d[dreg];
            self.writeEa(ea, 4, r);
            self.logicFlags(r, 4);
            return;
        }
        if (opmode == 0b111) { // CMPA.L
            const ea = try self.decodeEa(mode, reg, 4);
            _ = self.sub(self.a[dreg], self.readEa(ea, 4), 4);
            return;
        }
        // CMP: opmode 000=byte,001=word,010=long
        const size = stdSize(@truncate(opmode & 3));
        const ea = try self.decodeEa(mode, reg, size);
        _ = self.sub(self.d[dreg] & maskOf(size), self.readEa(ea, size), size);
    }

    fn opShift(self: *Cpu, w: u16) Error!void {
        // 1110 cnt dr ss i/r 0(ls/as) reg ; CFV1 longword, data reg only.
        const dr = w & 0x0100 != 0; // 1 = left
        const ir = w & 0x20 != 0; // 1 = count in Dy
        const logical = w & 0x18 == 0x08; // bits4:3: 00=AS,01=LS
        const reg: u3 = @truncate(w & 7);
        var cnt: u32 = (w >> 9) & 7;
        if (ir) cnt = self.d[cnt] & 63 else if (cnt == 0) cnt = 8;
        var val = self.d[reg];
        var carry: bool = self.ccr & C != 0;
        if (cnt == 0) {
            self.setf(C, false);
        } else {
            var i: u32 = 0;
            while (i < cnt) : (i += 1) {
                if (dr) {
                    carry = val & 0x8000_0000 != 0;
                    val <<= 1;
                } else {
                    carry = val & 1 != 0;
                    if (logical) val >>= 1 else val = @bitCast(@as(i32, @bitCast(val)) >> 1); // ASR sign-fills
                }
            }
            self.setf(C, carry);
            self.setf(X, carry);
        }
        self.d[reg] = val;
        self.setNZ(val, 4);
        self.setf(V, false);
    }

    fn opBitAndImm(self: *Cpu, w: u16, op_pc: u32) Error!void {
        // Static bit ops: 0000 1000 tt EA, ext word = bit number.
        if (w & 0xFF00 == 0x0800) {
            const btype: u2 = @truncate((w >> 6) & 3); // 00 BTST,01 BCHG,10 BCLR,11 BSET
            const mode: u3 = @truncate((w >> 3) & 7);
            const reg: u3 = @truncate(w & 7);
            const bitnum = self.fetch16() & 0x1F; // Dn: mod 32
            const dstIsReg = mode == 0;
            const size: u3 = if (dstIsReg) 4 else 1;
            const bit: u32 = @as(u32, 1) << @truncate(if (dstIsReg) bitnum else bitnum & 7);
            const ea = try self.decodeEa(mode, reg, size);
            const cur = self.readEa(ea, size);
            self.setf(Z, cur & bit == 0);
            const nv = switch (btype) {
                0 => cur, // BTST
                1 => cur ^ bit, // BCHG
                2 => cur & ~bit, // BCLR
                3 => cur | bit, // BSET
            };
            if (btype != 0) self.writeEa(ea, size, nv);
            return;
        }
        self.unimpl(w, op_pc);
    }

    fn op4(self: *Cpu, w: u16, op_pc: u32) Error!void {
        // LEA
        if (w & 0xF1C0 == 0x41C0) {
            const areg: u3 = @truncate((w >> 9) & 7);
            const ea = try self.eaOf(w, 4);
            self.a[areg] = try eaAddr(ea);
            return;
        }
        if (w & 0xFFC0 == 0x4E80) { // JSR <ea>
            const ea = try self.eaOf(w, 4);
            const target = try eaAddr(ea);
            self.push32(self.pc);
            self.pc = target;
            return;
        }
        if (w & 0xFFC0 == 0x4EC0) { // JMP <ea>
            const ea = try self.eaOf(w, 4);
            self.pc = try eaAddr(ea);
            return;
        }
        if (w & 0xFF00 == 0x4200) { // CLR
            const size = stdSize(@truncate((w >> 6) & 3));
            const ea = try self.eaOf(w, size);
            self.writeEa(ea, size, 0);
            self.setf(N, false);
            self.setf(Z, true);
            self.setf(V, false);
            self.setf(C, false);
            return;
        }
        if (w & 0xFF00 == 0x4A00) { // TST
            const size = stdSize(@truncate((w >> 6) & 3));
            const ea = try self.eaOf(w, size);
            self.logicFlags(self.readEa(ea, size), size);
            return;
        }
        if (w & 0xFFC0 == 0x4680) { // NOT.L (0100 0110 10 EA)
            const ea = try self.eaOf(w, 4);
            const r = ~self.readEa(ea, 4);
            self.writeEa(ea, 4, r);
            self.logicFlags(r, 4);
            return;
        }
        if (w & 0xFFC0 == 0x4480) { // NEG.L
            const ea = try self.eaOf(w, 4);
            const r = self.sub(0, self.readEa(ea, 4), 4);
            self.writeEa(ea, 4, r);
            return;
        }
        if (w & 0xFFC0 == 0x46C0) { // MOVE to SR
            const ea = try self.eaOf(w, 2);
            const v = self.readEa(ea, 2);
            self.sr_hi = @truncate(v >> 8);
            self.ccr = @truncate(v);
            return;
        }
        if (w & 0xFFC0 == 0x44C0) { // MOVE to CCR
            const ea = try self.eaOf(w, 2);
            self.ccr = @truncate(self.readEa(ea, 2));
            return;
        }
        self.unimpl(w, op_pc);
    }

    fn opBranch(self: *Cpu, w: u16) void {
        const cc_code: u4 = @truncate((w >> 8) & 0xF);
        var disp: i32 = @as(i8, @bitCast(@as(u8, @truncate(w))));
        const base = self.pc; // word after opword
        if (@as(u8, @truncate(w)) == 0x00) {
            disp = @as(i16, @bitCast(self.fetch16()));
        }
        const target: u32 = @bitCast(@as(i32, @bitCast(base)) + disp);
        if (cc_code == 1) { // BSR
            self.push32(self.pc);
            self.pc = target;
            return;
        }
        if (self.cond(cc_code)) self.pc = target;
    }
    fn cond(self: *Cpu, c: u4) bool {
        const cc = self.ccr & C != 0;
        const zz = self.ccr & Z != 0;
        const nn = self.ccr & N != 0;
        const vv = self.ccr & V != 0;
        return switch (c) {
            0 => true, // T (BRA)
            1 => true, // F; BSR handled earlier, treat as always
            2 => !cc and !zz, // HI
            3 => cc or zz, // LS
            4 => !cc, // CC/HS
            5 => cc, // CS/LO
            6 => !zz, // NE
            7 => zz, // EQ
            8 => !vv, // VC
            9 => vv, // VS
            10 => !nn, // PL
            11 => nn, // MI
            12 => nn == vv, // GE
            13 => nn != vv, // LT
            14 => (nn == vv) and !zz, // GT
            15 => zz or (nn != vv), // LE
        };
    }

    fn add(self: *Cpu, x: u32, y: u32, size: u3) u32 {
        const m = maskOf(size);
        const r = (x +% y) & m;
        const sm = msb(size);
        const sum: u64 = @as(u64, x & m) + (y & m);
        self.setf(C, sum > m);
        self.setf(V, (x ^ r) & (y ^ r) & sm != 0);
        self.setf(X, self.ccr & C != 0);
        self.setNZ(r, size);
        return r;
    }
    fn sub(self: *Cpu, x: u32, y: u32, size: u3) u32 {
        const m = maskOf(size);
        const r = (x -% y) & m;
        const sm = msb(size);
        self.setf(C, (x & m) < (y & m));
        self.setf(V, (x ^ y) & (x ^ r) & sm != 0);
        self.setf(X, self.ccr & C != 0);
        self.setNZ(r, size);
        return r;
    }

    pub fn run(self: *Cpu, max_steps: usize) Error!void {
        self.halted = false;
        var n: usize = 0;
        while (n < max_steps) : (n += 1) {
            if (!try self.step()) return;
            if (self.bad_opcode != 0) return error.UnimplementedOpcode;
        }
        return error.Runaway;
    }
};

const testing = std.testing;

test "cfv1 core: MOVE/MOVEQ/ADD and HALT self-halt" {
    var cpu = try Cpu.init(testing.allocator, 0xFF9820, 0x0, 0x1FFFF);
    defer cpu.deinit();
    // MOVEQ #5,D0 ; MOVEQ #3,D1 ; ADD.L D1,D0 ; MOVE.L D0,(0x00800400) ; HALT
    const prog = [_]u8{
        0x70, 0x05, // MOVEQ #5,D0
        0x72, 0x03, // MOVEQ #3,D1
        0xD0, 0x81, // ADD.L D1,D0  (opmode 010, ea=D1)
        0x23, 0xC0, 0x00, 0x80, 0x04, 0x00, // MOVE.L D0,(0x00800400).L
        0x4A, 0xC8, // HALT
    };
    cpu.hostWrite(0x00800A9C, &prog);
    cpu.pc = 0x00800A9C;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u32, 8), cpu.d[0]);
    try testing.expectEqual(@as(u32, 8), cpu.read32flash(0x00800400));
}

test "cfv1 flash controller: longword program + mass erase" {
    var cpu = try Cpu.init(testing.allocator, 0xFF9820, 0x0, 0x1FFFF);
    defer cpu.deinit();
    cpu.write32(0x1000, 0xDEADBEEF); // latch
    cpu.fcmd = CMD_WORD_PROG;
    cpu.latched = true;
    cpu.ctrlWrite(FSTAT, FCBEF); // launch
    try testing.expectEqual(@as(u32, 0xDEADBEEF), cpu.read32flash(0x1000));
    try testing.expectEqual(@as(u8, FCBEF | FCCF), cpu.fstat);
    cpu.write32(0x0, 0);
    cpu.fcmd = CMD_MASS_ERASE;
    cpu.latched = true;
    cpu.ctrlWrite(FSTAT, FCBEF);
    try testing.expectEqual(@as(u32, 0xFFFFFFFF), cpu.read32flash(0x1000));
}
