//! Hardware PC breakpoints per family: `hcs08` (DBG module), `hcs12` (classic-S12
//! DBGV1), `cfv1` (PBR debug registers); program on-chip comparators to halt on PC match.

const std = @import("std");
const flash = @import("flash.zig");

pub const Mem = flash.Mem;

/// Two-comparator PC-breakpoint set (HCS08/HCS12 DBG), compacted so the first
/// live breakpoint maps to comparator A.
fn ComparatorSet(comptime armFn: fn (Mem, ?u16, ?u16) anyerror!void) type {
    return struct {
        const Self = @This();
        slots: [2]?u16 = .{ null, null },

        pub fn has(self: Self, addr: u16) bool {
            for (self.slots) |s| if (s == addr) return true;
            return false;
        }
        pub fn add(self: *Self, addr: u16) error{NoSlot}!void {
            if (self.has(addr)) return;
            for (&self.slots) |*s| {
                if (s.* == null) {
                    s.* = addr;
                    return;
                }
            }
            return error.NoSlot;
        }
        pub fn remove(self: *Self, addr: u16) void {
            for (&self.slots) |*s| {
                if (s.* == addr) s.* = null;
            }
        }
        fn comparators(self: Self) struct { a: ?u16, b: ?u16 } {
            var a: ?u16 = null;
            var b: ?u16 = null;
            for (self.slots) |s| {
                if (s == null) continue;
                if (a == null) a = s else b = s;
            }
            return .{ .a = a, .b = b };
        }
        pub fn apply(self: Self, mem: Mem) anyerror!void {
            const c = self.comparators();
            try armFn(mem, c.a, c.b);
        }
    };
}

/// HCS08 DBG module (0x1810-0x1818, comparators A/B). TAG mode = break before
/// the tagged opcode (GDB Z1), end-trace. Ref MC9S08 RM (DBG chapter). ENBDM
/// (BDCSCR) must be set at connect or a match vectors through SWI.
pub const hcs08 = struct {
    pub const DBGCAH: u32 = 0x1810; // comparator A, ADDR[15:8]
    pub const DBGCAL: u32 = 0x1811; // comparator A, ADDR[7:0]
    pub const DBGCBH: u32 = 0x1812; // comparator B, ADDR[15:8]
    pub const DBGCBL: u32 = 0x1813; // comparator B, ADDR[7:0]
    pub const DBGC: u32 = 0x1816; // control
    pub const DBGT: u32 = 0x1817; // trigger

    pub const DBGEN: u8 = 0x80;
    pub const ARM: u8 = 0x40;
    pub const TAG: u8 = 0x20;
    pub const BRKEN: u8 = 0x10;
    // DBGT bits: TRGSEL(0x80)=tag, BEGIN(0x40)=begin-trace (we want end-trace, 0),
    // TRG[3:0]: 0=A-only, 1=A-OR-B.
    pub const TRGSEL: u8 = 0x80;
    pub const TRG_A_ONLY: u8 = 0x00;
    pub const TRG_A_OR_B: u8 = 0x01;

    /// Program the DBG module: `a`->comparator A, `b`->comparator B (optional). Must
    /// disarm before changing comparators/DBGT. Both null -> disarmed.
    pub fn program(mem: Mem, a_in: ?u16, b_in: ?u16) anyerror!void {
        try mem.write(DBGC, 0x00); // disarm; comparators/DBGT writable only while ARM=0
        // Map a single comparator to A: TRG_A_ONLY arms A, so a B-only trigger would
        // otherwise arm A's stale address and never fire.
        var a = a_in;
        var b = b_in;
        if (a == null) {
            a = b;
            b = null;
        }
        const ca = a orelse return;

        try mem.write(DBGCAH, @intCast(ca >> 8));
        try mem.write(DBGCAL, @truncate(ca));
        if (b) |addr| {
            try mem.write(DBGCBH, @intCast(addr >> 8));
            try mem.write(DBGCBL, @truncate(addr));
        }
        const trg: u8 = if (b != null) TRG_A_OR_B else TRG_A_ONLY;
        try mem.write(DBGT, TRGSEL | trg); // tag, end-trace
        try mem.write(DBGC, DBGEN | ARM | TAG | BRKEN); // 0xF0: arm
    }

    pub const Set = ComparatorSet(program);
};

// CFV1 PC hardware breakpoints. Unlike HCS08's memory-mapped DBG, ColdFire debug
// registers are written over BDM (WRITE_DREG): PBR0..PBR3 halt on PC match, TDR
// arms them. Ref GdbBreakpoints_CFV1.
pub const cfv1 = struct {
    // Debug-register numbers (CFV1_DReg*, USBDM_API.h).
    pub const DRegTDR: u16 = 0x07;
    pub const DRegPBR0: u16 = 0x08;
    pub const DRegPBMR: u16 = 0x09; // mask for PBR0
    pub const DRegPBR1: u16 = 0x18;
    pub const DRegPBR2: u16 = 0x1A;
    pub const DRegPBR3: u16 = 0x1B;

    pub const TDR_TRC_HALT: u32 = 1 << 30; // trigger response = halt
    pub const TDR_L1T: u32 = 1 << 14;
    pub const TDR_L1EBL: u32 = 1 << 13; // enable level-1 breakpoint
    pub const TDR_L1EPC: u32 = 1 << 1; // enable PC breakpoint
    pub const tdr_pc_halt: u32 = TDR_TRC_HALT | TDR_L1T | TDR_L1EBL | TDR_L1EPC;

    pub const max_pc_breakpoints = 4; // PBR0..PBR3

    pub const DRegWriter = struct {
        ctx: *anyopaque,
        writeFn: *const fn (ctx: *anyopaque, reg: u16, value: u32) anyerror!void,
        fn write(self: DRegWriter, reg: u16, value: u32) anyerror!void {
            return self.writeFn(self.ctx, reg, value);
        }
    };

    pub const BreakpointSet = struct {
        slots: [max_pc_breakpoints]?u32 = .{ null, null, null, null },

        pub fn has(self: BreakpointSet, addr: u32) bool {
            for (self.slots) |s| if (s == addr) return true;
            return false;
        }
        pub fn add(self: *BreakpointSet, addr: u32) error{NoSlot}!void {
            if (self.has(addr)) return;
            for (&self.slots) |*s| {
                if (s.* == null) {
                    s.* = addr;
                    return;
                }
            }
            return error.NoSlot;
        }
        pub fn remove(self: *BreakpointSet, addr: u32) void {
            for (&self.slots) |*s| {
                if (s.* == addr) s.* = null;
            }
        }

        /// Reprogram the debug module: PBR0 + zero mask, PBR1..3 with valid bit
        /// (unused cleared), TDR armed iff any slot live. Ref activate/deactivate.
        pub fn apply(self: BreakpointSet, w: DRegWriter) anyerror!void {
            var tdr: u32 = 0;
            if (self.slots[0]) |a| {
                tdr = tdr_pc_halt;
                try w.write(DRegPBR0, a & ~@as(u32, 1));
                try w.write(DRegPBMR, 0);
            }
            const rest = [_]struct { reg: u16, slot: usize }{
                .{ .reg = DRegPBR1, .slot = 1 },
                .{ .reg = DRegPBR2, .slot = 2 },
                .{ .reg = DRegPBR3, .slot = 3 },
            };
            for (rest) |r| {
                if (self.slots[r.slot]) |a| {
                    tdr = tdr_pc_halt;
                    try w.write(r.reg, a | 1); // bit0 = valid flag on PBR1..3
                } else {
                    try w.write(r.reg, 0);
                }
            }
            try w.write(DRegTDR, tdr);
        }
    };
};

// Classic S12 (MC9S12C/D/GC, DBGV1) HW PC breakpoints via the DBG module at
// 0x0020 (MC9S12C RM ch.7): comparators A/B, tagged -> background (DBGC2.BDM).
// NOT S12X/S12G. ENBDM (BDMSTS 0xFF01 bit7) at connect, else a match -> SWI.
pub const hcs12 = struct {
    pub const DBGC1: u32 = 0x0020; // control 1
    pub const DBGSC: u32 = 0x0021; // status + TRG[3:0] trigger select
    pub const DBGC2: u32 = 0x0028; // control 2 (BKABEN, BDM route, ...)
    pub const DBGC3: u32 = 0x0029; // control 3 (address masks, R/W qualifiers)
    pub const DBGCAX: u32 = 0x002A; // comparator A extended/page (0 for 16-bit PC)
    pub const DBGCAH: u32 = 0x002B; // comparator A addr[15:8]
    pub const DBGCAL: u32 = 0x002C; // comparator A addr[7:0]
    pub const DBGCBX: u32 = 0x002D; // comparator B extended
    pub const DBGCBH: u32 = 0x002E; // comparator B addr[15:8]
    pub const DBGCBL: u32 = 0x002F; // comparator B addr[7:0]

    pub const DBGEN: u8 = 0x80;
    pub const ARM: u8 = 0x40;
    pub const TRGSEL: u8 = 0x20; // tagged: break before the tagged opcode
    pub const DBGBRK: u8 = 0x08; // enable CPU break request
    pub const DBGC2_BDM: u8 = 0x20; // DBG mode (BKABEN=0), break -> background
    pub const TRG_A_ONLY: u8 = 0x00;
    pub const TRG_A_OR_B: u8 = 0x01;

    /// Arm comparator A (and optional B) as tagged PC breakpoints entering
    /// background on match; both null -> disarmed. Disarm first: only DBGEN/ARM
    /// are writable while armed, so comparators must be set with ARM=0.
    pub fn program(mem: Mem, a_in: ?u16, b_in: ?u16) anyerror!void {
        try mem.write(DBGC1, 0x00);
        var a = a_in;
        var b = b_in;
        if (a == null) { // fold a lone B onto A (TRG_A_ONLY only arms A)
            a = b;
            b = null;
        }
        const ca = a orelse return;
        try mem.write(DBGC2, DBGC2_BDM);
        try mem.write(DBGC3, 0x00); // exact address, R/W not qualified (tagged)
        try mem.write(DBGCAX, 0x00);
        try mem.write(DBGCAH, @intCast(ca >> 8));
        try mem.write(DBGCAL, @truncate(ca));
        if (b) |addr| {
            try mem.write(DBGCBX, 0x00);
            try mem.write(DBGCBH, @intCast(addr >> 8));
            try mem.write(DBGCBL, @truncate(addr));
        }
        try mem.write(DBGSC, if (b != null) TRG_A_OR_B else TRG_A_ONLY);
        try mem.write(DBGC1, DBGEN | ARM | TRGSEL | DBGBRK); // 0xE8: arm
    }

    pub const Set = ComparatorSet(program);
};

const testing = std.testing;

const Recorder = struct {
    writes: std.ArrayList(struct { addr: u32, val: u8 }) = .empty,
    gpa: std.mem.Allocator,

    fn deinit(self: *Recorder) void {
        self.writes.deinit(self.gpa);
    }
    fn writeFn(ctx: *anyopaque, addr: u32, val: u8) anyerror!void {
        const self: *Recorder = @ptrCast(@alignCast(ctx));
        try self.writes.append(self.gpa, .{ .addr = addr, .val = val });
    }
    fn mem(self: *Recorder) Mem {
        return .{ .ctx = self, .writeFn = writeFn };
    }
    fn find(self: *Recorder, addr: u32) ?u8 {
        var v: ?u8 = null;
        for (self.writes.items) |w| if (w.addr == addr) {
            v = w.val;
        };
        return v;
    }
};

test "hcs08: one breakpoint arms comparator A in tag/end-trace mode" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs08.program(r.mem(), 0xE00A, null);

    try testing.expectEqual(@as(?u8, 0xE0), r.find(hcs08.DBGCAH));
    try testing.expectEqual(@as(?u8, 0x0A), r.find(hcs08.DBGCAL));
    try testing.expectEqual(@as(?u8, hcs08.TRGSEL | hcs08.TRG_A_ONLY), r.find(hcs08.DBGT)); // 0x80
    try testing.expectEqual(@as(?u8, hcs08.DBGEN | hcs08.ARM | hcs08.TAG | hcs08.BRKEN), r.find(hcs08.DBGC)); // 0xF0
    // DBGC 0x00 (disarm) then 0xF0 (arm)
    try testing.expectEqual(@as(u8, 0x00), r.writes.items[0].val);
    try testing.expectEqual(@as(u32, hcs08.DBGC), r.writes.items[0].addr);
}

test "hcs08: two breakpoints use A-OR-B trigger" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs08.program(r.mem(), 0x8000, 0x8100);
    try testing.expectEqual(@as(?u8, 0x80), r.find(hcs08.DBGCAH));
    try testing.expectEqual(@as(?u8, 0x81), r.find(hcs08.DBGCBH));
    try testing.expectEqual(@as(?u8, 0x00), r.find(hcs08.DBGCBL));
    try testing.expectEqual(@as(?u8, hcs08.TRGSEL | hcs08.TRG_A_OR_B), r.find(hcs08.DBGT)); // 0x81
}

test "hcs08: a B-only breakpoint is normalized onto comparator A (not lost)" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs08.program(r.mem(), null, 0x9ABC);
    // arms 0x9ABC via A (TRG_A_ONLY), not a stale A addr
    try testing.expectEqual(@as(?u8, 0x9A), r.find(hcs08.DBGCAH));
    try testing.expectEqual(@as(?u8, 0xBC), r.find(hcs08.DBGCAL));
    try testing.expectEqual(@as(?u8, null), r.find(hcs08.DBGCBH));
    try testing.expectEqual(@as(?u8, hcs08.TRGSEL | hcs08.TRG_A_ONLY), r.find(hcs08.DBGT));
    try testing.expectEqual(@as(?u8, hcs08.DBGEN | hcs08.ARM | hcs08.TAG | hcs08.BRKEN), r.find(hcs08.DBGC));
}

test "hcs08: no breakpoints just disarms" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs08.program(r.mem(), null, null);
    try testing.expectEqual(@as(usize, 1), r.writes.items.len);
    try testing.expectEqual(@as(u8, 0x00), r.writes.items[0].val);
}

test "hcs08.Set add/remove/compact and NoSlot" {
    var set = hcs08.Set{};
    try set.add(0x1000);
    try set.add(0x2000);
    try testing.expectError(error.NoSlot, set.add(0x3000));
    try testing.expect(set.has(0x1000));
    set.remove(0x1000);
    try testing.expect(!set.has(0x1000));
    // removing A compacts the remaining bp to A
    const c = set.comparators();
    try testing.expectEqual(@as(?u16, 0x2000), c.a);
    try testing.expectEqual(@as(?u16, null), c.b);
    try set.add(0x3000);
    try testing.expect(set.has(0x3000));
}

test "hcs08.Set.apply programs the DBG module" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    var set = hcs08.Set{};
    try set.add(0xABCD);
    try set.apply(r.mem());
    try testing.expectEqual(@as(?u8, 0xAB), r.find(hcs08.DBGCAH));
    try testing.expectEqual(@as(?u8, 0xCD), r.find(hcs08.DBGCAL));
}

const DRegRecorder = struct {
    writes: std.ArrayList(struct { reg: u16, val: u32 }) = .empty,
    gpa: std.mem.Allocator,
    fn deinit(self: *DRegRecorder) void {
        self.writes.deinit(self.gpa);
    }
    fn writeFn(ctx: *anyopaque, reg: u16, val: u32) anyerror!void {
        const self: *DRegRecorder = @ptrCast(@alignCast(ctx));
        try self.writes.append(self.gpa, .{ .reg = reg, .val = val });
    }
    fn writer(self: *DRegRecorder) cfv1.DRegWriter {
        return .{ .ctx = self, .writeFn = writeFn };
    }
    fn find(self: *DRegRecorder, reg: u16) ?u32 {
        var v: ?u32 = null;
        for (self.writes.items) |w| if (w.reg == reg) {
            v = w.val;
        };
        return v;
    }
};

test "cfv1: one PC breakpoint sets PBR0 + mask and arms TDR" {
    var r = DRegRecorder{ .gpa = testing.allocator };
    defer r.deinit();
    var set = cfv1.BreakpointSet{};
    try set.add(0x00001234);
    try set.apply(r.writer());
    try testing.expectEqual(@as(?u32, 0x00001234), r.find(cfv1.DRegPBR0)); // addr & ~1
    try testing.expectEqual(@as(?u32, 0), r.find(cfv1.DRegPBMR)); // exact match
    try testing.expectEqual(@as(?u32, cfv1.tdr_pc_halt), r.find(cfv1.DRegTDR));
    try testing.expectEqual(@as(?u32, 0), r.find(cfv1.DRegPBR1));
}

test "cfv1: a second breakpoint uses PBR1 with the valid bit; clearing disarms TDR" {
    var r = DRegRecorder{ .gpa = testing.allocator };
    defer r.deinit();
    var set = cfv1.BreakpointSet{};
    try set.add(0x8000);
    try set.add(0x9000);
    try set.apply(r.writer());
    try testing.expectEqual(@as(?u32, 0x8000), r.find(cfv1.DRegPBR0));
    try testing.expectEqual(@as(?u32, 0x9000 | 1), r.find(cfv1.DRegPBR1)); // valid flag
    try testing.expectEqual(@as(?u32, cfv1.tdr_pc_halt), r.find(cfv1.DRegTDR));
    set.remove(0x8000);
    set.remove(0x9000);
    var r2 = DRegRecorder{ .gpa = testing.allocator };
    defer r2.deinit();
    try set.apply(r2.writer());
    try testing.expectEqual(@as(?u32, 0), r2.find(cfv1.DRegTDR));
}

test "hcs12: one PC breakpoint arms DBG comparator A, tagged, break-to-background" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs12.program(r.mem(), 0x4321, null);
    try testing.expectEqual(@as(?u8, 0x43), r.find(hcs12.DBGCAH));
    try testing.expectEqual(@as(?u8, 0x21), r.find(hcs12.DBGCAL));
    try testing.expectEqual(@as(?u8, 0x00), r.find(hcs12.DBGCAX)); // 16-bit PC, no page
    try testing.expectEqual(@as(?u8, hcs12.DBGC2_BDM), r.find(hcs12.DBGC2)); // 0x20
    try testing.expectEqual(@as(?u8, hcs12.TRG_A_ONLY), r.find(hcs12.DBGSC));
    try testing.expectEqual(@as(?u8, hcs12.DBGEN | hcs12.ARM | hcs12.TRGSEL | hcs12.DBGBRK), r.find(hcs12.DBGC1)); // 0xE8
    // disarm (0x00) is written before the comparators
    try testing.expectEqual(@as(u8, 0x00), r.writes.items[0].val);
    try testing.expectEqual(@as(u32, hcs12.DBGC1), r.writes.items[0].addr);
}

test "hcs12: two breakpoints set comparator B and TRG=A-or-B" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs12.program(r.mem(), 0x8000, 0xC100);
    try testing.expectEqual(@as(?u8, 0x80), r.find(hcs12.DBGCAH));
    try testing.expectEqual(@as(?u8, 0xC1), r.find(hcs12.DBGCBH));
    try testing.expectEqual(@as(?u8, 0x00), r.find(hcs12.DBGCBL));
    try testing.expectEqual(@as(?u8, hcs12.TRG_A_OR_B), r.find(hcs12.DBGSC));
}

test "hcs12: a lone comparator-B breakpoint is folded onto A" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs12.program(r.mem(), null, 0xABCD);
    try testing.expectEqual(@as(?u8, 0xAB), r.find(hcs12.DBGCAH));
    try testing.expectEqual(@as(?u8, 0xCD), r.find(hcs12.DBGCAL));
    try testing.expectEqual(@as(?u8, null), r.find(hcs12.DBGCBH));
    try testing.expectEqual(@as(?u8, hcs12.TRG_A_ONLY), r.find(hcs12.DBGSC));
}

test "hcs12: no breakpoints just disarms" {
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try hcs12.program(r.mem(), null, null);
    try testing.expectEqual(@as(usize, 1), r.writes.items.len);
    try testing.expectEqual(@as(u8, 0x00), r.writes.items[0].val);
}

test "hcs12.Set add/compact and apply program comparator A" {
    var set = hcs12.Set{};
    try set.add(0x5000);
    try set.add(0x6000);
    try testing.expectError(error.NoSlot, set.add(0x7000));
    var r = Recorder{ .gpa = testing.allocator };
    defer r.deinit();
    try set.apply(r.mem());
    try testing.expectEqual(@as(?u8, 0x50), r.find(hcs12.DBGCAH));
    try testing.expectEqual(@as(?u8, 0x60), r.find(hcs12.DBGCBH));
}
