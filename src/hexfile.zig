//! Motorola S-record / Intel HEX / raw binary image codec. A parsed `Image` is
//! address-sorted, non-overlapping segments (adjacent records merged at parse time).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Io = std.Io;

pub const Format = enum { binary, srec, ihex };

/// Classify content: S-record, Intel HEX, else binary (tolerates leading whitespace).
pub fn detect(bytes: []const u8) Format {
    const t = std.mem.trim(u8, bytes, " \t\r\n");
    if (t.len >= 2) {
        if (t[0] == 'S' and t[1] >= '0' and t[1] <= '9') return .srec;
        if (t[0] == ':' and isHexDigit(t[1])) return .ihex;
    }
    return .binary;
}

pub const Segment = struct {
    addr: u32,
    data: []u8,
};

pub const Image = struct {
    /// Sorted by address, non-overlapping (adjacent ranges merged at parse).
    segments: []Segment,

    pub fn deinit(self: *Image, gpa: Allocator) void {
        for (self.segments) |seg| gpa.free(seg.data);
        gpa.free(self.segments);
        self.* = .{ .segments = &.{} };
    }

    pub fn totalBytes(self: Image) usize {
        var n: usize = 0;
        for (self.segments) |seg| n += seg.data.len;
        return n;
    }
};

pub const ParseError = error{
    InvalidHex,
    InvalidRecord,
    UnsupportedRecord,
    BadChecksum,
    BadRecordCount,
    Overlap,
    AddressOverflow,
} || Allocator.Error;

/// Parse an image (via detect()). Binary -> one segment @0; caller owns (Image.deinit).
pub fn parse(gpa: Allocator, bytes: []const u8) ParseError!Image {
    switch (detect(bytes)) {
        .binary => {
            const segs = try gpa.alloc(Segment, 1);
            errdefer gpa.free(segs);
            segs[0] = .{ .addr = 0, .data = try gpa.dupe(u8, bytes) };
            return .{ .segments = segs };
        },
        .srec, .ihex => |fmt| return parseRecords(gpa, bytes, fmt),
    }
}

const Rec = struct { addr: u32, data: []u8 };

fn parseRecords(gpa: Allocator, bytes: []const u8, format: Format) ParseError!Image {
    var arena_state = std.heap.ArenaAllocator.init(gpa);
    defer arena_state.deinit();
    const arena = arena_state.allocator();

    var recs: std.ArrayList(Rec) = .empty; // arena-backed
    var data_record_count: u32 = 0;
    var ihex_base: u32 = 0;

    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    line_loop: while (lines.next()) |raw_line| {
        const line = std.mem.trim(u8, raw_line, " \t");
        if (line.len == 0) continue;

        switch (format) {
            .srec => {
                if (line[0] != 'S' or line.len < 4) return error.InvalidRecord;
                const decoded = try decodeHex(arena, line[2..]);
                if (decoded.len < 2) return error.InvalidRecord;
                const count = decoded[0];
                if (@as(usize, count) + 1 != decoded.len) return error.InvalidRecord;
                var sum: u32 = 0;
                for (decoded) |b| sum += b;
                if (sum & 0xFF != 0xFF) return error.BadChecksum;
                const payload = decoded[1 .. decoded.len - 1]; // addr + data
                switch (line[1]) {
                    '0' => {}, // header record, ignored
                    '1', '2', '3' => {
                        const alen: usize = switch (line[1]) {
                            '1' => 2,
                            '2' => 3,
                            else => 4,
                        };
                        if (payload.len < alen) return error.InvalidRecord;
                        const addr = readAddr(payload[0..alen]);
                        data_record_count += 1;
                        const d = payload[alen..];
                        if (d.len != 0) try recs.append(arena, .{ .addr = addr, .data = d });
                    },
                    '5', '6' => {
                        const alen: usize = if (line[1] == '5') 2 else 3;
                        if (payload.len != alen) return error.InvalidRecord;
                        if (readAddr(payload[0..alen]) != data_record_count)
                            return error.BadRecordCount;
                    },
                    '7', '8', '9' => {
                        const alen: usize = switch (line[1]) {
                            '7' => 4,
                            '8' => 3,
                            else => 2,
                        };
                        // start-address: validate shape, ignore value
                        if (payload.len != alen) return error.InvalidRecord;
                    },
                    else => return error.UnsupportedRecord,
                }
            },
            .ihex => {
                if (line[0] != ':') return error.InvalidRecord;
                const decoded = try decodeHex(arena, line[1..]);
                if (decoded.len < 5) return error.InvalidRecord;
                const len = decoded[0];
                if (@as(usize, len) + 5 != decoded.len) return error.InvalidRecord;
                var sum: u32 = 0;
                for (decoded) |b| sum += b;
                if (sum & 0xFF != 0) return error.BadChecksum;
                const off16 = (@as(u32, decoded[1]) << 8) | decoded[2];
                const rtype = decoded[3];
                const d = decoded[4 .. decoded.len - 1];
                switch (rtype) {
                    0x00 => if (d.len != 0)
                        try recs.append(arena, .{ .addr = ihex_base + off16, .data = d }),
                    0x01 => {
                        if (len != 0) return error.InvalidRecord;
                        break :line_loop;
                    },
                    0x02 => {
                        if (len != 2) return error.InvalidRecord;
                        ihex_base = ((@as(u32, d[0]) << 8) | d[1]) << 4;
                    },
                    0x04 => {
                        if (len != 2) return error.InvalidRecord;
                        ihex_base = ((@as(u32, d[0]) << 8) | d[1]) << 16;
                    },
                    0x03, 0x05 => {}, // start-address records, ignored
                    else => return error.UnsupportedRecord,
                }
            },
            .binary => unreachable,
        }
    }

    std.mem.sort(Rec, recs.items, {}, struct {
        fn lt(_: void, a: Rec, b: Rec) bool {
            return a.addr < b.addr;
        }
    }.lt);

    var segs: std.ArrayList(Segment) = .empty;
    errdefer {
        for (segs.items) |s| gpa.free(s.data);
        segs.deinit(gpa);
    }

    var i: usize = 0;
    while (i < recs.items.len) {
        const first = recs.items[i];
        var end: u64 = @as(u64, first.addr) + first.data.len;
        var j = i + 1;
        while (j < recs.items.len) : (j += 1) {
            const r = recs.items[j];
            if (r.addr < end) return error.Overlap;
            if (r.addr > end) break;
            end += r.data.len;
        }
        if (end > 1 << 32) return error.AddressOverflow;
        const buf = try gpa.alloc(u8, @intCast(end - first.addr));
        errdefer gpa.free(buf);
        var off: usize = 0;
        for (recs.items[i..j]) |r| {
            @memcpy(buf[off .. off + r.data.len], r.data);
            off += r.data.len;
        }
        try segs.append(gpa, .{ .addr = first.addr, .data = buf });
        i = j;
    }

    return .{ .segments = try segs.toOwnedSlice(gpa) };
}

pub const EmitError = error{
    MultipleSegments,
    AddressOverflow,
} || Io.Writer.Error;

/// Write an image. Binary requires exactly one segment (gaps unrepresentable); srec/ihex any.
pub fn emit(out: *Io.Writer, image: Image, format: Format) EmitError!void {
    switch (format) {
        .binary => {
            if (image.segments.len != 1) return error.MultipleSegments;
            try out.writeAll(image.segments[0].data);
        },
        .srec => try emitSrec(out, image),
        .ihex => try emitIhex(out, image),
    }
}

fn emitSrec(out: *Io.Writer, image: Image) EmitError!void {
    var max_addr: u64 = 0;
    for (image.segments) |seg| {
        if (seg.data.len == 0) continue;
        const end = @as(u64, seg.addr) + seg.data.len;
        if (end > 1 << 32) return error.AddressOverflow;
        if (end - 1 > max_addr) max_addr = end - 1;
    }
    const alen: u8 = if (max_addr <= 0xFFFF) 2 else if (max_addr <= 0xFFFFFF) 3 else 4;
    const data_digit: u8 = '0' + (alen - 1); // S1/S2/S3
    const term_digit: u8 = '0' + (11 - alen); // S9/S8/S7

    try writeSrecRecord(out, '0', 2, 0, "usbdm");
    for (image.segments) |seg| {
        var off: usize = 0;
        while (off < seg.data.len) {
            const n = @min(16, seg.data.len - off);
            const addr: u32 = @intCast(seg.addr + off);
            try writeSrecRecord(out, data_digit, alen, addr, seg.data[off .. off + n]);
            off += n;
        }
    }
    try writeSrecRecord(out, term_digit, alen, 0, "");
}

fn writeSrecRecord(out: *Io.Writer, type_digit: u8, addr_len: u8, addr: u32, data: []const u8) Io.Writer.Error!void {
    const count: u8 = @intCast(addr_len + data.len + 1);
    var sum: u32 = count;
    try out.print("S{c}{X:0>2}", .{ type_digit, count });
    var i: u8 = addr_len;
    while (i > 0) {
        i -= 1;
        const b: u8 = @truncate(addr >> @intCast(8 * i));
        sum += b;
        try out.print("{X:0>2}", .{b});
    }
    for (data) |b| {
        sum += b;
        try out.print("{X:0>2}", .{b});
    }
    try out.print("{X:0>2}\n", .{0xFF - @as(u8, @truncate(sum))});
}

fn emitIhex(out: *Io.Writer, image: Image) EmitError!void {
    var cur_base: u32 = 0;
    for (image.segments) |seg| {
        var off: usize = 0;
        while (off < seg.data.len) {
            const addr = @as(u64, seg.addr) + off;
            const base = addr >> 16;
            if (base > 0xFFFF) return error.AddressOverflow;
            if (base != cur_base) {
                const bb = [2]u8{ @intCast(base >> 8), @intCast(base & 0xFF) };
                try writeIhexRecord(out, 0, 0x04, &bb);
                cur_base = @intCast(base);
            }
            const off16: u16 = @truncate(addr);
            // data record must not cross a 64K boundary
            const to_boundary = 0x10000 - @as(usize, off16);
            const n = @min(@min(@as(usize, 16), seg.data.len - off), to_boundary);
            try writeIhexRecord(out, off16, 0x00, seg.data[off .. off + n]);
            off += n;
        }
    }
    try writeIhexRecord(out, 0, 0x01, "");
}

fn writeIhexRecord(out: *Io.Writer, addr: u16, rtype: u8, data: []const u8) Io.Writer.Error!void {
    const len: u8 = @intCast(data.len);
    var sum: u32 = @as(u32, len) + (addr >> 8) + (addr & 0xFF) + rtype;
    try out.print(":{X:0>2}{X:0>4}{X:0>2}", .{ len, addr, rtype });
    for (data) |b| {
        sum += b;
        try out.print("{X:0>2}", .{b});
    }
    try out.print("{X:0>2}\n", .{@as(u8, @truncate(0 -% sum))});
}

fn isHexDigit(c: u8) bool {
    return switch (c) {
        '0'...'9', 'a'...'f', 'A'...'F' => true,
        else => false,
    };
}

fn nibble(c: u8) ParseError!u4 {
    return switch (c) {
        '0'...'9' => @intCast(c - '0'),
        'a'...'f' => @intCast(c - 'a' + 10),
        'A'...'F' => @intCast(c - 'A' + 10),
        else => error.InvalidHex,
    };
}

fn decodeHex(a: Allocator, s: []const u8) ParseError![]u8 {
    if (s.len % 2 != 0) return error.InvalidRecord;
    const buf = try a.alloc(u8, s.len / 2);
    for (buf, 0..) |*b, k| {
        b.* = (@as(u8, try nibble(s[2 * k])) << 4) | try nibble(s[2 * k + 1]);
    }
    return buf;
}

fn readAddr(bytes: []const u8) u32 {
    return std.mem.readVarInt(u32, bytes, .big);
}

const ExpSeg = struct { addr: u32, data: []const u8 };

fn expectSegments(expected: []const ExpSeg, img: Image) !void {
    try std.testing.expectEqual(expected.len, img.segments.len);
    for (expected, img.segments) |e, a| {
        try std.testing.expectEqual(e.addr, a.addr);
        try std.testing.expectEqualSlices(u8, e.data, a.data);
    }
}

fn expectImagesEqual(a: Image, b: Image) !void {
    try std.testing.expectEqual(a.segments.len, b.segments.len);
    for (a.segments, b.segments) |x, y| {
        try std.testing.expectEqual(x.addr, y.addr);
        try std.testing.expectEqualSlices(u8, x.data, y.data);
    }
}

fn expectRoundtrip(img: Image, format: Format) !void {
    const gpa = std.testing.allocator;
    var buf: [4096]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try emit(&w, img, format);
    var img2 = try parse(gpa, w.buffered());
    defer img2.deinit(gpa);
    try expectImagesEqual(img, img2);
}

test "detect classifies formats and tolerates leading whitespace" {
    try std.testing.expectEqual(Format.srec, detect("S00600004844521B"));
    try std.testing.expectEqual(Format.srec, detect("\n\n  S1070010DEADBEEFB0\n"));
    try std.testing.expectEqual(Format.ihex, detect(":00000001FF"));
    try std.testing.expectEqual(Format.ihex, detect(" \t\n:04001000DEADBEEFB4\n"));
    try std.testing.expectEqual(Format.binary, detect("\x7fELF"));
    try std.testing.expectEqual(Format.binary, detect(""));
    try std.testing.expectEqual(Format.binary, detect("SX not a record"));
}

test "parse srec S1 golden record" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S0080000757362646DDC\nS1070010DEADBEEFB0\nS9030000FC\n");
    defer img.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0x10, .data = &.{ 0xDE, 0xAD, 0xBE, 0xEF } }}, img);
    try std.testing.expectEqual(@as(usize, 4), img.totalBytes());
}

test "parse srec S2 and S3 addressing" {
    const gpa = std.testing.allocator;
    var img2 = try parse(gpa, "S205012345AAE7\n");
    defer img2.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0x012345, .data = &.{0xAA} }}, img2);

    var img3 = try parse(gpa, "S306DEADBEEF01C0\n");
    defer img3.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0xDEADBEEF, .data = &.{0x01} }}, img3);
}

test "parse srec rejects bad checksum, bad hex, malformed records" {
    const gpa = std.testing.allocator;
    try std.testing.expectError(error.BadChecksum, parse(gpa, "S1070010DEADBEEFB1\n"));
    try std.testing.expectError(error.InvalidHex, parse(gpa, "S107001GDEADBEEFB0\n"));
    try std.testing.expectError(error.InvalidRecord, parse(gpa, "S1070010DEADBEEFB\n"));
    // count byte != record length
    try std.testing.expectError(error.InvalidRecord, parse(gpa, "S1080010DEADBEEFAF\n"));
    // S4 reserved: valid shape, unsupported
    try std.testing.expectError(error.UnsupportedRecord, parse(gpa, "S4030000FC\n"));
}

test "parse srec merges adjacent records including out-of-order input" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S10500020304F1\nS10500000102F7\nS9030000FC\n");
    defer img.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0, .data = &.{ 1, 2, 3, 4 } }}, img);
}

test "parse srec keeps non-adjacent records as separate segments" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S10500000102F7\nS1040010FFEC\n");
    defer img.deinit(gpa);
    try expectSegments(&.{
        .{ .addr = 0x0, .data = &.{ 1, 2 } },
        .{ .addr = 0x10, .data = &.{0xFF} },
    }, img);
}

test "parse srec rejects overlapping records" {
    const gpa = std.testing.allocator;
    try std.testing.expectError(error.Overlap, parse(gpa, "S10500000102F7\nS104000103F7\n"));
}

test "parse srec validates S5/S6 record counts" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S10500000102F7\nS10500020304F1\nS5030002FA\n");
    defer img.deinit(gpa);
    try std.testing.expectError(
        error.BadRecordCount,
        parse(gpa, "S10500000102F7\nS10500020304F1\nS5030003F9\n"),
    );
    var img6 = try parse(gpa, "S10500000102F7\nS10500020304F1\nS604000002F9\n");
    defer img6.deinit(gpa);
}

test "emit srec golden output" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S1070010DEADBEEFB0\n");
    defer img.deinit(gpa);
    var buf: [256]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try emit(&w, img, .srec);
    try std.testing.expectEqualStrings(
        "S0080000757362646DDC\nS1070010DEADBEEFB0\nS9030000FC\n",
        w.buffered(),
    );
}

test "srec roundtrip: multi-segment, S2/S3 widths, >16-byte segments" {
    var big: [40]u8 = undefined;
    for (&big, 0..) |*b, k| b.* = @intCast(k * 3 & 0xFF);
    var d1 = [_]u8{ 0xCA, 0xFE };
    var d3 = [_]u8{0x42};

    // S1 range (16-bit addresses)
    var segs1 = [_]Segment{
        .{ .addr = 0x0000, .data = &big },
        .{ .addr = 0x8000, .data = &d1 },
    };
    try expectRoundtrip(.{ .segments = &segs1 }, .srec);

    // S2 range (24-bit)
    var segs2 = [_]Segment{.{ .addr = 0x123456, .data = &d1 }};
    try expectRoundtrip(.{ .segments = &segs2 }, .srec);

    // S3 range (32-bit)
    var segs3 = [_]Segment{
        .{ .addr = 0x10, .data = &d1 },
        .{ .addr = 0xFFFFFFFE, .data = &d3 },
    };
    try expectRoundtrip(.{ .segments = &segs3 }, .srec);
}

test "parse ihex golden record" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, ":04001000DEADBEEFB4\n:00000001FF\n");
    defer img.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0x10, .data = &.{ 0xDE, 0xAD, 0xBE, 0xEF } }}, img);
}

test "parse ihex extended segment (02) and linear (04) addressing" {
    const gpa = std.testing.allocator;
    var lin = try parse(gpa, ":020000040001F9\n:01000000AA55\n:00000001FF\n");
    defer lin.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0x10000, .data = &.{0xAA} }}, lin);

    var seg = try parse(gpa, ":020000021000EC\n:01000000AA55\n:00000001FF\n");
    defer seg.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0x10000, .data = &.{0xAA} }}, seg);
}

test "parse ihex rejects bad checksum and unknown record type" {
    const gpa = std.testing.allocator;
    try std.testing.expectError(error.BadChecksum, parse(gpa, ":04001000DEADBEEFB5\n"));
    try std.testing.expectError(error.UnsupportedRecord, parse(gpa, ":020000060001F7\n"));
    try std.testing.expectError(error.InvalidRecord, parse(gpa, ":05001000DEADBEEFB3\n"));
}

test "parse ihex stops at EOF record" {
    const gpa = std.testing.allocator;
    // garbage after EOF ignored
    var img = try parse(gpa, ":01000000AA55\n:00000001FF\nnot a record\n");
    defer img.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0, .data = &.{0xAA} }}, img);
}

test "emit ihex golden output including type-04 base record" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, ":04001000DEADBEEFB4\n:00000001FF\n");
    defer img.deinit(gpa);
    var buf: [256]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try emit(&w, img, .ihex);
    try std.testing.expectEqualStrings(":04001000DEADBEEFB4\n:00000001FF\n", w.buffered());

    var img4 = try parse(gpa, ":020000040001F9\n:01000000AA55\n:00000001FF\n");
    defer img4.deinit(gpa);
    var w4: Io.Writer = .fixed(&buf);
    try emit(&w4, img4, .ihex);
    try std.testing.expectEqualStrings(
        ":020000040001F9\n:01000000AA55\n:00000001FF\n",
        w4.buffered(),
    );
}

test "ihex roundtrip: segment crossing a 64K boundary" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, ":02FFFE000102FE\n:020000040001F9\n:020000000304F7\n:00000001FF\n");
    defer img.deinit(gpa);
    // crossing records merge into one segment
    try expectSegments(&.{.{ .addr = 0xFFFE, .data = &.{ 1, 2, 3, 4 } }}, img);
    // emit splits at the boundary again, losslessly
    try expectRoundtrip(img, .ihex);

    var buf: [256]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try emit(&w, img, .ihex);
    try std.testing.expectEqualStrings(
        ":02FFFE000102FE\n:020000040001F9\n:020000000304F7\n:00000001FF\n",
        w.buffered(),
    );
}

test "binary parse and emit roundtrip" {
    const gpa = std.testing.allocator;
    const raw = "\x00\x01\x02hello\xff";
    var img = try parse(gpa, raw);
    defer img.deinit(gpa);
    try expectSegments(&.{.{ .addr = 0, .data = raw }}, img);

    var buf: [64]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try emit(&w, img, .binary);
    try std.testing.expectEqualStrings(raw, w.buffered());
}

test "emit binary rejects multi-segment images" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S10500000102F7\nS1040010FFEC\n");
    defer img.deinit(gpa);
    var buf: [64]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try std.testing.expectError(error.MultipleSegments, emit(&w, img, .binary));
}

test "cross-format roundtrip srec -> ihex -> srec" {
    const gpa = std.testing.allocator;
    var img = try parse(gpa, "S10500000102F7\nS1040010FFEC\nS205012345AAE7\n");
    defer img.deinit(gpa);
    try expectRoundtrip(img, .srec);
    try expectRoundtrip(img, .ihex);
}

// parse() reads arbitrary user files: must never panic or leak.
fn fuzzRecByte(rand: std.Random) u8 {
    return switch (rand.intRangeAtMost(u8, 0, 6)) {
        0 => '\n',
        1 => 'S',
        2 => ':',
        3 => ' ',
        else => "0123456789ABCDEF"[rand.intRangeAtMost(usize, 0, 15)],
    };
}

test "fuzz: hexfile.parse never panics or leaks on adversarial input" {
    const gpa = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0xF022C0DE);
    const rand = prng.random();
    var scratch: [600]u8 = undefined;
    var it: usize = 0;
    while (it < 20000) : (it += 1) {
        const b = scratch[0..rand.intRangeAtMost(usize, 0, scratch.len)];
        switch (rand.intRangeAtMost(u8, 0, 3)) {
            // record-shaped input hits the SREC/IHEX parsers; random hits detection + binary
            1 => for (b, 0..) |*c, i| {
                c.* = if (i == 0) 'S' else fuzzRecByte(rand);
            },
            2 => for (b, 0..) |*c, i| {
                c.* = if (i == 0) ':' else fuzzRecByte(rand);
            },
            else => rand.bytes(b),
        }
        if (parse(gpa, b)) |img| {
            var m = img;
            m.deinit(gpa);
        } else |_| {}
    }
}
