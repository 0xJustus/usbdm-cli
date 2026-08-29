//! RSP packet framing: `$<payload>#<checksum>` with `}` escape and `*` RLE.
//! Ref: GDB "Remote Protocol" appendix.

const std = @import("std");

/// Sum of payload bytes, modulo 256.
pub fn checksum(payload: []const u8) u8 {
    var sum: u8 = 0;
    for (payload) |b| sum +%= b;
    return sum;
}

/// Escaped payload bytes: `$ # } *`, written `}` then (byte ^ 0x20).
pub fn mustEscape(b: u8) bool {
    return b == '$' or b == '#' or b == '}' or b == '*';
}

pub const EncodeError = std.Io.Writer.Error;

/// Write a `$<escaped-payload>#<hh>` packet. No RLE (optional; a stub may omit it).
pub fn writePacket(out: *std.Io.Writer, payload: []const u8) EncodeError!void {
    var sum: u8 = 0;
    try out.writeByte('$');
    for (payload) |b| {
        if (mustEscape(b)) {
            try out.writeByte('}');
            const esc = b ^ 0x20;
            try out.writeByte(esc);
            sum +%= '}';
            sum +%= esc;
        } else {
            try out.writeByte(b);
            sum +%= b;
        }
    }
    try out.writeByte('#');
    try out.print("{x:0>2}", .{sum});
}

pub const DecodeError = error{
    NoPacketStart,
    Truncated,
    BadChecksum,
    BadEscape,
    Overflow,
};

pub const Decoded = struct {
    /// Decoded payload in `buf` (escapes resolved, RLE expanded).
    payload: []u8,
    /// Raw bytes consumed (incl. `$`, `#`, 2 checksum digits) - to advance the stream.
    consumed: usize,
};

/// Decode one packet from `raw` into `buf`. Leading non-`$` bytes (e.g. acks)
/// are skipped; resolves `}` escapes and `*` RLE; verifies the checksum.
pub fn decodePacket(raw: []const u8, buf: []u8) DecodeError!Decoded {
    var start: usize = 0;
    while (start < raw.len and raw[start] != '$') start += 1;
    if (start >= raw.len) return error.NoPacketStart;

    var i = start + 1;
    var out_len: usize = 0;
    var sum: u8 = 0;
    while (true) {
        if (i >= raw.len) return error.Truncated;
        const c = raw[i];
        if (c == '#') break;
        sum +%= c;
        i += 1;
        if (c == '}') {
            if (i >= raw.len) return error.Truncated;
            const e = raw[i];
            sum +%= e;
            i += 1;
            if (out_len >= buf.len) return error.Overflow;
            buf[out_len] = e ^ 0x20;
            out_len += 1;
        } else if (c == '*') {
            // RLE: repeat previous byte; count = next byte - 29 (printable)
            if (i >= raw.len) return error.Truncated;
            const rep = raw[i];
            sum +%= rep;
            i += 1;
            if (out_len == 0 or rep < 29) return error.BadEscape;
            const count: usize = @as(usize, rep) - 29;
            const last = buf[out_len - 1];
            if (out_len + count > buf.len) return error.Overflow;
            for (0..count) |_| {
                buf[out_len] = last;
                out_len += 1;
            }
        } else {
            if (out_len >= buf.len) return error.Overflow;
            buf[out_len] = c;
            out_len += 1;
        }
    }
    // raw[i]=='#': need 2 checksum digits after
    if (i + 2 >= raw.len) return error.Truncated;
    const hi = hexDigit(raw[i + 1]) orelse return error.BadChecksum;
    const lo = hexDigit(raw[i + 2]) orelse return error.BadChecksum;
    const want = (@as(u8, hi) << 4) | lo;
    if (want != sum) return error.BadChecksum;
    return .{ .payload = buf[0..out_len], .consumed = i + 3 };
}

fn hexDigit(c: u8) ?u4 {
    return switch (c) {
        '0'...'9' => @intCast(c - '0'),
        'a'...'f' => @intCast(c - 'a' + 10),
        'A'...'F' => @intCast(c - 'A' + 10),
        else => null,
    };
}

/// Append `value` as `nbytes` of big-endian hex (2 chars/byte) to `out`.
pub fn appendHexInt(out: *std.Io.Writer, value: u64, nbytes: usize) std.Io.Writer.Error!void {
    var i = nbytes;
    while (i > 0) {
        i -= 1;
        const byte: u8 = @truncate(value >> @intCast(i * 8));
        try out.print("{x:0>2}", .{byte});
    }
}

/// Write `bytes` as a hex string (2 chars/byte) to `out`.
pub fn writeHexBytes(out: *std.Io.Writer, bytes: []const u8) std.Io.Writer.Error!void {
    for (bytes) |b| try out.print("{x:0>2}", .{b});
}

/// Parse `text` as hex into `value`, returning null on any non-hex char.
pub fn parseHex(text: []const u8) ?u64 {
    if (text.len == 0 or text.len > 16) return null;
    var v: u64 = 0;
    for (text) |c| {
        const d = hexDigit(c) orelse return null;
        v = (v << 4) | d;
    }
    return v;
}

/// Decode hex-pair bytes from `text` into `out`; returns the byte count.
pub fn parseHexBytes(text: []const u8, out: []u8) ?usize {
    if (text.len % 2 != 0) return null;
    const n = text.len / 2;
    if (n > out.len) return null;
    for (0..n) |i| {
        const hi = hexDigit(text[i * 2]) orelse return null;
        const lo = hexDigit(text[i * 2 + 1]) orelse return null;
        out[i] = (@as(u8, hi) << 4) | lo;
    }
    return n;
}

const testing = std.testing;

test "checksum is sum mod 256" {
    // "OK" -> 'O'(0x4F) + 'K'(0x4B) = 0x9A
    try testing.expectEqual(@as(u8, 0x9A), checksum("OK"));
}

test "writePacket frames and checksums" {
    var buf: [64]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    try writePacket(&w, "OK");
    try testing.expectEqualStrings("$OK#9a", w.buffered());
}

test "writePacket escapes special bytes" {
    var buf: [64]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    try writePacket(&w, "a}b"); // '}' -> '}' , 0x7D^0x20=0x5D ']'
    // payload on the wire: 'a' '}' ']' 'b'
    const expect_sum = checksum("a}]b");
    var eb: [64]u8 = undefined;
    const s = std.fmt.bufPrint(&eb, "$a}}]b#{x:0>2}", .{expect_sum}) catch unreachable;
    try testing.expectEqualStrings(s, w.buffered());
}

test "decodePacket round-trips a simple packet" {
    var buf: [64]u8 = undefined;
    const d = try decodePacket("$OK#9a", &buf);
    try testing.expectEqualStrings("OK", d.payload);
    try testing.expectEqual(@as(usize, 6), d.consumed);
}

test "decodePacket skips leading acks" {
    var buf: [64]u8 = undefined;
    const d = try decodePacket("+$OK#9a", &buf);
    try testing.expectEqualStrings("OK", d.payload);
    try testing.expectEqual(@as(usize, 7), d.consumed);
}

test "decodePacket resolves escapes" {
    var buf: [64]u8 = undefined;
    // payload bytes 'a' '}' ']' 'b' -> decodes to 'a' '}' 'b'
    var eb: [64]u8 = undefined;
    const pkt = std.fmt.bufPrint(&eb, "$a}}]b#{x:0>2}", .{checksum("a}]b")}) catch unreachable;
    const d = try decodePacket(pkt, &buf);
    try testing.expectEqualStrings("a}b", d.payload);
}

test "decodePacket expands run-length compression" {
    // RLE: "0* " -> '0' + (0x20-29=3) more '0' = "0000"
    var eb: [64]u8 = undefined;
    const body = "0* "; // space is 0x20 = 32 -> repeat 32-29 = 3 times
    const pkt = std.fmt.bufPrint(&eb, "${s}#{x:0>2}", .{ body, checksum(body) }) catch unreachable;
    var buf: [64]u8 = undefined;
    const d = try decodePacket(pkt, &buf);
    try testing.expectEqualStrings("0000", d.payload);
}

test "decodePacket rejects bad checksum" {
    var buf: [64]u8 = undefined;
    try testing.expectError(error.BadChecksum, decodePacket("$OK#00", &buf));
}

test "decodePacket reports truncation" {
    var buf: [64]u8 = undefined;
    try testing.expectError(error.Truncated, decodePacket("$OK#9", &buf));
    try testing.expectError(error.Truncated, decodePacket("$OK", &buf));
}

test "appendHexInt is big-endian" {
    var buf: [32]u8 = undefined;
    var w = std.Io.Writer.fixed(&buf);
    try appendHexInt(&w, 0x1234, 2);
    try testing.expectEqualStrings("1234", w.buffered());
    w = std.Io.Writer.fixed(&buf);
    try appendHexInt(&w, 0xAB, 1);
    try testing.expectEqualStrings("ab", w.buffered());
}

test "parseHex and parseHexBytes" {
    try testing.expectEqual(@as(?u64, 0x1a2b), parseHex("1a2b"));
    try testing.expectEqual(@as(?u64, null), parseHex("xyz"));
    var out: [4]u8 = undefined;
    const n = parseHexBytes("deadbeef", &out).?;
    try testing.expectEqual(@as(usize, 4), n);
    try testing.expectEqualSlices(u8, &.{ 0xde, 0xad, 0xbe, 0xef }, out[0..n]);
    try testing.expectEqual(@as(?usize, null), parseHexBytes("abc", &out)); // odd length
}

// decodePacket parses untrusted input: must never panic on any bytes.
test "fuzz: decodePacket never panics on adversarial input" {
    var prng = std.Random.DefaultPrng.init(0x6DB57A9E);
    const rand = prng.random();
    var raw: [600]u8 = undefined;
    var out: [512]u8 = undefined;
    const hexd = "0123456789abcdef";
    var it: usize = 0;
    while (it < 50000) : (it += 1) {
        const len = rand.intRangeAtMost(usize, 0, raw.len);
        const r = raw[0..len];
        switch (rand.intRangeAtMost(u8, 0, 3)) {
            0 => rand.bytes(r), // pure random
            1 => { // packet-shaped
                if (len > 0) r[0] = '$';
                if (len > 1) rand.bytes(r[1..]);
            },
            2 => for (r) |*b| { // escape/RLE-heavy (the classic bug surface)
                b.* = switch (rand.intRangeAtMost(u8, 0, 5)) {
                    0 => '*',
                    1 => '}',
                    2 => '#',
                    3 => '$',
                    else => rand.int(u8),
                };
            },
            else => if (len >= 4) { // valid frame shape, random checksum
                r[0] = '$';
                r[len - 3] = '#';
                for (r[1 .. len - 3]) |*b| b.* = rand.int(u8);
                r[len - 2] = hexd[rand.intRangeAtMost(usize, 0, 15)];
                r[len - 1] = hexd[rand.intRangeAtMost(usize, 0, 15)];
            } else rand.bytes(r),
        }
        const olen = rand.intRangeAtMost(usize, 0, out.len); // small buffers hit Overflow
        _ = decodePacket(r, out[0..olen]) catch {};
    }
}

// writePacket escapes (no RLE), so encode->decode is exact identity.
test "fuzz: writePacket -> decodePacket round-trips any payload" {
    var prng = std.Random.DefaultPrng.init(0xC0FFEE11);
    const rand = prng.random();
    var payload: [256]u8 = undefined;
    var pktbuf: [1024]u8 = undefined; // fits '$' + 2x escaped + "#hh"
    var out: [256]u8 = undefined;
    var it: usize = 0;
    while (it < 20000) : (it += 1) {
        const p = payload[0..rand.intRangeAtMost(usize, 0, payload.len)];
        rand.bytes(p);
        var w = std.Io.Writer.fixed(&pktbuf);
        writePacket(&w, p) catch continue;
        const d = try decodePacket(w.buffered(), &out);
        try testing.expectEqualSlices(u8, p, d.payload);
        try testing.expectEqual(w.buffered().len, d.consumed);
    }
}
