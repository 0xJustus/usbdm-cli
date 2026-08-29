//! 16-byte-per-line hex dump with ASCII gutter; styling via `tty.Style`
//! (`.none` gives plain `hexdump -C`-style output).

const std = @import("std");
const Io = std.Io;
const tty = @import("tty.zig");

pub fn dump(out: *Io.Writer, st: tty.Style, base: usize, bytes: []const u8) Io.Writer.Error!void {
    var offset: usize = 0;
    while (offset < bytes.len) : (offset += 16) {
        const line = bytes[offset..@min(offset + 16, bytes.len)];

        try out.print("{s}{x:0>8}{s}  ", .{ st.dim, base + offset, st.reset });

        for (0..16) |i| {
            if (i == 8) try out.writeAll(" ");
            if (i < line.len) {
                if (line[i] == 0) {
                    try out.print("{s}00{s} ", .{ st.dim, st.reset });
                } else {
                    try out.print("{x:0>2} ", .{line[i]});
                }
            } else {
                try out.writeAll("   ");
            }
        }

        try out.print(" {s}|{s}", .{ st.dim, st.reset });
        for (line) |b| {
            if (std.ascii.isPrint(b)) {
                try out.print("{s}{c}{s}", .{ st.cyan, b, st.reset });
            } else {
                try out.print("{s}.{s}", .{ st.dim, st.reset });
            }
        }
        try out.print("{s}|{s}\n", .{ st.dim, st.reset });
    }
}

test "plain dump, full line" {
    var input: [16]u8 = undefined;
    for (&input, 0..) |*b, i| b.* = @intCast(i);

    var buf: [256]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try dump(&w, .none, 0, &input);
    try std.testing.expectEqualStrings(
        "00000000  00 01 02 03 04 05 06 07  08 09 0a 0b 0c 0d 0e 0f  |................|\n",
        w.buffered(),
    );
}

test "plain dump, partial line with printable ascii and base offset" {
    const input = [_]u8{ 0x16, 0xd0, 0x05, 0x67, 0x4c };
    var buf: [256]u8 = undefined;
    var w: Io.Writer = .fixed(&buf);
    try dump(&w, .none, 0x100, &input);
    try std.testing.expectEqualStrings(
        "00000100  16 d0 05 67 4c" ++ " " ** 36 ++ "|...gL|\n",
        w.buffered(),
    );
}
