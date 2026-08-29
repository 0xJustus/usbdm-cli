//! Terminal styling. Colors are plain string fields so call sites can embed
//! them in format strings; `Style.none` makes every code an empty string.

const std = @import("std");

extern "c" fn isatty(fd: c_int) c_int;
extern "c" fn getenv(name: [*:0]const u8) ?[*:0]const u8;

pub const Style = struct {
    reset: []const u8 = "",
    bold: []const u8 = "",
    dim: []const u8 = "",
    green: []const u8 = "",
    yellow: []const u8 = "",
    cyan: []const u8 = "",

    pub const none: Style = .{};

    pub const ansi: Style = .{
        .reset = "\x1b[0m",
        .bold = "\x1b[1m",
        .dim = "\x1b[2m",
        .green = "\x1b[32m",
        .yellow = "\x1b[33m",
        .cyan = "\x1b[36m",
    };
};

/// Color when stdout is a tty, unless NO_COLOR or TERM=dumb.
pub fn stdoutSupportsColor() bool {
    if (getenv("NO_COLOR") != null) return false;
    if (getenv("TERM")) |term| {
        if (std.mem.eql(u8, std.mem.span(term), "dumb")) return false;
    }
    return isatty(1) != 0;
}

/// stdin is a tty (can we prompt).
pub fn stdinIsTty() bool {
    return isatty(0) != 0;
}

test "none style is empty" {
    const st: Style = .none;
    try std.testing.expectEqualStrings("", st.reset);
    try std.testing.expectEqualStrings("", st.green);
}
