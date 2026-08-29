#!/usr/bin/env bash
set -euo pipefail

BIN="${1:-zig-out/bin/usbdm}"
[ -x "$BIN" ] || { echo "sim-check: no executable at '$BIN'"; exit 1; }

d="$(mktemp -d)"; trap 'rm -rf "$d"' EXIT

mkS19() {
  python3 - "$1" "$2" "$3" <<'PY'
import sys
addr = int(sys.argv[1], 0); data = bytes.fromhex(sys.argv[2]); path = sys.argv[3]
b = [len(data) + 3, (addr >> 8) & 0xFF, addr & 0xFF] + list(data)
cks = (~(sum(b) & 0xFF)) & 0xFF
open(path, "w").write("S1" + "".join(f"{x:02X}" for x in b) + f"{cks:02X}\nS9030000FC\n")
PY
}

check() {
  local desc="$1"; shift
  printf '  %-24s' "$desc"
  local out
  if ! out="$("$@" 2>&1)"; then
    printf 'FAIL (exit %d)\n%s\n' "$?" "$out"; exit 1
  fi
  if ! grep -q verified <<<"$out"; then
    printf "FAIL (no 'verified')\n%s\n" "$out"; exit 1
  fi
  echo ok
}

mkS19 0xE000 11223344         "$d/h08.s19"
mkS19 0x4000 aabbccdd11223344 "$d/h12.s19"
mkS19 0x0000 0102030405060708 "$d/cf.s19"
mkS19 0x0000 deadbeef         "$d/ee.s19"

echo "sim-check: $BIN"
check "HCS08 program+verify" "$BIN" --sim --part mc9s08sh8 program "$d/h08.s19" -f
check "HCS12 program+verify" "$BIN" --sim --target hcs12 --part mc9s12c32 program "$d/h12.s19" -f
check "CFV1 program+verify"  "$BIN" --sim --target cfv1 --part mcf51qe128 program "$d/cf.s19" -f
check "S12G program+verify"  "$BIN" --sim --target hcs12 --part mc9s12g240 program "$d/h12.s19" -f
check "S12G EEPROM program"  "$BIN" --sim --target hcs12 --part mc9s12g240 eeprom "$d/ee.s19" -f

idout="$("$BIN" --sim --target hcs12 --part mc9s12c32 identify 2>&1)" || true
if ! grep -q 'mc9s12c32' <<<"$idout"; then
  printf '  %-24sFAIL\n%s\n' "identify (SDID)" "$idout"; exit 1
fi
printf '  %-24s%s\n' "identify (SDID)" ok

"$BIN" --version >/dev/null
"$BIN" parts --json | python3 -c 'import sys,json; d=json.load(sys.stdin); assert len(d) > 20, d' \
  || { echo "  parts --json            FAIL"; exit 1; }
printf '  %-24s%s\n' "introspection" ok

echo "sim-check: PASS"
