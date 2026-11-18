"""Frame parser and capture loader for the vl53l7_eval firmware.

Wire format (see firmware/vl53l7_eval/protocol.h):
    0xA5 0x5A | type u8 | len u16 | payload[len] | crc16 u16     (little-endian)
crc16 is CRC-16/CCITT-FALSE over type, len and payload.

A capture (.vlog) is the raw byte stream from the board in binary mode, so the
same parser reads a live port and a file.
"""
from __future__ import annotations

import json
import re
import struct
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

SYNC = b"\xa5\x5a"
FT_RANGE, FT_CNH, FT_INFO, FT_LOG = 0x01, 0x02, 0x03, 0x04
MAX_PAYLOAD = 16384
DRIVERS = {0: "off", 1: "cx", 2: "ch"}
PORTS = {0: "A", 1: "B"}
CNH_BIN_MM = 37.5348  # one device histogram bin, from ST's vl53lmz_plugin_cnh.h
VALID_STATUS = (5, 9)  # ST: 5 = range valid, 9 = valid with large pulse (merged target)

_RANGE_HDR = struct.Struct("<BBBBBbHIqI")  # ver port driver zones targets temp rsv seq t_us read_us
_CNH_HDR = struct.Struct("<BBBBBBBBhHIqI")  # ver port zones aggs bins sub merge cols start rsv seq t_us ref_res


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(ftype: int, payload: bytes) -> bytes:
    """Build a frame exactly as the firmware does (used by tests)."""
    body = struct.pack("<BH", ftype, len(payload)) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class StreamParser:
    """Splits a byte stream into frames; bytes outside frames become text lines.

    feed() returns a list of events: ("frame", type, payload) or ("text", line).
    Text covers boot messages and anything printed before `out bin`.
    """

    def __init__(self) -> None:
        self.buf = bytearray()
        self.text = bytearray()
        self.frames = 0
        self.crc_errors = 0

    def _text(self, data: bytes, events: list) -> None:
        self.text += data
        while b"\n" in self.text:
            line, _, rest = bytes(self.text).partition(b"\n")
            self.text = bytearray(rest)
            s = line.decode("utf-8", "replace").rstrip("\r")
            if s:
                events.append(("text", s))

    def feed(self, data: bytes) -> list:
        events: list = []
        self.buf += data
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # Keep a trailing 0xA5 in case the next chunk starts with 0x5A.
                keep = 1 if self.buf.endswith(SYNC[:1]) else 0
                self._text(bytes(self.buf[: len(self.buf) - keep]), events)
                del self.buf[: len(self.buf) - keep]
                return events
            if i:
                self._text(bytes(self.buf[:i]), events)
                del self.buf[:i]
            if len(self.buf) < 5:
                return events
            ftype = self.buf[2]
            length = self.buf[3] | (self.buf[4] << 8)
            if length > MAX_PAYLOAD or ftype not in (FT_RANGE, FT_CNH, FT_INFO, FT_LOG):
                del self.buf[:1]  # false sync
                continue
            end = 5 + length + 2
            if len(self.buf) < end:
                return events
            body = bytes(self.buf[2 : 5 + length])
            crc = self.buf[end - 2] | (self.buf[end - 1] << 8)
            if crc16(body) != crc:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            events.append(("frame", ftype, body[3:]))
            self.frames += 1
            del self.buf[:end]


def decode_range(p: bytes) -> dict:
    ver, port, drv, nz, nt, temp, _, seq, t_us, read_us = _RANGE_HDR.unpack_from(p)
    o = _RANGE_HDR.size

    def take(dtype, n):
        nonlocal o
        a = np.frombuffer(p, dtype=dtype, count=n, offset=o)
        o += a.nbytes
        return a

    d = dict(ver=ver, port=PORTS.get(port, str(port)), driver=DRIVERS.get(drv, str(drv)), zones=nz,
             targets=nt, temp_c=temp, seq=seq, t_us=t_us, read_us=read_us)
    d["nb_target_detected"] = take("<u1", nz)
    d["ambient_per_spad"] = take("<u4", nz)
    d["nb_spads_enabled"] = take("<u4", nz)
    d["distance_mm"] = take("<i2", nz * nt).reshape(nz, nt)
    d["range_sigma_mm"] = take("<u2", nz * nt).reshape(nz, nt)
    d["signal_per_spad"] = take("<u4", nz * nt).reshape(nz, nt)
    d["reflectance"] = take("<u1", nz * nt).reshape(nz, nt)
    d["target_status"] = take("<u1", nz * nt).reshape(nz, nt)
    if o != len(p):
        raise ValueError(f"range frame length mismatch: parsed {o}, got {len(p)}")
    return d


def decode_cnh(p: bytes) -> dict:
    ver, port, nz, n_agg, n_bins, sub, merge, cols, start, _, seq, t_us, ref_res = _CNH_HDR.unpack_from(p)
    o = _CNH_HDR.size
    a = np.frombuffer(p, dtype="<f4", count=n_agg * (n_bins + 1), offset=o).reshape(n_agg, n_bins + 1)
    if o + a.nbytes != len(p):
        raise ValueError(f"cnh frame length mismatch: parsed {o + a.nbytes}, got {len(p)}")
    return dict(ver=ver, port=PORTS.get(port, str(port)), zones=nz, aggregates=n_agg, bins=n_bins, sub=sub,
                merge=merge, cols=cols, start_bin=start, seq=seq, t_us=t_us, ref_residual=ref_res,
                ambient=a[:, 0], hist=a[:, 1:])


def bin_centers_mm(start_bin: int, bins: int, sub: int) -> np.ndarray:
    """Nominal distance of each CNH output bin's centre (before offset calibration)."""
    return (start_bin + (np.arange(bins) + 0.5) * sub) * CNH_BIN_MM


def zone_to_aggregate(zone: int, zones: int, merge: int, cols: int) -> int:
    side = 8 if zones == 64 else 4
    r, c = divmod(zone, side)
    return (r // merge) * cols + c // merge


@dataclass
class PortData:
    """All frames from one port, stacked. Frames whose shape differs from the
    first one (config changed mid-capture) are counted in `skipped`."""

    name: str
    range: dict = field(default_factory=dict)  # arrays with a leading frame axis
    cnh: dict = field(default_factory=dict)
    info: list = field(default_factory=list)  # INFO dicts in arrival order
    skipped: int = 0

    @property
    def frames(self) -> int:
        return len(self.range.get("seq", []))

    @property
    def driver(self) -> str:
        return self.range.get("driver", "?")

    @property
    def side(self) -> int:
        return 8 if self.range.get("zones", 64) == 64 else 4

    def fps(self) -> float:
        t = self.range.get("t_us")
        if t is None or len(t) < 2:
            return 0.0
        return (len(t) - 1) / ((t[-1] - t[0]) / 1e6)


_SCALARS = ("ver", "port", "driver", "zones", "targets", "aggregates", "bins", "sub", "merge", "cols", "start_bin")


def _stack(frames: list[dict]) -> tuple[dict, int]:
    if not frames:
        return {}, 0
    first = frames[0]
    key = [k for k in _SCALARS if k in first]
    same = [f for f in frames if all(f[k] == first[k] for k in key)]
    out = {k: first[k] for k in key}
    for k, v in first.items():
        if k in key:
            continue
        out[k] = np.stack([f[k] for f in same]) if isinstance(v, np.ndarray) else np.array([f[k] for f in same])
    return out, len(frames) - len(same)


@dataclass
class Capture:
    path: Path
    ports: dict  # "A"/"B" -> PortData
    logs: list
    meta: dict
    crc_errors: int


_TEXT_HDR = re.compile(r"\[([AB]) (\w+) (\d+)x\d+\] frame (\d+)\s+read (\d+) us\s+(-?\d+) C")


def parse_text_log(path: str | Path) -> list[dict]:
    """Frames from a saved Arduino Serial Monitor (text mode) log.

    Text mode prints one frame per port every TEXT_MS, target 0 only, and
    only zones with a valid status; other zones become NaN.
    """
    frames: list[dict] = []
    cur = None
    for line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        m = _TEXT_HDR.search(line)
        if m:
            side = int(m.group(3))
            cur = dict(port=m.group(1), driver=m.group(2), side=side, frame=int(m.group(4)),
                       read_us=int(m.group(5)), temp_c=int(m.group(6)), rows=[])
            frames.append(cur)
            continue
        if cur is None or len(cur["rows"]) >= cur["side"]:
            continue
        toks = line.split()
        if len(toks) == cur["side"] and all(t == "." or t.lstrip("-").isdigit() for t in toks):
            cur["rows"].append([np.nan if t == "." else float(t) for t in toks])
    out = []
    for f in frames:
        if len(f["rows"]) == f["side"]:
            f["distance_mm"] = np.array(f.pop("rows"))
            out.append(f)
    return out


def load(path: str | Path) -> Capture:
    path = Path(path)
    parser = StreamParser()
    events = parser.feed(path.read_bytes())
    ranges: dict = {"A": [], "B": []}
    cnhs: dict = {"A": [], "B": []}
    infos: dict = {"A": [], "B": []}
    logs = []
    for ev in events:
        if ev[0] == "text":
            logs.append(ev[1])
            continue
        _, ftype, payload = ev
        if ftype == FT_RANGE:
            d = decode_range(payload)
            ranges.setdefault(d["port"], []).append(d)
        elif ftype == FT_CNH:
            d = decode_cnh(payload)
            cnhs.setdefault(d["port"], []).append(d)
        elif ftype == FT_INFO:
            j = json.loads(payload.decode("utf-8", "replace"))
            infos.setdefault(j.get("port", "?"), []).append(j)
        elif ftype == FT_LOG:
            logs.append(payload.decode("utf-8", "replace"))
    ports = {}
    for name in sorted(set(ranges) | set(cnhs) | set(infos)):
        r, skip_r = _stack(ranges.get(name, []))
        c, skip_c = _stack(cnhs.get(name, []))
        ports[name] = PortData(name, r, c, infos.get(name, []), skip_r + skip_c)
    meta_path = path.with_suffix(".json")
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.exists() else {}
    return Capture(path, ports, logs, meta, parser.crc_errors)
