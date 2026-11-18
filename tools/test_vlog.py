"""Offline tests for the parser and analysis: py -m pytest tools  (or: py tools/test_vlog.py)

Frames are packed the way firmware/vl53l7_eval/tof_port.cpp packs them.
"""
from __future__ import annotations

import json
import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze  # noqa: E402
import vlog  # noqa: E402


def range_frame(port, driver, seq, t_us, dist, status, ntd, nz=64, nt=1):
    hdr = struct.pack("<BBBBBbHIqI", 1, port, driver, nz, nt, 31, 0, seq, t_us, 2500)
    body = (np.asarray(ntd, "<u1").tobytes() + np.full(nz, 5, "<u4").tobytes() + np.full(nz, 900, "<u4").tobytes()
            + np.asarray(dist, "<i2").tobytes() + np.full(nz * nt, 2, "<u2").tobytes()
            + np.full(nz * nt, 1000, "<u4").tobytes() + np.full(nz * nt, 40, "<u1").tobytes()
            + np.asarray(status, "<u1").tobytes())
    return vlog.encode_frame(vlog.FT_RANGE, hdr + body)


def cnh_frame(port, seq, t_us, hist, ambient, start=0, sub=2, merge=1, cols=8, nz=64):
    n_agg, n_bins = hist.shape
    hdr = struct.pack("<BBBBBBBBhHIqI", 1, port, nz, n_agg, n_bins, sub, merge, cols, start, 0, seq, t_us, 2048)
    body = np.column_stack([ambient, hist]).astype("<f4").tobytes()
    return vlog.encode_frame(vlog.FT_CNH, hdr + body)


def pulse(x_mm, center_mm, amp, width_mm=150.0):
    return amp * np.exp(-0.5 * ((x_mm - center_mm) / width_mm) ** 2)


def synthetic_capture(path: Path, frames=40) -> None:
    rng = np.random.default_rng(1)
    x = vlog.bin_centers_mm(0, 18, 2)
    out = bytearray(b"ets Jun  8 2016 00:22:57\r\nrst:0x1 (POWERON_RESET)\r\n")
    for port in (0, 1):
        info = {"port": "AB"[port], "event": "started", "loaded": ["cx", "ch"][port], "device_id": 0xF0,
                "revision_id": 2, "last_error": ""}
        out += vlog.encode_frame(vlog.FT_INFO, json.dumps(info).encode())
    out += vlog.encode_frame(vlog.FT_LOG, b"port A: cx driver loaded in 1234 ms")
    for i in range(frames):
        t = 1_000_000 + i * 100_000
        dist = 500 + rng.normal(0, 3, 64).round()
        status = np.full(64, 5)
        ntd = np.ones(64)
        status[0], ntd[0] = 255, 0  # zone 0: no target
        out += range_frame(0, 1, i + 1, t, dist, status, ntd)
        out += range_frame(1, 2, i + 1, t + 300, dist + 20, status, ntd)
        hist = np.stack([pulse(x, 520, 1000) for _ in range(64)]) + rng.normal(0, 2, (64, 18))
        hist[28] += pulse(x, 1100, 300)  # zone (3,4): a weaker second return
        out += cnh_frame(1, i + 1, t + 300, hist, np.full(64, 3.0))
        if i == 5:
            out += b"\xa5\x5a\x01\x10\x00garbage-with-bad-crc"  # corrupted frame mid-stream
    path.write_bytes(bytes(out))


def test_crc_check_value():
    assert vlog.crc16(b"123456789") == 0x29B1  # CRC-16/CCITT-FALSE


def test_parser_split_feed_and_text():
    data = b"hello\n" + vlog.encode_frame(vlog.FT_LOG, b"x") + b"bye\n"
    p = vlog.StreamParser()
    ev = []
    for i in range(len(data)):  # one byte at a time, incl. a split sync word
        ev += p.feed(data[i:i + 1])
    assert ev == [("text", "hello"), ("frame", vlog.FT_LOG, b"x"), ("text", "bye")]


def test_load_and_decode():
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "t.vlog"
        synthetic_capture(path)
        cap = vlog.load(path)
    assert cap.crc_errors >= 1
    a, b = cap.ports["A"], cap.ports["B"]
    assert a.frames == b.frames == 40
    assert a.driver == "cx" and b.driver == "ch"
    assert a.range["distance_mm"].shape == (40, 64, 1)
    assert b.cnh["hist"].shape == (40, 64, 18)
    assert abs(a.fps() - 10.0) < 0.01
    assert b.info[0]["loaded"] == "ch"
    assert any("loaded" in s for s in cap.logs)


def test_peak_finder():
    x = vlog.bin_centers_mm(0, 18, 2)
    rng = np.random.default_rng(0)
    floor = 6 * 2.0  # k x noise sigma
    for _ in range(50):
        noise = rng.normal(10, 2, 18)
        assert len(analyze.find_peaks(noise, 3, 0.1, floor)) == 0
        assert len(analyze.find_peaks(pulse(x, 520, 1000) + noise, 3, 0.1, floor)) == 1
        two = analyze.find_peaks(pulse(x, 520, 1000) + pulse(x, 1100, 300) + noise, 3, 0.1, floor)
        assert len(two) == 2 and abs(x[two[1]] - 1100) < 100


def test_analyze_runs(capsys):
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "t.vlog"
        synthetic_capture(path)
        args = type("A", (), {"peak_rel": 0.1, "peak_k": 6.0})()
        analyze.summarise(vlog.load(path), args)
    out = capsys.readouterr().out
    assert "port A [cx]" in out and "port B [ch]" in out
    assert "bin->mm offset" in out  # single-distance scene: offset only, no slope
    # The second-peak grid: 100% at zone (3,4) where the extra return was planted, 0% elsewhere.
    lines = out.splitlines()
    i = next(n for n, s in enumerate(lines) if "2nd peak" in s)
    rows = [[float(v) for v in lines[i + 1 + r].split()] for r in range(8)]
    assert rows[3][4] == 100.0
    assert sum(sum(r) for r in rows) == 100.0


if __name__ == "__main__":
    import pytest

    sys.exit(pytest.main([__file__, "-q"]))
