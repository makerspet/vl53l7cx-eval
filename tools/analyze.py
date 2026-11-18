"""Summarise a capture and compare the two ports.

    py analyze.py captures/20261118_101500_wall_50cm.vlog
    py analyze.py captures/...vlog --zone 3,4 --plot

Per port: frame rate, I2C read time, and per-zone valid fraction, median
distance and temporal noise (std). If CNH histograms are present it also:
  - fits the CNH bin -> mm mapping against the CH driver's own ranged distance;
  - counts frames where a zone's histogram has a second peak while the ranging
    output reports a single target there, i.e. information the depth output
    dropped. This is the core CX-vs-CH question.
"""
from __future__ import annotations

import argparse
import sys
import warnings

import numpy as np

import vlog


def valid_mask(r: dict) -> np.ndarray:
    """[frames, zones] True where target 0 is valid."""
    return (r["nb_target_detected"] > 0) & np.isin(r["target_status"][:, :, 0], vlog.VALID_STATUS)


def grid(values: np.ndarray, side: int, fmt: str) -> str:
    rows = []
    for r in range(side):
        cells = values[r * side:(r + 1) * side]
        rows.append(" ".join(" " * len(fmt.format(0.0)) if np.isnan(v) else fmt.format(v) for v in cells))
    return "\n".join("    " + row for row in rows)


def zone_stats(r: dict) -> dict:
    ok = valid_mask(r)
    d = r["distance_mm"][:, :, 0].astype(float)
    dv = np.where(ok, d, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN zones
        return dict(valid=ok.mean(0) * 100, median=np.nanmedian(dv, 0), std=np.nanstd(dv, 0))


def prominence(h: np.ndarray, i: int) -> float:
    """Height of peak i above the higher of the two valleys separating it from taller bins (or the edges)."""
    left = h[:i][::-1]
    stop = np.argmax(left > h[i]) if (left > h[i]).any() else len(left)
    lmin = left[:stop].min() if stop else h[i]
    right = h[i + 1:]
    stop = np.argmax(right > h[i]) if (right > h[i]).any() else len(right)
    rmin = right[:stop].min() if stop else h[i]
    return float(h[i] - max(lmin, rmin))


def find_peaks(h: np.ndarray, min_sep: int, rel: float, floor: float) -> np.ndarray:
    """Local maxima whose prominence is >= floor and >= rel * the main peak's.

    The caller supplies `floor` (k x noise). A single 18-bin histogram is too
    short to estimate its own noise: the laser pulse can cover half of it.
    """
    cand = [i for i in range(len(h)) if (i == 0 or h[i] > h[i - 1]) and (i == len(h) - 1 or h[i] >= h[i + 1])]
    prom = {i: prominence(h, i) for i in cand}
    top = max(prom.values(), default=0.0)
    cand = [i for i in cand if prom[i] >= floor and prom[i] >= rel * top]
    cand.sort(key=lambda i: -prom[i])
    kept: list[int] = []
    for i in cand:
        if all(abs(i - j) >= min_sep for j in kept):
            kept.append(i)
    return np.array(sorted(kept), dtype=int)


def parabolic(h: np.ndarray, i: int) -> float:
    if 0 < i < len(h) - 1:
        a, b, c = h[i - 1], h[i], h[i + 1]
        den = a - 2 * b + c
        if den != 0:
            return i + 0.5 * (a - c) / den
    return float(i)


def cnh_analysis(p: vlog.PortData, args) -> None:
    c, r = p.cnh, p.range
    n_frames = len(c["seq"])
    print(f"  CNH: {n_frames} frames, {c['aggregates']} aggregates x {c['bins']} bins, start_bin {c['start_bin']}, "
          f"sub {c['sub']} ({c['sub'] * vlog.CNH_BIN_MM:.1f} mm/bin), merge {c['merge']}")
    if c["merge"] != 1:
        print("  (peak/ranging comparison needs merge 1: one aggregate per zone)")
        return
    # Join CNH frames to range frames of the same port by sequence number.
    idx = {s: i for i, s in enumerate(r["seq"])}
    pairs = [(ci, idx[s]) for ci, s in enumerate(c["seq"]) if s in idx]
    if not pairs:
        print("  no CNH frames match range frames")
        return
    ok = valid_mask(r)
    ntd = r["nb_target_detected"]
    dist = r["distance_mm"][:, :, 0].astype(float)
    x_nom = vlog.bin_centers_mm(c["start_bin"], c["bins"], c["sub"])
    step = c["sub"] * vlog.CNH_BIN_MM
    # The laser pulse spans ~10 device bins; peaks closer than ~half that merge.
    min_sep = max(1, int(round(5 / c["sub"])))
    # Noise per aggregate: median over bins of each bin's spread across frames.
    # Right for the static scenes this evaluation uses; conservative if things move.
    noise = np.median(c["hist"].std(axis=0), axis=-1)

    fit_x, fit_y = [], []
    extra = np.zeros(c["aggregates"])
    single = np.zeros(c["aggregates"])
    no_target_peak = np.zeros(c["aggregates"])
    no_target = np.zeros(c["aggregates"])
    for ci, ri in pairs:
        for z in range(c["aggregates"]):
            h = c["hist"][ci, z]
            pk = find_peaks(h, min_sep, args.peak_rel, args.peak_k * noise[z])
            if ok[ri, z] and ntd[ri, z] == 1:
                single[z] += 1
                if len(pk) >= 2:
                    extra[z] += 1
                if len(pk):
                    main = pk[np.argmax(h[pk])]
                    fit_x.append(x_nom[0] + parabolic(h, main) * step)
                    fit_y.append(dist[ri, z])
            elif ntd[ri, z] == 0:
                no_target[z] += 1
                if len(pk):
                    no_target_peak[z] += 1

    if len(fit_x) > 10:
        fx, fy = np.array(fit_x), np.array(fit_y)
        if np.ptp(fy) >= 300:  # enough depth spread for a slope
            a, b = np.polyfit(fx, fy, 1)
            resid = fy - (a * fx + b)
            print(f"  bin->mm fit vs CH ranging: distance = {a:.3f} * nominal + {b:.0f} mm  "
                  f"(residual std {resid.std():.0f} mm, n={len(fx)})")
        else:  # one distance only: an offset is all the data supports
            off = np.median(fy - fx)
            print(f"  bin->mm offset vs CH ranging: distance = nominal + {off:.0f} mm  "
                  f"(spread {np.std(fy - fx - off):.0f} mm, n={len(fx)}; capture a range of distances for a slope)")
    side = 8 if c["zones"] == 64 else 4
    with np.errstate(invalid="ignore", divide="ignore"):
        pct = np.where(single > 0, extra / single * 100, np.nan)
        pct0 = np.where(no_target > 0, no_target_peak / no_target * 100, np.nan)
    print(f"  % of single-target frames whose histogram shows a 2nd peak "
          f"(prominence >= {args.peak_rel:.0%} of main and >= {args.peak_k:g} x temporal noise):")
    print(grid(pct, side, "{:5.1f}"))
    print("  % of no-target frames whose histogram still shows a peak:")
    print(grid(pct0, side, "{:5.1f}"))


def summarise(cap: vlog.Capture, args) -> None:
    print(f"{cap.path.name}: {cap.meta.get('label', '')} {cap.meta.get('notes', '')}".rstrip())
    print(f"  CRC errors: {cap.crc_errors}")
    for name, p in cap.ports.items():
        if not p.frames:
            last = p.info[-1] if p.info else {}
            print(f"\nport {name}: no frames ({last.get('event', 'no info')}: {last.get('last_error', '')})")
            continue
        r = p.range
        info = p.info[-1] if p.info else {}
        dur = (r["t_us"][-1] - r["t_us"][0]) / 1e6
        gaps = np.diff(r["seq"])
        print(f"\nport {name} [{p.driver}] ids 0x{info.get('device_id', 0):02X}/0x{info.get('revision_id', 0):02X}, "
              f"{p.side}x{p.side}, {p.frames} frames in {dur:.1f} s = {p.fps():.1f} fps, "
              f"I2C read {np.mean(r['read_us']) / 1000:.1f} ms avg, temp {np.median(r['temp_c']):.0f} C"
              + (f", {p.skipped} frames skipped (config changed)" if p.skipped else "")
              + (f", seq gaps {int((gaps != 1).sum())}" if (gaps != 1).any() else ""))
        s = zone_stats(r)
        print("  valid %:")
        print(grid(s["valid"], p.side, "{:5.0f}"))
        print("  median distance, mm:")
        print(grid(s["median"], p.side, "{:5.0f}"))
        print("  temporal std, mm:")
        print(grid(s["std"], p.side, "{:5.1f}"))
        if p.cnh:
            cnh_analysis(p, args)


def plot(cap: vlog.Capture, args) -> None:
    import matplotlib.pyplot as plt

    ports = [p for p in cap.ports.values() if p.frames]
    fig, axes = plt.subplots(3, len(ports), figsize=(5 * len(ports), 11), squeeze=False)
    for i, p in enumerate(ports):
        s = zone_stats(p.range)
        for row, (key, label, cmap) in enumerate((("median", "median mm", "viridis_r"), ("std", "std mm", "magma"),
                                                  ("valid", "valid %", "Greens"))):
            ax = axes[row, i]
            im = ax.imshow(s[key].reshape(p.side, p.side), cmap=cmap)
            ax.set_title(f"port {p.name} [{p.driver}] {label}")
            ax.set_xticks([])
            ax.set_yticks([])
            fig.colorbar(im, ax=ax, fraction=0.046)
    plt.tight_layout()

    if args.zone:
        rr, cc = (int(v) for v in args.zone.split(","))
        fig2, ax = plt.subplots(1 + sum(1 for p in ports if p.cnh), 1, figsize=(10, 8), squeeze=False)
        ax = ax[:, 0]
        for p in ports:
            r = p.range
            z = rr * p.side + cc
            ok = valid_mask(r)[:, z]
            t = (r["t_us"] - r["t_us"][0]) / 1e6
            ax[0].plot(t[ok], r["distance_mm"][ok, z, 0], ".", ms=3, label=f"port {p.name} [{p.driver}]")
        ax[0].set_ylabel(f"zone ({rr},{cc}) distance, mm")
        ax[0].legend()
        k = 1
        for p in ports:
            if not p.cnh:
                continue
            c = p.cnh
            agg = vlog.zone_to_aggregate(rr * p.side + cc, c["zones"], c["merge"], c["cols"])
            x = vlog.bin_centers_mm(c["start_bin"], c["bins"], c["sub"])
            t = (c["t_us"] - c["t_us"][0]) / 1e6
            half = c["sub"] * vlog.CNH_BIN_MM / 2
            ax[k].imshow(c["hist"][:, agg].T, aspect="auto", origin="lower", cmap="inferno",
                         extent=(t[0], t[-1], x[0] - half, x[-1] + half))
            r = p.range
            okz = valid_mask(r)[:, rr * p.side + cc]
            tr = (r["t_us"] - c["t_us"][0]) / 1e6
            ax[k].plot(tr[okz], r["distance_mm"][okz, rr * p.side + cc, 0], "c.", ms=2, label="ranged distance")
            ax[k].set_ylabel(f"port {p.name} CNH agg {agg}\nnominal mm")
            ax[k].legend(loc="upper right", fontsize=8)
            k += 1
        ax[-1].set_xlabel("time, s")
        plt.tight_layout()
    plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="+", help=".vlog file(s)")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--zone", help="row,col to plot over time (with --plot)")
    ap.add_argument("--peak-rel", type=float, default=0.1,
                    help="a 2nd peak's prominence must be >= this fraction of the main peak's")
    ap.add_argument("--peak-k", type=float, default=6.0,
                    help="a peak's prominence must be >= k x the zone's temporal noise")
    args = ap.parse_args()
    for path in args.capture:
        cap = vlog.load(path)
        summarise(cap, args)
        if args.plot:
            plot(cap, args)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
