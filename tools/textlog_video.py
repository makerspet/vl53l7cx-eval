"""Render a saved Serial Monitor text log into a video, for debugging and demos.

    py tools/textlog_video.py data/usbcable-jumperwire-breakoutpcb.txt
    py tools/textlog_video.py log.txt --out log.gif --speed 2 --mirror

Per frame: the depth map, each zone's height above a floor plane fitted to
that frame, a side view, and a top-down view, plus the fitted sensor height,
tilt and roll. Uses the ffmpeg bundled with imageio-ffmpeg if ffmpeg is not on
PATH; a .gif output needs only Pillow.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as manim  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import floor  # noqa: E402
import vlog  # noqa: E402

FLOOR_C, ABOVE_C, BELOW_C, NONE_C = "#9a9a9a", "#d62728", "#1f77b4", "#e8e8e8"


def tolerance(z: np.ndarray) -> np.ndarray:
    # A zone spans 7.5 deg, so a return can sit anywhere within about +-6.5% of
    # its range from the zone-centre ray. Far zones also extrapolate the plane
    # fitted on near zones, so each degree of tilt error adds ~1.7% of range.
    # 10% of range covers both; never claim better than 10 mm.
    return np.maximum(10.0, 0.10 * z)


def classify(h: np.ndarray, z: np.ndarray) -> np.ndarray:
    c = np.full(z.shape, "none", dtype=object)
    ok = np.isfinite(z)
    tol = tolerance(np.where(ok, z, 0))
    c[ok & (np.abs(h) <= tol)] = "floor"
    c[ok & (h > tol)] = "above"
    c[ok & (h < -tol)] = "below"
    return c


def draw_map(ax, values, side, title, cmap, vmin, vmax, fmt, mirror, cbar_holder, label):
    img = values[:, ::-1] if mirror else values
    im = ax.imshow(np.ma.masked_invalid(img), origin="lower", cmap=cmap, vmin=vmin, vmax=vmax)
    im.cmap.set_bad(NONE_C)
    for r in range(side):
        for c in range(side):
            v = img[r, c]
            ax.text(c, r, "-" if not np.isfinite(v) else fmt.format(v), ha="center", va="center", fontsize=8,
                    color="#555" if not np.isfinite(v) else "black")
    ax.set_title(title, fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    if label not in cbar_holder:
        cbar_holder[label] = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.02, label=label)


def render(frames, args, out: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 9), dpi=100)
    side = frames[0]["side"]
    half_fov = np.radians(floor.ZONE_DEG_8X8 * 4)
    t0 = frames[0]["frame"]
    cbars: dict = {}
    if out.suffix.lower() == ".gif":
        writer = manim.PillowWriter(fps=args.out_fps)
    else:
        if not shutil.which("ffmpeg"):
            try:
                import imageio_ffmpeg

                matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
            except ImportError:
                raise SystemExit("no ffmpeg: install imageio-ffmpeg (pip) or write a .gif")
        writer = manim.FFMpegWriter(fps=args.out_fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p"])

    with writer.saving(fig, str(out), dpi=100):
        for i, f in enumerate(frames):
            z = f["distance_mm"]
            fit = floor.fit_floor(z)
            for ax in axes.flat:
                ax.cla()
            t = (f["frame"] - t0) / args.sensor_hz
            head = f"{Path(args.log).name}   port {f['port']} [{f['driver']}]   frame {f['frame']}   t = {t:4.1f} s"
            draw_map(axes[0, 0], z, side, "distance, mm (nearest row at bottom)", "viridis_r", 0, args.max_mm,
                     "{:.0f}", args.mirror, cbars, "mm")
            if fit is None:
                fig.suptitle(head + "\nfloor plane: not enough points", fontsize=11)
                writer.grab_frame()
                continue
            P = floor.points(z)
            h = fit.heights(P)
            cls = classify(h, z)
            draw_map(axes[0, 1], np.clip(h, -999, 999), side, "height above fitted floor, mm", "RdBu_r",
                     -80, 80, "{:.0f}", args.mirror, cbars, "mm above floor")
            fig.suptitle(f"{head}\nsensor {fit.height_mm:.0f} mm above floor, axis {fit.tilt_deg:.0f}° down, "
                         f"roll {fit.roll_deg:+.0f}°, floor fit rms {fit.rms_mm:.1f} mm on "
                         f"{int(fit.inliers.sum())} zones", fontsize=11)

            F = fit.floor_frame(P)  # forward, left, up
            colors = {"floor": FLOOR_C, "above": ABOVE_C, "below": BELOW_C}

            # Side view.
            ax = axes[1, 0]
            ax.axhline(0, color="k", lw=1.5)
            ax.fill_between([0, args.max_mm], -1000, 0, color="#f3efe6", zorder=0)
            hs = fit.height_mm
            tilt = np.radians(fit.tilt_deg)
            for a in (tilt - half_fov, tilt + half_fov):
                if a > 1e-3:
                    ax.plot([0, hs / np.tan(a)], [hs, 0], ":", color="gray", lw=1)
                else:  # edge ray at or above the horizon
                    ax.plot([0, args.max_mm], [hs, hs - args.max_mm * np.tan(a)], ":", color="gray", lw=1)
            ax.plot([0], [hs], "ks", ms=8)
            for k, col in colors.items():
                sel = cls == k
                ax.scatter(F[..., 0][sel], F[..., 2][sel], s=22, c=col, label=k, edgecolors="none")
            ax.set_xlim(-30, args.max_mm)
            ax.set_ylim(-320, 260)
            ax.set_xlabel("forward along floor, mm")
            ax.set_ylabel("height above floor, mm")
            ax.set_title("side view", fontsize=10)
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(alpha=0.3)

            # Top-down view.
            ax = axes[1, 1]
            for s in (-1, 1):
                ax.plot([0, s * args.max_mm * np.tan(half_fov)], [0, args.max_mm], ":", color="gray", lw=1)
            ax.plot([0], [0], "ks", ms=8)
            left = -F[..., 1] if args.mirror else F[..., 1]
            for k, col in colors.items():
                sel = cls == k
                ax.scatter(-left[sel], F[..., 0][sel], s=22, c=col, edgecolors="none")
            ax.set_xlim(-args.max_mm * 0.6, args.max_mm * 0.6)
            ax.set_ylim(-30, args.max_mm)
            ax.set_aspect("equal")
            ax.set_xlabel("right, mm")
            ax.set_ylabel("forward, mm")
            ax.set_title("top view (colour = above / on / below the floor)", fontsize=10)
            ax.grid(alpha=0.3)

            plt.tight_layout(rect=(0, 0, 1, 0.94))
            # Hold each logged frame for its real duration (scaled by --speed).
            nxt = frames[i + 1]["frame"] if i + 1 < len(frames) else f["frame"] + 6
            reps = max(1, round((nxt - f["frame"]) / args.sensor_hz * args.out_fps / args.speed))
            for _ in range(reps):
                writer.grab_frame()
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="Serial Monitor text log")
    ap.add_argument("--out", help="output .mp4 or .gif (default: next to the log)")
    ap.add_argument("--port", help="A or B (default: the first port in the log)")
    ap.add_argument("--sensor-hz", type=float, default=10.0, help="ranging rate the log was taken at")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed, 1 = real time")
    ap.add_argument("--out-fps", type=int, default=10)
    ap.add_argument("--max-mm", type=float, default=1200, help="range shown in maps and views")
    ap.add_argument("--mirror", action="store_true", help="flip left/right if the view looks mirrored")
    args = ap.parse_args()

    frames = vlog.parse_text_log(args.log)
    if not frames:
        raise SystemExit("no frames found in the log")
    port = args.port or frames[0]["port"]
    frames = [f for f in frames if f["port"] == port]
    out = Path(args.out) if args.out else Path(args.log).with_suffix(".mp4")
    render(frames, args, out)
    print(f"wrote {out} ({len(frames)} logged frames, port {port})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
