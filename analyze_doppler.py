"""
Doppler cart data analysis.
Scatter plot: encoder velocity vs. frequency shift, with amplitude threshold sweep.
"""

import csv
import math
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from pathlib import Path

# ── データ読み込み ────────────────────────────────────────
DATA_FILES = [
    Path.home() / "Desktop/55ms-1.txt",
    Path.home() / "Desktop/55ms-2.txt",
    Path.home() / "Desktop/55ms-3.txt",
    Path.home() / "Desktop/55ms-4.txt",
]

RUNS_COLORS = ["#0077cc", "#cc6600", "#4a8c00", "#9933aa"]
RUNS_LABELS = ["Run 1", "Run 2", "Run 3", "Run 4"]

V_SOUND = 343.0   # m/s
V_ART   = 5.0     # |vel| > this → encoder artifact, excluded

def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                freq = float(row["freq_hz"])
                amp  = float(row["amplitude"])
                vel  = float(row["vel_ms"])
                rows.append(dict(freq=freq, amp=amp, vel=vel))
            except (ValueError, KeyError):
                pass
    return rows

all_rows = [load_csv(p) for p in DATA_FILES]

# ── f_ref: 各runで|vel|<0.05 かつ freq>0 の中央値 ─────────
def get_fref(rows, amp_min):
    stationary = [r["freq"] for r in rows
                  if r["freq"] > 0 and abs(r["vel"]) < 0.05 and r["amp"] >= amp_min]
    if len(stationary) < 5:
        # フォールバック: amp条件緩めて全静止点
        stationary = [r["freq"] for r in rows
                      if r["freq"] > 0 and abs(r["vel"]) < 0.05]
    return float(np.median(stationary)) if stationary else 997.0

# ── 理論曲線 ──────────────────────────────────────────────
# データから: vel と Δf は同符号 → Δf = f0*v/(vs - v)
v_th = np.linspace(-3.5, 3.5, 400)
def delta_f_theory(v, f0):
    return f0 * v / (V_SOUND - v)

# ── 図1: amplitude閾値ごとの散布図 (2×2) ────────────────
AMP_THRESHOLDS = [50, 100, 150, 200]

fig, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
fig.suptitle("Doppler Cart — Velocity vs. Frequency Shift\n"
             "(encoder velocity vs. Δf = f_obs − f_ref)", fontsize=13)

for ax, amp_min in zip(axes.flat, AMP_THRESHOLDS):
    for rows, color, label in zip(all_rows, RUNS_COLORS, RUNS_LABELS):
        fref = get_fref(rows, amp_min)
        vels, dfs = [], []
        for r in rows:
            if r["freq"] <= 0:        continue  # 検出失敗
            if r["amp"] < amp_min:    continue  # 振幅フィルタ
            if abs(r["vel"]) > V_ART: continue  # エンコーダー異常
            if abs(r["vel"]) < 0.05:  continue  # 静止点は除外（f_ref推定に使用済み）
            vels.append(r["vel"])
            dfs.append(r["freq"] - fref)

        ax.scatter(vels, dfs, s=18, alpha=0.55, color=color, label=label, linewidths=0)

    # 理論曲線（代表的な f_ref ≈ 997 Hz で描画）
    f0_rep = np.median([get_fref(rows, amp_min) for rows in all_rows])
    ax.plot(v_th, delta_f_theory(v_th, f0_rep), "k-", lw=1.8,
            label=f"Theory (f₀={f0_rep:.1f} Hz)", zorder=5)
    ax.axhline(0, color="#888", lw=0.8, ls="--")
    ax.axvline(0, color="#888", lw=0.8, ls="--")

    ax.set_xlabel("Velocity  v  (m/s)")
    ax.set_ylabel("Δf  (Hz)")
    ax.set_title(f"Amplitude ≥ {amp_min}")
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-12, 12)
    ax.legend(fontsize=7, markerscale=1.5)
    ax.grid(True, alpha=0.25)

fig.savefig("doppler_scatter_thresholds.png", dpi=150)
plt.close(fig)
print("Saved: doppler_scatter_thresholds.png")

# ── 図2: best threshold の大きな散布図 ──────────────────
AMP_BEST = 150

fig2, ax2 = plt.subplots(figsize=(8, 6), constrained_layout=True)
ax2.set_title(f"Doppler Effect — Velocity vs. Δf  (amplitude ≥ {AMP_BEST})\n"
              f"All 4 runs combined", fontsize=12)

all_v, all_df = [], []
for rows, color, label in zip(all_rows, RUNS_COLORS, RUNS_LABELS):
    fref = get_fref(rows, AMP_BEST)
    vels, dfs = [], []
    for r in rows:
        if r["freq"] <= 0:          continue
        if r["amp"] < AMP_BEST:     continue
        if abs(r["vel"]) > V_ART:   continue
        if abs(r["vel"]) < 0.05:    continue
        vels.append(r["vel"])
        dfs.append(r["freq"] - fref)
    ax2.scatter(vels, dfs, s=30, alpha=0.65, color=color, label=label, linewidths=0)
    all_v.extend(vels); all_df.extend(dfs)

# 理論曲線
f0_rep = np.median([get_fref(rows, AMP_BEST) for rows in all_rows])
ax2.plot(v_th, delta_f_theory(v_th, f0_rep), "k-", lw=2.2,
         label=f"Theory: Δf = f₀v/(vₛ−v)\n(f₀={f0_rep:.1f} Hz, vₛ=343 m/s)", zorder=6)

# 線形近似（参考）
if len(all_v) >= 5:
    coeffs = np.polyfit(all_v, all_df, 1)
    v_fit  = np.linspace(min(all_v), max(all_v), 100)
    ax2.plot(v_fit, np.polyval(coeffs, v_fit), "r--", lw=1.4,
             label=f"Linear fit: slope={coeffs[0]:.2f} Hz/(m/s)\n"
                   f"  theory: {f0_rep/V_SOUND:.2f} Hz/(m/s)")

ax2.axhline(0, color="#888", lw=0.8, ls="--")
ax2.axvline(0, color="#888", lw=0.8, ls="--")
ax2.set_xlabel("Velocity  v  (m/s)", fontsize=12)
ax2.set_ylabel("Δf = f_obs − f_ref  (Hz)", fontsize=12)
ax2.set_xlim(-3.5, 3.5)
ax2.set_ylim(-12, 12)
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

fig2.savefig("doppler_scatter_best.png", dpi=150)
plt.close(fig2)
print("Saved: doppler_scatter_best.png")

# ── 図3: 時系列（各run, 生データ vs 振幅フィルタ済み） ────
fig3, axes3 = plt.subplots(4, 2, figsize=(14, 16), constrained_layout=True)
fig3.suptitle("Time Series: Velocity & Frequency (raw vs. amplitude-filtered)", fontsize=12)

for row_i, (rows, color, label) in enumerate(zip(all_rows, RUNS_COLORS, RUNS_LABELS)):
    fref = get_fref(rows, AMP_BEST)
    times = list(range(len(rows)))  # index as proxy for time (equal interval)

    # elapsed_s が欲しいので再読み込み
    with open(DATA_FILES[row_i]) as f:
        reader2 = csv.DictReader(f)
        tdata = []
        for r in reader2:
            try:
                tdata.append(dict(
                    t=float(r["elapsed_s"]),
                    vel=float(r["vel_ms"]),
                    freq=float(r["freq_hz"]),
                    amp=float(r["amplitude"]),
                ))
            except (ValueError, KeyError):
                pass

    ts_all   = [d["t"]    for d in tdata]
    vel_all  = [d["vel"]  for d in tdata]
    freq_all = [d["freq"] for d in tdata]
    amp_all  = [d["amp"]  for d in tdata]

    # フィルタ済み（freq > 0 かつ amp >= AMP_BEST）
    ts_f    = [d["t"]             for d in tdata if d["freq"] > 0 and d["amp"] >= AMP_BEST]
    vel_f   = [d["vel"]           for d in tdata if d["freq"] > 0 and d["amp"] >= AMP_BEST]
    df_f    = [d["freq"] - fref   for d in tdata if d["freq"] > 0 and d["amp"] >= AMP_BEST]

    # 左: velocity
    ax_v = axes3[row_i, 0]
    ax_v.plot(ts_all, vel_all, color="#ccc", lw=0.9, label="raw")
    ax_v.scatter(ts_f, vel_f, s=12, color=color, alpha=0.8, label=f"amp≥{AMP_BEST}", zorder=3)
    ax_v.axhline(0, color="#aaa", lw=0.7, ls="--")
    ax_v.set_ylabel("vel (m/s)")
    ax_v.set_title(f"{label} — Velocity")
    ax_v.legend(fontsize=7)
    ax_v.grid(True, alpha=0.2)

    # 右: Δf
    ax_f = axes3[row_i, 1]
    df_all_clean = [d["freq"] - fref for d in tdata if d["freq"] > 0]
    ts_clean     = [d["t"]           for d in tdata if d["freq"] > 0]
    ax_f.scatter(ts_clean, df_all_clean, s=8, color="#ddd", label="raw (freq>0)")
    ax_f.scatter(ts_f, df_f, s=12, color=color, alpha=0.8, label=f"amp≥{AMP_BEST}", zorder=3)
    ax_f.axhline(0, color="#aaa", lw=0.7, ls="--")
    ax_f.set_ylabel("Δf (Hz)")
    ax_f.set_title(f"{label} — Freq shift")
    ax_f.legend(fontsize=7)
    ax_f.grid(True, alpha=0.2)
    if row_i == 3:
        ax_v.set_xlabel("elapsed (s)")
        ax_f.set_xlabel("elapsed (s)")

fig3.savefig("doppler_timeseries_filtered.png", dpi=150)
plt.close(fig3)
print("Saved: doppler_timeseries_filtered.png")

# ── コンソール統計 ────────────────────────────────────────
print("\n=== Point counts by amplitude threshold ===")
print(f"{'Threshold':>10}  {'Total pts':>10}  {'Amp-filtered':>13}  {'Moving (|v|>0.05)':>18}")
for amp_min in [50, 100, 150, 200]:
    total = moving = filtered_moving = 0
    for rows in all_rows:
        for r in rows:
            total += 1
            if abs(r["vel"]) > 0.05 and abs(r["vel"]) < V_ART and r["freq"] > 0:
                moving += 1
                if r["amp"] >= amp_min:
                    filtered_moving += 1
    print(f"{amp_min:>10}  {total:>10}  {filtered_moving:>13}  {moving:>18}")
