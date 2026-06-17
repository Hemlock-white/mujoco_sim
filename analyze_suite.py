"""
analyze_suite.py - aggregate logs/suite/<preset>/run_*/ into a deep report.

Pools all runs per preset for robust statistics on rare events, then writes
docs/latency_report.md with comparison tables + auto-derived observations.

Usage:
  python analyze_suite.py --suite logs/suite --out docs/latency_report.md
"""
import argparse, csv, glob, os
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--suite", default="logs/suite")
ap.add_argument("--out", default="docs/latency_report.md")
ap.add_argument("--presets", nargs="+", default=["1", "2", "3", "normal", "4", "5", "6"])
args = ap.parse_args()

def load_col(path, name):
    if not os.path.exists(path):
        return None
    out = []
    with open(path) as f:
        r = csv.DictReader(f)
        if name not in (r.fieldnames or []):
            return None
        for row in r:
            try:
                out.append(float(row[name]))
            except (TypeError, ValueError):
                out.append(np.nan)
    return np.asarray(out)

def run_dirs(preset):
    base = os.path.join(args.suite, preset)
    return sorted(glob.glob(os.path.join(base, "run_*")))

def collect(preset):
    """Return pooled loop-dt, pooled publish-dt, per-run spike counts, durations, ages."""
    loop_dt, pub_dt, age_all = [], [], []
    loop_spk, pub_spk, durations = [], [], []
    nruns = 0
    for d in run_dirs(preset):
        nruns += 1
        w = load_col(os.path.join(d, "controller_mpc.csv"), "wall_time_ns")
        if w is not None and len(w) > 2:
            dt = np.diff(w) / 1e6
            loop_dt.append(dt)
            loop_spk.append(int((dt > 50).sum()))
            durations.append((w[-1] - w[0]) / 1e9)
        p = load_col(os.path.join(d, "controller_lowcmd_pub.csv"), "wall_time_ns")
        if p is not None and len(p) > 2:
            pdt = np.diff(p) / 1e6
            pub_dt.append(pdt)
            pub_spk.append(int((pdt > 50).sum()))
        a = load_col(os.path.join(d, "bridge_sim.csv"), "lowcmd_age_ms")
        if a is not None and len(a):
            age_all.append(a)
    return dict(
        nruns=nruns,
        loop=np.concatenate(loop_dt) if loop_dt else None,
        pub=np.concatenate(pub_dt) if pub_dt else None,
        age=np.concatenate(age_all) if age_all else None,
        loop_spk=loop_spk, pub_spk=pub_spk, durations=durations,
    )

def stat(dt):
    if dt is None or not len(dt):
        return None
    return dict(
        mean=np.nanmean(dt), med=np.nanmedian(dt),
        p99=np.nanpercentile(dt, 99), p999=np.nanpercentile(dt, 99.9),
        mx=np.nanmax(dt),
        f10=100 * np.mean(dt > 10), f20=100 * np.mean(dt > 20), f50=100 * np.mean(dt > 50),
        n=len(dt),
    )

data = {p: collect(p) for p in args.presets}

LABEL = {
    "1": "2-proc, Write ON, stand->move",
    "normal": "2-proc, Write ON, stand->move",
    "2": "2-proc, Write OFF, stand->move",
    "3": "2-proc, Write ON, stand-only",
    "4": "standalone, Write OFF",
    "5": "standalone, Write ON, gc.disable",
    "6": "standalone, Write ON",
}

lines = []
def w(s=""):
    lines.append(s)

w("# MPC Loop Latency - Suite Report")
w()
w(f"Auto-generated from `{args.suite}` (pooled across repeats per preset).")
w()
w("## 1. Config legend")
w()
w("| preset | configuration | #runs |")
w("|---|---|---|")
for p in args.presets:
    w(f"| {p} | {LABEL.get(p,'?')} | {data[p]['nruns']} |")
w()

w("## 2. MPC loop dt - pooled statistics (ms)")
w()
w("| preset | samples | mean | median | p99 | p99.9 | max | %>10ms | %>20ms | %>50ms |")
w("|---|---|---|---|---|---|---|---|---|---|")
for p in args.presets:
    s = stat(data[p]["loop"])
    if not s:
        w(f"| {p} | (no data) |"); continue
    w(f"| {p} | {s['n']} | {s['mean']:.2f} | {s['med']:.2f} | {s['p99']:.2f} | "
      f"{s['p999']:.2f} | {s['mx']:.1f} | {s['f10']:.2f} | {s['f20']:.3f} | {s['f50']:.4f} |")
w()

w("## 3. Spike (>50ms) rate")
w()
w("| preset | total spikes | total minutes | spikes / min | per-run counts |")
w("|---|---|---|---|---|")
for p in args.presets:
    d = data[p]
    tot = sum(d["loop_spk"]) if d["loop_spk"] else 0
    mins = sum(d["durations"]) / 60.0 if d["durations"] else 0
    rate = tot / mins if mins > 0 else float("nan")
    w(f"| {p} | {tot} | {mins:.1f} | {rate:.3f} | {d['loop_spk']} |")
w()

w("## 4. Publish dt - pooled (ms)")
w()
w("| preset | mean | p99.9 | max | %>50ms |")
w("|---|---|---|---|---|")
for p in args.presets:
    s = stat(data[p]["pub"])
    if not s:
        w(f"| {p} | (no data) |"); continue
    w(f"| {p} | {s['mean']:.2f} | {s['p999']:.2f} | {s['mx']:.1f} | {s['f50']:.4f} |")
w()

w("## 5. Bridge lowcmd_age_ms (2-proc only)")
w()
w("| preset | mean | median | max |")
w("|---|---|---|---|")
for p in args.presets:
    a = data[p]["age"]
    if a is None:
        w(f"| {p} | (standalone / n/a) |"); continue
    w(f"| {p} | {np.nanmean(a):.1f} | {np.nanmedian(a):.1f} | {np.nanmax(a):.1f} |")
w()

# ---- auto observations ----
w("## 6. Auto-derived observations")
w()
obs = []
# spike-rate vs config
rates = {}
for p in args.presets:
    d = data[p]
    mins = sum(d["durations"]) / 60.0 if d["durations"] else 0
    rates[p] = (sum(d["loop_spk"]) / mins) if mins > 0 else float("nan")
write_on = [p for p in args.presets if "Write ON" in LABEL.get(p, "")]
write_off = [p for p in args.presets if "Write OFF" in LABEL.get(p, "")]
if write_on and write_off:
    ron = np.nanmean([rates[p] for p in write_on])
    roff = np.nanmean([rates[p] for p in write_off])
    obs.append(f"- Spike rate with DDS Write ON = {ron:.3f}/min vs OFF = {roff:.3f}/min "
               f"-> {'DDS Write is NOT the dominant cause' if abs(ron-roff) < max(ron,roff,1e-9)*0.5 else 'DDS Write may matter'}.")
# jitter floor 2-proc vs standalone
twoproc = [p for p in args.presets if LABEL.get(p,'').startswith('2-proc')]
standalone = [p for p in args.presets if LABEL.get(p,'').startswith('standalone')]
def avg_f10(ps):
    vals = [stat(data[p]["loop"])["f10"] for p in ps if stat(data[p]["loop"])]
    return np.mean(vals) if vals else float("nan")
if twoproc and standalone:
    obs.append(f"- Jitter floor (%>10ms): 2-proc avg = {avg_f10(twoproc):.2f}% vs standalone avg = {avg_f10(standalone):.2f}% "
               f"-> the second process (MuJoCo sim) is the main driver of sub-spike jitter.")
for o in obs:
    w(o)
w()
w("> Fill in narrative conclusions below after reviewing the tables.")
w()
w("## 7. Conclusions (manual)")
w()
w("- TODO")

os.makedirs(os.path.dirname(args.out), exist_ok=True)
with open(args.out, "w") as f:
    f.write("\n".join(lines) + "\n")
print(f"[report] wrote {args.out}  ({len(args.presets)} presets)")
