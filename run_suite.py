"""
run_suite.py - launch test_mpc_locomotion_sdk2.py N times per config.

Each run is a FRESH process (clean DDS/GC each time) writing to
  logs/suite/<preset>/run_<k>/

Resilient: each run has a hard timeout; if a run hangs (e.g. CycloneDDS
participant-creation deadlock on rapid restart), the whole process group is
killed and the suite CONTINUES with the next run.

Usage:
  # standalone presets (nothing else needed):
  python run_suite.py --presets 4 5 6 --repeats 10 --duration 120

  # 2-proc presets (START mujoco_sim_sdk2.py FIRST, in another terminal):
  python run_suite.py --presets 1 2 3 normal --repeats 10 --duration 120
"""
import argparse, os, signal, subprocess, sys, time

ap = argparse.ArgumentParser()
ap.add_argument("--presets", nargs="+", required=True, help="any of: 1 2 3 normal 4 5 6")
ap.add_argument("--repeats", type=int, default=10)
ap.add_argument("--duration", type=float, default=120.0)
ap.add_argument("--warmup", type=float, default=5.0)
ap.add_argument("--out", default="logs/suite")
ap.add_argument("--python", default=sys.executable)
ap.add_argument("--grace", type=float, default=60.0, help="extra seconds before a run is declared hung")
ap.add_argument("--gap", type=float, default=5.0, help="seconds between runs (DDS teardown)")
args = ap.parse_args()

timeout = args.duration + args.grace
results = []  # (preset, k, status, secs)

def kill_group(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass

try:
    for preset in args.presets:
        for k in range(args.repeats):
            out_dir = os.path.join(args.out, preset, f"run_{k}")
            os.makedirs(out_dir, exist_ok=True)
            cmd = [args.python, "test_mpc_locomotion_sdk2.py",
                   "--preset", preset,
                   "--duration", str(args.duration),
                   "--warmup", str(args.warmup),
                   "--debug-log", "--debug-log-dir", out_dir]
            print(f"\n=== preset {preset}  run {k+1}/{args.repeats} -> {out_dir} ===")
            t0 = time.time()
            # start_new_session=True -> child gets its own process group so we can
            # kill it AND its threads/children if it hangs.
            p = subprocess.Popen(cmd, start_new_session=True)
            try:
                rc = p.wait(timeout=timeout)
                status = f"ok(rc={rc})"
            except subprocess.TimeoutExpired:
                kill_group(p)
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
                status = "HUNG-killed"
            secs = time.time() - t0
            results.append((preset, k, status, secs))
            print(f"=== {status} in {secs:.0f}s ===")
            time.sleep(args.gap)   # let DDS sockets/ports fully release
except KeyboardInterrupt:
    print("\n[run_suite] interrupted by user; partial results kept.")

# summary
print("\n================ SUITE SUMMARY ================")
ok = sum(1 for _,_,s,_ in results if s.startswith("ok"))
print(f"completed: {ok}/{len(results)} runs")
for preset, k, status, secs in results:
    flag = "" if status.startswith("ok") else "  <-- check"
    print(f"  preset {preset:>6}  run {k}: {status:>12}  {secs:5.0f}s{flag}")
print("\nNext: python analyze_suite.py --suite logs/suite --out docs/latency_report.md")
