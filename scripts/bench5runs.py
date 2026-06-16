"""
bench5runs.py — Run test_metaheuristics_5s 5 times (seeds 42-46) on Cordeau
instances pr01-pr20 with vehicle count v=4, 60s timelimit (no iter cap).

Writes:
  scripts/bench5runs_raw.tsv     — every (instance, solver, seed, reward) row
  scripts/bench5runs_summary.tsv — mean / best / std across seeds per solver

Usage (from repo root):
    python scripts/bench5runs.py
"""

import subprocess, sys, statistics
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXE  = REPO / "build" / "examples" / "test_metaheuristics_5s.exe"

SEEDS     = [42, 43, 44, 45, 46]
INSTANCES = [f"pr{i:02d}" for i in range(1, 21)]   # pr01..pr20
VEHICLE   = "4"     # argv[6]: run only v=4
TIME_LIMIT  = "60"
MAX_ITERS   = "0"   # lift iteration cap — time-bound only

# results[solver][inst] -> list of rewards across seeds
results: dict = defaultdict(lambda: defaultdict(list))
raw_rows: list[str] = []

def run_one(inst: str, seed: int) -> list[dict]:
    """Run the binary for one (instance, seed); return parsed rows."""
    cmd = [
        str(EXE),
        TIME_LIMIT,
        MAX_ITERS,
        inst,       # argv[3]: instance filter
        "",         # argv[4]: solver filter (empty = all)
        str(seed),  # argv[5]: seed
        VEHICLE,    # argv[6]: fixed vehicle count
    ]
    proc = subprocess.run(
        cmd, capture_output=True, text=True, cwd=str(REPO)
    )
    rows = []
    for line in proc.stdout.splitlines():
        parts = line.split()
        if len(parts) < 6 or not parts[2].isdigit():
            continue
        try:
            reward = float(parts[3])
        except ValueError:
            continue
        feas = parts[6] if len(parts) > 6 else "?"
        rows.append({
            "inst":   parts[0].replace(".txt", ""),
            "solver": parts[1],
            "v":      int(parts[2]),
            "reward": reward,
            "feas":   feas,
        })
    if not rows:
        print(f"    [WARN] no rows (exit={proc.returncode})", flush=True)
        if proc.stderr:
            print(proc.stderr[:300], flush=True)
    return rows


script_dir = Path(__file__).parent
script_dir.mkdir(exist_ok=True)
raw_path  = script_dir / "bench5runs_raw.tsv"
summ_path = script_dir / "bench5runs_summary.tsv"

total = len(SEEDS) * len(INSTANCES)
done  = 0

with open(raw_path, "w") as raw_f:
    raw_f.write("Instance\tSolver\tVehicles\tSeed\tReward\tFeasible\n")

    for seed in SEEDS:
        for inst in INSTANCES:
            done += 1
            pct = 100 * done / total
            print(f"[{done:3d}/{total}  {pct:5.1f}%]  inst={inst}  seed={seed}", flush=True)

            rows = run_one(inst, seed)
            for r in rows:
                results[r["solver"]][r["inst"]].append(r["reward"])
                raw_f.write(
                    f"{r['inst']}\t{r['solver']}\t{r['v']}\t"
                    f"{seed}\t{r['reward']:.2f}\t{r['feas']}\n"
                )
                raw_f.flush()

print(f"\nRaw data written to {raw_path}", flush=True)

# --- Build summary ---
solvers   = sorted(results.keys())
all_insts = INSTANCES

with open(summ_path, "w") as sf:
    sf.write("Instance\tSolver\tMean\tBest\tStd\tN\n")
    for inst in all_insts:
        for solver in solvers:
            vals = results[solver][inst]
            if not vals:
                continue
            mean = statistics.mean(vals)
            best = max(vals)
            std  = statistics.stdev(vals) if len(vals) > 1 else 0.0
            sf.write(f"{inst}\t{solver}\t{mean:.2f}\t{best:.2f}\t{std:.2f}\t{len(vals)}\n")

print(f"Summary written to {summ_path}", flush=True)

# --- Print table grouped by solver ---
print()
print("=" * 90)
print(f"BENCHMARK: pr01-pr20 | v=4 | 60s time limit | 5 seeds (42-46)")
print("=" * 90)

# Header
col_w = 8
hdr_parts = ["Solver         "] + [f"{inst:>{col_w}}" for inst in all_insts] + ["    Mean"]
print("".join(hdr_parts))
print("-" * (16 + col_w * len(all_insts) + 8))

for solver in solvers:
    cells = []
    all_means = []
    for inst in all_insts:
        vals = results[solver][inst]
        if vals:
            m = statistics.mean(vals)
            all_means.append(m)
            cells.append(f"{m:>{col_w}.1f}")
        else:
            cells.append(f"{'—':>{col_w}}")
    grand = f"{statistics.mean(all_means):8.1f}" if all_means else "       —"
    print(f"{solver:<16}" + "".join(cells) + grand)

print()
print("Values = mean reward over 5 seeds. Grand mean across pr01-pr20.")
