import csv
import statistics
from collections import defaultdict

# Read the CSV file
results = []
with open('results/toptw_benchmark_260612_211235.csv', 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        results.append(row)

print("=" * 80)
print("TOPTW BENCHMARK SUMMARY - All Solvers on 76 Instances (1-4 Vehicles)")
print("=" * 80)
print()

# Basic statistics
print(f"Total runs completed: {len(results)}")
print(f"Unique instances: {len(set(r['Instance'] for r in results))}")
print(f"Solvers tested: {len(set(r['Solver'] for r in results))}")
print(f"Vehicle configurations: {len(set(r['Vehicles'] for r in results))}")
print()

# Group by solver
solver_stats = defaultdict(list)
for r in results:
    cpu_time = float(r['CPU_ms'])
    reward = float(r['Reward'])
    solver_stats[r['Solver']].append(cpu_time)

print("SOLVER PERFORMANCE (CPU Time in milliseconds):")
print("-" * 80)
print(f"{'Solver':<15} {'Min':<10} {'Avg':<10} {'Max':<10} {'Median':<10}")
print("-" * 80)
for solver in sorted(solver_stats.keys()):
    times = solver_stats[solver]
    print(f"{solver:<15} {min(times):>9.2f} {statistics.mean(times):>9.2f} {max(times):>9.2f} {statistics.median(times):>9.2f}")
print()

# Group by vehicle count
vehicle_stats = defaultdict(list)
for r in results:
    cpu_time = float(r['CPU_ms'])
    vehicles = int(r['Vehicles'])
    vehicle_stats[vehicles].append(cpu_time)

print("PERFORMANCE BY VEHICLE COUNT:")
print("-" * 80)
print(f"{'Vehicles':<12} {'Min':<10} {'Avg':<10} {'Max':<10} {'Median':<10} {'Runs':<8}")
print("-" * 80)
for vehicles in sorted(vehicle_stats.keys()):
    times = vehicle_stats[vehicles]
    print(f"{vehicles:<12} {min(times):>9.2f} {statistics.mean(times):>9.2f} {max(times):>9.2f} {statistics.median(times):>9.2f} {len(times):>7}")
print()

# Rewards analysis
print("REWARD STATISTICS:")
print("-" * 80)
rewards = [float(r['Reward']) for r in results]
print(f"Total zero-reward runs: {sum(1 for r in rewards if r == 0.0)} / {len(rewards)}")
print(f"Non-zero rewards found: {sum(1 for r in rewards if r != 0.0)}")
print(f"Average reward: {statistics.mean(rewards):.4f}")
print()

# Instance size analysis
instance_sizes = {}
for r in results:
    inst = r['Instance']
    nodes = int(r['Nodes'])
    if inst not in instance_sizes:
        instance_sizes[inst] = nodes

print("INSTANCE SIZE DISTRIBUTION:")
print("-" * 80)
size_ranges = [(0, 50), (50, 100), (100, 200), (200, 300), (300, 400)]
for low, high in size_ranges:
    count = sum(1 for n in instance_sizes.values() if low < n <= high)
    print(f"Instances with {low:>3}-{high:<3} nodes: {count:>2}")
print()

# Solver-instance combinations (fastest/slowest)
print("TOP 10 FASTEST RUNS (Greedy excluded):")
print("-" * 80)
non_greedy = [(float(r['CPU_ms']), r['Instance'], r['Solver'], r['Vehicles']) 
              for r in results if r['Solver'] != 'greedy']
for cpu_time, inst, solver, vehicles in sorted(non_greedy)[:10]:
    print(f"{inst:<12} {solver:<15} V={vehicles:<1}  {cpu_time:>8.2f} ms")
print()

print("TOP 10 SLOWEST RUNS:")
print("-" * 80)
for cpu_time, inst, solver, vehicles in sorted(non_greedy, reverse=True)[:10]:
    print(f"{inst:<12} {solver:<15} V={vehicles:<1}  {cpu_time:>8.2f} ms")
print()

# Success rate
errors = sum(1 for r in results if r['Status'] != 'OK')
print(f"All runs completed successfully: {errors == 0} ({len(results)-errors}/{len(results)} OK)")

