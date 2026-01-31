#!/usr/bin/env python3
"""Compare benchmark JSON outputs (moved from tools/).

Usage: python3 scripts/compare_benchmarks.py --baseline <dir> --current <dir> [--threshold 1.05]
"""
import argparse
import json
import pathlib
import sys


def load_results(path):
    results = {}
    path = pathlib.Path(path)
    for f in path.glob('*.json'):
        with open(f, 'r') as fh:
            data = json.load(fh)
            for bench in data.get('benchmarks', []):
                name = bench.get('name')
                time = bench.get('cpu_time') or bench.get('real_time')
                if name and time is not None:
                    results[name] = time
    return results


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--baseline', required=True)
    p.add_argument('--current', required=True)
    p.add_argument('--threshold', type=float, default=1.05)
    args = p.parse_args()

    base = load_results(args.baseline)
    cur = load_results(args.current)

    regressions = []
    for name, btime in base.items():
        if name not in cur:
            print(f"Warning: {name} missing in current results")
            continue
        ratio = cur[name] / btime
        if ratio > args.threshold:
            regressions.append((name, btime, cur[name], ratio))

    if regressions:
        print("Regressions detected:")
        for r in regressions:
            print(f"- {r[0]}: baseline={r[1]:.6f} current={r[2]:.6f} ratio={r[3]:.3f}")
        sys.exit(2)

    print("No regressions detected.")


if __name__ == '__main__':
    main()
