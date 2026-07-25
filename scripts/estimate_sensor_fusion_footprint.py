#!/usr/bin/env python3
"""Estimate persistent sensor-fusion state from an ABI-specific size probe."""

import argparse
import json
import re
import sys
from pathlib import Path


PLANNED_TYPES = (
    ("mahony_orientations", "orientation_mahony_imu9", "Mahony orientation estimator"),
    ("mekf_orientations", "orientation_mekf_imu9", "MEKF orientation estimator"),
    ("mahony_relative_pairs", "relative_mahony_imu9_pair", "Mahony relative estimator pair"),
    ("mekf_relative_pairs", "relative_mekf_imu9_pair", "MEKF relative estimator pair"),
)
STACK_USAGE_PATTERN = re.compile(r"^(?P<function>.+?)\s+(?P<bytes>\d+)\s+(?P<kind>static|dynamic|bounded)$")


def non_negative(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def load_probe(path: Path) -> dict[str, object]:
    try:
        contents = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read size probe {path}: {error}") from error

    if contents.get("schema_version") != 1:
        raise ValueError("unsupported size probe schema")
    if not isinstance(contents.get("abi"), dict) or not isinstance(contents.get("types"), dict):
        raise ValueError("size probe must contain abi and types objects")
    return contents


def load_stack_usage(paths: list[Path]) -> list[tuple[int, str, str]]:
    records: list[tuple[int, str, str]] = []
    for path in paths:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            raise ValueError(f"cannot read stack report {path}: {error}") from error

        for line in lines:
            match = STACK_USAGE_PATTERN.match(line)
            if match:
                records.append((int(match.group("bytes")), match.group("kind"), match.group("function")))
    return records


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate persistent sensor-fusion state using a generated MicroLA size probe"
    )
    parser.add_argument("size_probe", type=Path, help="JSON emitted by microla_resource_plan")
    parser.add_argument("--mahony-orientations", type=non_negative, default=0)
    parser.add_argument("--mekf-orientations", type=non_negative, default=0)
    parser.add_argument("--mahony-relative-pairs", type=non_negative, default=0)
    parser.add_argument("--mekf-relative-pairs", type=non_negative, default=0)
    parser.add_argument("--ram-budget-bytes", type=non_negative)
    parser.add_argument(
        "--stack-report",
        type=Path,
        action="append",
        default=[],
        help="GCC .su file emitted by a target resource smoke build; may be repeated",
    )
    args = parser.parse_args()

    try:
        probe = load_probe(args.size_probe)
        stack_records = load_stack_usage(args.stack_report)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    type_info = probe["types"]
    assert isinstance(type_info, dict)
    total_bytes = 0

    print("MicroLA sensor-fusion resource plan")
    print(f"ABI: pointer={probe['abi']['pointer_bytes']} bytes, size_t={probe['abi']['size_t_bytes']} bytes")
    print()
    print(f"{'Component':<34} {'Instances':>10} {'Bytes each':>12} {'Persistent':>12}")
    print("-" * 72)

    for argument_name, type_name, label in PLANNED_TYPES:
        count = getattr(args, argument_name)
        details = type_info.get(type_name)
        if not isinstance(details, dict) or not isinstance(details.get("bytes"), int):
            print(f"ERROR: size probe does not contain {type_name}", file=sys.stderr)
            return 1

        type_bytes = details["bytes"]
        persistent_bytes = count * type_bytes
        total_bytes += persistent_bytes
        print(f"{label:<34} {count:>10} {type_bytes:>12} {persistent_bytes:>12}")

    print("-" * 72)
    print(f"{'Persistent estimator state':<58} {total_bytes:>12} B")
    print(f"{'Persistent estimator state':<58} {total_bytes / 1024:>11.2f} KiB")

    if args.ram_budget_bytes is not None:
        remaining = args.ram_budget_bytes - total_bytes
        print(f"{'Application RAM budget':<58} {args.ram_budget_bytes:>12} B")
        print(f"{'Remaining after estimator state':<58} {remaining:>12} B")
        if remaining < 0:
            print("ERROR: persistent estimator state exceeds the supplied RAM budget", file=sys.stderr)
            return 2

    if stack_records:
        maximum_bytes, stack_kind, function_name = max(stack_records, key=lambda record: record[0])
        print()
        print("Compiler stack report")
        print(f"Maximum individual frame: {maximum_bytes} B ({stack_kind})")
        print(f"Function: {function_name}")
        print("This is not a composed call-path or interrupt-nesting stack bound.")

    print()
    print("Sizes are ABI-specific. Rebuild the probe with the deployment compiler and review linked firmware RAM,")
    print("stack, interrupts, and peripheral allocations separately before setting a production budget.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
