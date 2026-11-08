"""Read a business request JSON; write a machine-readable plan JSON."""

import argparse
import json
from pathlib import Path

from route_plan_sdk import plan


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("request", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    request = json.loads(args.request.read_text(encoding="utf-8"))
    response = plan(request)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(response.to_dict(), ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                           encoding="utf-8")
    print(f"{response.status}; {len(response.goals)} goals; output={args.output}")
    return 0 if response.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
