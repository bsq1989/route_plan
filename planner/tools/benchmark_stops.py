"""Release regression: existing length plan vs incremental pickup-stop search."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import random

from route_plan_sim import Scene
from route_plan_planner import LengthPlanner, PlannerOptions, StopsPlanner
from route_plan_planner.motion import replay
from route_plan_planner.records import describe_scene, restore_scene, write_json
from route_plan_planner.stops import at_stop_scene, materialize_stops

SEEDS = (41, 57, 28, 38, 49, 70, 1, 97, 85, 54)


def clustered(seed):
    scene = Scene()
    scene.targets.clear()
    rng = random.Random(seed)
    centers = []
    while len(centers) < 10:
        center = (rng.uniform(1.2, 10.8), rng.uniform(1.2, 7.8))
        if math.dist(center, scene.pose[:2]) < 1.4 or any(math.dist(center, other) < 1.35 for other in centers):
            continue
        centers.append(center)
    scene.add_targets((cx + dx, cy + dy) for cx, cy in centers
                      for dx, dy in ((-.07, -.07), (-.07, .07), (.07, -.07), (.07, .07)))
    assert len(scene.targets) == 40 and all(len(group) == 1 for group in scene.target_groups)
    scene.seed = seed
    return scene


def benchmark(output, render=False):
    output.mkdir(parents=True, exist_ok=True)
    rows = []
    cases = [(f"random-{seed}", Scene(scenario="random", seed=seed)) for seed in SEEDS]
    cases += [(f"cluster-{seed}", clustered(seed)) for seed in range(10)]
    for name, contact in cases:
        seed = LengthPlanner(PlannerOptions(timeout=1., ompl=True)).plan(contact)
        if seed.status != "success":
            raise ValueError(f"Length seed failed: {name}: {seed.reason}")
        scene = at_stop_scene(contact)
        a = materialize_stops(scene, seed)
        b = StopsPlanner(PlannerOptions(timeout=3., ompl=True)).plan(scene, initial=seed)
        if b.status != "success":
            raise ValueError(f"Stop search failed: {name}: {b.reason}")
        for result in (a, b):
            for dt in (.013, .7):
                final = replay(scene, result.goals, dt=dt, pickup_stops=result.pickup_stops)
                if final.failure or final.targets or set(final.collected) != set(scene.targets):
                    raise ValueError(f"Replay failed: {name}, dt={dt}")
        assert (len(b.pickup_stops), round(b.length, 7)) <= (len(a.pickup_stops), round(a.length, 7))
        for label, result in (("A", a), ("B", b)):
            write_json(output / f"{label}-{name}.json", {"schema_version": 1, "initial": describe_scene(scene),
                                                        "result": result.to_dict()})
        row = dict(case=name, type="cluster" if name.startswith("cluster") else "random", seed=contact.seed,
                   targets=len(scene.targets), a_stops=len(a.pickup_stops), b_stops=len(b.pickup_stops),
                   a_length=a.length, b_length=b.length, saved_stops=len(a.pickup_stops) - len(b.pickup_stops),
                   a_elapsed=seed.elapsed, b_elapsed=b.elapsed, success=True)
        rows.append(row)
        print(f"{name}: stops {row['a_stops']} -> {row['b_stops']}; "
              f"length {a.length:.2f} -> {b.length:.2f}m; two replay rates passed", flush=True)
    summary = dict(schema_version=1, sdk_version="0.2.0", rule_version=2, collection_mode="at_stop",
                   a="length mode; exact contacts materialized as individual pickup stops",
                   b="stops mode; same verified A seed; stop count first, length second",
                   budget="A: 1s; B: 3s total including A elapsed; independent replay may add time",
                   replay_dt=[.013, .7], total=len(rows), success=len(rows), targets=sum(row["targets"] for row in rows),
                   a_stops=sum(row["a_stops"] for row in rows), b_stops=sum(row["b_stops"] for row in rows),
                   a_length=sum(row["a_length"] for row in rows), b_length=sum(row["b_length"] for row in rows),
                   cases=rows)
    summary["saved_stop_fraction"] = 1 - summary["b_stops"] / summary["a_stops"]
    summary["stop_improved_cases"] = sum(row["saved_stops"] > 0 for row in rows)
    for category in ("random", "cluster"):
        subset = [row for row in rows if row["type"] == category]
        summary[category] = {key: sum(row[key] for row in subset)
                             for key in ("a_stops", "b_stops", "a_length", "b_length")}
    write_json(output / "summary.json", summary)
    if render:
        render_results(output, summary)
    return summary


def render_results(output, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from route_plan_sim.app import configure_chinese_font
    from route_plan_planner.visualize import save_animation
    configure_chinese_font()
    rows = summary["cases"]
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), dpi=130)
    for ax, a_key, b_key, title in ((axes[0], "a_stops", "b_stops", "拾取停靠次数（首要优化目标）"),
                                  (axes[1], "a_length", "b_length", "行驶距离 / m（停靠次数相同时继续优化）")):
        positions = list(range(len(rows)))
        ax.bar([i - .2 for i in positions], [row[a_key] for row in rows], width=.4, color="#6086b9", label="A 原长度模式")
        ax.bar([i + .2 for i in positions], [row[b_key] for row in rows], width=.4, color="#24a582", label="B 停靠优化")
        ax.set_xticks(positions, [row["case"].replace("random-", "R").replace("cluster-", "C") for row in rows], rotation=45)
        ax.set_title(title, fontsize=12)
        ax.axvline(9.5, color="#939ba8", linestyle="--", alpha=.5)
        ax.grid(axis="y", alpha=.15)
        ax.legend(fontsize=9)
    fig.suptitle(f"停靠优化 A/B · 20/20 场景全部清除且无碰撞 · 停靠减少 {summary['saved_stop_fraction']:.1%}", fontsize=15)
    fig.text(.5, .02, "R：随机分布，C：10 簇 × 4 个非重叠目标。B 复用 A 初解并继续搜索，3 秒总预算包含 A 的 1 秒。", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .05, 1, .95))
    fig.savefig(output / "stops-ab.png")
    plt.close(fig)
    selected = max((row for row in rows if row["type"] == "cluster"), key=lambda row: row["saved_stops"])
    name = selected["case"]
    records = [json.loads((output / f"{label}-{name}.json").read_text()) for label in ("A", "B")]
    scene = restore_scene(records[0]["initial"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), dpi=130)
    for ax, record, label, color in zip(axes, records, ("A 原长度模式", "B 停靠优化"), ("#6086b9", "#159477")):
        result = record["result"]
        final = replay(scene, result["goals"], pickup_stops=result["pickup_stops"])
        ax.set(xlim=(0, scene.width), ylim=(0, scene.height), aspect="equal", xlabel="x / m", ylabel="y / m")
        ax.grid(alpha=.15)
        for point in scene.targets.values():
            ax.add_patch(Circle(point, scene.target_radius, facecolor="#efa638", edgecolor="#a77329", zorder=4))
        ax.plot(*zip(*final.path), color=color, linewidth=1.6)
        ax.scatter(*scene.pose[:2], color=color, s=35)
        for stop in result["pickup_stops"]:
            probe = scene.clone()
            probe.pose = tuple(stop["pose"])
            ax.add_patch(Polygon(probe.vertices(probe.collector), facecolor=color, alpha=.18, edgecolor=color))
            ax.scatter(*stop["pose"][:2], color=color, s=11, marker="s")
        ax.set_title(f"{label} · {len(result['pickup_stops'])} 次停靠\n{result['length']:.2f} m · 40/40 清除", fontsize=13)
    fig.suptitle(f"{name} · 方框为真正的拾取停靠位，色块为该位置的消除区", fontsize=14)
    fig.tight_layout(rect=(0, .02, 1, .92))
    fig.savefig(output / "cluster-comparison.png")
    plt.close(fig)
    best = records[1]["result"]
    final = save_animation(scene, best["goals"], output / "cluster-pickup.gif", dt=.16, fps=14,
                           pickup_stops=best["pickup_stops"])
    assert not final.targets and final.failure is None


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("sim/output/stops-release"))
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    benchmark(args.output, args.render)
