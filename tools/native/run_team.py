#!/usr/bin/env python3
"""Launch three native soccer players; optionally verify attacker-loss acceptance."""
import argparse
import json
import os
import pathlib
import re
import shutil
import signal
import subprocess
import tempfile
import time

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
STATE = re.compile(
    r"TEAM_STATE id=(\d+) t=([\d.]+) role=(\d+) pos=\(([-\d.e+]+),([-\d.e+]+)\) target=\(([-\d.e+]+),([-\d.e+]+)\) fallen=(\w+)"
)
RX = re.compile(
    r"TEAM_RX self=(\d+) peer=(\d+) role=(\d+) active=(\w+) intention=(\w+) target=\(([-\d.e+]+),([-\d.e+]+)\) walk=\(([-\d.e+]+),([-\d.e+]+),([-\d.e+]+)\) t=([\d.]+)"
)


def clean(path):
    return re.sub(r"\x1b\[[0-9;]*m", "", path.read_text())


def configure(source, target, updates):
    data = yaml.safe_load(source.read_text())
    data.update(updates)
    target.write_text(yaml.safe_dump(data, sort_keys=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument(
        "--verify", action="store_true", help="Stop the initial attacker after 20 seconds and check takeover"
    )
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("/tmp/nusim-native-team"))
    parser.add_argument("--domain-base", type=int, default=48)
    parser.add_argument("--port-base", type=int, default=24120)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.domain_base <= 58 or not 1024 <= args.port_base <= 65532:
        parser.error("DDS base must be 0–58 and port base 1024–65532")
    if args.verify and args.duration < 30:
        parser.error("Verification requires at least 30 seconds")
    sim = ROOT.parent / "NUSim"
    args.output.mkdir(parents=True, exist_ok=True)
    processes, handles = [], []
    stopped = None
    stop_time = None
    with tempfile.TemporaryDirectory(prefix="nusim-team-") as temporary:
        work = pathlib.Path(temporary)
        sim_config = work / "sim-config"
        shutil.copytree(sim / "mujoco/config", sim_config)
        roster = yaml.safe_load((sim_config / "match_3v3.yaml").read_text())
        for robot in roster["robots"]:
            robot["dds_domain"] = args.domain_base + robot["robot_id"] - 1
        (sim_config / "match_3v3.yaml").write_text(yaml.safe_dump(roster))
        try:
            handle = open(args.output / "sim.log", "w")
            handles.append(handle)
            simulator = subprocess.Popen(
                [
                    str(sim / "mujoco/build-native/k1_mujoco_sim"),
                    "--game",
                    "3",
                    "--on-field-positions",
                    "--match",
                    "match_3v3.yaml",
                    "--config-dir",
                    str(sim_config),
                ]
                + ([] if args.viewer else ["--headless"]),
                stdout=handle,
                stderr=subprocess.STDOUT,
            )
            processes.append(simulator)
            time.sleep(2)
            players = {}
            for player in range(1, 4):
                directory = work / f"player{player}"
                directory.mkdir()
                shutil.copytree(ROOT / "build-player/config", directory / "config")
                for model in (ROOT / "build-player").glob("*.onnx"):
                    (directory / model.name).symlink_to(model)
                config = directory / "config"
                configure(
                    config / "RobotCommunication.yaml",
                    config / "RobotCommunication.yaml",
                    {
                        "startup_delay": 1,
                        "local_player_ports": {i: args.port_base + i for i in range(1, 4)},
                        "udp_filter_address": "127.0.0.1",
                    },
                )
                configure(config / "FieldDescription.yaml", config / "FieldDescription.yaml", {"field_type": "l3"})
                configure(
                    config / "PlanWalkPath.yaml",
                    config / "PlanWalkPath.yaml",
                    {
                        "max_velocity": [0.25, 0.12, 0.5],
                        "min_velocity": [0.07, 0.07, 0.15],
                        "zero_tolerance": [0.02, 0.02, 0.03],
                        "adjust_vx_limit": 0.25,
                        "adjust_vy_limit": 0.12,
                        "adjust_vtheta_limit": 0.5,
                    },
                )
                env = dict(
                    os.environ,
                    NUSIM_TEAM_PLAYER="1",
                    NUSIM_ROBOT_ID=str(player),
                    NUSIM_DDS_DOMAIN=str(args.domain_base + player - 1),
                )
                handle = open(args.output / f"player{player}.log", "w")
                handles.append(handle)
                players[player] = subprocess.Popen(
                    [str(ROOT / "build-player/bin/team")],
                    cwd=directory,
                    env=env,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                )
                processes.append(players[player])
            start = time.monotonic()
            while time.monotonic() - start < args.duration:
                time.sleep(1)
                for process in processes:
                    if process.poll() is not None and process is not players.get(stopped):
                        raise RuntimeError(f"Process exited unexpectedly: {process.returncode}")
                if args.verify and stopped is None and time.monotonic() - start >= 20:
                    states = {i: list(STATE.finditer(clean(args.output / f"player{i}.log"))) for i in players}
                    if not all(states.values()):
                        raise RuntimeError("Missing player diagnostics")
                    attackers = [i for i, samples in states.items() if int(samples[-1][3]) == 2]
                    if len(attackers) != 1:
                        raise RuntimeError(f"Expected one initial attacker; found {attackers}")
                    stopped = attackers[0]
                    stop_time = float(states[stopped][-1][2])
                    players[stopped].send_signal(signal.SIGINT)
                    print(f"Stopped attacker {stopped} at simulator time {stop_time:.3f}", flush=True)
        finally:
            for process in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for handle in handles:
                handle.close()
    if args.verify:
        result = {"stopped_attacker": stopped, "stop_sim_time": stop_time, "players": {}}
        takeover = []
        all_states = {
            i: [m.groups() for m in STATE.finditer(clean(args.output / f"player{i}.log"))] for i in range(1, 4)
        }
        baseline = {i: int([s for s in all_states[i] if float(s[1]) <= stop_time][-1][2]) for i in range(1, 4)}
        if sorted(baseline.values()) != [2, 3, 4]:
            raise RuntimeError(f"Team did not settle into attacker/defender/supporter: {baseline}")
        result["initial_roles"] = baseline
        for i in range(1, 4):
            text = clean(args.output / f"player{i}.log")
            states = [m.groups() for m in STATE.finditer(text)]
            packets = [m.groups() for m in RX.finditer(text)]
            before = [p for p in packets if float(p[10]) <= stop_time]
            peers = sorted({int(p[1]) for p in before})
            if peers != sorted(set(range(1, 4)) - {i}):
                raise RuntimeError(f"Player {i} did not receive both teammates: {peers}")
            if not any(p[4] == "true" for p in before) or len({p[2] for p in before}) < 2:
                raise RuntimeError(f"Player {i} missed teammate roles and intention messages")
            support = [s for s in states if int(s[2]) == 4 and float(s[1]) < stop_time]
            support_motion = 0.0
            if len(support) >= 2:
                x, y = map(float, support[0][3:5])
                support_motion = max(((float(s[3]) - x) ** 2 + (float(s[4]) - y) ** 2) ** 0.5 for s in support)
                if support_motion < 0.15:
                    raise RuntimeError(f"Supporter {i} did not reposition: {support_motion}")
            after = [s for s in states if float(s[1]) > stop_time + 2 and int(s[2]) == 2]
            if i != stopped:
                activity = re.findall(rf"TEAM_PEER self={i} peer={stopped} active=(\w+)", text)
                if len(activity) < 2 or activity[-1] != "false" or "true" not in activity:
                    raise RuntimeError(f"Player {i} did not expire the missing attacker")
                if after:
                    takeover.append(i)
            result["players"][i] = {
                "received_from": peers,
                "packets": len(packets),
                "roles": sorted({int(s[2]) for s in states}),
                "support_displacement_m": support_motion,
            }
        if not takeover or not any(p["support_displacement_m"] >= 0.15 for p in result["players"].values()):
            raise RuntimeError("No supporter movement or attacker takeover")
        for replacement in takeover:
            for receiver in set(range(1, 4)) - {stopped, replacement}:
                packets = [m.groups() for m in RX.finditer(clean(args.output / f"player{receiver}.log"))]
                if not any(
                    int(p[1]) == replacement
                    and p[2] == "2"
                    and p[3] == "true"
                    and p[4] == "true"
                    and float(p[10]) > stop_time + 2
                    for p in packets
                ):
                    raise RuntimeError("Replacement role/intention was not received by the remaining teammate")
        result["replacement_attackers"] = takeover
        (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
    print(args.output)


if __name__ == "__main__":
    main()
