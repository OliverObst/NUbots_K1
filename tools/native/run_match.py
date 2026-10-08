#!/usr/bin/env python3
"""Launch a repeatable native 3v3 match with local GameController fan-out and scenarios."""
import argparse
import json
import math
import os
import pathlib
import platform
import signal
import socket
import statistics
import struct
import subprocess
import time

import yaml

from run_team import ROOT, RX, STATE, clean, configure

MATCH = __import__("re").compile(
    r"MATCH_STATE robot=(\d+) team=(\d+) player=(\d+) t=([\d.]+) generation=(\d+) "
    r"pos=\(([-\d.e+]+),([-\d.e+]+)\) ball=\(([-\d.e+]+),([-\d.e+]+)\) "
    r"fallen=(\w+) ready=(\w+) gc_live=(\w+)"
)


def packet(number, phase, penalties, kicking_team=125):
    """Official GameController v20 receive layout (158 bytes, little endian)."""
    header = struct.pack("<4s10Bhh", b"RGme", 20, number % 256, 3, 0, 0, 0, phase, 0, 1, kicking_team, 600, 0)
    teams = []
    for team_id in (125, 126):
        team = struct.pack("<6BHH", team_id, team_id - 125, team_id - 125, 0, 0, 0, 0, 1200)
        team += b"".join(
            struct.pack("3B", penalties.get((team_id, i), 0), 30 if penalties.get((team_id, i)) else 0, 0)
            for i in range(1, 21)
        )
        teams.append(team)
    return header + b"".join(teams)


def cpu_seconds(text):
    days, _, clock = text.rpartition("-")
    pieces = list(map(float, (clock or text).split(":")))
    return (int(days) * 86400 if days else 0) + sum(value * 60**i for i, value in enumerate(reversed(pieces)))


def resource_sample(processes, previous, now, encoder_pid=None):
    """ps cumulative CPU time and resident memory, no optional Python packages required."""
    live = {p.pid: name for name, p in processes.items() if p.poll() is None}
    if encoder_pid:
        live[encoder_pid] = "encoder"
    if not live:
        return {}
    output = subprocess.check_output(["ps", "-p", ",".join(map(str, live)), "-o", "pid=,time=,rss="], text=True)
    result = {}
    for row in output.splitlines():
        pid, cpu, rss = row.split()
        pid, cpu = int(pid), cpu_seconds(cpu)
        prior = previous.get(pid)
        result[live[pid]] = {
            "pid": pid,
            "cpu_seconds": cpu,
            "rss_mib": int(rss) / 1024,
            "cpu_percent": max(0, (cpu - prior[0]) / (now - prior[1]) * 100) if prior else None,
        }
        previous[pid] = (cpu, now)
    return result


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def latest(output):
    result = {}
    for i in range(1, 7):
        matches = list(MATCH.finditer(clean(output / f"player{i}.log")))
        if matches:
            m = matches[-1]
            result[i] = dict(
                robot=int(m[1]),
                team=int(m[2]),
                player=int(m[3]),
                t=float(m[4]),
                generation=int(m[5]),
                pos=[float(m[6]), float(m[7])],
                ball=[float(m[8]), float(m[9])],
                fallen=m[10] == "true",
                ready=m[11] == "true",
                gc_live=m[12] == "true",
            )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=70)
    parser.add_argument(
        "--verify", action="store_true", help="Run kickoff, six penalties, relocation, fall, reconnections and reset"
    )
    parser.add_argument("--output", type=pathlib.Path, default=pathlib.Path("/tmp/nusim-native-match"))
    parser.add_argument("--domain-base", type=int, default=48)
    parser.add_argument(
        "--port-base", type=int, default=24200, help="Reserve 6 team, 7 GameController and 1 scenario ports"
    )
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument(
        "--video", action="store_true", help="Record simulator-only MP4 at 1280x720, 15 fps (requires ffmpeg)"
    )
    parser.add_argument("--target-rtf", type=float, default=0.95)
    args = parser.parse_args()
    if args.video:
        args.viewer = True

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    if not 0 <= args.domain_base <= 58 or not 1024 <= args.port_base <= 65515:
        parser.error("DDS base must be 0–58 and port base 1024–65515")
    if args.duration < 5 or not math.isfinite(args.duration):
        parser.error("Duration must be finite and at least five seconds")
    if args.verify and args.duration < 65:
        parser.error("Scenario verification requires at least 65 seconds")
    if args.output.exists():
        parser.error("Use a new output directory to preserve earlier run evidence")
    args.output.mkdir(parents=True)
    work = args.output / "configs"
    work.mkdir()
    sim = ROOT.parent / "NUSim"
    import shutil

    sim_config = work / "simulator"
    shutil.copytree(sim / "mujoco/config", sim_config)
    roster = yaml.safe_load((sim_config / "match_3v3.yaml").read_text())
    placements = []
    spawn = [(-2.0, 0.0), (-6.5, 0.0), (-3.0, 2.5)]
    for robot in roster["robots"]:
        i, player = robot["robot_id"], robot["player_id"]
        robot["dds_domain"] = args.domain_base + i - 1
        sign = 1 if robot["team_id"] == 125 else -1
        x, y = spawn[player - 1]
        pose = dict(x=sign * x, y=sign * y, z=0.555, yaw=0 if sign == 1 else math.pi)
        placements.append(
            dict(
                robot_id=i,
                body="Trunk" if i == 1 else f"sub{i-1:02d}_Trunk",
                team_id=robot["team_id"],
                player_id=player,
                home_pose=pose,
                penalty_pose=dict(x=sign * (-4 + player), y=sign * 5.0, z=0.555, yaw=pose["yaw"]),
            )
        )
    (sim_config / "match_3v3.yaml").write_text(yaml.safe_dump(roster))
    configure(
        sim_config / "supervisor.yaml",
        sim_config / "supervisor.yaml",
        dict(enabled=True, gc_port=args.port_base + 10, robots=placements),
    )
    configure(sim_config / "simulation.yaml", sim_config / "simulation.yaml", dict(scenario_port=args.port_base + 20))
    configure(sim_config / "locomotion.yaml", sim_config / "locomotion.yaml", dict(low_cmd_timeout=0.25))
    directories, environments = {}, {}
    for robot in roster["robots"]:
        i = robot["robot_id"]
        directory = work / f"player{i}"
        directory.mkdir()
        shutil.copytree(ROOT / "build-player/config", directory / "config")
        for model in (ROOT / "build-player").glob("*.onnx"):
            (directory / model.name).symlink_to(model)
        config = directory / "config"
        offset = 0 if robot["team_id"] == 125 else 3
        configure(
            config / "RobotCommunication.yaml",
            config / "RobotCommunication.yaml",
            dict(
                startup_delay=1,
                local_player_ports={p: args.port_base + offset + p for p in range(1, 4)},
                udp_filter_address="127.0.0.1",
            ),
        )
        configure(config / "FieldDescription.yaml", config / "FieldDescription.yaml", dict(field_type="l3"))
        configure(
            config / "PlanWalkPath.yaml",
            config / "PlanWalkPath.yaml",
            dict(
                max_velocity=[0.25, 0.12, 0.5],
                min_velocity=[0.07, 0.07, 0.15],
                zero_tolerance=[0.02, 0.02, 0.03],
                adjust_vx_limit=0.25,
                adjust_vy_limit=0.12,
                adjust_vtheta_limit=0.5,
            ),
        )
        environment = dict(
            NUSIM_TEAM_PLAYER="1",
            NUSIM_ROBOT_ID=str(i),
            NUSIM_DDS_DOMAIN=str(robot["dds_domain"]),
            NUSIM_GC_PORT=str(args.port_base + 10 + i),
        )
        (directory / "environment.json").write_text(json.dumps(environment, indent=2) + "\n")
        directories[i], environments[i] = directory, environment
    metadata = dict(
        arguments=vars(args) | {"output": str(args.output)},
        host=platform.platform(),
        simulator_commit=subprocess.check_output(["git", "-C", str(sim), "rev-parse", "HEAD"], text=True).strip(),
        player_commit=subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
    )
    for label, repo in [("simulator", sim), ("players", ROOT)]:
        (args.output / f"{label}.patch").write_bytes(
            subprocess.check_output(["git", "-C", str(repo), "diff", "HEAD", "--", ".", ":!**/__pycache__/**"])
        )
        untracked = subprocess.check_output(
            ["git", "-C", str(repo), "ls-files", "--others", "--exclude-standard"], text=True
        )
        for name in untracked.splitlines():
            source = repo / name
            if source.suffix in (".py", ".cpp", ".hpp", ".proto", ".role", ".md"):
                destination = args.output / "source" / label / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
    import hashlib

    metadata["models_sha256"] = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT / "build-player").glob("*.onnx")
    }
    (args.output / "session.json").write_text(json.dumps(metadata, indent=2) + "\n")
    handles, processes, previous, events = [], {}, {}, []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    encoder_pid = None
    phase, penalties, gc_live, sequence = (0 if args.verify else 3), {}, True, 0

    def launch_player(i):
        handle = open(args.output / f"player{i}.log", "a")
        handles.append(handle)
        process = subprocess.Popen(
            [str(ROOT / "build-player/bin/team")],
            cwd=directories[i],
            env=dict(os.environ, **environments[i]),
            stdout=handle,
            stderr=subprocess.STDOUT,
        )
        processes[f"player{i}"] = process

    def scenario(action, **values):
        sock.sendto(json.dumps(dict(action=action, **values)).encode(), ("127.0.0.1", args.port_base + 20))

    def event(name, elapsed):
        record = dict(name=name, wall_seconds=elapsed, states=latest(args.output))
        events.append(record)
        (args.output / "events.json").write_text(json.dumps(events, indent=2) + "\n")
        print(f"{elapsed:.1f}s: {name}", flush=True)

    timeline = [(3, "ready"), (6, "set"), (8, "kickoff"), (9.5, "kickoff_observed")]
    for i in range(1, 7):
        timeline.extend([(10 + 2 * (i - 1), f"penalise{i}"), (11 + 2 * (i - 1), f"unpenalise{i}")])
    timeline.extend(
        [
            (24, "ball_relocation"),
            (27, "ball_observed"),
            (30, "fall"),
            (33, "fall_observed"),
            (36, "player_disconnect"),
            (39, "player_expired"),
            (40, "player_reconnect"),
            (44, "gc_disconnect"),
            (47, "gc_expired"),
            (48, "gc_reconnect"),
            (53, "before_reset"),
            (54, "match_reset"),
            (57, "reset_ready"),
            (59, "reset_set"),
            (61, "reset_kickoff"),
            (64, "reset_observed"),
        ]
    )
    try:
        handle = open(args.output / "sim.log", "w")
        handles.append(handle)
        processes["simulator"] = subprocess.Popen(
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
        time.sleep(2)
        for i in range(1, 7):
            launch_player(i)
        if args.video:
            if not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
                raise RuntimeError("--video requires ffmpeg and ffprobe on PATH")
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                sock.sendto(packet(sequence, phase, penalties), ("127.0.0.1", args.port_base + 10))
                for i in range(1, 7):
                    sock.sendto(packet(sequence, phase, penalties), ("127.0.0.1", args.port_base + 10 + i))
                sequence += 1
                states = latest(args.output)
                if len(states) == 6 and all(s["ready"] and s["gc_live"] for s in states.values()):
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("All six players must be ready before video starts")
            scenario("record", path=str((args.output / "demo.mp4").resolve()), duration=args.duration)
        start, next_resource = time.monotonic(), 0.0
        with open(args.output / "resources.jsonl", "w") as resources:
            while (elapsed := time.monotonic() - start) < args.duration:
                for name, process in processes.items():
                    if process.poll() is not None and name != "disconnected":
                        raise RuntimeError(f"{name} exited unexpectedly: {process.returncode}")
                if args.verify:
                    while timeline and elapsed >= timeline[0][0]:
                        _, name = timeline.pop(0)
                        if name in ("ready", "reset_ready"):
                            phase = 1
                        elif name in ("set", "reset_set"):
                            phase = 2
                        elif name in ("kickoff", "reset_kickoff"):
                            phase = 3
                        elif name.startswith("penalise") or name.startswith("unpenalise"):
                            i = int(name[-1])
                            player = (i - 1) % 3 + 1
                            penalties[(125 if i <= 3 else 126, player)] = 5 if name.startswith("penalise") else 0
                        elif name == "ball_relocation":
                            scenario("ball", x=0.0, y=-2.0)
                        elif name == "fall":
                            scenario("fall", robot_id=3)
                        elif name == "player_disconnect":
                            process = processes.pop("player1")
                            stop(process)
                            processes["disconnected"] = process
                        elif name == "player_reconnect":
                            launch_player(1)
                        elif name == "gc_disconnect":
                            gc_live = False
                        elif name == "gc_reconnect":
                            gc_live = True
                        elif name == "match_reset":
                            phase = 0
                            penalties.clear()
                        event(name, elapsed)
                if gc_live:
                    wire = packet(sequence, phase, penalties)
                    for port in range(args.port_base + 10, args.port_base + 17):
                        sock.sendto(wire, ("127.0.0.1", port))
                    sequence += 1
                if elapsed >= next_resource:
                    if args.video and encoder_pid is None:
                        match = __import__("re").search(
                            r"VIDEO started .* encoder_pid (\d+)", clean(args.output / "sim.log")
                        )
                        if match:
                            encoder_pid = int(match[1])
                    resources.write(
                        json.dumps(
                            dict(
                                wall_seconds=elapsed,
                                processes=resource_sample(processes, previous, time.monotonic(), encoder_pid),
                            )
                        )
                        + "\n"
                    )
                    resources.flush()
                    next_resource = elapsed + 1
                time.sleep(0.1)
    finally:
        for process in reversed(list(processes.values())):
            stop(process)
        for handle in handles:
            handle.close()
        sock.close()
    if args.video:
        probe = subprocess.check_output(
            [
                "ffprobe",
                "-v",
                "error",
                "-show_entries",
                "format=duration:stream=width,height,nb_frames",
                "-of",
                "json",
                str(args.output / "demo.mp4"),
            ],
            text=True,
        )
        (args.output / "video.json").write_text(probe)
        duration = float(json.loads(probe)["format"]["duration"])
        if abs(duration - args.duration) > 0.1:
            raise RuntimeError(f"Video duration mismatch: {duration}")
    result = analyse(args.output, args.target_rtf, args.verify)
    (args.output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if args.verify and (not all(result["checks"].values()) or not result["performance"]["target_met"]):
        raise SystemExit("Acceptance failed; see results.json for separate functional and performance outcomes")


def analyse(output, target, verify):
    import re

    simlog = clean(output / "sim.log")
    metric = re.findall(r"SIM_METRICS rtf ([\d.e+-]+) physics_step_ms ([\d.e+-]+)", simlog)
    rtfs = [float(m[0]) for m in metric[5:] if float(m[0]) > 0]
    controls = re.findall(r"CONTROL_METRICS robot (\d+) mailbox_to_apply_ms ([\d.e+-]+) samples (\d+)", simlog)
    control = {int(i): dict(mean_ms=float(ms), samples=int(n)) for i, ms, n in controls}
    rows = [json.loads(row) for row in (output / "resources.jsonl").read_text().splitlines()]
    resource = {}
    for name in ("simulator", *(f"player{i}" for i in range(1, 7)), "encoder"):
        if not any(name in r["processes"] for r in rows):
            continue
        samples = [r["processes"][name] for r in rows if name in r["processes"]]
        resource[name] = dict(
            mean_cpu_percent=statistics.mean(s["cpu_percent"] for s in samples if s["cpu_percent"] is not None),
            peak_rss_mib=max(s["rss_mib"] for s in samples),
        )
    aggregate = [sum(s["rss_mib"] for s in row["processes"].values()) for row in rows]
    result = dict(
        performance=dict(
            peak_total_rss_mib=max(aggregate, default=0),
            mean_rtf=statistics.mean(rtfs) if rtfs else 0.0,
            min_rtf=min(rtfs, default=0.0),
            proposed_target=target,
            target_met=bool(rtfs) and min(rtfs) >= target,
            mean_mujoco_step_ms=float(metric[-1][1]) if metric else None,
            control_mailbox_to_apply=control,
            resources=resource,
        ),
        checks={},
        players={},
    )
    for i in range(1, 7):
        text = clean(output / f"player{i}.log")
        packets = [m.groups() for m in RX.finditer(text)]
        peers = sorted({int(p[1]) for p in packets})
        states = [m.groups() for m in MATCH.finditer(text)]
        phases = sorted(set(map(int, re.findall(r"TEAM_GC .*?phase=(\d+)", text))))
        decisions = list(STATE.finditer(text))
        result["players"][i] = dict(
            roles_seen=sorted({int(m[3]) for m in decisions}),
            fallen_sample_fraction=sum(s[9] == "true" for s in states) / len(states) if states else None,
            received_from=peers,
            packets=len(packets),
            phases=phases,
            identity=[int(states[-1][0]), int(states[-1][1]), int(states[-1][2])] if states else None,
        )
        result["checks"][f"robot{i}_identity"] = bool(states) and all(
            int(s[0]) == i and int(s[1]) == (125 if i <= 3 else 126) and int(s[2]) == (i - 1) % 3 + 1 for s in states
        )
        result["checks"][f"robot{i}_team_communication"] = peers == sorted({1, 2, 3} - {(i - 1) % 3 + 1}) and any(
            p[4] == "true" for p in packets
        )
        result["checks"][f"robot{i}_control"] = control.get(i, {}).get("samples", 0) > 100
        if verify:
            result["checks"][f"robot{i}_phases"] = all(p in phases for p in (1, 2, 3, 4))
            penalty = re.findall(r"TEAM_GC .*?penalty=(\d+)", text)
            result["checks"][f"robot{i}_penalty"] = "5" in penalty and penalty[-1] == "0"
            body = "Trunk" if i == 1 else f"sub{i-1:02d}_Trunk"
            result["checks"][f"robot{i}_placement"] = f"{body} penalised" in simlog and f"{body} unpenalised" in simlog
    if verify:
        events = {e["name"]: e for e in json.loads((output / "events.json").read_text())}
        kickoff = events["kickoff_observed"]["states"]
        result["checks"]["kickoff_ball"] = len(kickoff) == 6 and all(
            math.hypot(*s["ball"]) < 0.3 for s in kickoff.values()
        )
        result["checks"]["ball_relocation"] = (
            all(
                abs(s["ball"][0]) < 0.3 and abs(s["ball"][1] + 2) < 0.3
                for s in events["ball_observed"]["states"].values()
            )
            and len(events["ball_observed"]["states"]) == 6
        )
        fall_begin = events["fall"]["states"]["3"]["t"]
        fall_end = events["fall_observed"]["states"]["3"]["t"]
        result["checks"]["fallen_player"] = any(
            float(m[4]) >= fall_begin and float(m[4]) <= fall_end and m[10] == "true"
            for m in MATCH.finditer(clean(output / "player3.log"))
        )
        result["checks"]["player_disconnect_reconnect"] = all(
            re.search(
                r"TEAM_PEER self="
                + str(i)
                + r" peer=1 active=true.*TEAM_PEER self="
                + str(i)
                + r" peer=1 active=false.*TEAM_PEER self="
                + str(i)
                + r" peer=1 active=true",
                clean(output / f"player{i}.log"),
                re.S,
            )
            is not None
            for i in (2, 3)
        ) and events["reset_observed"]["states"].get("1", {}).get("ready", False)
        result["checks"]["gc_disconnect_reconnect"] = all(
            "gc_live=false" in clean(output / f"player{i}.log")
            and "Brief stop called: standing still" in clean(output / f"player{i}.log")
            and "Brief stop cleared: resuming play" in clean(output / f"player{i}.log")
            and events["reset_observed"]["states"].get(str(i), {}).get("gc_live", False)
            for i in range(1, 7)
        )
        before, after = events["before_reset"]["states"], events["reset_observed"]["states"]
        result["checks"]["match_reset"] = len(after) == 6 and all(
            after[str(i)]["generation"] > before[str(i)]["generation"] and after[str(i)]["t"] < before[str(i)]["t"]
            for i in range(1, 7)
        )
    return result


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Session interrupted; owned processes have stopped", flush=True)
        raise SystemExit(130)
