#!/usr/bin/env python3
"""Run the real native player and simulator together, preserving diagnostic logs."""
import argparse, json, os, pathlib, re, shutil, signal, subprocess, time, tempfile

parser = argparse.ArgumentParser()
parser.add_argument("--scenario", choices=["one", "two", "recovery"], default="one")
parser.add_argument("--duration", type=int, default=40)
args = parser.parse_args()
root = pathlib.Path(__file__).resolve().parents[2]
sim = root.parent / "NUSim"
logs = pathlib.Path("/tmp/nusim-native-policy") / args.scenario
logs.mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="nusim-player-") as temporary:
    config = pathlib.Path(temporary) / "config"
    shutil.copytree(sim / "mujoco/config", config)
    text = (config / "dds.yaml").read_text().replace("domain: 0", "domain: 45")
    (config / "dds.yaml").write_text(text)
    handles = []
    processes = []
    try:
        log = open(logs / "sim.log", "w")
        handles.append(log)
        processes.append(
            subprocess.Popen(
                [
                    str(sim / "mujoco/build-native/k1_mujoco_sim"),
                    "--headless",
                    "--field",
                    "kidsize",
                    "--robots",
                    "2" if args.scenario == "two" else "1",
                    "--config-dir",
                    str(config),
                ]
                + (["--keyframe", "lying_front"] if args.scenario == "recovery" else []),
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        )
        time.sleep(2)
        for robot in range(1, 3 if args.scenario == "two" else 2):
            log = open(logs / f"player{robot}.log", "w")
            handles.append(log)
            env = dict(os.environ, NUSIM_ROBOT_ID=str(robot), NUSIM_DDS_DOMAIN=str(44 + robot))
            if args.scenario == "two":
                env["NUSIM_TEST_VELOCITY"] = "0.20" if robot == 1 else "-0.20"
            if args.scenario == "recovery":
                env["NUSIM_TEST_VELOCITY"] = "0.0"
            processes.append(
                subprocess.Popen(
                    [str(root / "build-player/bin/player")],
                    cwd=root / "build-player",
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            )
        for _ in range(args.duration):
            time.sleep(1)
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"Process exited: {process.returncode}")
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
print(logs)

pattern = re.compile(
    r"PLAYER (\d+) t ([\d.]+) ball ([\d.e+-]+) ([\d.e+-]+) robot ([\d.e+-]+) ([\d.e+-]+) worldball ([\d.e+-]+) ([\d.e+-]+) kick ([01]) done ([01])"
)
results = {}
for log in logs.glob("player*.log"):
    text = re.sub(r"\x1b\[[0-9;]*m", "", log.read_text())
    samples = [list(map(float, match.groups())) for match in pattern.finditer(text)]
    if len(samples) < 4:
        raise RuntimeError(f"Too few robot samples in {log}")
    first, last = samples[0], samples[-1]
    result = {
        "robot_id": int(first[0]),
        "x_displacement": last[4] - first[4],
        "ball_displacement": ((last[6] - first[6]) ** 2 + (last[7] - first[7]) ** 2) ** 0.5,
    }
    if args.scenario == "one":
        start = re.search(r"Kick started 1 t ([\d.]+) worldball ([\d.e+-]+) ([\d.e+-]+)", text)
        end = re.search(r"Kick completed 1 t ([\d.]+) worldball ([\d.e+-]+) ([\d.e+-]+)", text)
        if not start or not end:
            raise RuntimeError("Kick did not execute and finish")
        start_t, start_x, start_y = map(float, start.groups())
        end_t, end_x, end_y = map(float, end.groups())
        kick = [sample for sample in samples if start_t <= sample[1] <= end_t]
        positions = [(sample[6], sample[7]) for sample in kick] + [(end_x, end_y)]
        result["ball_displacement_during_kick"] = max(
            ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5 for x, y in positions
        )
        if result["x_displacement"] < 0.8 or result["ball_displacement_during_kick"] < 0.03:
            raise RuntimeError(f"Approach/kick acceptance failed: {result}")
    elif args.scenario == "recovery":
        if "Finished getting up (policy)" not in text:
            raise RuntimeError("Get-up did not finish")
    else:
        if (result["x_displacement"] if int(first[0]) == 1 else -result["x_displacement"]) < 0.3:
            raise RuntimeError(f"Independent motion failed: {result}")
    results[log.stem] = result
(logs / "results.json").write_text(json.dumps(results, indent=2) + "\n")
print(json.dumps(results, indent=2))
