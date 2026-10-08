# Native motion-policy player for NUSim

The `player` role runs the walk, kick and get-up policies with ONNX Runtime CPU.
Each process selects one robot and DDS domain. The existing Director and `K1Servos`
provider arbitrate the low-level stream: get-up takes priority over kick, then walk.

## Build

Keep this checkout beside the modified NUSim checkout. Build NUSim first (`./b build`)
so its generated DDS types and static `k1sim_idl` library are available. On Apple Silicon,
install Xcode Command Line Tools and Homebrew, then run from `NUbots_K1`:

```bash
brew install cmake ninja boost yaml-cpp uv
bash tools/native/install_deps.sh
bash tools/native/configure.sh
cmake --build build-player --parallel 8
ctest --test-dir build-player --output-on-failure
```

The native profile reuses the saved macOS prototype's portable NUClear, Protobuf, Eigen,
fmt, libuv and ONNX Runtime 1.24.1 dependencies. `NUBOTS_NATIVE_PLAYER` defaults to OFF;
normal robot roles retain their existing build selection. The installer currently targets
Apple Silicon. Configure `-DNUSIM_ROOT=/path/to/NUSim -DNUSIM_BUILD=/path/to/NUSim/mujoco/build-native`
to select another checkout/build. Rebuild both projects after changing the IDL.

Sources and selected checkpoints come from `jmontano/nusim-local` at
`f45cae12498f86b20d08b208eb04705b05278424`:

| Policy | Checkpoint | Observations | Actions |
| --- | --- | --- | --- |
| Walk | `k1_walk_v3_20260812_102M.onnx` | 79 | 22 |
| Kick | `k1_kick_v7_20260728_2504M.onnx` | 80 | 22 |
| Get-up | `k1_getup_v21_20260725_984M.onnx` | 72 | 22 |

Observation assembly, head masking, phase ordering, serial joint order, gains and scales
are retained. Walk and kick actions offset the default pose; get-up actions offset measured
joint positions. Native inference runs on the 50 Hz timer; Director task replacement retains
the previous servo subtask without an extra inference. The kick checkpoint uses a unit
lateral sweep direction, as in its source branch. It predates the later training contract
that uses commanded ball velocity in those observation slots.

## Run

Start NUSim, then launch each player from its build directory:

```bash
# NUSim: six robots, roster domains 0..5
./b run sim/soccer --game 3 --match match_3v3.yaml --on-field-positions

# NUbots_K1: a separate terminal per player
cd build-player
NUSIM_ROBOT_ID=1 NUSIM_DDS_DOMAIN=0 bin/player
# Robot 2: NUSIM_ROBOT_ID=2 NUSIM_DDS_DOMAIN=1 bin/player
```

Robot ID is match-wide; select its domain from the simulator roster. The bridge observes
CUSTOM mode before sending joint commands. `NUSIM_TEST_VELOCITY=0.20` selects a constant
forward command; a negative value walks backwards. Otherwise the minimal behaviour
approaches the ball, settles its stance, performs the side-foot kick, then stands.
Six copies do not yet implement team tactics.

The adapter validates the schema, roster, finite measurements and quaternion norms.
It rejects captures older than 250 ms and samples arriving out of order. Session/reset
changes clear behaviour tasks. Expired truth cancels tasks and stops publishing joint
commands; NUSim then holds the last targets. This is not a damping or emergency-stop command.

Odometry world `{w}` equals simulator world `{s}`. The adapter publishes this robot's
`RawSensors`, mapped `Sensors.servo` entries, `Htw`, yaw-only `Hrw`, `vTw`, and normal
localisation `Field`, `Ball` and `Robots`. `Field.Hfw` rotates by 180 degrees for the opposing
team. Capture time supplies measurement timestamps. Covariance diagonals default to `1e-6`;
a positive `NUSIM_COVARIANCE_FLOOR` overrides them. Missing ball data produces zero confidence.
`Robots.id` uses simulator robot IDs; `Purpose.player_id` carries the team-local player ID. Tactical purpose is not
inferred from physics.

The [three-player team role](NATIVE_TEAM.md) adds existing soccer behaviours and teammate
communication. Full foot/joint kinematics, camera metadata, GameController half changes
and complete competition behaviour coverage remain subsequent work. Camera processing and the saved
macOS vision prototype are not required to run this role.

## Behavioural acceptance

Run the actual simulator and player processes through DDS with temporary configuration
on domains 45 and 46:

```bash
build-deps/venv/bin/python tools/native/try_player.py --scenario one --duration 40
build-deps/venv/bin/python tools/native/try_player.py --scenario two --duration 14
# Optional recovery check
build-deps/venv/bin/python tools/native/try_player.py --scenario recovery --duration 30
```

The first check requires at least 0.8 m of approach, a completed kick and 3 cm of ball
movement while the kick owns the motion path. Later walking contact is excluded from this
metric. The checkpoint produces a modest strike; passing distance and accuracy are unverified.
The second check requires opposite signed displacement of at least 0.3 m for both robots in
one world. Logs and JSON measurements are saved under `/tmp/nusim-native-policy/<scenario>/`.

### Validated on 8 October 2026

On this Apple Silicon Mac, the actual native player/DDS/simulator checks passed:

- One player approached 1.26 m and moved the ball 8.6 cm during the four-second kick.
  It remained standing after handing control back to the walk policy.
- Two player processes moved independently: +1.47 m and -1.40 m along the world X axis.
- The get-up policy completed recovery from `lying_front`, then returned to standing control.
- Native portability and all three policy inference contracts passed (2 tests).
- NUSim passed its macOS suite (12 tests) and Linux ARM64 suite (11 tests), including
  independent controllers, DDS routing and the new ground-truth capture test.

The selected ONNX files match their source-branch Git blobs exactly. This validates a
working motion path; it does not establish six-player tactics or robust recovery from every fall.
