# Native six-player 3v3 scenarios

The `team` role runs six independent soccer decision trees, native ONNX Runtime CPU
walk/kick/get-up policies, symbolic localisation and direct team communication.
Team 125 controls simulator robots 1–3; team 126 controls robots 4–6. Both teams
use player IDs 1–3. The simulator supplies physics and state; the existing player
behaviours choose attacker, defender and supporter independently.

## Launch

Build the native player as described in [NATIVE_PLAYER.md](NATIVE_PLAYER.md), then:

```bash
bash tools/native/configure.sh -DROLE_team=ON
cmake --build build-player --target team player check_team_strategy -j 6
ctest --test-dir build-player --output-on-failure
# NUSim must be the sibling checkout, with mujoco/build-native/k1_mujoco_sim built.
caffeinate -i build-deps/venv/bin/python tools/native/run_match.py \
  --verify --output /tmp/nusim-match-001
```

Use a **new output directory** for each session. Without `--verify`, all six players
run continuously in PLAYING. Add `--duration 180 --video` for an interactive run.
Recording uses a full-frame 1280 × 720 simulator render at 15 fps, with team/player
labels and no desktop or window borders. `--video` enables the viewer and requires
`ffmpeg`/`ffprobe` on PATH. It waits until all six controllers are ready, records
`demo.mp4` for the requested duration and verifies the resulting movie duration.
`video.json` stores the encoder metadata; encoder CPU/RSS are measured separately.

The measured acceptance target applies to the headless session; camera rendering and the viewer
need separate benchmarks. The saved macOS vision prototype is not required.

Defaults reserve DDS domains 48–53, team UDP ports 24201–24206, GameController ports
24210–24216 and scenario port 24220. `--domain-base` and `--port-base` select another
range. Each team has its own communication port map; there is no tactical relay.
All six use the 14 × 9 m middle field and the previously validated conservative
walk planner limits. The scenario runs one half; attack directions follow the roster and switching ends
at half-time requires a future frame/roster update. All six are field players; this role does not add a goalie policy.

The launcher keeps generated configurations, model links, environment settings,
model SHA256s, source revisions, local patches/new sources, simulator and individual
player logs, `events.json`, `resources.jsonl` and `results.json`. Model binaries remain
in the companion build directory. It checks unexpected process exits and stops all
owned processes on normal completion, Ctrl-C or SIGTERM, using SIGINT then a bounded
wait and kill fallback. Existing simulations must use different domains and ports.

## GameController and placement

`NUSIM_GC_PORT` opts the player into a localhost GameController v20 receiver. Each
player receives its own copy of the same official packet layout; the supervisor
receives a separate copy. This avoids six local processes competing for port 3838.
The launcher currently generates packets, rather than running the graphical official
GameController. For live officiating, a future relay can forward official packets to
these dedicated ports. This receive-only adapter does not send GameController replies.

The adapter supplies phase, mode/set play, kickoff ownership, team colours/scores,
player penalties and time deadlines. Self-penalty changes emit the existing soccer
penalisation/unpenalisation events. A missing GameController for two seconds stops
the local behaviour tree; fresh packets resume it. Missing simulator state or an
unready controller also stops it. Without `NUSIM_GC_PORT`, the earlier three-player
launcher retains its synthetic PLAYING contract.

The generated supervisor configuration covers all six body names and identities.
Penalty transitions place a robot outside its own touchline; unpenalisation returns
it to its configured starting position in its own half. Each relocation clears only
that robot's controller and advances its individual reset count. Match reset restores
all starting poses, ball and controllers, advances the world reset generation and
resets simulation time. The bridge observes world resets explicitly and treats a controller leaving CUSTOM
as a local reset. Individual simulator reset counts remain internal to NUSim; the
existing world-v1 wire schema is unchanged.

The optional `simulation.yaml: scenario_port` is disabled by default. When enabled,
it accepts loopback UDP JSON actions `ball` (world x/y), `fall` (physical robot ID) and
`reset`. The launcher uses these deterministic perturbations. It enables a 250 ms
`locomotion.yaml: low_cmd_timeout` for disconnected players; omit it to preserve the
existing one-shot servo command behaviour.

## Repeatable scenario sequence

With `--verify`, the 70-second schedule exercises INITIAL → READY → SET → PLAYING,
then penalises and returns each robot in turn. It relocates the ball to (0, −2),
forces robot 3 prone, disconnects/restarts player 1, interrupts/restores the
GameController packet stream, then resets the match and repeats kickoff.

Checks require all six sensor identities and controller streams, both teammate
senders with role/intention messages, every phase and penalty, supervisor placement,
ball relocation observed by all players, a recorded fallen sensor state, teammate
expiry/reactivation, GameController expiry/resumption and world reset observed by all
players. Fall recovery can complete before the three-second observation checkpoint;
the check accepts the recorded fall in that interval. Motion policy robustness,
ball passing, realistic perception and complete competition rules remain separate work.

## Measurements

`results.json` separates:

- **Physics:** `mean_mujoco_step_ms` times `mj_step` itself, excluding controllers,
  snapshots and pacing sleep. RTF samples measure all shared-world advancement versus
  wall time, including controller and scheduling overhead. The first five samples are
  excluded as warm-up. Every remaining sample must meet the proposed 0.95 RTF target.
- **Control:** per-robot mean `mailbox_to_apply_ms` and counts measure a new servo
  command reaching the simulator controller mailbox to its first physics application.
  Overwritten mailbox commands are not counted. This excludes DDS transport, policy
  inference and sensor-to-action latency; it is not an end-to-end latency benchmark.
- **CPU:** one-second `ps` cumulative CPU deltas, per process; 100% means one full core.
- **Memory:** per-process peak RSS and peak simultaneous total RSS. This is resident
  memory, rather than the macOS Activity Monitor memory footprint.

The launcher returns a failure if a functional check or the RTF target fails, while
retaining the separate outcomes and raw logs for diagnosis. Repeated runs share the
same initial conditions and scenario schedule; asynchronous decisions and learned
motion mean their trajectories need not be byte-for-byte identical.

## Validated on 8 October 2026

The headless scenario session passed all six-player identity, control, communication,
phase, penalty/placement, kickoff, relocation, fall, reconnection and GameController
match-reset checks. Its logs are saved in the sibling NUSim checkout under
`mujoco/build-native/sessions/3v3-acceptance-20261008`.

The subsequent full-frame three-minute recording ran all six player processes
throughout: 1280 × 720, 15 fps, 2,700 frames and exactly 180 seconds. Every player
received both teammates, with 720–722 packets each, and all session processes
including the encoder exited cleanly. The recorded run measured:

| Measurement | Result |
| --- | --- |
| Mean / minimum measured RTF | 1.000000 / 0.999823 |
| Mean MuJoCo step time | 0.1066 ms |
| Per-robot mean command mailbox-to-application delay | 0.223–0.554 ms |
| Simulator CPU with viewer and recording | 127.9% (1.28 cores) |
| Individual player CPU | 12.4–26.7% |
| Encoder CPU | 4.9% |
| Peak simulator RSS | 1,609 MiB |
| Individual player peak RSS | 48.7–50.3 MiB |
| Peak encoder RSS | 116 MiB |

The longer run exposed a motion-policy limitation: robot 125/1 was marked fallen
in approximately 72% of diagnostic samples. Other players continued playing and
communication/control remained live. This is evidence for follow-up on get-up
stability, rather than evidence of robust three-minute soccer motion. The recording
and detailed measurements are in
`NUSim/mujoco/build-native/sessions/3v3-demo-20261008`.
