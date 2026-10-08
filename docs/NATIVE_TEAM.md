# Three native NUSim soccer players

For both teams, GameController phases, penalties and repeatable scenarios, use
[the six-player launcher](NATIVE_MATCH.md). The three-player launcher below remains
available for the original attacker-loss check.

The dedicated `team` role combines `NUSimPlayer`, `NUSimTeam`, `RobotCommunication`,
`Soccer`, `FieldPlayer`, `Attack`, `Defend`, `Support`, the existing walking/kicking
planners and the native CPU walk, kick and get-up policies. It runs one decision tree
in each player process. There is no simulator-side attacker assignment or team controller.

## Build and run

Follow [the native player build](NATIVE_PLAYER.md), then enable and build the team role:

```bash
bash tools/native/configure.sh -DROLE_team=ON
cmake --build build-player --target team check_team_strategy -j 6
ctest --test-dir build-player --output-on-failure
build-deps/venv/bin/python tools/native/run_team.py --duration 120 --viewer
```

The launcher starts a six-body, 3v3 NUSim world on the 14 × 9 m middle field. Three
independent player processes control robots 1–3 on team 125, with player IDs 1–3.
The opposing three robots retain their simulator controllers; no opposition behaviours
are launched. The existing behaviour may assign attack, defence or support to any of
our three players: the launcher's player IDs are identities, not fixed tactical roles.

Player configuration and model links are created in separate temporary working directories.
Defaults reserve DDS domains 48–53 and localhost UDP ports 24121–24123; override
`--domain-base` or `--port-base` for another match. Each process uses `NUSIM_TEAM_PLAYER=1`,
its own `NUSIM_ROBOT_ID` and `NUSIM_DDS_DOMAIN`. The launcher matches `FieldDescription`
to the middle field and limits the existing walk planner to the previously validated
CPU policy speeds (0.25 m/s forward, 0.12 m/s sideways, 0.5 rad/s turning).

## State, decisions and communication

The bridge supplies each robot's sensors, field transform, ball and physical robot poses.
For this role the internal field frame has the opponent's goal on **negative X**, as the
existing soccer behaviours expect. World-space messages remain in the simulator's frame;
the player's `Field.Hfw` handles the rotation. `Robots.id` is the simulator robot ID and
`Purpose.player_id` is the team-local player ID.

`NUSimTeam` adapts the roster into the player's expected `Robots` messages. Teammate
purpose and availability come from received communication, never from a central decision.
A teammate is inactive until a packet arrives, and becomes inactive after two seconds
without one. Its physical pose remains observable, so it can still be an obstacle.
Fallen, unready or stale players advertise unavailability. Opponents are physical
observations without inferred tactics.

`RobotCommunication` sends the normal RoboCup protobuf at 2 Hz. Its optional NUbots
field 101 carries the full `Purpose` (attack/defend/support, player ID and active flag);
`going_for_ball`, walking command and target pose carry intentions. Receivers without
that extension can still use the original `going_for_ball` flag. Within this role a
legacy packet supplies attack/support rather than a full role classification.

On this Mac, `127.255.255.255` broadcast did not reach the local sockets. The optional
`RobotCommunication.yaml` configuration below uses independent localhost UDP listeners
and direct peer sends. It has no relay, shared tactical state or central coordinator:

```yaml
local_player_ports:
  1: 24121
  2: 24122
  3: 24123
udp_filter_address: "127.0.0.1"
```

Omit `local_player_ports` to use the existing team-port broadcast transport. Each local
team must have its own port map. A physical or mixed-team broadcast match still uses
its configured broadcast address and `10000 + team_id` port.

The adapter supplies a synthetic, unpenalised `PLAYING` game for this first teamwork
experiment. It pauses the local soccer tree if simulator data becomes stale or its
controller is unready. The six-player launcher opts into the GameController receiver described in
[NATIVE_MATCH.md](NATIVE_MATCH.md). Clock synchronisation across machines is outside
these same-host experiments. All players are field players;
this role does not introduce a goalie policy, passing strategy or full competition vision.
The saved macOS vision prototype is not required.

## Acceptance check

```bash
# Keep this Mac awake during the experiment.
caffeinate -i build-deps/venv/bin/python tools/native/run_team.py --verify --duration 45
```

The check starts the actual simulator and all three player processes, records their
UDP reception and physical movement, then stops whichever player is currently attacking
at 20 seconds. It requires:

- all three processes receive both teammates' role and intention packets;
- a settled attacker/defender/supporter combination, selected by `FieldPlayer`;
- at least 15 cm of physical supporter movement;
- both remaining players expire the stopped attacker, and a replacement attacks;
- the remaining teammate receives the replacement's updated role and attacking intention.

Logs and `results.json` are written to `/tmp/nusim-native-team/` (or `--output`).
`TEAM_RX`, `TEAM_PEER` and `TEAM_STATE` distinguish received communication, availability
transitions and local decisions. Use fresh simulator runs for repeatable starting positions.
Initial roles can differ briefly before the first teammate packets arrive.

This integration also fixes the existing ranking helper's initial winner: a player with
a larger ID could previously retain itself despite a faster lower-ID teammate. Regression
checks cover every player's view, observation order, ties and removal of the attacker.
The support behaviour's head request is now optional, allowing its walking task to run
when the parent `FieldPlayer` already owns ball tracking.

### Validated on 8 October 2026

Two successive 45-second runs on this Apple Silicon Mac passed the complete teamwork
check. In the final run:

| Player | Settled role | Teammates received | Total received packets |
| --- | --- | --- | --- |
| 1 | Attacker | 2, 3 | 72 |
| 2 | Defender | 1, 3 | 122 |
| 3 | Supporter, then attacker | 1, 2 | 122 |

Player 3 repositioned **1.31 m** while supporting. The test stopped player 1 at simulation
time 21.80 s. Both remaining processes expired its availability, player 3 took over, and
player 2 received player 3's updated attack role and `going_for_ball` intention. The second
run's logs and measurements are in `/tmp/nusim-native-team-final/`.

All three native tests passed (portability, the walk/kick/get-up inference contracts and
team ranking). The original single-player approach/kick check passed on repeat: 1.31 m
of approach and 15.5 cm of ball movement during the kick. An earlier regression run fell
during the kick and did not complete; the learned motion remains sensitive to timing and
stance. This milestone validates distributed coordination, not robust kicking or complete
competition play.
