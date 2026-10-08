#include "NUSimTeam.hpp"

#include <cstdlib>
#include <cstring>
#include <fmt/format.h>

#include "message/input/GameEvents.hpp"
#include "message/input/GameState.hpp"
#include "message/input/NUSimRoster.hpp"
#include "message/input/NUSimStatus.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Robot.hpp"
#include "message/planning/WalkPath.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/support/GlobalConfig.hpp"
namespace module::input {
    NUSimTeam::NUSimTeam(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        if (!std::getenv("NUSIM_TEAM_PLAYER"))
            throw std::invalid_argument("team role requires NUSIM_TEAM_PLAYER=1");
        if (const char* port = std::getenv("NUSIM_GC_PORT")) {
            use_gc = true;
            on<UDP, Sync<NUSimTeam>>(std::stoi(port)).then([this](const UDP::Packet& packet) {
                using namespace gamecontroller;
                GameControllerPacket wire{};
                if (packet.remote.address != "127.0.0.1" || packet.payload.size() != sizeof(wire))
                    return;
                std::memcpy(&wire, packet.payload.data(), sizeof(wire));
                if (std::memcmp(wire.header.data(), RECEIVE_HEADER, 4) || wire.version != 20 || int(wire.state) > 4
                    || wire.players_per_team > MAX_NUM_PLAYERS || !team_id)
                    return;
                const Team* own      = nullptr;
                const Team* opponent = nullptr;
                for (const auto& team : wire.teams) {
                    if (team.team_id == team_id)
                        own = &team;
                    else
                        opponent = &team;
                }
                if (!own || !opponent || player_id < 1 || player_id > wire.players_per_team)
                    return;
                using GS                    = message::input::GameState;
                const auto previous_penalty = game.self.penalty_reason;
                game.phase =
                    wire.state == State::FINISHED ? GS::Phase(GS::Phase::FINISHED) : GS::Phase(int(wire.state) + 1);
                game.mode = GS::Mode(int(wire.game_phase) + 1);
                if (wire.set_play != SetPlay::NONE) {
                    static const GS::Mode modes[] = {GS::Mode::NORMAL,
                                                     GS::Mode::DIRECT_FREEKICK,
                                                     GS::Mode::INDIRECT_FREEKICK,
                                                     GS::Mode::PENALTYKICK,
                                                     GS::Mode::THROW_IN,
                                                     GS::Mode::GOAL_KICK,
                                                     GS::Mode::CORNER_KICK};
                    if (int(wire.set_play) > 6)
                        return;
                    game.mode = modes[int(wire.set_play)];
                }
                game.first_half   = wire.first_half;
                game.our_kick_off = wire.kicking_team == team_id;
                game.stopped      = wire.stopped;
                auto fill         = [](GS::Team& target, const Team& source) {
                    target.team_id     = source.team_id;
                    target.team_colour = GS::TeamColour(int(source.field_player_colour) + 1);
                    target.score       = source.score;
                    target.players.clear();
                    for (unsigned i = 0; i < 3; ++i) {
                        GS::Robot robot;
                        robot.id             = i + 1;
                        robot.goalie         = source.goalkeeper == i + 1;
                        robot.penalty_reason = GS::PenaltyReason(int(source.players[i].penalty_state));
                        robot.unpenalised =
                            NUClear::clock::now() + std::chrono::seconds(source.players[i].penalised_time_left);
                        target.players.push_back(robot);
                    }
                };
                fill(game.team, *own);
                fill(game.opponent, *opponent);
                game.self           = game.team.players.at(player_id - 1);
                game.primary_time   = NUClear::clock::now() + std::chrono::seconds(wire.secs_remaining);
                game.secondary_time = NUClear::clock::now() + std::chrono::seconds(wire.secondary_time);
                gc_received         = NUClear::clock::now();
                publish_game(true);
                if (previous_penalty != game.self.penalty_reason) {
                    using GE = message::input::GameEvents;
                    if (game.self.penalty_reason == GS::PenaltyReason::UNPENALISED) {
                        auto event      = std::make_unique<GE::Unpenalisation>();
                        event->context  = GE::Context::SELF;
                        event->robot_id = player_id;
                        emit(std::move(event));
                    }
                    else {
                        auto event      = std::make_unique<GE::Penalisation>();
                        event->context  = GE::Context::SELF;
                        event->robot_id = player_id;
                        event->reason   = game.self.penalty_reason;
                        event->ends     = game.self.unpenalised;
                        emit(std::move(event));
                    }
                }
                log<INFO>(fmt::format("TEAM_GC team={} player={} phase={} penalty={} kickoff={} stopped={}",
                                      team_id,
                                      player_id,
                                      int(game.phase),
                                      int(game.self.penalty_reason),
                                      game.our_kick_off,
                                      game.stopped));
            });
        }
        on<Trigger<message::input::NUSimStatus>, Sync<NUSimTeam>>().then(
            [this](const message::input::NUSimStatus& status) {
                received = NUClear::clock::now();
                sim_time = status.sim_time;
                if (player_id != status.player_id || team_id != status.team_id) {
                    player_id         = status.player_id;
                    team_id           = status.team_id;
                    auto config       = std::make_unique<message::support::GlobalConfig>();
                    config->player_id = player_id;
                    config->team_id   = team_id;
                    emit(std::move(config));
                }
                if (status.reset) {
                    teammates.clear();
                    availability.clear();
                }
                publish_game(status.ready);
            });
        on<Trigger<message::input::Message>, Sync<NUSimTeam>>().then([this](const message::input::Message& msg) {
            if (msg.current_pose.player_id == player_id || msg.current_pose.player_id == 0)
                return;
            teammates[msg.current_pose.player_id] = {msg, NUClear::clock::now()};
            log<INFO>(
                fmt::format("TEAM_RX self={} peer={} role={} active={} intention={} target=({:.3f},{:.3f}) "
                            "walk=({:.3f},{:.3f},{:.3f}) t={:.3f}",
                            player_id,
                            msg.current_pose.player_id,
                            int(msg.team_purpose.purpose),
                            msg.state != message::input::State::PENALISED && msg.team_purpose.active,
                            msg.going_for_ball,
                            msg.target_pose.position.x(),
                            msg.target_pose.position.y(),
                            msg.walk_command.x(),
                            msg.walk_command.y(),
                            msg.walk_command.z(),
                            sim_time));
        });
        on<Trigger<message::input::NUSimRoster>, Sync<NUSimTeam>>().then(
            [this](const message::input::NUSimRoster& roster) {
                auto robots = std::make_unique<message::localisation::Robots>(roster.robots);
                for (auto& robot : robots->robots) {
                    if (!robot.teammate)
                        continue;
                    const auto id         = robot.purpose.player_id;
                    robot.purpose.purpose = message::purpose::SoccerPosition::UNKNOWN;
                    robot.purpose.active  = false;
                    const auto found      = teammates.find(id);
                    if (found != teammates.end()
                        && NUClear::clock::now() - found->second.received < std::chrono::seconds(2)) {
                        const auto& msg = found->second.message;
                        if (msg.team_purpose.player_id == id)
                            robot.purpose = msg.team_purpose;
                        else {
                            robot.purpose.purpose = msg.going_for_ball ? message::purpose::SoccerPosition::ATTACK
                                                                       : message::purpose::SoccerPosition::SUPPORT;
                            robot.purpose.active  = true;
                        }
                        robot.purpose.active = robot.purpose.active && msg.state != message::input::State::PENALISED;
                    }
                    if (!availability.count(id) || availability[id] != robot.purpose.active) {
                        availability[id] = robot.purpose.active;
                        log<INFO>(
                            fmt::format("TEAM_PEER self={} peer={} active={}", player_id, id, robot.purpose.active));
                    }
                }
                emit(std::move(robots));
            });
        on<Every<1, std::chrono::seconds>,
           With<message::input::NUSimStatus>,
           With<message::input::Sensors>,
           With<message::purpose::Purpose>,
           Optional<With<message::planning::WalkToDebug>>,
           Sync<NUSimTeam>>()
            .then([this](const message::input::NUSimStatus& status,
                         const message::input::Sensors& sensors,
                         const message::purpose::Purpose& purpose,
                         const std::shared_ptr<const message::planning::WalkToDebug>& target) {
                const Eigen::Vector3d pos = sensors.Hrw.inverse().translation();
                log<INFO>(fmt::format(
                    "TEAM_STATE id={} t={:.3f} role={} pos=({:.4f},{:.4f}) target=({:.4f},{:.4f}) fallen={}",
                    player_id,
                    status.sim_time,
                    int(purpose.purpose),
                    pos.x(),
                    pos.y(),
                    target ? target->Hrd.translation().x() : 0.,
                    target ? target->Hrd.translation().y() : 0.,
                    status.fallen));
            });
        on<Every<1, std::chrono::seconds>,
           With<message::input::NUSimStatus>,
           With<message::localisation::Ball>,
           With<message::input::Sensors>,
           Sync<NUSimTeam>>()
            .then([this](const message::input::NUSimStatus& status,
                         const message::localisation::Ball& ball,
                         const message::input::Sensors& sensors) {
                const Eigen::Vector3d pos = sensors.Hrw.inverse().translation();
                log<INFO>(
                    fmt::format("MATCH_STATE robot={} team={} player={} t={:.3f} generation={} "
                                "pos=({:.3f},{:.3f}) ball=({:.3f},{:.3f}) fallen={} ready={} gc_live={}",
                                status.robot_id,
                                team_id,
                                player_id,
                                status.sim_time,
                                status.reset_generation,
                                pos.x(),
                                pos.y(),
                                ball.rBWw.x(),
                                ball.rBWw.y(),
                                status.fallen,
                                status.ready,
                                !use_gc || NUClear::clock::now() - gc_received <= std::chrono::seconds(2)));
            });
        on<Every<10, Per<std::chrono::seconds>>, Sync<NUSimTeam>>().then([this] {
            if (player_id) {
                const bool ready = NUClear::clock::now() - received <= std::chrono::milliseconds(250);
                publish_game(ready);
            }
        });
    }
    void NUSimTeam::publish_game(bool ready) {
        using GS = message::input::GameState;
        if (!use_gc) {
            game.phase            = GS::Phase::PLAYING;
            game.mode             = GS::Mode::NORMAL;
            game.our_kick_off     = true;
            game.first_half       = true;
            game.team.team_id     = team_id;
            game.team.team_colour = GS::TeamColour::BLUE;
            game.self.id          = player_id;
        }
        auto output     = std::make_unique<GS>(game);
        output->stopped = output->stopped || !ready || output->phase == GS::Phase::INITIAL
                          || output->phase == GS::Phase::FINISHED
                          || (use_gc && NUClear::clock::now() - gc_received > std::chrono::seconds(2));
        emit(std::make_unique<GS::Phase>(output->phase));
        emit<Scope::INLINE>(std::move(output));
    }
}  // namespace module::input
