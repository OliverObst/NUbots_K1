#include "NUSimApproach.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "message/behaviour/state/Stability.hpp"
#include "message/input/NUSimStatus.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/skill/GetUp.hpp"
#include "message/skill/Kick.hpp"
#include "message/skill/Walk.hpp"
namespace module::strategy {
    NUSimApproach::NUSimApproach(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {
        on<Trigger<message::input::NUSimStatus>,
           With<message::input::Sensors>,
           With<message::localisation::Field>,
           With<message::localisation::Ball>,
           With<message::behaviour::state::Stability>,
           Sync<NUSimApproach>>()
            .then([this](const message::input::NUSimStatus& status,
                         const message::input::Sensors& sensors,
                         const message::localisation::Field& field,
                         const message::localisation::Ball& ball,
                         const message::behaviour::state::Stability& stability) {
                received = NUClear::clock::now();
                if (status.reset_generation != generation || status.reset) {
                    emit<Task>(std::unique_ptr<message::skill::Walk>{});
                    emit<Task>(std::unique_ptr<message::skill::Kick>{});
                    emit<Task>(std::unique_ptr<message::skill::GetUp>{});
                    kicking       = false;
                    finished      = false;
                    recovering    = false;
                    settled_since = -1;
                    generation    = status.reset_generation;
                }
                if (!status.ready || !field.localised || ball.confidence < 0.5) {
                    emit<Task>(std::unique_ptr<message::skill::Walk>{});
                    emit<Task>(std::unique_ptr<message::skill::Kick>{});
                    emit<Task>(std::unique_ptr<message::skill::GetUp>{});
                    return;
                }
                if (status.fallen && !recovering) {
                    emit<Task>(std::unique_ptr<message::skill::Walk>{});
                    emit<Task>(std::unique_ptr<message::skill::Kick>{});
                    recovering = true;
                    kicking    = false;
                    emit<Task>(std::make_unique<message::skill::GetUp>(), 100);
                    return;
                }
                if (recovering) {
                    if (stability != message::behaviour::state::Stability::STANDING) {
                        emit<Task>(std::make_unique<message::skill::GetUp>(), 100);
                        return;
                    }
                    emit<Task>(std::unique_ptr<message::skill::GetUp>{});
                    recovering = false;
                }
                const Eigen::Vector3d relative = sensors.Hrw * ball.rBWw;
                if (status.sim_time - last_log >= 1) {
                    log<INFO>("PLAYER",
                              status.robot_id,
                              "t",
                              status.sim_time,
                              "ball",
                              relative.x(),
                              relative.y(),
                              "robot",
                              sensors.Hrw.inverse().translation().x(),
                              sensors.Hrw.inverse().translation().y(),
                              "worldball",
                              ball.rBWw.x(),
                              ball.rBWw.y(),
                              "kick",
                              kicking,
                              "done",
                              finished);
                    last_log = status.sim_time;
                }
                if (kicking && status.sim_time - kick_start < 4.0) {
                    emit<Task>(std::make_unique<message::skill::Kick>(), 50);
                    return;
                }
                if (kicking) {
                    emit<Task>(std::unique_ptr<message::skill::Kick>{});
                    kicking  = false;
                    finished = true;
                    log<INFO>("Kick completed",
                              status.robot_id,
                              "t",
                              status.sim_time,
                              "worldball",
                              ball.rBWw.x(),
                              ball.rBWw.y());
                }
                auto walk                 = std::make_unique<message::skill::Walk>();
                walk->velocity_target     = Eigen::Vector3d::Zero();
                const char* test_velocity = std::getenv("NUSIM_TEST_VELOCITY");
                if (test_velocity) {
                    walk->velocity_target.x() = std::stod(test_velocity);
                }
                else if (!finished) {
                    // The trained kick is a side-foot sweep. Place the ball forward and just to the left.
                    const double heading  = std::atan2(relative.y() - 0.10, relative.x());
                    walk->velocity_target = {std::clamp(0.8 * (relative.x() - 0.08), -0.12, 0.25),
                                             std::clamp(0.8 * (relative.y() - 0.10), -0.12, 0.12),
                                             std::clamp(heading, -0.5, 0.5)};
                    if (relative.x() > 0.02 && relative.x() < 0.15 && std::abs(relative.y() - 0.10) < 0.08) {
                        walk->velocity_target = Eigen::Vector3d::Zero();
                        emit<Task>(std::move(walk), 10);
                        if (settled_since < 0)
                            settled_since = status.sim_time;
                        if (status.sim_time - settled_since < 1.0)
                            return;
                        emit<Task>(std::unique_ptr<message::skill::Walk>{});
                        kicking    = true;
                        kick_start = status.sim_time;
                        log<INFO>("Kick started",
                                  status.robot_id,
                                  "t",
                                  status.sim_time,
                                  "worldball",
                                  ball.rBWw.x(),
                                  ball.rBWw.y());
                        emit<Task>(std::make_unique<message::skill::Kick>(), 50);
                        return;
                    }
                }
                emit<Task>(std::move(walk), 10);
            });
        on<Every<10, Per<std::chrono::seconds>>, Sync<NUSimApproach>>().then([this] {
            if (NUClear::clock::now() - received > std::chrono::milliseconds(250)) {
                emit<Task>(std::unique_ptr<message::skill::Walk>{});
                emit<Task>(std::unique_ptr<message::skill::Kick>{});
                emit<Task>(std::unique_ptr<message::skill::GetUp>{});
            }
        });
    }
}  // namespace module::strategy
