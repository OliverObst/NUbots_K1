#include "NUSimPlayer.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include "LowCmdPubSubTypes.h"
#include "RpcReqMsgPubSubTypes.h"
#include "WorldSnapshotPubSubTypes.h"
#include "shared/k1/BoosterApi.hpp"

#include "message/behaviour/state/Stability.hpp"
#include "message/booster/BoosterLowCmd.hpp"
#include "message/booster/BoosterMode.hpp"
#include "message/booster/BoosterModeState.hpp"
#include "message/input/NUSimStatus.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/localisation/Robot.hpp"
#include "message/platform/RawSensors.hpp"

#include "utility/math/euler.hpp"
#include "utility/platform/RawSensors.hpp"

namespace module::input {
    using Dds = k1sim::module::sdkbridge::DdsParticipant;
    using namespace eprosima::fastdds::dds;
    using namespace message;
    namespace {
        int env_int(const char* name, int fallback) {
            const char* value = std::getenv(name);
            return value ? std::stoi(value) : fallback;
        }
        Eigen::Vector3d vector(const std::array<double, 3>& v) {
            return {v[0], v[1], v[2]};
        }
        Eigen::Isometry3d pose(const nusim_msgs::msg::dds_::BodyTruth_& body) {
            Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
            const auto& q       = body.orientation_body_to_s();
            H.linear()          = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
            H.translation()     = vector(body.position_s());
            return H;
        }
        template <std::size_t N>
        bool finite(const std::array<double, N>& value) {
            for (double x : value)
                if (!std::isfinite(x))
                    return false;
            return true;
        }
        bool quaternion(const std::array<double, 4>& q) {
            if (!finite(q))
                return false;
            double norm = 0;
            for (double x : q)
                norm += x * x;
            return std::abs(norm - 1) < 1e-3;
        }
        bool valid_world(const nusim_msgs::msg::dds_::WorldSnapshot_& world) {
            if (world.robots().empty() || !std::isfinite(world.stamp().sim_time()) || world.stamp().sim_time() < 0)
                return false;
            if (world.teams()[0].team_id() == world.teams()[1].team_id()
                || world.teams()[0].attacks_positive_x() == world.teams()[1].attacks_positive_x())
                return false;
            for (std::size_t i = 0; i < world.robots().size(); ++i) {
                const auto& robot = world.robots()[i];
                const auto& body  = robot.torso();
                if (robot.identity().robot_id() != i + 1 || robot.identity().model_index() != i
                    || robot.identity().player_id() < 1 || robot.identity().player_id() > 11)
                    return false;
                if (robot.identity().team_id() != world.teams()[0].team_id()
                    && robot.identity().team_id() != world.teams()[1].team_id())
                    return false;
                if (!finite(body.position_s()) || !finite(body.linear_velocity_s())
                    || !finite(body.angular_velocity_s()) || !quaternion(body.orientation_body_to_s()))
                    return false;
                if (!finite(robot.imu().gyro_imu()) || !finite(robot.imu().acceleration_imu())
                    || !quaternion(robot.imu().orientation_imu_to_s()))
                    return false;
                for (const auto& joint : robot.joints())
                    if (!std::isfinite(joint.q()) || !std::isfinite(joint.dq()) || !std::isfinite(joint.ddq())
                        || !std::isfinite(joint.tau()))
                        return false;
            }
            return !world.ball().valid()
                   || (finite(world.ball().centre_s()) && finite(world.ball().linear_velocity_s())
                       && finite(world.ball().angular_velocity_s()));
        }

    }  // namespace
    void NUSimPlayer::change_mode() {
        booster_msgs::msg::dds_::RpcReqMsg_ req;
        req.uuid("native-player-" + std::to_string(robot_id) + "-" + std::to_string(++request));
        req.header("{\"api_id\":2000}");
        req.body("{\"mode\":3}");
        rpc->write(&req);
    }
    NUSimPlayer::NUSimPlayer(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        robot_id = env_int("NUSIM_ROBOT_ID", 1);
        if (const char* value = std::getenv("NUSIM_COVARIANCE_FLOOR"))
            covariance_floor = std::stod(value);
        if (!std::isfinite(covariance_floor) || covariance_floor <= 0)
            throw std::invalid_argument("Covariance floor must be positive and finite");
        const int domain = env_int("NUSIM_DDS_DOMAIN", robot_id - 1);
        if (robot_id < 1 || robot_id > 22 || domain < 0 || domain > 63)
            throw std::invalid_argument("Invalid robot/domain");
        dds    = std::make_unique<Dds>(domain, true);
        joints = dds->create_writer<booster_interface::msg::dds_::LowCmd_PubSubType>(k1sim::booster::TOPIC_JOINT_CTRL,
                                                                                     Dds::state_writer_qos());
        rpc    = dds->create_writer<booster_msgs::msg::dds_::RpcReqMsg_PubSubType>(k1sim::booster::TOPIC_RPC_REQUEST,
                                                                                Dds::state_writer_qos());
        reader = dds->create_reader<nusim_msgs::msg::dds_::WorldSnapshot_PubSubType>("rt/nusim/gt/world_v1",
                                                                                     Dds::rpc_request_reader_qos(2),
                                                                                     nullptr);
        log<INFO>("Native player", robot_id, "DDS domain", domain);
        on<Every<50, Per<std::chrono::seconds>>, Single>().then([this] {
            std::lock_guard<std::mutex> lock(mutex);
            nusim_msgs::msg::dds_::WorldSnapshot_ world;
            SampleInfo info;
            while (reader->take_next_sample(&world, &info) == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
                if (!info.valid_data || world.stamp().schema_version() != 1 || !valid_world(world))
                    continue;
                const auto& stamp = world.stamp();
                // Reject old captures and out-of-order samples in this simulator session.
                const auto unix_now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::system_clock::now().time_since_epoch())
                                          .count();
                if (stamp.capture_time_unix_ns() > uint64_t(unix_now) + 100000000ULL
                    || uint64_t(unix_now) > stamp.capture_time_unix_ns() + 250000000ULL)
                    continue;
                if (seen && session == stamp.session_id()
                    && (stamp.reset_generation() < generation
                        || (stamp.reset_generation() == generation && stamp.sample_sequence() <= sequence)))
                    continue;
                const nusim_msgs::msg::dds_::RobotTruth_* self = nullptr;
                for (const auto& robot : world.robots())
                    if (robot.identity().robot_id() == robot_id)
                        self = &robot;
                if (!self)
                    continue;
                const bool reset   = !seen || session != stamp.session_id() || generation != stamp.reset_generation();
                session            = stamp.session_id();
                generation         = stamp.reset_generation();
                sequence           = stamp.sample_sequence();
                seen               = true;
                last_sample        = NUClear::clock::now();
                current_mode       = self->mode();
                const auto now     = NUClear::clock::time_point(std::chrono::duration_cast<NUClear::clock::duration>(
                    std::chrono::nanoseconds(stamp.capture_time_unix_ns())));
                auto raw           = std::make_unique<platform::RawSensors>();
                raw->timestamp     = now;
                raw->gyroscope     = vector(self->imu().gyro_imu()).cast<float>();
                raw->accelerometer = vector(self->imu().acceleration_imu()).cast<float>();
                const auto& q      = self->imu().orientation_imu_to_s();
                const auto R       = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized().toRotationMatrix();
                raw->imu_rpy       = utility::math::euler::mat_to_rpy_intrinsic(R).cast<float>();
                platform::RawSensors::Servo* servos[] = {
                    &raw->servo.head_pan,         &raw->servo.head_tilt,       &raw->servo.l_shoulder_pitch,
                    &raw->servo.l_shoulder_roll,  &raw->servo.l_elbow,         &raw->servo.l_elbow_yaw,
                    &raw->servo.r_shoulder_pitch, &raw->servo.r_shoulder_roll, &raw->servo.r_elbow,
                    &raw->servo.r_elbow_yaw,      &raw->servo.l_hip_pitch,     &raw->servo.l_hip_roll,
                    &raw->servo.l_hip_yaw,        &raw->servo.l_knee,          &raw->servo.l_ankle_pitch,
                    &raw->servo.l_ankle_roll,     &raw->servo.r_hip_pitch,     &raw->servo.r_hip_roll,
                    &raw->servo.r_hip_yaw,        &raw->servo.r_knee,          &raw->servo.r_ankle_pitch,
                    &raw->servo.r_ankle_roll};
                for (std::size_t i = 0; i < 22; ++i) {
                    servos[i]->present_position = self->joints()[i].q();
                    servos[i]->present_velocity = self->joints()[i].dq();
                    servos[i]->torque_enabled   = true;
                }
                auto sensors       = std::make_unique<message::input::Sensors>();
                sensors->timestamp = now;
                for (uint32_t id = 0; id < 22; ++id) {
                    const auto& servo = utility::platform::get_raw_servo(id, *raw);
                    sensors->servo.emplace_back(0,
                                                id,
                                                true,
                                                0,
                                                0,
                                                0,
                                                0,
                                                0,
                                                servo.present_position,
                                                servo.present_velocity,
                                                servo.present_current,
                                                0,
                                                0);
                }
                const auto Hst         = pose(self->torso());
                sensors->Htw           = Hst.inverse();
                Eigen::Isometry3d Hsr  = Eigen::Isometry3d::Identity();
                const double yaw       = std::atan2(Hst.linear()(1, 0), Hst.linear()(0, 0));
                Hsr.linear()           = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                Hsr.translation()      = Eigen::Vector3d(Hst.translation().x(), Hst.translation().y(), 0);
                sensors->Hrw           = Hsr.inverse();
                sensors->vTw           = vector(self->torso().linear_velocity_s());
                sensors->gyroscope     = raw->gyroscope.cast<double>();
                sensors->accelerometer = raw->accelerometer.cast<double>();
                auto field             = std::make_unique<localisation::Field>();
                field->Hfw             = Eigen::Isometry3d::Identity();
                field->covariance      = covariance_floor * Eigen::Matrix3d::Identity();
                field->localised       = true;
                for (const auto& team : world.teams())
                    if (team.team_id() == self->identity().team_id() && !team.attacks_positive_x())
                        field->Hfw.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                if (world.ball().valid()) {
                    auto ball                 = std::make_unique<localisation::Ball>();
                    ball->rBWw                = vector(world.ball().centre_s());
                    ball->vBw                 = vector(world.ball().linear_velocity_s());
                    ball->average_rBWw        = ball->rBWw;
                    ball->confidence          = 1;
                    ball->covariance          = covariance_floor * Eigen::Matrix4d::Identity();
                    ball->time_of_measurement = now;
                    emit(std::move(ball));
                }
                else {
                    auto ball                 = std::make_unique<localisation::Ball>();
                    ball->confidence          = 0;
                    ball->time_of_measurement = now;
                    emit(std::move(ball));
                }
                auto robots = std::make_unique<localisation::Robots>();
                for (const auto& other : world.robots())
                    if (other.identity().robot_id() != robot_id) {
                        localisation::Robot robot;
                        robot.id                  = other.identity().player_id();
                        robot.rRWw                = vector(other.torso().position_s());
                        robot.vRw                 = vector(other.torso().linear_velocity_s());
                        robot.covariance          = covariance_floor * Eigen::Matrix4d::Identity();
                        robot.time_of_measurement = now;
                        robot.teammate            = other.identity().team_id() == self->identity().team_id();
                        robots->robots.push_back(robot);
                    }
                auto status              = std::make_unique<message::input::NUSimStatus>();
                status->robot_id         = robot_id;
                status->team_id          = self->identity().team_id();
                status->player_id        = self->identity().player_id();
                status->sim_time         = stamp.sim_time();
                status->ready            = current_mode == 3;
                status->fallen           = self->fall_state() == 2;
                status->reset_generation = generation;
                status->reset            = reset;
                emit(std::make_unique<booster::BoosterModeState>(booster::K1Mode(current_mode)));
                emit(std::move(raw));
                emit(std::move(sensors));
                emit(std::move(field));
                emit(std::move(robots));
                emit(std::move(status));
            }
        });
        on<Every<1, std::chrono::seconds>>().then([this] {
            std::lock_guard<std::mutex> lock(mutex);
            if (current_mode != 3)
                change_mode();
        });
        on<Trigger<booster::BoosterMode>>().then([this](const booster::BoosterMode& mode) {
            std::lock_guard<std::mutex> lock(mutex);
            if (int(mode.mode) == 3 && current_mode != 3)
                change_mode();
        });
        on<Trigger<booster::BoosterLowCmd>>().then([this](const booster::BoosterLowCmd& command) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!seen || current_mode != 3 || NUClear::clock::now() - last_sample > std::chrono::milliseconds(250)
                || command.motor_cmd.size() != 22)
                return;
            booster_interface::msg::dds_::LowCmd_ wire;
            wire.cmd_type(booster_interface::msg::dds_::SERIAL);
            for (const auto& c : command.motor_cmd) {
                if (!std::isfinite(c.q) || !std::isfinite(c.kp) || !std::isfinite(c.kd) || !std::isfinite(c.dq)
                    || !std::isfinite(c.tau))
                    return;
                booster_interface::msg::dds_::MotorCmd_ motor;
                motor.mode(c.mode);
                motor.q(c.q);
                motor.dq(c.dq);
                motor.tau(c.tau);
                motor.kp(c.kp);
                motor.kd(c.kd);
                motor.weight(c.weight);
                wire.motor_cmd().push_back(motor);
            }
            joints->write(&wire);
        });
    }
}  // namespace module::input
