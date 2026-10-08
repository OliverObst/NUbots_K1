#include <iostream>
#include <stdexcept>

#include "utility/strategy/soccer_strategy.hpp"

int main() {
    using message::localisation::Robot;
    using message::localisation::Robots;
    const Eigen::Vector3d ball(0, 0, 0);
    const Eigen::Vector2d goal(-7, 0);
    const auto Hfw = Eigen::Isometry3d::Identity();
    auto check     = [&](const std::vector<std::pair<unsigned int, Eigen::Vector3d>>& positions,
                     unsigned int expected,
                     const std::vector<unsigned int>& inactive) {
        for (const auto& [self, position] : positions) {
            if (std::find(inactive.begin(), inactive.end(), self) != inactive.end())
                continue;
            Robots others;
            for (const auto& [id, p] : positions) {
                if (id == self)
                    continue;
                Robot robot;
                robot.purpose.player_id = id;
                robot.purpose.active    = std::find(inactive.begin(), inactive.end(), id) == inactive.end();
                robot.rRWw              = p;
                robot.vRw               = Eigen::Vector3d::Zero();
                robot.teammate          = true;
                others.robots.push_back(robot);
            }
            auto Hfr          = Eigen::Isometry3d::Identity();
            Hfr.translation() = position;
            Hfr.linear()      = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            const auto winner = utility::strategy::fastest_to_ball_on_team(ball,
                                                                           goal,
                                                                           .4,
                                                                           others,
                                                                           Hfw,
                                                                           Hfr.inverse(),
                                                                           .7,
                                                                           1.,
                                                                           .5,
                                                                           self,
                                                                           inactive);
            const auto closest =
                utility::strategy::closest_to_ball_on_team(ball, others, Hfw, Hfr.inverse(), .1, self, inactive);
            if (winner != expected || closest != expected)
                throw std::runtime_error("Inconsistent team ranking for self=" + std::to_string(self));
            std::reverse(others.robots.begin(), others.robots.end());
            if (utility::strategy::
                    fastest_to_ball_on_team(ball, goal, .4, others, Hfw, Hfr.inverse(), .7, 1., .5, self, inactive)
                != expected)
                throw std::runtime_error("Team ranking depends on observation order");
        }
    };
    check({{1, {2, 0, 0}}, {2, {6.5, 0, 0}}, {3, {3, -2.5, 0}}}, 1, {});
    check({{1, {2, 0, 0}}, {2, {6.5, 0, 0}}, {3, {3, -2.5, 0}}}, 3, {1});
    check({{1, {2, 0, 0}}, {2, {2, 0, 0}}, {3, {2, 0, 0}}}, 3, {});
    std::cout << "Team ranking agrees for every player, tie and unavailable attacker\n";
}
