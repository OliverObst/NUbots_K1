#pragma once
#include <iostream>
#include <map>
#include <nuclear>

#include "../../GameController/src/GameControllerData.hpp"

#include "message/input/GameState.hpp"
#include "message/input/Robocup.hpp"
namespace module::input {
    class NUSimTeam : public NUClear::Reactor {
    public:
        explicit NUSimTeam(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Teammate {
            message::input::Message message;
            NUClear::clock::time_point received;
        };
        std::map<uint32_t, Teammate> teammates;
        std::map<uint32_t, bool> availability;
        void publish_game(bool ready);
        message::input::GameState game;
        NUClear::clock::time_point gc_received{};
        bool use_gc        = false;
        uint32_t player_id = 0, team_id = 0;
        double sim_time = 0;
        NUClear::clock::time_point received{};
    };
}  // namespace module::input
