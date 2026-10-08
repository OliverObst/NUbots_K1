#pragma once
#include <nuclear>

#include "extension/Behaviour.hpp"
namespace module::strategy {
    class NUSimApproach : public ::extension::behaviour::BehaviourReactor {
    public:
        explicit NUSimApproach(std::unique_ptr<NUClear::Environment> environment);

    private:
        bool kicking = false, finished = false, recovering = false;
        double kick_start = 0, last_log = -1, settled_since = -1;
        uint64_t generation = 0;
        NUClear::clock::time_point received{};
    };
}  // namespace module::strategy
