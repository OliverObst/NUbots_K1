#pragma once
#include <mutex>
#include <nuclear>

#include "module/SdkBridge/src/DdsParticipant.hpp"
namespace module::input {
    class NUSimPlayer : public NUClear::Reactor {
    public:
        explicit NUSimPlayer(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::unique_ptr<k1sim::module::sdkbridge::DdsParticipant> dds;
        eprosima::fastdds::dds::DataReader* reader = nullptr;
        eprosima::fastdds::dds::DataWriter* joints = nullptr;
        eprosima::fastdds::dds::DataWriter* rpc    = nullptr;
        std::mutex mutex;
        double covariance_floor = 1e-6;
        int robot_id            = 1;
        int current_mode        = -1;
        uint64_t generation = 0, sequence = 0, request = 0;
        bool seen = false;
        std::array<char, 36> session{};
        NUClear::clock::time_point last_sample{};
        void change_mode();
    };
}  // namespace module::input
