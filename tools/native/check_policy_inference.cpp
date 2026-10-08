#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

#include "utility/motion/OnnxPolicy.hpp"
template <std::size_t N>
void check(const char* path) {
    utility::motion::OnnxPolicy<N, 22> policy;
    policy.load(path);
    std::array<float, N> observation{};
    observation[5] = -1;
    for (int i = 0; i < 25; ++i)
        policy.infer(observation);
    observation[0] = std::numeric_limits<float>::quiet_NaN();
    try {
        policy.infer(observation);
        throw std::logic_error("Non-finite input accepted");
    }
    catch (const std::runtime_error&) {
    }
    std::printf("%s: %zu observations, 22 finite CPU actions; rejects non-finite input\n", path, N);
}
int main(int argc, char** argv) {
    if (argc != 4)
        return 1;
    try {
        check<79>(argv[1]);
        check<80>(argv[2]);
        check<72>(argv[3]);
        try {
            utility::motion::OnnxPolicy<79, 22> wrong;
            wrong.load(argv[2]);
            return 1;
        }
        catch (const std::runtime_error&) {
        }
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
