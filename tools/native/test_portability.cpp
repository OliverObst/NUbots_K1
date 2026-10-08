#include <chrono>
#include <cstdio>

#include "message/conversion/proto_conversion.hpp"

#include "utility/support/si_unit.hpp"

int main() {
    using Conversion = message::conversion::
        Convert<message::conversion::Chrono, NUClear::clock::time_point, google::protobuf::Timestamp>;
    for (int nanos : {0, 1, 123456789, 999999999}) {
        google::protobuf::Timestamp original;
        original.set_seconds(1700000000);
        original.set_nanos(nanos);
        const auto timestamp = Conversion::call(original);
        const auto roundtrip = Conversion::call(timestamp);
        // The wire has nanoseconds; macOS system_clock has microseconds.
        const auto precision = std::chrono::duration_cast<std::chrono::nanoseconds>(NUClear::clock::duration(1));
        if (roundtrip.seconds() != original.seconds() || original.nanos() - roundtrip.nanos() < 0
            || original.nanos() - roundtrip.nanos() >= precision.count()) {
            std::fprintf(stderr, "Timestamp conversion lost more than the native clock precision\n");
            return 1;
        }
    }
    if (utility::support::si_unit(1000).second != "k" || utility::support::si_unit(0.001).second != "m") {
        std::fprintf(stderr, "SI prefix conversion failed\n");
        return 1;
    }
    std::puts("Native timestamp and integer portability passed");
}
