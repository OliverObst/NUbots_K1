#pragma once
#include <array>
#include <cmath>
#include <memory>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <string>

namespace utility::motion {
    // CPU-only session: one thread avoids contention between independent players.
    template <std::size_t Observations, std::size_t Actions>
    class OnnxPolicy {
        Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "K1Policy"};
        std::unique_ptr<Ort::Session> session;
        std::string input_name, output_name;

    public:
        void load(const std::string& path) {
            Ort::SessionOptions options;
            options.SetIntraOpNumThreads(1);
            options.SetInterOpNumThreads(1);
            session = std::make_unique<Ort::Session>(env, path.c_str(), options);
            if (session->GetInputCount() != 1 || session->GetOutputCount() != 1)
                throw std::runtime_error("Policy requires one input/output");
            const auto in  = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            const auto out = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            if (in != std::vector<int64_t>{1, Observations} || out != std::vector<int64_t>{1, Actions})
                throw std::runtime_error("Policy observation/action shape mismatch");
            Ort::AllocatorWithDefaultOptions allocator;
            input_name  = session->GetInputNameAllocated(0, allocator).get();
            output_name = session->GetOutputNameAllocated(0, allocator).get();
        }
        std::array<float, Actions> infer(std::array<float, Observations>& observation) {
            for (auto value : observation)
                if (!std::isfinite(value))
                    throw std::runtime_error("Non-finite observation");
            const std::array<int64_t, 2> shape{1, Observations};
            auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            auto input  = Ort::Value::CreateTensor<float>(memory, observation.data(), Observations, shape.data(), 2);
            const char* inputs[]  = {input_name.c_str()};
            const char* outputs[] = {output_name.c_str()};
            auto result           = session->Run(Ort::RunOptions{nullptr}, inputs, &input, 1, outputs, 1);
            if (result[0].GetTensorTypeAndShapeInfo().GetElementCount() != Actions)
                throw std::runtime_error("Policy output size mismatch");
            const auto* values = result[0].template GetTensorData<float>();
            std::array<float, Actions> action{};
            for (std::size_t i = 0; i < Actions; ++i) {
                if (!std::isfinite(values[i]))
                    throw std::runtime_error("Non-finite action");
                action[i] = values[i];
            }
            return action;
        }
    };
}  // namespace utility::motion
