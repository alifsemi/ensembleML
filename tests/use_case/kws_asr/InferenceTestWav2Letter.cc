/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "hal.h"
#include "TensorFlowLiteMicro.hpp"
#include "Wav2LetterModel.hpp"
#include "TestData_asr.hpp"

#include <catch.hpp>
#include <random>

namespace test {
namespace asr {

bool RunInference(arm::app::Model& model, const int8_t vec[], const size_t copySz)
{
    TfLiteTensor* inputTensor = model.GetInputTensor(0);
    REQUIRE(inputTensor);

    memcpy(inputTensor->data.data, vec, copySz);

    return model.RunInference();
}

bool RunInferenceRandom(arm::app::Model& model)
{
    TfLiteTensor* inputTensor = model.GetInputTensor(0);
    REQUIRE(inputTensor);

    std::random_device rndDevice;
    std::mt19937 mersenneGen{rndDevice()};
    std::uniform_int_distribution<short> dist {-128, 127};

    auto gen = [&dist, &mersenneGen](){
                   return dist(mersenneGen);
               };

    std::vector<int8_t> randomAudio(inputTensor->bytes);
    std::generate(std::begin(randomAudio), std::end(randomAudio), gen);

    REQUIRE(RunInference(model, randomAudio.data(), inputTensor->bytes));
    return true;
}

TEST_CASE("Running random inference with Tflu and Wav2LetterModel Int8", "[Wav2Letter]")
{
    arm::app::Wav2LetterModel model{};

    REQUIRE_FALSE(model.IsInited());
    REQUIRE(model.Init());
    REQUIRE(model.IsInited());

    REQUIRE(RunInferenceRandom(model));
}


template<typename T>
void TestInference(const T* input_goldenFV, const T* output_goldenFV, arm::app::Model& model)
{
    TfLiteTensor* inputTensor = model.GetInputTensor(0);
    REQUIRE(inputTensor);

    REQUIRE(RunInference(model, input_goldenFV, inputTensor->bytes));

    TfLiteTensor* outputTensor = model.GetOutputTensor(0);

    REQUIRE(outputTensor);
    REQUIRE(outputTensor->bytes == OFM_DATA_SIZE);
    auto tensorData = tflite::GetTensorData<T>(outputTensor);
    REQUIRE(tensorData);

    for (size_t i = 0; i < outputTensor->bytes; i++) {
        REQUIRE((int)tensorData[i] == (int)((T)output_goldenFV[i]));
    }
}

TEST_CASE("Running inference with Tflu and Wav2LetterModel Int8", "[Wav2Letter]")
{
    for (uint32_t i = 0 ; i < NUMBER_OF_FM_FILES; ++i) {
        auto input_goldenFV = get_ifm_data_array(i);;
        auto output_goldenFV = get_ofm_data_array(i);

        DYNAMIC_SECTION("Executing inference with re-init")
        {
            arm::app::Wav2LetterModel model{};

            REQUIRE_FALSE(model.IsInited());
            REQUIRE(model.Init());
            REQUIRE(model.IsInited());

            TestInference<int8_t>(input_goldenFV, output_goldenFV, model);

        }
    }
}

} //namespace
} //namespace
