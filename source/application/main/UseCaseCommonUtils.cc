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
#include "UseCaseCommonUtils.hpp"
#include "InputFiles.hpp"
#include <inttypes.h>

#include "lvgl.h"

extern lv_obj_t *labelResult1;
extern lv_obj_t *labelResult2;
extern lv_obj_t *labelResult3;
//extern lv_obj_t *labelTime;


void DisplayCommonMenu()
{
    printf("\n\n");
    printf("User input required\n");
    printf("Enter option number from:\n\n");
    printf("  %u. Classify next ifm\n", common::MENU_OPT_RUN_INF_NEXT);
    printf("  %u. Classify ifm at chosen index\n", common::MENU_OPT_RUN_INF_CHOSEN);
    printf("  %u. Run classification on all ifm\n", common::MENU_OPT_RUN_INF_ALL);
    printf("  %u. Show NN model info\n", common::MENU_OPT_SHOW_MODEL_INFO);
    printf("  %u. List ifm\n\n", common::MENU_OPT_LIST_IFM);
    printf("  Choice: ");
    fflush(stdout);
}

void image::ConvertImgToInt8(void* data, const size_t kMaxImageSize)
{
    auto* tmp_req_data = (uint8_t*) data;
    auto* tmp_signed_req_data = (int8_t*) data;

    for (size_t i = 0; i < kMaxImageSize; i++) {
        tmp_signed_req_data[i] = (int8_t) (
            (int32_t) (tmp_req_data[i]) - 128);
    }
}

bool image::PresentInferenceResult(hal_platform& platform,
                                       const std::vector<arm::app::ClassificationResult>& results)
{
    //return PresentInferenceResult(platform, results, false);

    lv_label_set_text_fmt(labelResult1, "%s (%d%%)", results[0].m_label.c_str(), 
            (uint32_t)(results[0].m_normalisedVal * 100));
    lv_label_set_text_fmt(labelResult2, "%s (%d%%)", results[1].m_label.c_str(), 
            (uint32_t)(results[1].m_normalisedVal * 100));
    lv_label_set_text_fmt(labelResult3, "%s (%d%%)", results[2].m_label.c_str(), 
            (uint32_t)(results[2].m_normalisedVal * 100));
    return true;
}

bool image::PresentInferenceResult(hal_platform &platform,
                                   const std::vector<arm::app::ClassificationResult> &results,
                                   const time_t infTimeMs)
{
    return PresentInferenceResult(platform, results, true, infTimeMs);
}


bool image::PresentInferenceResult(hal_platform &platform,
                                        const std::vector<arm::app::ClassificationResult> &results,
                                        bool profilingEnabled,
                                        const time_t infTimeMs)
{
    constexpr uint32_t dataPsnTxtStartX1 = 150;
    constexpr uint32_t dataPsnTxtStartY1 = 30;

    constexpr uint32_t dataPsnTxtStartX2 = 10;
    constexpr uint32_t dataPsnTxtStartY2 = 150;

    constexpr uint32_t dataPsnTxtYIncr = 16;  /* Row index increment. */

    if(profilingEnabled)
    {
        platform.data_psn->set_text_color(COLOR_YELLOW);

        /* If profiling is enabled, and the time is valid. */
        info("Final results:\n");
        info("Total number of inferences: 1\n");
        if (infTimeMs)
        {
            std::string strInf =
                    std::string{"Inference: "} +
                    std::to_string(infTimeMs) +
                    std::string{"ms"};
            platform.data_psn->present_data_text(
                    strInf.c_str(), strInf.size(),
                    dataPsnTxtStartX1, dataPsnTxtStartY1, 0);
        }
    }
    platform.data_psn->set_text_color(COLOR_GREEN);

    /* Display each result. */
    uint32_t rowIdx1 = dataPsnTxtStartY1 + 2 * dataPsnTxtYIncr;
    uint32_t rowIdx2 = dataPsnTxtStartY2;

    if(!profilingEnabled)
    {
        info("Final results:\n");
        info("Total number of inferences: 1\n");
    }

    for (uint32_t i = 0; i < results.size(); ++i) {
        std::string resultStr =
                std::to_string(i + 1) + ") " +
                std::to_string(results[i].m_labelIdx) +
                " (" + std::to_string(results[i].m_normalisedVal) + ")";

        platform.data_psn->present_data_text(
                resultStr.c_str(), resultStr.size(),
                dataPsnTxtStartX1, rowIdx1, 0);
        rowIdx1 += dataPsnTxtYIncr;

        resultStr = std::to_string(i + 1) + ") " + results[i].m_label;
        platform.data_psn->present_data_text(
                resultStr.c_str(), resultStr.size(),
                dataPsnTxtStartX2, rowIdx2, 0);
        rowIdx2 += dataPsnTxtYIncr;

        if(profilingEnabled)
        {
            info("%" PRIu32 ") %" PRIu32 " (%f) -> %s\n", i, results[i].m_labelIdx,
                 results[i].m_normalisedVal, results[i].m_label.c_str());
        }
        else
        {
            info("%" PRIu32 ") %" PRIu32 " (%f) -> %s\n", i,
                    results[i].m_labelIdx, results[i].m_normalisedVal,
                    results[i].m_label.c_str());
        }
    }

    return true;
}

void IncrementAppCtxIfmIdx(arm::app::ApplicationContext& ctx, std::string useCase)
{
    auto curImIdx = ctx.Get<uint32_t>(useCase);

    if (curImIdx + 1 >= NUMBER_OF_FILES) {
        ctx.Set<uint32_t>(useCase, 0);
        return;
    }
    ++curImIdx;
    ctx.Set<uint32_t>(useCase, curImIdx);
}

bool SetAppCtxIfmIdx(arm::app::ApplicationContext& ctx, uint32_t idx, std::string ctxIfmName)
{
    if (idx >= NUMBER_OF_FILES) {
        printf_err("Invalid idx %" PRIu32 " (expected less than %u)\n",
                   idx, NUMBER_OF_FILES);
        return false;
    }
    ctx.Set<uint32_t>(ctxIfmName, idx);
    return true;
}


namespace arm {
namespace app {


bool RunInference(arm::app::Model& model, Profiler& profiler)
{
    profiler.StartProfiling("Inference");
    bool runInf = model.RunInference();
    profiler.StopProfiling();

    return runInf;
}

int ReadUserInputAsInt(hal_platform& platform)
{
    char chInput[128];
    memset(chInput, 0, sizeof(chInput));

    platform.data_acq->get_input(chInput, sizeof(chInput));
    return atoi(chInput);
}

void DumpTensorData(const uint8_t* tensorData,
                    size_t size,
                    size_t lineBreakForNumElements)
{
    char strhex[8];
    std::string strdump;

    for (size_t i = 0; i < size; ++i) {
        if (0 == i % lineBreakForNumElements) {
            printf("%s\n\t", strdump.c_str());
            strdump.clear();
        }
        snprintf(strhex, sizeof(strhex) - 1,
                 "0x%02x, ", tensorData[i]);
        strdump += std::string(strhex);
    }

    if (!strdump.empty()) {
        printf("%s\n", strdump.c_str());
    }
}

void DumpTensor(const TfLiteTensor* tensor, const size_t lineBreakForNumElements)
{
    if (!tensor) {
        printf_err("invalid tensor\n");
        return;
    }

    const uint32_t tensorSz = tensor->bytes;
    const uint8_t* tensorData = tflite::GetTensorData<uint8_t>(tensor);

    DumpTensorData(tensorData, tensorSz, lineBreakForNumElements);
}

bool ListFilesHandler(ApplicationContext& ctx)
{
    auto& model = ctx.Get<Model&>("model");
    auto& platform = ctx.Get<hal_platform&>("platform");

    constexpr uint32_t dataPsnTxtStartX = 20;
    constexpr uint32_t dataPsnTxtStartY = 40;

    if (!model.IsInited()) {
        printf_err("Model is not initialised! Terminating processing.\n");
        return false;
    }

    /* Clear the LCD */
    platform.data_psn->clear(COLOR_BLACK);

    /* Show the total number of embedded files. */
    std::string strNumFiles = std::string{"Total Number of Files: "} +
                               std::to_string(NUMBER_OF_FILES);
    platform.data_psn->present_data_text(strNumFiles.c_str(),
                                         strNumFiles.size(),
                                         dataPsnTxtStartX,
                                         dataPsnTxtStartY,
                                         false);

#if NUMBER_OF_FILES > 0
        constexpr uint32_t dataPsnTxtYIncr = 16;
        info("List of Files:\n");
        uint32_t yVal = dataPsnTxtStartY + dataPsnTxtYIncr;
        for (uint32_t i = 0; i < NUMBER_OF_FILES; ++i, yVal += dataPsnTxtYIncr) {

            std::string currentFilename{get_filename(i)};
            platform.data_psn->present_data_text(currentFilename.c_str(),
                                                 currentFilename.size(),
                                                 dataPsnTxtStartX, yVal, false);

            info("\t%" PRIu32 " => %s\n", i, currentFilename.c_str());
        }
#endif /* NUMBER_OF_FILES > 0 */

        return true;
}

} /* namespace app */
} /* namespace arm */
