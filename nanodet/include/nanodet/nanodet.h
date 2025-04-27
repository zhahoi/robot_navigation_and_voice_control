// Tencent is pleased to support the open source community by making ncnn available.
//
// Copyright (C) 2021 THL A29 Limited, a Tencent company. All rights reserved.
//
// Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// https://opensource.org/licenses/BSD-3-Clause
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#ifndef NANODET_H
#define NANODET_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <benchmark.h>
#include <net.h>
#include "cpu.h"

#define PARAM_PATH "/home/hit/cartographer_ws/src/nanodet/weights/nanodet-m.param"
#define BIN_PATH "/home/hit/cartographer_ws/src/nanodet/weights/nanodet-m.bin"

struct Object
{
    cv::Rect_<float> rect;
    int label;
    float prob;
};

class NanoDet
{
public:
    NanoDet(const int& m_target_size, const float& m_prob_threshold, const float& m_nms_threshold, bool m_use_gpu);

    ~NanoDet();

    int load();

    int detect(const cv::Mat& rgb, std::vector<Object>& objects);

    int draw(cv::Mat& rgb, const std::vector<Object>& objects);

    int draw_unsupported(cv::Mat& rgb);

    int draw_fps(cv::Mat& rgb);

private:
    ncnn::Net nanodet;

    int class_nums = 80;
    int target_size = 320;
    float mean_vals[3] = {103.53f, 116.28f, 123.675f};
    float norm_vals[3] = {1.f / 57.375f, 1.f / 57.12f, 1.f / 58.395f};
    bool use_gpu = false;
    float prob_threshold = 0.45f;
    float nms_threshold = 0.5f;

    ncnn::UnlockedPoolAllocator blob_pool_allocator;
    ncnn::PoolAllocator workspace_pool_allocator;
};

#endif // NANODET_H