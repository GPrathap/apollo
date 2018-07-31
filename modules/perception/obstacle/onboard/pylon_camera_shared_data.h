/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_OBSTACLE_ONBOARD_CAMERA1_SHARED_DATA_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBOARD_CAMERA1_SHARED_DATA_H_

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/object_supplement.h"
#include "modules/perception/onboard/common_shared_data.h"

namespace apollo {
    namespace perception {

        struct ImageInfo {
            cv::Mat image_src_mat;
            SeqId seq_num = 0u;
            double timestamp = 0.0;
        };

        class Camera2SharedData : public CommonSharedData<ImageInfo> {
        public:
            Camera2SharedData() = default;
            virtual ~Camera2SharedData() = default;

            std::string name() const override {
                return "Camera1SharedData";
            }

        private:
            DISALLOW_COPY_AND_ASSIGN(Camera2SharedData);
        };

        REGISTER_SHAREDDATA(Camera2SharedData);

    }  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBOARD_OBJECT_SHARED_DATA_H
