
#ifndef IMAGE_PUBLISHER_NODE_FLAREN_DEPTH_ESTIMATOR_H
#define IMAGE_PUBLISHER_NODE_FLAREN_DEPTH_ESTIMATOR_H

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/cuda_util/network.h"
#include "modules/perception/cuda_util/region_output.h"
#include "modules/perception/cuda_util/util.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/interface/base_camera_depth_estimator.h"

namespace apollo {
    namespace perception {

        class FlarenDepthEstimator : public BaseCameraDepthEstimator {
        public:
            FlarenDepthEstimator() : BaseCameraDepthEstimator() {}

            virtual ~FlarenDepthEstimator() {}

            bool Init(const CameraDepthEstimatorInitOptions &options =
            CameraDepthEstimatorInitOptions()) override;

            bool CalculateDepth(const cv::Mat &frame, const CameraDepthEstimatorOptions &options,
                        std::vector<std::shared_ptr<VisualObject>> *objects) override;

            bool Multitask(const cv::Mat &frame, const CameraDetectorOptions &options,
                           std::vector<std::shared_ptr<VisualObject>> *objects,
                           cv::Mat *mask);

            std::string Name() const override;

        protected:

            void load_intrinsic(const CameraDepthEstimatorInitOptions &options);

            void init_anchor(const std::string &yolo_root);

            bool get_objects_cpu(std::vector<std::shared_ptr<VisualObject>> *objects);
            bool get_objects_gpu(std::vector<std::shared_ptr<VisualObject>> *objects);

        private:
            int height_ = 0;
            int width_ = 0;
            float min_2d_height_ = 0.0f;
            float min_3d_height_ = 0.0f;
            int top_k_ = 1000;
            int obj_size_ = 0;
            int output_height_ = 0;
            int output_width_ = 0;
            int lane_output_height_ = 0;
            int lane_output_width_ = 0;
            int num_anchors_ = 10;


            std::vector<ObjectType> types_;
            int offset_y_ = 0;

            float inter_cls_nms_thresh_ = 1.0f;
            float cross_class_merge_threshold_ = 1.0f;
            float confidence_threshold_ = 0.1f;
            int image_height_ = 0;
            int image_width_ = 0;

            // parameters for lane detection
            float confidence_threshold_lane_ = 0.95;
            int offset_y_lane_ = 0;
            int lane_output_height_lane_ = 0;
            int lane_output_width_lane_ = 0;
            int ignored_height_ = 0;
        };

        REGISTER_CAMERA_DEPTH_ESTIMATOR(FlarenDepthEstimator);

    }  // namespace perception
}  // namespace apollo

#endif  //
