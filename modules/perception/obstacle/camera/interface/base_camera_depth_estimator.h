
// @brief: The base class of camera #d depth estimator

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DEPTH_ESTIMATOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DEPTH_ESTIMATOR_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/camera.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"

namespace apollo {
namespace perception {

struct CameraDepthEstimatorInitOptions {
  std::shared_ptr<CameraDistortD> intrinsic;
};

struct CameraDepthEstimatorOptions {
  cv::Mat gray_frame;
  cv::Mat range_frame;
  std::shared_ptr<CameraDistortD> intrinsic;
  std::shared_ptr<Eigen::Matrix4d> extrinsic_ground2camera;
  std::shared_ptr<Eigen::Matrix4d> extrinsic_stereo;
};

class BaseCameraDepthEstimator {
 public:
  BaseCameraDepthEstimator() {}
  virtual ~BaseCameraDepthEstimator() {}

  virtual bool Init(const CameraDepthEstimatorInitOptions& options =
                        CameraDepthEstimatorInitOptions()) = 0;

  // @brief: Estimate Depth on image from camera
  // @param [in]: image frame from camera
  // @param [in/out]: disparity map
  virtual bool CalculateDepth(const cv::Mat& frame,
                      const CameraDepthEstimatorOptions& options,
                      std::vector<std::shared_ptr<VisualObject>>* objects) = 0;

  virtual bool Multitask(const cv::Mat& frame,
                         const CameraDepthEstimatorOptions& options,
                         std::vector<std::shared_ptr<VisualObject>>* objects,
                         cv::Mat* mask) {
    return true;
  }

  // @brief: Extract deep learning ROI features for each object
  // @param [in/out]: detected objects, with 2D bbox and its features
  //virtual bool Extract(std::vector<std::shared_ptr<VisualObject>>* objects) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseCameraDepthEstimator);
};

REGISTER_REGISTERER(BaseCameraDepthEstimator);
#define REGISTER_CAMERA_DEPTH_ESTIMATOR(name) REGISTER_CLASS(BaseCameraDepthEstimator, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DEPTH_ESTIMATOR_H_
