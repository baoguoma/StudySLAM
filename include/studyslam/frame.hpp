#ifndef FRAME_HEAD
#define FRAME_HEAD

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace studyslam {
class MapPoint;
class Frame {
 public:
  typedef std::shared_ptr<Frame> sh_ptr;
  unsigned long frame_id_;        // id of this frame
  double frame_time_stamp_;       // time stamp of this frame
  SE3 T_w2c;                      // transform from world to camera
  Camera::sh_ptr sh_ptr_camera_;  // camera model
  Mat mat_color_, mat_depth_;     // color and depth image;
  bool is_key_frame_;

 public:  // data members
  Frame();
  Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(),
        Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
  ~Frame();

  static Frame::Ptr createFrame();

  // find the depth in depth map
  double findDepth(const cv::KeyPoint& kp);

  // Get Camera Center
  Vector3d getCamCenter() const;

  void setPose(const SE3& T_c_w);

  // check if a point is in this frame
  bool isInFrame(const Vector3d& pt_world);
};
}  // namespace studyslam
#endif