#include "studyslam/frame.hpp"

namespace studyslam {
Frame::Frame()
    : frame_id_(-1),
      frame_time_stamp_(-1),
      sh_ptr_camera_(nullptr),
      is_key_frame_(false) {}

Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::sh_ptr camera,
             Mat color, Mat depth)
    : id_(id),
      time_stamp_(time_stamp),
      T_c_w_(T_c_w),
      sh_pre_camera_(camera),
      color_(color),
      depth_(depth),
      is_key_frame_(false) {}

Frame::~Frame() {}

Frame::sh_ptr Frame::createFrame() {
  static long factory_id = 0;
  Frame::sh_ptr ptr(new Frame(factory_id++));
  return ptr;
}

double Frame::findDepth(const cv::KeyPoint& kp) {
  int x = cvRound(kp.pt.x);
  int y = cvRound(kp.pt.y);
  ushort d = mat_depth_.ptr<ushort>(y)[x];
  if (d != 0) {
    return double(d) / sh_ptr_camera_->depth_scale_;
  } else {
    // check the nearby points
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, -1, 0, 1};
    for (int i = 0; i < 4; i++) {
      d = mat_depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
      if (d != 0) {
        return double(d) / sh_ptr_camera_->depth_scale_;
      }
    }
  }
  return -1.0;
}

void Frame::setPose(const SE3& T_w2c) { T_w2c = T_w2c; }

Vector3d Frame::getCamCenter() const { return T_w2c.inverse().translation(); }

bool Frame::isInFrame(const Vector3d& pt_world) {
  Vector3d p_cam = sh_ptr_camera_->world2camera(pt_world, T_w2c);
  // cout<<"P_cam = "<<p_cam.transpose()<<endl;
  if (p_cam(2, 0) < 0) return false;
  Vector2d pixel = sh_ptr_camera_->world2pixel(pt_world, T_w2c);
  // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
  return pixel(0, 0) > 0 && pixel(1, 0) > 0 && pixel(0, 0) < mat_color_.cols &&
         pixel(1, 0) < mat_color_.rows;
}
}  // namespace studyslam