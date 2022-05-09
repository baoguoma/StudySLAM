#ifndef VISUALODOMETRY_HEAD
#define VISUALODOMETRY_HEAD

#include <opencv2/features2d/features2d.hpp>

#include "studyslam/common_include.hpp"
#include "studyslam/map.hpp"

namespace studyslam {
class VisualOdometry {
 public:
  typedef shared_ptr<VisualOdometry> sh_ptr;
  enum VOState { INITIALIZING = -1, OK = 0, LOST };

  VOState state_;           // current VO status
  Map::sh_ptr sh_ptr_map_;  // map with all frames and map points

  Frame::sh_ptr sh_ptr_ref_;   // reference key-frame
  Frame::sh_ptr sh_ptr_curr_;  // current frame

  cv::Ptr<cv::ORB> ptr_orb_;                 // orb detector and computer
  vector<cv::KeyPoint> vec_keypoints_curr_;  // keypoints in current frame
  Mat mat_descriptors_curr_;                 // descriptor in current fram

  cv::FlannBasedMatcher matcher_flann_;       // flann matcher
  vector<MapPoint::sh_ptr> vec_match_3dpts_;  // matched 3d points
  vector<int> vec_match_2dkp_index_;  // matched 2d pixels (index of kp_curr)

  SE3 T_w2c_estimated_;  // the estimated pose of current frame
  int num_inliers_;      // number of inlier features in icp
  int num_lost_;         // number of lost times

  int num_of_features_;           // number of features
  double scale_factor_;           // scale in image pyramid
  int level_pyramid_;             // number of pyramid levels
  float match_ratio_;             // ratio for selecting  good matches
  int max_num_lost_;              // max number of continuous lost times
  int min_inliers_;               // minimum inliers
  double key_frame_min_rot;       // minimal rotation of two key-frames
  double key_frame_min_trans;     // minimal translation of two key-frames
  double map_point_erase_ratio_;  // remove map point ratio

 public:
  VisualOdometry();
  ~VisualOdometry();

  bool addFrame(Frame::sh_ptr sh_ptr_frame);

 protected:
  // inner operation
  void extractKeyPoints();
  void computeDescriptors();
  void featureMatching();
  void poseEstimationPnP();
  void optimizeMap();

  void addKeyFrame();
  void addMapPoints();
  bool checkEstimatedPose();
  bool checkKeyFrame();

  double getViewAngle(Frame::sh_ptr frame, MapPoint::sh_ptr point);
};
}  // namespace studyslam

#endif