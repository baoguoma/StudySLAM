#ifndef MAPPOINT_HEAD
#define MAPPOINT_HEAD

#include "studyslam/common_include.hpp"

namespace myslam {

class Frame;
class MapPoint {
 public:
  typedef shared_ptr<MapPoint> sh_ptr;
  unsigned long id_;                 // ID
  static unsigned long factory_id_;  // factory id
  bool good_;                        // wheter a good point
  Vector3d pos_;                     // Position in world
  Vector3d norm_;                    // Normal of viewing direction
  Mat descriptor_;                   // Descriptor for matching

  list<Frame*> observed_frames_;  // key-frames that can observe this point

  int matched_times_;  // being an inliner in pose estimation
  int visible_times_;  // being visible in current frame

  MapPoint();
  MapPoint(unsigned long id, const Vector3d& position, const Vector3d& norm,
           Frame* frame = nullptr, const Mat& descriptor = Mat());

  inline cv::Point3f getPositionCV() const {
    return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
  }

  static MapPoint::sh_ptr createMapPoint();
  static MapPoint::sh_ptr createMapPoint(const Vector3d& pos_world,
                                         const Vector3d& norm_,
                                         const Mat& descriptor, Frame* frame);
};
}  // namespace myslam

#endif  // MAPPOINT_H