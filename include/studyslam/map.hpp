#ifndef MAP_HEAD
#define MAP_HEAD

#include "studyslam/common_include.h"
#include "studyslam/frame.h"
#include "studyslam/mappoint.h"

namespace studyslam {
class Map {
 public:
  typedef shared_ptr<Map> sh_ptr;
  unordered_map<unsigned long, MapPoint::Ptr> map_points_;  // all landmarks
  unordered_map<unsigned long, Frame::Ptr> keyframes_;      // all key-frames

  Map() {}

  void insertKeyFrame(Frame::Ptr frame);
  void insertMapPoint(MapPoint::Ptr map_point);
};
}  // namespace studyslam
#endif