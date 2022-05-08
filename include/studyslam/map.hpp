#ifndef MAP_HEAD
#define MAP_HEAD

#include "studyslam/common_include.hpp"
#include "studyslam/frame.hpp"
#include "studyslam/mappoint.hpp"

namespace studyslam {
class Map {
 public:
  typedef shared_ptr<Map> sh_ptr;
  unordered_map<unsigned long, MapPoint::sh_ptr> map_points_;  // all landmarks
  unordered_map<unsigned long, Frame::sh_ptr> keyframes_;      // all key-frames

  Map() {}

  void insertKeyFrame(Frame::sh_ptr frame);
  void insertMapPoint(MapPoint::sh_ptr map_point);
};
}  // namespace studyslam
#endif