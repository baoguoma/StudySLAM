#include "studyslam/map.hpp"

namespace studyslam {
void Map::insertKeyFrame(Frame::sh_ptr sh_ptr_frame) {
  cout << "Key frame size = " << keyframes_.size() << endl;
  if (keyframes_.find(sh_ptr_frame->id_) == keyframes_.end()) {
    keyframes_.insert(make_pair(sh_ptr_frame->id_, frame));
  } else {
    keyframes_[sh_ptr_frame->id_] = frame;
  }
}

void Map::insertMapPoint(MapPoint::Ptr sh_ptr_map_point) {
  if (map_points_.find(sh_ptr_map_point->id_) == map_points_.end()) {
    map_points_.insert(make_pair(sh_ptr_map_point->id_, map_point));
  } else {
    map_points_[sh_ptr_map_point->id_] = map_point;
  }
}

}  // namespace studyslam