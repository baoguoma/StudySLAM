#include "studyslam/visual_odometry.hpp"

#include <algorithm>
#include <boost/timer.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "studyslam/config.hpp"
#include "studyslam/g2o_types.hpp"

namespace studyslam {
VisualOdometry::VisualOdometry()
    : state_(INITIALIZING),
      ref_(nullptr),
      curr_(nullptr),
      map_(new Map),
      num_lost_(0),
      num_inliers_(0),
      matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2)) {
  num_of_features_ = Config::get<int>("number_of_features");
  scale_factor_ = Config::get<double>("scale_factor");
  level_pyramid_ = Config::get<int>("level_pyramid");
  match_ratio_ = Config::get<float>("match_ratio");
  max_num_lost_ = Config::get<float>("max_num_lost");
  min_inliers_ = Config::get<int>("min_inliers");
  key_frame_min_rot = Config::get<double>("keyframe_rotation");
  key_frame_min_trans = Config::get<double>("keyframe_translation");
  map_point_erase_ratio_ = Config::get<double>("map_point_erase_ratio");
  ptr_orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry() {}

visual_odometry::~visual_odometry() {}

bool VisualOdometry::addFrame(Frame::Ptr frame) {
  switch (state_) {
    case IINITIALIZING: {
      state_ = OK;
      sh_ptr_curr_ = sh_ptr_ref_ = frame;
      extractKeyPoints();
      computeDescriptors();
      addKeyFrame();  // the first frame is a key-frame
      break;
    }
    case OK: {
      sh_ptr_curr_ = frame;
      sh_ptr_curr_->T_w2c_ = ref_->T_w2c_;
      extractKeyPoints();
      computeDescriptors();
      featureMatching();
      poseEstimationPnP();
      if (checkEstimatedPose() == true)  // a good estimation
      {
        curr_->T_c_w_ = T_c_w_estimated_;
        optimizeMap();
        num_lost_ = 0;
        if (checkKeyFrame() == true)  // is a key-frame
        {
          addKeyFrame();
        }
      } else  // bad estimation due to various reasons
      {
        num_lost_++;
        if (num_lost_ > max_num_lost_) {
          state_ = LOST;
        }
        return false;
      }
      break;
    }
    case LOST: {
      cout << "vo has lost." << endl;
      break;
    }
  }
  return true;
}

void VisualOdometry::extractKeyPoints() {
  boost::timer timer;
  orb_->detect(curr_->color_, keypoints_curr_);
  cout << "extract keypoints cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::computeDescriptors() {
  boost::timer timer;
  orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
  cout << "descriptor computation cost time: " << timer.elapsed() << endl;
}

void VisualOdometry::featureMatching() {
  boost::timer timer;
  vector<cv::DMatch> matches;
  // select the candidates in map
  Mat desp_map;
  vector<MapPoint::Ptr> candidate;
  for (auto& allpoints : map_->map_points_) {
    MapPoint::Ptr& p = allpoints.second;
    // check if p in curr frame image
    if (curr_->isInFrame(p->pos_)) {
      // add to candidate
      p->visible_times_++;
      candidate.push_back(p);
      desp_map.push_back(p->descriptor_);
    }
  }

  matcher_flann_.match(desp_map, descriptors_curr_, matches);
  // select the best matches
  float min_dis =
      std::min_element(matches.begin(), matches.end(),
                       [](const cv::DMatch& m1, const cv::DMatch& m2) {
                         return m1.distance < m2.distance;
                       })
          ->distance;

  match_3dpts_.clear();
  match_2dkp_index_.clear();
  for (cv::DMatch& m : matches) {
    if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
      match_3dpts_.push_back(candidate[m.queryIdx]);
      match_2dkp_index_.push_back(m.trainIdx);
    }
  }
  cout << "good matches: " << match_3dpts_.size() << endl;
  cout << "match cost time: " << timer.elapsed() << endl;
}

}  // namespace studyslam
