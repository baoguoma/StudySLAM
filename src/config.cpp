#include "studyslam/config.hpp"

namespace studyslam {

void Config::setParameterFile(const std::string& filename) {
  if (config_ptr == nullptr) config_ptr = shared_ptr<Config>(new Config);
  config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
  if (config_->file_.isOpened() == false) {
    std::cerr << "parameter file " << filename << " does not exist."
              << std::endl;
    config_->file_.release();
    return;
  }
}

Config::~Config() {
  if (file_.isOpened()) file_.release();
}

shared_ptr<Config> Config::config_ptr = nullptr;

}  // namespace myslam