#ifndef CONFIG_HEAD
#define CONFIG_HEAD

#include "studyslam/common_include.hpp"

namespace studyslam {
class Config {
 private:
  static std::shared_ptr<Config> config_ptr;
  cv::FileStorage file_;

  Config() {}  // private constructor makes a singleton
 public:
  ~Config();  // close the file when deconstructing

  // set a new config file
  static void setParameterFile(const std::string& filename);

  // access the parameter values
  template <typename T>
  static T get(const std::string& key) {
    return T(Config::config_ptr->file_[key]);
  }
}
}  // namespace studyslam

#endif