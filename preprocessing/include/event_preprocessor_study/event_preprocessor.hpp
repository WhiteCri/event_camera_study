#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>

// this runs at offline (not real-time)
// for real_time purposes, use core functions like preprocess_events(), ...
class EventPreprocessor{
  struct PreprocessingOutputType{
    PreprocessingOutputType(): seq(0){}
    size_t seq;
    cv::Mat image_SAE_positive, image_SAE_negative, image_BIN_positive, image_BIN_negative; //todo: add variables with its own purpose
    cv::Mat image_eros;
    cv::Mat image_SORT;
    cv::Mat image_SITS[2]; // 0: positive, 1: negative
    cv::Mat image_TOS; // luvHarris
  };
public:
  EventPreprocessor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void load_ros_parameters(ros::NodeHandle& pnh);
  void organize_preprocessing_logic(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // input type: "file"
  void parse_event_txt_file(std::string event_txt_path, dvs_msgs::EventArray& event_array_from_file);

  // preprocessing
  void events_callback(const ros::TimerEvent&); // if output_type == "ros"
  void preprocess_events(dvs_msgs::EventArray& event_slice, std::string preprocess_type, PreprocessingOutputType& output);
  void handle_preprocessed_results(PreprocessingOutputType& output);

  // sampler
  bool sample_events(dvs_msgs::EventArray& event_slice, std::string sample_type, bool on_start=true); // return if sample is last one

  void run();

private:
  std::string input_type, output_type;
  
  // input related
  std::string event_txt_path;
  dvs_msgs::EventArray event_array;

  // preprocessing and sampling related
  std::string sampling_method, preprocessing_type;
  double sampling_duration;
  int sampling_number;

  /* preprocessing parameters */
  //BIN
  int bin_r;
  double bin_tau;
  //EROS
  double eros_d;
  int eros_k;
  int eros_apply_gaussian_blur;
  //SITS
  int sits_r;
  //luvHarris
  int luv_k_tos;

  // output related
  std::string image_file_output_dir;
  size_t output_max_seq;
  bool output_apply_colormap;
  int enable_gen_image_file_dir;
};

/*
  opencv get minmax
        double minVal; 
      double maxVal; 
      cv::Point minLoc; 
      cv::Point maxLoc;

      minMaxLoc(output.image_SAE_negative, &minVal, &maxVal, &minLoc, &maxLoc );
      cout << "pos max, min: " << maxVal << ' ' << minVal << endl;
      minMaxLoc(SAE_negative_8U, &minVal, &maxVal, &minLoc, &maxLoc );
      cout << "pos8U max, min: " << maxVal << ' ' << minVal << endl;
*/