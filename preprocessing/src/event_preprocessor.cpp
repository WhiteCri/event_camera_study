#include <ros/ros.h>
#include <event_preprocessor_study/event_preprocessor.hpp>
#include <fstream>
#include <filesystem>

using std::cout;
using std::endl;
using std::vector;
using std::string;
namespace fs = std::filesystem;

template <typename T>
static void get_param(ros::NodeHandle& nh, std::string name, T& v){
  if (false == nh.getParam(name, v)){
    std::string err =  std::string() + "param " + name + " is not set";
    throw std::runtime_error(err.c_str());
  }
}

static void error(std::string msg){
  std::string err =  std::string() + "error: " + msg;
  throw std::runtime_error(err.c_str());
}

EventPreprocessor::EventPreprocessor(ros::NodeHandle& nh, ros::NodeHandle& pnh){
  load_ros_parameters(pnh);
  
  if (input_type == "file"){
    parse_event_txt_file(event_txt_path, event_array);
  }
  else{
    error(std::string() + "input_type: " + input_type + " is not supported");
  }

  organize_preprocessing_logic(nh, pnh);
}

void EventPreprocessor::load_ros_parameters(ros::NodeHandle& pnh){
  // camera info 
  int width, height;
  get_param(pnh, "ecam_width", width);
  get_param(pnh, "ecam_height", height);
  event_array.height  = height;
  event_array.width   = width;

  // input type
  get_param(pnh, "input_type", input_type);
  if (input_type == "file"){
    get_param(pnh, "event_txt_path", event_txt_path);
  }

  // preprocessing typoe
  get_param(pnh, "preprocessing_type", preprocessing_type);

  // sub sampling method
  get_param(pnh, "sampling_method", sampling_method);
  if (sampling_method == "duration"){
    get_param(pnh, "sampling_duration", sampling_duration);
  }

  // output of preprocessing
  get_param(pnh, "output_type", output_type);
  int output_max_seq_; get_param(pnh, "output_max_seq", output_max_seq_);
  output_max_seq = output_max_seq_;
  int output_apply_colormap_;
  get_param(pnh, "output_apply_colormap", output_apply_colormap_);
  output_apply_colormap = output_apply_colormap_ ? true : false;

  if (output_type == "image_file"){
    get_param(pnh, "image_file_output_dir", image_file_output_dir);
  }
}

void EventPreprocessor::parse_event_txt_file(
  std::string event_txt_path, dvs_msgs::EventArray& event_array)
{
  std::ifstream event_txt_file(event_txt_path);
  if (false == event_txt_file.is_open())
    error(std::string() + "failed to open: " + event_txt_path);

  dvs_msgs::Event ev;
  double t;
  int pola;
  while(false == event_txt_file.eof()){
    event_txt_file >> t >> ev.x >> ev.y >> pola;

    ev.ts.fromSec(t);
    ev.polarity = pola ? 1 : 0;
    event_array.events.push_back(ev);
  }

  // size check
  if (event_array.events.size() == 0)
    error(std::string() + "event_array_from_file.events.size() == 0");

  // check timing violation: only for debugging some files
  // for(size_t i = 0; i < event_array_from_file.events.size()-1; ++i){
  //   if (event_array_from_file.events[i].ts > event_array_from_file.events[i+1].ts){
  //     error(std::string() + "event_array_from_file[i].ts > event_array_from_file[i+1].ts");
  //   }
  //   if (false == ros::ok()) return; // for fast aborting
  // }

  cout << "-------------------------- statistics -----------------------------------" << endl;
  cout << "# of events: " << event_array.events.size() << endl;
  cout << "min timestamp: " << event_array.events.front().ts << endl;
  cout << "max timestamp: " << event_array.events.back().ts << endl;
  cout << "-------------------------------------------------------------------------" << endl;
}

void EventPreprocessor::organize_preprocessing_logic(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  if (input_type == "file"){
    if (output_type == "image_file"){
      // nothing to do here
    }
    else if (output_type == "ros"){
      // register callback...
      // register ros publisher...
    }
    else{
      error(std::string() + "output_type: " + output_type + " is not supported");
    }
  }
  else{
    error(std::string() + "input_type: " + input_type + " is not supported");
  }
  
}

void EventPreprocessor::run(){
  // this function assue event_array is already filled
  if (0 == event_array.events.size()) error("event_array_from_file.events.size() == 0"); 

  if (output_type == "image_file"){
    dvs_msgs::EventArray sub_event_array;
    PreprocessingOutputType preprocessing_output;

    bool is_last = false;
    bool on_start = true;
    while(false == is_last){
      is_last = sample_events(sub_event_array, sampling_method, on_start); on_start = false;
      cout << "length of sub_event_array: " << sub_event_array.events.size() << endl;
      preprocess_events(sub_event_array, preprocessing_type, preprocessing_output);
      handle_preprocessed_results(preprocessing_output);

      if (preprocessing_output.seq >= output_max_seq) break;
    }
  }
  else error(std::string() + "output_type: " + output_type + " is not supported");
  
  cout << "preprocessing finished" << endl;
}

bool EventPreprocessor::sample_events(dvs_msgs::EventArray& event_slice, std::string sample_type, bool on_start){
  static size_t idx_start, idx_end;
  if (on_start){
    idx_start = 0;
  }
  else{
    idx_start = idx_end + 1;
  }
  idx_end = idx_start + 1;

  event_slice.events.clear();
  event_slice.height  = event_array.height;
  event_slice.width   = event_array.width;
  
  if (sampling_method == "duration"){
    ros::Time start = event_array.events[idx_start].ts;
    event_slice.events.push_back(event_array.events[idx_start]);

    while (ros::ok() 
      && (idx_end < event_array.events.size()) // if not last
      && ((event_array.events[idx_end].ts - start).toSec() <= sampling_duration)) // if duration length is not enough
    { 
      event_slice.events.push_back(event_array.events[idx_end++]);
    }

    if (idx_end == event_array.events.size()) return true; // is last
    else return false; // is not last
  }
  else error(std::string() + "sampling_method: " + sampling_method + " is not supported");
  
  return false; // control not reaches here but in order to suppress compiler warnings..
}

void EventPreprocessor::preprocess_events(
  dvs_msgs::EventArray& event_slice, 
  std::string preprocess_type, 
  PreprocessingOutputType& output)
{
  output.seq++;
  if (preprocessing_type == "SAE"){ // normalized by very first time
    output.image_SAE_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_SAE_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);

    for(size_t i = 0; i < event_slice.events.size(); ++i){
      double t = (event_slice.events[i].ts - event_slice.events[0].ts).toSec();
      if (event_slice.events[i].polarity){
        output.image_SAE_positive.at<double>(event_slice.events[i].y, event_slice.events[i].x) = t;
      }
      else{
        output.image_SAE_negative.at<double>(event_slice.events[i].y, event_slice.events[i].x) = t;
      }
    }
  }
  else error(std::string() + "preprocessing_type: " + preprocessing_type + " is not supported");
}

void EventPreprocessor::handle_preprocessed_results(PreprocessingOutputType& output){
  if (output_type == "image_file"){
    vector<cv::Mat> images_to_be_saved;
    vector<std::string> names;
    if (preprocessing_type == "SAE"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_SAE_positive.png";
      fs::path png_path = dir / name;
      cv::Mat SAE_positive_8U;
      cv::normalize(output.image_SAE_positive, SAE_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(SAE_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_SAE_negative.png";
      png_path = dir / name;
      cv::Mat SAE_negative_8U;
      cv::normalize(output.image_SAE_negative, SAE_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(SAE_negative_8U);
      names.push_back(png_path.string());
    }

    if (images_to_be_saved.size() != names.size()) error("images_to_be_saved.size() != names.size()");
    size_t n_images = images_to_be_saved.size();

    for (size_t i = 0; i < n_images; ++i){
      cv::Mat* imgp, img;
      if (false == output_apply_colormap){
        imgp = &images_to_be_saved[i];
      }
      else {
        cv::applyColorMap(images_to_be_saved[i], img, cv::COLORMAP_PINK);
        imgp = &img;
      }
      cv::imwrite(names[i], *imgp);
    }
  }
  else error(std::string() + "output_type: " + output_type + " is not supported");
}