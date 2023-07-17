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

  /* preprocessing type */
  get_param(pnh, "preprocessing_type", preprocessing_type);
  //BIN
  get_param(pnh, "bin_r", bin_r);
  get_param(pnh, "bin_tau", bin_tau);

  //EROS: Gava, Luna, et al. "Puck: Parallel surface and convolution-kernel tracking for event-based cameras."
  get_param(pnh, "eros_k", eros_k);
  this->eros_d = std::pow(0.3, 1.0/eros_k);
  get_param(pnh, "eros_apply_gaussian_blur", eros_apply_gaussian_blur);
  //SITS
  get_param(pnh, "sits_r", sits_r);
  //luvHarris
  get_param(pnh, "luv_k_tos", luv_k_tos);

  // sub sampling method
  get_param(pnh, "sampling_method", sampling_method);
  if (sampling_method == "duration"){
    get_param(pnh, "sampling_duration", sampling_duration);
  }
  else if (sampling_method == "number"){
    get_param(pnh, "sampling_number", sampling_number);
  }
  else error(std::string() + "unknown sampling_method: " + sampling_method);

  // output of preprocessing
  get_param(pnh, "output_type", output_type);
  int output_max_seq_; get_param(pnh, "output_max_seq", output_max_seq_);
  output_max_seq = output_max_seq_;
  int output_apply_colormap_;
  get_param(pnh, "output_apply_colormap", output_apply_colormap_);
  output_apply_colormap = output_apply_colormap_ ? true : false;

  if (output_type == "image_file"){
    get_param(pnh, "image_file_output_dir", image_file_output_dir);
    get_param(pnh, "enable_gen_image_file_dir", enable_gen_image_file_dir);
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
  ros::shutdown();
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
  else if (sampling_method == "number"){
    int event_count = 1;
    event_slice.events.push_back(event_array.events[idx_start]);

    while (ros::ok() 
      && (idx_end < event_array.events.size()) // if not last
      && (++event_count <= sampling_number)) // if not enough events
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
  else if (preprocessing_type == "testSAE"){
    output.image_SAE_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_SAE_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    int R = 0;

    for(size_t i = 0; i < event_slice.events.size(); ++i){
      for(int ux = -R; ux <= R; ux++){
        for(int uy = -R; uy <= R; uy++){
          int newX = event_slice.events[i].x + ux;
          int newY = event_slice.events[i].y + uy;
          if(newX >= 0 && newX < static_cast<int>(event_slice.width) && newY >= 0 && newY < static_cast<int>(event_slice.height)){
            for(int j = i; j>=0; j--){
              if(newX == event_slice.events[j].x && newY == event_slice.events[j].y && event_slice.events[i].polarity==event_slice.events[j].polarity){
                if (event_slice.events[j].polarity) {
                  output.image_SAE_positive.at<double>(newY, newX) = (event_slice.events[j].ts-event_slice.events[0].ts).toSec();
                }
                else {
                  output.image_SAE_negative.at<double>(newY, newX) = (event_slice.events[j].ts-event_slice.events[0].ts).toSec();
                }
                break;
              }
            }
          }
        }
      }
    }
  }
  else if (preprocessing_type == "BIN"){
    output.image_BIN_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_BIN_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    double tau = 0.007;
    for(size_t i = 0; i < event_slice.events.size(); ++i){
      for(int ux = -bin_r; ux <= bin_r; ux++){
        for(int uy = -bin_r; uy <= bin_r; uy++){
          int newX = event_slice.events[i].x + ux;
          int newY = event_slice.events[i].y + uy;
          if(newX >= 0 && newX < static_cast<int>(event_slice.width) && newY >= 0 && newY < static_cast<int>(event_slice.height)){
            for(int j = i; j>=0; j--){
              double t = (event_slice.events[i].ts - event_slice.events[j].ts).toSec();
              if(newX == event_slice.events[j].x && newY == event_slice.events[j].y && event_slice.events[i].polarity==event_slice.events[j].polarity){
                if (event_slice.events[j].polarity) {
                  if (t<=tau) {
                    output.image_BIN_positive.at<double>(newY, newX) = 1;
                  }
                  else {
                    output.image_BIN_positive.at<double>(newY, newX) = 0;
                  }
                }
                else {                  
                  if (t<=tau) {
                    output.image_BIN_negative.at<double>(newY, newX) = 1;
                  }
                  else {
                    output.image_BIN_negative.at<double>(newY, newX) = 0;
                  }
                }
                break;
              }
            }
          }
        }
      }
    }
  }
  else if (preprocessing_type == "BIN2"){
    cv::Mat last_ts_pos =  cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    cv::Mat last_ts_neg =  cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_BIN_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_BIN_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);

    // update last ts
    for(size_t i = 0; i < event_slice.events.size(); ++i){
      double t = (event_slice.events[i].ts - event_slice.events[0].ts).toSec();
      if (event_slice.events[i].polarity){
        last_ts_pos.at<double>(event_slice.events[i].y, event_slice.events[i].x) = t;
      }
      else{
        last_ts_neg.at<double>(event_slice.events[i].y, event_slice.events[i].x) = t;
      }
    }

    //calc SAE considering neighbors
    // iterate last_ts map
    for (int xi = bin_r; xi < (int)event_slice.width - bin_r; ++xi){
      for (int yi = bin_r; yi < (int)event_slice.height - bin_r; ++yi){ 
        double biggest_ts_pos = 0;
        double biggest_ts_neg = 0;
        for (int xii = xi - bin_r; xii <= xi + bin_r; ++xii){ // iterate neighbors
          for (int yii = yi - bin_r; yii <= yi + bin_r; ++yii){
            if (last_ts_pos.at<double>(yi, xi) > biggest_ts_pos) biggest_ts_pos =  last_ts_pos.at<double>(yi, xi);
            if (last_ts_neg.at<double>(yi, xi) > biggest_ts_neg) biggest_ts_neg =  last_ts_neg.at<double>(yi, xi);
          }
        }
        last_ts_pos.at<double>(yi, xi) = biggest_ts_pos;
        last_ts_neg.at<double>(yi, xi) = biggest_ts_neg;
      }
    }

    // apply binary thresholding
    double ti = event_slice.events.back().ts.toSec() - event_slice.events.front().ts.toSec();
    for (int xi = bin_r; xi < (int)event_slice.width - bin_r; ++xi){
      for (int yi = bin_r; yi < (int)event_slice.height - bin_r; ++yi){ 
        if (ti - last_ts_pos.at<double>(yi, xi) <= bin_tau) output.image_BIN_positive.at<double>(yi, xi) = 1;
        else output.image_BIN_positive.at<double>(yi, xi) = 0;

        if (ti - last_ts_neg.at<double>(yi, xi) <= bin_tau) output.image_BIN_negative.at<double>(yi, xi) = 1;
        else output.image_BIN_negative.at<double>(yi, xi) = 0;
      }
    }
  }
  else if (preprocessing_type == "LIN"){
    output.image_BIN_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_BIN_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    int R = 1;
    double tau = 0.001;

    for(size_t i = 0; i < event_slice.events.size(); ++i){
      for(int ux = -R; ux <= R; ux++){
        for(int uy = -R; uy <= R; uy++){
          int newX = event_slice.events[i].x + ux;
          int newY = event_slice.events[i].y + uy;
          if(newX >= 0 && newX < static_cast<int>(event_slice.width) && newY >= 0 && newY < static_cast<int>(event_slice.height)){
            for(int j = i; j>=0; j--){ 
              double t = (event_slice.events[i].ts - event_slice.events[j].ts).toSec();
              if(newX == event_slice.events[j].x && newY == event_slice.events[j].y && event_slice.events[i].polarity==event_slice.events[j].polarity){
                if (event_slice.events[j].polarity) {
                  if (t>=tau) {
                    output.image_BIN_positive.at<double>(newY, newX) = -(1-t/tau);
                  }
                  else {
                    output.image_BIN_positive.at<double>(newY, newX) = 0;
                  }
                }
                else {                  
                  if (t>=tau) {
                    output.image_BIN_negative.at<double>(newY, newX) = -(1-t/tau);
                  }
                  else {
                    output.image_BIN_negative.at<double>(newY, newX) = 0;
                  }
                }
                break;
              }
            }
          }
        }
      }
    }
  }

  else if (preprocessing_type == "EXP"){
    output.image_BIN_positive = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    output.image_BIN_negative = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    int R = 1;
    double tau = 100;

    for(size_t i = 0; i < event_slice.events.size(); ++i){
      for(int ux = -R; ux <= R; ux++){
        for(int uy = -R; uy <= R; uy++){
          int newX = event_slice.events[i].x + ux;
          int newY = event_slice.events[i].y + uy;
          if(newX >= 0 && newX < static_cast<int>(event_slice.width) && newY >= 0 && newY < static_cast<int>(event_slice.height)){
            for(int j = i; j>=0; j--){
              double t = (event_slice.events[i].ts - event_slice.events[j].ts).toSec();
              if(newX == event_slice.events[j].x && newY == event_slice.events[j].y && event_slice.events[i].polarity==event_slice.events[j].polarity){
                if (event_slice.events[j].polarity) {
                  output.image_BIN_positive.at<double>(newY, newX) = cv::exp(-tau*t);
                }
                else {                  
                  output.image_BIN_negative.at<double>(newY, newX) = cv::exp(-tau*t);
                }
                break;
              }
            }
          }
        }
      }
    }
  }
  else if (preprocessing_type == "EROS"){ 
    static bool is_first = true;
    if (is_first){
      // output.image_eros = cv::Mat::ones(event_slice.height, event_slice.width, CV_8UC1)*255;
      output.image_eros = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
      is_first = false;
    }
    for(size_t idx_e = 0; idx_e < event_slice.events.size(); ++idx_e){
      if (event_slice.events[idx_e].x - eros_k < 0) continue;
      if (event_slice.events[idx_e].x + eros_k >= (int)event_slice.width) continue;
      if (event_slice.events[idx_e].y - eros_k < 0) continue;
      if (event_slice.events[idx_e].y + eros_k >= (int)event_slice.height) continue;

      for(int u = -eros_k; u <= eros_k; ++u){
        for(int v = -eros_k; v <= eros_k; ++v){
          output.image_eros.at<double>(
            event_slice.events[idx_e].y+u, 
            event_slice.events[idx_e].x+v) *= eros_d;
        }
      }
      output.image_eros.at<double>(
        event_slice.events[idx_e].y, 
        event_slice.events[idx_e].x
      ) = 255.0;
    }
  }
  else if (preprocessing_type == "SORT"){
    /* sort events by timestamp */
    // polarity is not considered
    using XY_COOR = std::pair<int, int>;
    
    // 1. find most recent events
    std::map<XY_COOR, double> recent_events;
    for(size_t idx_e = 1; idx_e < event_slice.events.size(); ++idx_e){ // avoid first event
    // pixels where event did not occur will be considered as first event occured
      auto coor = XY_COOR(event_slice.events[idx_e].x, event_slice.events[idx_e].y);
      recent_events[coor] = (event_slice.events[idx_e].ts - event_slice.events[0].ts).toSec();
    } // until this line, everyline can run O(1) in realtime senario

    // sort by time. ascending order
    auto cmp = [](const std::pair<XY_COOR, ros::Time>& a, const std::pair<XY_COOR, ros::Time>& b){
      return a.second < b.second;
    };
    std::vector<std::pair<XY_COOR, ros::Time>> sorted_events(recent_events.begin(), recent_events.end());
    std::sort(sorted_events.begin(), sorted_events.end(), cmp); // O(NlogN)

    // assign order
    output.image_SORT = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
    int order = 1;
    for(auto&& e = sorted_events.begin(); e != sorted_events.end(); ++e){
      output.image_SORT.at<double>(e->first.second, e->first.first) = order++; // y, x
    }
  } 
  else if (preprocessing_type == "SITS"){
    bool is_first = true;
    if (is_first){
      output.image_SITS[0] = cv::Mat::ones(event_slice.height, event_slice.width, CV_32SC1);
      output.image_SITS[1] = cv::Mat::ones(event_slice.height, event_slice.width, CV_32SC1);
      is_first = false;
    }
    for(size_t idx_e = 0; idx_e < event_slice.events.size(); ++idx_e){ 
      if (event_slice.events[idx_e].x - sits_r < 0) continue;
      if (event_slice.events[idx_e].x + sits_r >= (int)event_slice.width) continue;
      if (event_slice.events[idx_e].y - sits_r < 0) continue;
      if (event_slice.events[idx_e].y + sits_r >= (int)event_slice.height) continue;
      
      int image_index = event_slice.events[idx_e].polarity ? 0 : 1;
      int sits_value = output.image_SITS[image_index].at<int>(event_slice.events[idx_e].y, event_slice.events[idx_e].x);
      for(int xi = event_slice.events[idx_e].x - sits_r; xi <= event_slice.events[idx_e].x + sits_r; xi++){
        for(int yi = event_slice.events[idx_e].y - sits_r; yi <= event_slice.events[idx_e].y + sits_r; yi++){
          if (output.image_SITS[image_index].at<int>(yi, xi) > sits_value){
            output.image_SITS[image_index].at<int>(yi, xi) -= 1;
          }
        }
      }
      output.image_SITS[image_index].at<int>(event_slice.events[idx_e].y, event_slice.events[idx_e].x) = sits_r*sits_r+1;
    }
  }
  else if (preprocessing_type == "luvHarris"){
    bool is_first = true;
    if (is_first){
      output.image_TOS = cv::Mat::zeros(event_slice.height, event_slice.width, CV_64FC1);
      is_first = false;
    }
 
    int luv_t_tos = 2*(2*luv_k_tos+1);
    for(size_t idx_e = 0; idx_e < event_slice.events.size(); ++idx_e){ 
      if (event_slice.events[idx_e].x - luv_k_tos < 0) continue;
      if (event_slice.events[idx_e].x + luv_k_tos >= (int)event_slice.width) continue;
      if (event_slice.events[idx_e].y - luv_k_tos < 0) continue;
      if (event_slice.events[idx_e].y + luv_k_tos >= (int)event_slice.height) continue;
      
      for(int xi = event_slice.events[idx_e].x - luv_k_tos; xi <= event_slice.events[idx_e].x + luv_k_tos; xi++){
        for(int yi = event_slice.events[idx_e].y - luv_k_tos; yi <= event_slice.events[idx_e].y + luv_k_tos; yi++){
          output.image_TOS.at<double>(yi, xi) -= 1;
          if (output.image_TOS.at<double>(yi, xi) < 255 - luv_t_tos)
            output.image_TOS.at<double>(yi, xi) = 0;
        }
      }
      output.image_TOS.at<double>(event_slice.events[idx_e].y, event_slice.events[idx_e].x) = 255;
    }
  }
  else error(std::string() + "preprocessing_type: " + preprocessing_type + " is not supported");
}

void EventPreprocessor::handle_preprocessed_results(PreprocessingOutputType& output){
  if (output_type == "image_file"){
    vector<cv::Mat> images_to_be_saved;
    vector<std::string> names;

    fs::path dir(image_file_output_dir);
    if (enable_gen_image_file_dir) dir = dir / preprocessing_type;
    fs::create_directory(dir);
    std::string name;

    if (preprocessing_type == "SAE"){
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
    ////////////////////test/////////////////////////////////
    else if (preprocessing_type == "testSAE"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_tSAE_positive.png";
      fs::path png_path = dir / name;
      cv::Mat SAE_positive_8U;
      cv::normalize(output.image_SAE_positive, SAE_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(SAE_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_tSAE_negative.png";
      png_path = dir / name;
      cv::Mat SAE_negative_8U;
      cv::normalize(output.image_SAE_negative, SAE_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(SAE_negative_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "BIN"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_BIN_positive.png";
      fs::path png_path = dir / name;
      cv::Mat BIN_positive_8U;
      cv::normalize(output.image_BIN_positive, BIN_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_BIN_negative.png";
      png_path = dir / name;
      cv::Mat BIN_negative_8U;
      cv::normalize(output.image_BIN_negative, BIN_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_negative_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "BIN2"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_BIN2_positive.png";
      fs::path png_path = dir / name;
      cv::Mat BIN_positive_8U;
      cv::normalize(output.image_BIN_positive, BIN_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_BIN2_negative.png";
      png_path = dir / name;
      cv::Mat BIN_negative_8U;
      cv::normalize(output.image_BIN_negative, BIN_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_negative_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "LIN"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_LIN_positive.png";
      fs::path png_path = dir / name;
      cv::Mat BIN_positive_8U;
      cv::normalize(output.image_BIN_positive, BIN_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_LIN_negative.png";
      png_path = dir / name;
      cv::Mat BIN_negative_8U;
      cv::normalize(output.image_BIN_negative, BIN_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_negative_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "EXP"){
      fs::path dir(image_file_output_dir);
      std::string name;
      
      name = std::to_string(output.seq) + "_EXP_positive.png";
      fs::path png_path = dir / name;
      cv::Mat BIN_positive_8U;
      cv::normalize(output.image_BIN_positive, BIN_positive_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_positive_8U);
      names.push_back(png_path.string());

      name = std::to_string(output.seq) + "_EXP_negative.png";
      png_path = dir / name;
      cv::Mat BIN_negative_8U;
      cv::normalize(output.image_BIN_negative, BIN_negative_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      images_to_be_saved.push_back(BIN_negative_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "EROS"){
      cv::Mat eros_8U;
      output.image_eros.copyTo(eros_8U);
      if (eros_apply_gaussian_blur){
        cv::GaussianBlur(eros_8U, eros_8U, cv::Size(eros_k, eros_k), 1);
        cv::normalize(eros_8U, eros_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        name = std::to_string(output.seq) + "_EROS_gaussian.png";
      }
      else {
        name = std::to_string(output.seq) + "_EROS.png";
      }
      fs::path png_path = dir / name;
      images_to_be_saved.push_back(eros_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "SORT"){
      cv::Mat sort_8U;
      cv::normalize(output.image_SORT, sort_8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      name = std::to_string(output.seq) + "_SORT.png";
      fs::path png_path = dir / name;
      images_to_be_saved.push_back(sort_8U);
      names.push_back(png_path.string());
    }
    else if (preprocessing_type == "SITS"){
      cv::Mat sits_8U[2];
      
      cv::normalize(output.image_SITS[0], sits_8U[0], 0, 255, cv::NORM_MINMAX, CV_8UC1);
      name = std::to_string(output.seq) + "_SITS_positive.png";
      fs::path png_path1 = dir / name;
      images_to_be_saved.push_back(sits_8U[0]);
      names.push_back(png_path1.string());
      
      cv::normalize(output.image_SITS[1], sits_8U[1], 0, 255, cv::NORM_MINMAX, CV_8UC1);
      name = std::to_string(output.seq) + "_SITS_negative.png";
      fs::path png_path2 = dir / name;
      images_to_be_saved.push_back(sits_8U[1]);
      names.push_back(png_path2.string());
    }
    else if (preprocessing_type == "luvHarris"){
      cv::Mat luvHarris;
      output.image_TOS.copyTo(luvHarris);

      cv::normalize(output.image_TOS, luvHarris, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      name = std::to_string(output.seq) + "_luvHarris.png";
      fs::path png_path = dir / name;
      images_to_be_saved.push_back(luvHarris);
      names.push_back(png_path.string());
    }
    else error(std::string() + "preprocessing_type: " + preprocessing_type + " is not supported");

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
      cout << "saved " << names[i] << endl;
    }
  }
  else error(std::string() + "output_type: " + output_type + " is not supported");
}
