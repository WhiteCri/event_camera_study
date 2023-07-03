#include <ros/ros.h>
#include <event_preprocessor_study/event_preprocessor.hpp>

using std::cout;
using std::endl;

int main(int argc, char *argv[]){
  ros::init(argc, argv, "event_preprocessor_ros");
  ros::NodeHandle nh, pnh("~");

  EventPreprocessor Event_preprocessor(nh, pnh);
  
  cout << "running... " << endl;
  Event_preprocessor.run();

  ros::spin();
}