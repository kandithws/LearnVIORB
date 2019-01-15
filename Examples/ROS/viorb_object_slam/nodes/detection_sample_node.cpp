#include <ros/ros.h>
#include <dnn/CVObjectDetector.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "detection_sample");
  ORB_SLAM2::CVObjectDetector detector("", "", CV_DNN_FRAMEWORK_DARKNET);
}