#include <ros/ros.h>
#include <memory>
#include <dnn/CVObjectDetector.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class CVObjectDetectorNode {
  public:
    CVObjectDetectorNode() :
        nh_("~")
    {
      std::string model_path, config_path, label_map_path;
      nh_.param<std::string>("model_path", model_path, "");
      nh_.param<std::string>("config_path", config_path, "");
      // nh_.param<std::string>("label_map_path", label_map_path);
      double conf_th, nms_th;
      bool apply_nms;
      nh_.param("confidence_threshold", conf_th, 0.5);
      nh_.param("nms_threshold", nms_th, 0.4);
      nh_.param("apply_nms", apply_nms, true);
      detector_ = std::make_shared<ORB_SLAM2::CVObjectDetector>(model_path, config_path, CV_DNN_FRAMEWORK_DARKNET);
      detector_->setConfidenceThreshold(conf_th);
      detector_->setApplyNMS(apply_nms);
      detector_->setNMSThreshold(nms_th);
      if (nh_.getParam("label_map_path", label_map_path)){
        detector_->parseLabelMap(label_map_path, '\n');
      }
      detector_->setInputSize(416);
      img_sub_ = nh_.subscribe("/camera0/image_raw", 1, &CVObjectDetectorNode::imgCallback, this);
      result_pub_ = nh_.advertise<sensor_msgs::Image>("/detection_out", 1);
      ROS_INFO("-------- Start detection----------");
      ros::spin();
    }

    void imgCallback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      std::vector<ORB_SLAM2::PredictedObject> preds;
      detector_->detectObject(cv_ptr->image, preds, true); //given as rgb
      detector_->drawPredictionBoxes(cv_ptr->image, preds);
      result_pub_.publish(cv_ptr->toImageMsg());
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<ORB_SLAM2::CVObjectDetector> detector_;
    ros::Subscriber img_sub_;
    ros::Publisher result_pub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "detection_sample");
  CVObjectDetectorNode node;
}