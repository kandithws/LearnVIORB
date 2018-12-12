/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "MsgSync/MsgSynchronizer.h"

#include "../../../src/IMU/imudata.h"
#include "../../../src/IMU/configparam.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

using namespace std;

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::ImageConstPtr &msg);

  ORB_SLAM2::System *mpSLAM;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "VI_ORBSLAM");
  //ros::start();
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  /*
  if (argc != 3) {
    cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
    ros::shutdown();
    return 1;
  }
   */
  std::string vocab_path, config_path, bagfile_path;
  bool realtime_mode = true;

  pnh.param<std::string>("vocab_path", vocab_path, "Vocabulary/ORBvoc.bin");
  pnh.param<std::string>("config_path", config_path, "config/euroc.yaml");

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(vocab_path, config_path, ORB_SLAM2::System::MONOCULAR, true);

  ORB_SLAM2::ConfigParam config(config_path);

  /**
   * @brief added data sync
   */
  double imageMsgDelaySec = config.GetImageDelayToIMU();
  ORBVIO::MsgSynchronizer msgsync(imageMsgDelaySec);

  ros::Subscriber imagesub;
  ros::Subscriber imusub;

  // Add this section to directly config from launch files instead of config
  if (!pnh.getParam("realtime_mode", realtime_mode)) {
    ROS_WARN("Realtime mode config is not given by ROS param, reading through VI-ORB Config");
    realtime_mode = ORB_SLAM2::ConfigParam::GetRealTimeFlag();
    
  }
  else{
    // Overiding VI-ORB file config with ROS Config (to use with internal SLAM classes)
    ORB_SLAM2::ConfigParam::SetRealTimeFlag(realtime_mode);
  }

  if (!pnh.getParam("bagfile_path", bagfile_path) && !realtime_mode) {
    bagfile_path = config._bagfile;
  }

  if (realtime_mode) {
    imagesub = nh.subscribe(config._imageTopic, /*200*/ 2, &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
    imusub = nh.subscribe(config._imuTopic, 200, &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
  }
  sensor_msgs::ImageConstPtr imageMsg;
  std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

  // 3dm imu output per g. 1g=9.80665 according to datasheet
  const double g3dm = 9.80665;
  const bool bAccMultiply98 = config.GetAccMultiply9p8();

  ros::Rate r(1000);

  if (!realtime_mode) {
    std::string bagfile = bagfile_path;
    ROS_WARN("Run Offline Mode:: Bagfile=%s", bagfile.c_str());
    rosbag::Bag bag;
    bag.open(bagfile, rosbag::bagmode::Read);
    ROS_WARN("---START----");
    std::vector<std::string> topics;
    std::string imutopic = config._imuTopic;
    std::string imagetopic = config._imageTopic;
    topics.push_back(imagetopic);
    topics.push_back(imutopic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    //while(ros::ok())
    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
            if (simu != NULL)
              msgsync.imuCallback(simu);
            sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if (simage != NULL)
              msgsync.imageCallback(simage);
            bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);

            if (bdata) {
              std::vector<ORB_SLAM2::IMUData> vimuData;
              //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
              for (unsigned int i = 0; i < vimuMsg.size(); i++) {
                sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                double ax = imuMsg->linear_acceleration.x;
                double ay = imuMsg->linear_acceleration.y;
                double az = imuMsg->linear_acceleration.z;
                if (bAccMultiply98) {
                  ax *= g3dm;
                  ay *= g3dm;
                  az *= g3dm;
                }
                ORB_SLAM2::IMUData
                    imudata(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z,
                            ax, ay, az, imuMsg->header.stamp.toSec());
                vimuData.push_back(imudata);
                //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
              }

              // Copy the ros image message to cv::Mat.
              cv_bridge::CvImageConstPtr cv_ptr;
              try {
                cv_ptr = cv_bridge::toCvShare(imageMsg);
              }
              catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
              }

              // Consider delay of image message
              //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
              cv::Mat im = cv_ptr->image.clone();
              {
                // To test relocalization
                static double startT = -1;
                if (startT < 0)
                  startT = imageMsg->header.stamp.toSec();
                // Below to test relocalizaiton
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if (imageMsg->header.stamp.toSec() < startT + config._testDiscardTime)
                  im = cv::Mat::zeros(im.rows, im.cols, im.type());
              }
              SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
              //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
              //cv::imshow("image",cv_ptr->image);

              // Wait local mapping end.
              bool bstop = false;
              while (!SLAM.bLocalMapAcceptKF()) {
                if (!ros::ok()) {
                  bstop = true;
                }
              };
              if (bstop)
                break;

            }

            //cv::waitKey(1);

            ros::spinOnce();
            r.sleep();
            if (!ros::ok())
              break;
          }
        ROS_INFO("------Bag DONE-----");
        ros::spin();
  } else {
    ROS_WARN("Run realtime");
    while (ros::ok()) {
      bool bdata = msgsync.getRecentMsgs(imageMsg, vimuMsg);

      if (bdata) {
        std::vector<ORB_SLAM2::IMUData> vimuData;
        //ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
        for (unsigned int i = 0; i < vimuMsg.size(); i++) {
          sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
          double ax = imuMsg->linear_acceleration.x;
          double ay = imuMsg->linear_acceleration.y;
          double az = imuMsg->linear_acceleration.z;
          if (bAccMultiply98) {
            ax *= g3dm;
            ay *= g3dm;
            az *= g3dm;
          }
          ORB_SLAM2::IMUData imudata(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z,
                                     ax, ay, az, imuMsg->header.stamp.toSec());
          vimuData.push_back(imudata);
          //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
        }

        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
          cv_ptr = cv_bridge::toCvShare(imageMsg);
        }
        catch (cv_bridge::Exception &e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return -1;
        }

        // Consider delay of image message
        //SLAM.TrackMonocular(cv_ptr->image, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        cv::Mat im = cv_ptr->image.clone();
        {
          // To test relocalization
          static double startT = -1;
          if (startT < 0)
            startT = imageMsg->header.stamp.toSec();
          // Below to test relocalizaiton
          //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
          if (imageMsg->header.stamp.toSec() < startT + config._testDiscardTime)
            im = cv::Mat::zeros(im.rows, im.cols, im.type());
        }
        SLAM.TrackMonoVI(im, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        //SLAM.TrackMonoVI(cv_ptr->image, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
        //cv::imshow("image",cv_ptr->image);

      }

      ros::spinOnce();
      //r.sleep();
      //if (!ros::ok())
      //  break;
    }
  }



  // Save camera trajectory
  //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


  SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath + "KeyFrameNavStateTrajectory.txt");

  cout << endl << endl << "Trajectory Saved!" << endl;

  // Stop all threads

  ros::shutdown();
  SLAM.Shutdown();
  return 0;
}

//void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
//{
//    // Copy the ros image message to cv::Mat.
//    cv_bridge::CvImageConstPtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvShare(msg);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
//}


