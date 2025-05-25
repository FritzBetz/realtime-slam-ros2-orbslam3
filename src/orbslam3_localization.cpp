//#define GL_GLEXT_PROTOTYPES
//#include <GL/glew.h>
//#include <GL/gl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib>   // For std::getenv
#include "rclcpp/rclcpp.hpp"
#include "Converter.h"

#include "System.h" // ORB_SLAM3 - if your build fails here, check CMakeLists for your installed ORB_SLAM3 directory

class ORBMonoNode : public rclcpp::Node {
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  ORBMonoNode()
  : Node("orbslam3_mono_node"), slam_(nullptr)
  {
    // TODO : make the config and settings file loading better
    std::string home_dir = std::getenv("HOME");
    std::string vocab_path = home_dir + "/ros2_ws/src/orbslam_cpp/src/ORBvoc.txt";
    std::string settings_path = home_dir + "/ros2_ws/src/orbslam_cpp/src/camera_param_default.yaml";

    this->declare_parameter<std::string>("vocab_path", vocab_path);
    this->declare_parameter<std::string>("settings_path", settings_path);

    vocab_path = this->get_parameter("vocab_path").as_string();
    settings_path = this->get_parameter("settings_path").as_string();

    slam_ = new ORB_SLAM3::System(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/Mavic_2_PRO/camera/image_color", 10,
      std::bind(&ORBMonoNode::image_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
     try {
      // TODO: Fix some axis or more is wrong in published tf
      cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
      double tstamp = rclcpp::Time(msg->header.stamp).seconds();
      slam_->TrackMonocular(img, tstamp);

      // Publish TF
      Sophus::SE3f Tcw_SE3 = slam_->TrackMonocular(img, tstamp);
      cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(Tcw_SE3.matrix());
      if (!Tcw.empty()) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "world";
        tf_msg.child_frame_id = "camera";

        // Translation
        tf_msg.transform.translation.x = Tcw.at<float>(0,3);
        tf_msg.transform.translation.y = Tcw.at<float>(1,3);
        tf_msg.transform.translation.z = Tcw.at<float>(2,3);

        // Rotation
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Transpose for world-to-camera
        tf2::Matrix3x3 tf2_rot(
          Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
          Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
          Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2)
        );
        tf2::Quaternion q;
        tf2_rot.getRotation(q);

        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  ~ORBMonoNode() {
    
    slam_->Shutdown();
    slam_->SaveKeyFrameTrajectoryTUM("CameraKeyFrames.txt");

    // I know that we are young, but I just cant be with you like this anymore
    if (slam_) delete slam_;
  }

private:
  ORB_SLAM3::System* slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ORBMonoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}