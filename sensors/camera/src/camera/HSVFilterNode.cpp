// Copyright 2023 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "camera/HSVFilterNode.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera
{

using std::placeholders::_1;

const std::string window_name = "Filtered Image";
const std::string low_h = "Low H";
const std::string high_h = "High H";
const std::string low_s = "Low S";
const std::string high_s = "High S";
const std::string low_v = "Low V";
const std::string high_v = "High V";
const uint8_t h_max_val = 360 / 2;
const uint8_t s_max_val = 255;
const uint8_t v_max_val = 255;

void create_trackbars(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
{
  cv::namedWindow(window_name);
  cv::createTrackbar(low_h, window_name, nullptr, h_max_val);
  cv::setTrackbarPos(low_h, window_name, h_min);
  cv::createTrackbar(high_h, window_name, nullptr, h_max_val);
  cv::setTrackbarPos(high_h, window_name, h_max);
  cv::createTrackbar(low_s, window_name, nullptr, s_max_val);
  cv::setTrackbarPos(low_s, window_name, s_min);
  cv::createTrackbar(high_s, window_name, nullptr, s_max_val);
  cv::setTrackbarPos(high_s, window_name, s_max);
  cv::createTrackbar(low_v, window_name, nullptr, v_max_val);
  cv::setTrackbarPos(low_v, window_name, v_min);
  cv::createTrackbar(high_v, window_name, nullptr, v_max_val);
  cv::setTrackbarPos(high_v, window_name, v_max);
}

HSVFilterNode::HSVFilterNode()
: Node("hsv_filter_node")
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&HSVFilterNode::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::SensorDataQoS().reliable(),
    std::bind(&HSVFilterNode::camera_info_callback, this, _1));

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "output_detection_2d", 100);

  declare_parameter("min_h", h_);
  declare_parameter("min_s", s_);
  declare_parameter("min_v", v_);
  declare_parameter("max_h", H_);
  declare_parameter("max_s", S_);
  declare_parameter("max_v", V_);
  declare_parameter("kernel_size", kernel_size_);
  declare_parameter("kernel_shape", kernel_shape_);

  get_parameter("min_h", h_);
  get_parameter("min_s", s_);
  get_parameter("min_v", v_);
  get_parameter("max_h", H_);
  get_parameter("max_s", S_);
  get_parameter("max_v", V_);
  get_parameter("kernel_size", kernel_size_);
  get_parameter("kernel_shape", kernel_shape_);

  if (kernel_size_ % 2 == 0) {
    kernel_size_++;
  }

  create_trackbars(h_, H_, s_, S_, v_, V_);
}

void
HSVFilterNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  model_ = std::make_shared<image_geometry::PinholeCameraModel>();
  model_->fromCameraInfo(*info);

  // No neccesary to receive more camera info, as it is fixed
  camera_info_sub_ = nullptr;
}

cv::Mat1b
HSVFilterNode::filter_image(cv::Mat & in, int h, int s, int v, int H, int S, int V)
{
  // Change space color BGR -> HSV
  cv::Mat img_hsv, out;
  cv::cvtColor(in, img_hsv, cv::COLOR_BGR2HSV);
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), out);

  // Erode and dilate to remove small regions
  cv::Mat kernel;
  if (kernel_shape_ == 0) { // Rectangle
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_));
  } else if (kernel_shape_ == 1) { // Ellipse
    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size_, kernel_size_));
  } else if (kernel_shape_ == 2) { // Cross
    kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(kernel_size_, kernel_size_));
  } else {
    RCLCPP_WARN(get_logger(), "Invalid kernel_shape_ value. Defaulting to MORPH_RECT.");
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_));
  }
  cv::erode(out, out, kernel);
  cv::dilate(out, out, kernel);
  return out;
}

void
HSVFilterNode::show_image_filtered(const cv::Mat & image, const cv::Mat1b & image_filtered)
{
  cv::waitKey(1);

  cv::Mat out_image;
  image.copyTo(out_image, image_filtered);
  cv::imshow(window_name, out_image);

  // show bounding box
  cv::Rect bbx = cv::boundingRect(image_filtered);
  cv::rectangle(out_image, bbx, cv::Scalar(0, 255, 0), 2);
  cv::imshow(window_name, out_image);
  cv::waitKey(1);
}

cv::Point2d
HSVFilterNode::get_detected_center(const cv::Mat & filtered)
{
  auto m = cv::moments(filtered, true);
  if (m.m00 < 0.000001) {return {0.0, 0.0};}
  return cv::Point2d(m.m10 / m.m00, m.m01 / m.m00);
}

std::tuple<float, float>
HSVFilterNode::get_detected_angles(
  const cv::Point2d & center, std::shared_ptr<image_geometry::PinholeCameraModel> model)
{
  cv::Point3d ray = model->projectPixelTo3dRay(model->rectifyPoint(center));

  ray = ray / ray.z;
  float yaw = atan2(ray.x, ray.z);
  float pitch = atan2(ray.y, ray.z);

  return {yaw, pitch};
}

void
HSVFilterNode::publish_detection(
  const sensor_msgs::msg::Image::ConstSharedPtr & image, const cv::Point2d & point,
  const cv::Rect & bbx)
{
  if (detection_pub_->get_subscription_count() > 0) {
    vision_msgs::msg::Detection2D detection_msg;
    detection_msg.header.frame_id = image->header.frame_id;
    detection_msg.header.stamp = image->header.stamp;
    detection_msg.bbox.center.position.x = point.x + bbx.width / 2;
    detection_msg.bbox.center.position.y = point.y + bbx.height / 2;
    detection_msg.bbox.size_x = bbx.width;
    detection_msg.bbox.size_y = bbx.height;

    vision_msgs::msg::Detection2DArray detection_array_msg;
    detection_array_msg.header.frame_id = image->header.frame_id;
    detection_array_msg.header.stamp = image->header.stamp;
    detection_array_msg.detections.push_back(detection_msg);

    detection_pub_->publish(detection_array_msg);
  }
}

void
HSVFilterNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  if (model_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Camera info not received yet");
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat & image_cv = cv_ptr->image;

  h_ = cv::getTrackbarPos(low_h, window_name);
  H_ = cv::getTrackbarPos(high_h, window_name);
  s_ = cv::getTrackbarPos(low_s, window_name);
  S_ = cv::getTrackbarPos(high_s, window_name);
  v_ = cv::getTrackbarPos(low_v, window_name);
  V_ = cv::getTrackbarPos(high_v, window_name);

  cv::Mat1b image_filtered = filter_image(image_cv, h_, s_, v_, H_, S_, V_);
  show_image_filtered(image_cv, image_filtered);

  cv::Rect bbx = cv::boundingRect(image_filtered);
  if (bbx.width == 0 || bbx.height == 0) {
    RCLCPP_WARN(get_logger(), "Object not detected");
    return;
  }

  cv::Point2d point = get_detected_center(image_filtered);

  if (model_->cameraInfo().distortion_model != "") {
    auto [yaw, pitch] = get_detected_angles(point, model_);

    RCLCPP_INFO(
      get_logger(), "Center at pos = (%lf, %lf) angle = [%f, %f]", point.x, point.y, yaw, pitch);
  }
  publish_detection(image, point, bbx);
}

}  // namespace camera
