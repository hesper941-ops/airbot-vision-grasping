#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <ai_msgs/msg/perception_targets.hpp>
#include <ai_msgs/msg/target.hpp>
#include <ai_msgs/msg/roi.hpp>
#include <ai_msgs/msg/point.hpp>

#include <dnn_node/dnn_node.h>
#include <dnn_node/dnn_node_data.h>
#include <dnn_node/util/image_proc.h>
#include <dnn_node/util/output_parser/detection/ptq_yolo8_output_parser.h>
#include <dnn_node/util/output_parser/perception_common.h>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <fstream>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_set>
#include <unordered_map>

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DNNInput;
using hobot::dnn_node::ImageProc;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::output_parser::DnnParserResult;

class DetectYoloNode : public DnnNode
{
public:
  DetectYoloNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : DnnNode(node_name, options)
  {
    RCLCPP_INFO(this->get_logger(), "DetectYoloNode initializing...");

    // Parameters
    this->declare_parameter<std::string>("model_file",
      "/opt/hobot/model/x5/basic/yolo11m_detect_bayese_640x640_nv12_modified.bin");
    this->declare_parameter<std::string>("config_file",
      "/home/sunrise/robot/Orbbec_ws/src/detect_yolo/config/yolov11workconfig.json");
    this->declare_parameter<double>("score_threshold", 0.4);
    this->declare_parameter<std::vector<std::string>>("forward_classes",
      std::vector<std::string>{"bottle", "cup", "bowl", "apple", "banana",
        "orange", "cake", "book", "cell phone", "remote", "scissors", "spoon", "fork",
        "knife", "vase", "teddy bear", "toothbrush"});
    this->declare_parameter<bool>("show_image", false);
    this->declare_parameter<int>("depth_range_min_mm", 80);
    this->declare_parameter<int>("depth_range_max_mm", 1500);

    model_file_ = this->get_parameter("model_file").as_string();
    config_file_ = this->get_parameter("config_file").as_string();
    score_threshold_ = this->get_parameter("score_threshold").as_double();
    show_image_ = this->get_parameter("show_image").as_bool();

    auto forward_classes = this->get_parameter("forward_classes").as_string_array();
    for (const auto &c : forward_classes) {
      forward_class_set_.insert(c);
    }

    RCLCPP_INFO(this->get_logger(), "Model: %s", model_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Config: %s", config_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "Score threshold: %.2f", score_threshold_);
    RCLCPP_INFO(this->get_logger(), "Show image: %s", show_image_ ? "true" : "false");

    // Load model config JSON for parser
    std::ifstream ifs(config_file_);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open config: %s", config_file_.c_str());
      return;
    }
    rapidjson::IStreamWrapper isw(ifs);
    config_doc_.ParseStream(isw);
    ifs.close();

    if (hobot::dnn_node::parser_yolov8::LoadConfig(config_doc_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO config");
      return;
    }

    // Load COCO class names from config
    loadClassNames();

    // Subscribers
    color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&DetectYoloNode::colorCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&DetectYoloNode::depthCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&DetectYoloNode::cameraInfoCallback, this, std::placeholders::_1));

    targets_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      "/yolo_detections", 10);


    // OpenCV display windows
    if (show_image_) {
      cv::namedWindow("YOLO Detector", cv::WINDOW_NORMAL);
      cv::resizeWindow("YOLO Detector", 960, 720);
    }

    RCLCPP_INFO(this->get_logger(), "DetectYoloNode ready. Waiting for images...");
  }

  ~DetectYoloNode() override
  {
    if (show_image_) {
      cv::destroyAllWindows();
    }
  }

protected:
  int SetNodePara() override
  {
    dnn_node_para_ptr_->model_file = model_file_;
    dnn_node_para_ptr_->model_task_type = ModelTaskType::ModelInferType;
    dnn_node_para_ptr_->task_num = 4;
    return 0;
  }

  int PostProcess(const std::shared_ptr<DnnNodeOutput> &output) override
  {
    if (!output) return -1;

    std::shared_ptr<DnnParserResult> parser_result = std::make_shared<DnnParserResult>();
    if (hobot::dnn_node::parser_yolov8::Parse(output, parser_result) != 0) {
      return -1;
    }

    if (parser_result->perception.det.empty()) return 0;

    const auto &detections = parser_result->perception.det;

    // Get depth and camera info (locked)
    cv::Mat depth_mat;
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;
    cv::Mat color_copy;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      depth_mat = latest_depth_.clone();
      cam_info = latest_camera_info_;
      if (show_image_ && !latest_color_.empty()) {
        color_copy = latest_color_.clone();
      }
    }

    bool have_depth = !depth_mat.empty();
    bool have_info = cam_info != nullptr;

    ai_msgs::msg::PerceptionTargets msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_color_optical_frame";
    msg.fps = 0;

    for (const auto &det : detections) {
      if (det.score < score_threshold_) continue;

      ai_msgs::msg::Target target;
      target.type = det.class_name ? det.class_name : "unknown";
      target.track_id = 0;

      ai_msgs::msg::Roi roi;
      roi.type = "body";
      roi.rect.x_offset = static_cast<uint32_t>(std::max(0.0f, det.bbox.xmin));
      roi.rect.y_offset = static_cast<uint32_t>(std::max(0.0f, det.bbox.ymin));
      roi.rect.width  = static_cast<uint32_t>(std::max(0.0f, det.bbox.xmax - det.bbox.xmin));
      roi.rect.height = static_cast<uint32_t>(std::max(0.0f, det.bbox.ymax - det.bbox.ymin));
      roi.confidence = det.score;
      target.rois.push_back(roi);

      if (have_depth && have_info) {
        float scale_x = depth_mat.empty() ? 1.0f : static_cast<float>(depth_mat.cols) / 640.0f;
        float scale_y = depth_mat.empty() ? 1.0f : static_cast<float>(depth_mat.rows) / 640.0f;
        int cx = static_cast<int>((det.bbox.xmin + det.bbox.xmax) / 2.0f * scale_x);
        int cy = static_cast<int>((det.bbox.ymin + det.bbox.ymax) / 2.0f * scale_y);

        int min_mm = this->get_parameter("depth_range_min_mm").as_int();
        int max_mm = this->get_parameter("depth_range_max_mm").as_int();
        float depth_mm = getDepthAt(depth_mat, cx, cy, min_mm, max_mm);

        if (depth_mm > 0) {
          ai_msgs::msg::Point pt;
          pt.type = "camera_3d";

          float depth_m = depth_mm / 1000.0f;
          float fx  = static_cast<float>(cam_info->k[0]);
          float fy  = static_cast<float>(cam_info->k[4]);
          float ppx = static_cast<float>(cam_info->k[2]);
          float ppy = static_cast<float>(cam_info->k[5]);

          geometry_msgs::msg::Point32 point3d;
          point3d.x = (cx - ppx) * depth_m / fx;
          point3d.y = (cy - ppy) * depth_m / fy;
          point3d.z = depth_m;
          pt.point.push_back(point3d);
          pt.confidence.push_back(det.score);
          target.points.push_back(pt);
        }
      }

      msg.targets.push_back(target);

      // Per-class PointStamped for forward classes
      std::string cls_name(target.type);
      if (forward_class_set_.count(cls_name) && have_depth && have_info &&
          !target.points.empty()) {
        publishClassPosition(cls_name, target.points[0].point[0], msg.header);
      }
    }

    targets_pub_->publish(msg);

    // Draw visualization
    if (show_image_ && !color_copy.empty()) {
      drawDetections(color_copy, detections, depth_mat, cam_info);
    }

    return 0;
  }

private:

  void loadClassNames()
  {
    // Extract cls_names_list path from config JSON
    if (config_doc_.HasMember("cls_names_list")) {
      std::string path = config_doc_["cls_names_list"].GetString();
      std::ifstream ifs(path);
      if (!ifs.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Cannot open: %s", path.c_str());
        return;
      }
      std::string line;
      while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        class_names_.push_back(line);
      }
      ifs.close();
      RCLCPP_INFO(this->get_logger(), "Loaded %zu COCO classes from %s",
        class_names_.size(), path.c_str());
    }
  }

  void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_camera_info_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Waiting for camera_info...");
        return;
      }
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    const cv::Mat &bgr = cv_ptr->image;
    if (bgr.empty()) return;

    // Store for visualization
    if (show_image_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_color_ = bgr.clone();
    }

    // Convert BGR to NV12 pyramid for BPU
    int model_w = 640, model_h = 640;
    auto nv12_input = ImageProc::GetNV12PyramidFromBGRImg(bgr, model_h, model_w);
    if (!nv12_input) {
      RCLCPP_ERROR(this->get_logger(), "NV12 pyramid creation failed");
      return;
    }

    std::vector<std::shared_ptr<DNNInput>> inputs;
    inputs.push_back(nv12_input);

    auto output = std::make_shared<DnnNodeOutput>();
    int ret = this->Run(inputs, output, nullptr, true);
    if (ret != 0) {
      RCLCPP_ERROR(this->get_logger(), "DNN Run failed: %d", ret);
    }
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (const cv_bridge::Exception &e) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_depth_ = cv_ptr->image;
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_camera_info_ = msg;
  }

  float getDepthAt(const cv::Mat &depth, int x, int y, int min_mm, int max_mm)
  {
    if (depth.empty()) return -1.0f;
    int h = depth.rows, w = depth.cols;
    float sum = 0.0f;
    int cnt = 0;
    const int k = 3;
    for (int dy = -k; dy <= k; dy += 2) {
      for (int dx = -k; dx <= k; dx += 2) {
        int nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
          uint16_t d = depth.at<uint16_t>(ny, nx);
          if (d >= static_cast<uint16_t>(min_mm) && d <= static_cast<uint16_t>(max_mm)) {
            sum += static_cast<float>(d);
            cnt++;
          }
        }
      }
    }
    if (cnt < 3) return -1.0f;
    return sum / cnt;
  }

  void drawDetections(cv::Mat &image,
    const std::vector<hobot::dnn_node::output_parser::Detection> &detections,
    const cv::Mat &depth,
    const sensor_msgs::msg::CameraInfo::SharedPtr &cam_info)
  {
    // Scale detections from model size (640x640) to image size
    float scale_x = static_cast<float>(image.cols) / 640.0f;
    float scale_y = static_cast<float>(image.rows) / 640.0f;

    for (const auto &det : detections) {
      if (det.score < score_threshold_) continue;

      int x1 = static_cast<int>(det.bbox.xmin * scale_x);
      int y1 = static_cast<int>(det.bbox.ymin * scale_y);
      int x2 = static_cast<int>(det.bbox.xmax * scale_x);
      int y2 = static_cast<int>(det.bbox.ymax * scale_y);

      x1 = std::max(0, x1); y1 = std::max(0, y1);
      x2 = std::min(image.cols - 1, x2); y2 = std::min(image.rows - 1, y2);

      const char *name = det.class_name ? det.class_name : "?";
      int cx = (x1 + x2) / 2, cy = (y1 + y2) / 2;

      // Color based on class
      cv::Scalar color(0, 255, 0);
      if (std::string(name) == "bottle") color = cv::Scalar(255, 128, 0);
      else if (std::string(name) == "cup") color = cv::Scalar(255, 0, 0);
      else if (std::string(name) == "apple") color = cv::Scalar(0, 0, 255);
      else if (std::string(name) == "cell phone") color = cv::Scalar(0, 255, 255);
      else if (std::string(name) == "book") color = cv::Scalar(128, 0, 255);
      else if (std::string(name) == "remote") color = cv::Scalar(255, 255, 0);

      cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

      // Get 3D position for label
      std::string label = std::string(name) + " " +
        std::to_string(static_cast<int>(det.score * 100)) + "%";

      if (!depth.empty() && cam_info) {
        int min_mm = this->get_parameter("depth_range_min_mm").as_int();
        int max_mm = this->get_parameter("depth_range_max_mm").as_int();
        float dm = getDepthAt(depth, cx, cy, min_mm, max_mm);
        if (dm > 0) {
          float dz = dm / 1000.0f;
          float fx = static_cast<float>(cam_info->k[0]);
          float fy = static_cast<float>(cam_info->k[4]);
          float ppx = static_cast<float>(cam_info->k[2]);
          float ppy = static_cast<float>(cam_info->k[5]);
          float dx = (cx - ppx) * dz / fx;
          float dy = (cy - ppy) * dz / fy;
          char buf[128];
          snprintf(buf, sizeof(buf), " (%.2f,%.2f,%.2f)m", dx, dy, dz);
          label += buf;
        }
      }

      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 2, &baseline);
      cv::rectangle(image,
        cv::Point(x1, y1 - text_size.height - 10),
        cv::Point(x1 + text_size.width, y1),
        color, -1);
      cv::putText(image, label, cv::Point(x1, y1 - 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
    }

    cv::imshow("YOLO Detector", image);
    cv::waitKey(1);
  }

  void publishClassPosition(const std::string &cls,
    const geometry_msgs::msg::Point32 &pt,
    const std_msgs::msg::Header &header)
  {
    auto it = class_publishers_.find(cls);
    if (it == class_publishers_.end()) {
      std::string topic = "/detect_yolo/" + cls + "_position";
      for (auto &c : topic) { if (c == ' ') c = '_'; }
      auto pub = this->create_publisher<geometry_msgs::msg::PointStamped>(topic, 10);
      it = class_publishers_.emplace(cls, pub).first;
      RCLCPP_INFO(this->get_logger(), "Created publisher: %s", topic.c_str());
    }

    geometry_msgs::msg::PointStamped msg;
    msg.header = header;
    msg.point.x = pt.x;
    msg.point.y = pt.y;
    msg.point.z = pt.z;
    it->second->publish(msg);
  }

  std::string model_file_;
  std::string config_file_;
  double score_threshold_;
  bool show_image_;
  rapidjson::Document config_doc_;
  std::vector<std::string> class_names_;
  std::unordered_set<std::string> forward_class_set_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr targets_pub_;
  std::unordered_map<std::string,
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr> class_publishers_;

  std::mutex data_mutex_;
  cv::Mat latest_depth_;
  cv::Mat latest_color_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<DetectYoloNode>("detect_yolo_node", options);

  if (node->Init() != 0) {
    RCLCPP_ERROR(node->get_logger(), "DNN Init failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "BPU YOLO detector ready.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
