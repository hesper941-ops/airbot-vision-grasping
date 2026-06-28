#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class EmotionLandmarkCppNode : public rclcpp::Node {
 public:
  EmotionLandmarkCppNode() : Node("emotion_landmark_cpp_node") {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    result_topic_ = this->declare_parameter<std::string>("result_topic", "/landmark/result");
    vis_topic_ = this->declare_parameter<std::string>("vis_topic", "/landmark/vis_image");
    model_path_ = this->declare_parameter<std::string>(
        "landmark_model_path",
        "/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm");
    show_image_ = this->declare_parameter<bool>("show_image", false);
    process_every_n_ = this->declare_parameter<int>("process_every_n", 3);
    min_face_size_ = this->declare_parameter<int>("min_face_size", 60);
    face_margin_ = this->declare_parameter<double>("face_margin", 0.20);

    LoadFaceDetector();
    LoadLandmarkModel();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        10,
        std::bind(&EmotionLandmarkCppNode::ImageCallback, this, std::placeholders::_1));

    result_pub_ = this->create_publisher<std_msgs::msg::String>(result_topic_, 10);
    vis_pub_ = this->create_publisher<sensor_msgs::msg::Image>(vis_topic_, 10);

    if (show_image_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(window_name_, 960, 720);
    }

    RCLCPP_INFO(this->get_logger(), "emotion_landmark_cpp_node started");
    RCLCPP_INFO(this->get_logger(), "subscribe: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish result: %s", result_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish vis: %s", vis_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "show_image: %s", show_image_ ? "true" : "false");
  }

  ~EmotionLandmarkCppNode() override {
    if (packed_handle_) {
      hbDNNRelease(packed_handle_);
      packed_handle_ = nullptr;
    }
    if (show_image_) {
      cv::destroyAllWindows();
    }
  }

 private:
  struct RoiBox {
    int left;
    int top;
    int right;
    int bottom;
  };

  void LoadFaceDetector() {
    std::vector<std::string> candidates = {
        "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"};

    for (const auto &p : candidates) {
      if (std::ifstream(p).good()) {
        face_detector_.load(p);
        if (!face_detector_.empty()) {
          RCLCPP_INFO(this->get_logger(), "loaded haar cascade: %s", p.c_str());
          return;
        }
      }
    }

    throw std::runtime_error("cannot load haarcascade_frontalface_default.xml");
  }

  void LoadLandmarkModel() {
    const char *model_files[] = {model_path_.c_str()};

    int ret = hbDNNInitializeFromFiles(&packed_handle_, model_files, 1);
    if (ret != 0) {
      throw std::runtime_error("hbDNNInitializeFromFiles failed");
    }

    const char **model_name_list = nullptr;
    int model_count = 0;
    ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_handle_);
    if (ret != 0 || model_count <= 0) {
      throw std::runtime_error("hbDNNGetModelNameList failed");
    }

    ret = hbDNNGetModelHandle(&dnn_handle_, packed_handle_, model_name_list[0]);
    if (ret != 0) {
      throw std::runtime_error("hbDNNGetModelHandle failed");
    }

    ret = hbDNNGetOutputCount(&output_count_, dnn_handle_);
    if (ret != 0) {
      throw std::runtime_error("hbDNNGetOutputCount failed");
    }

    RCLCPP_INFO(this->get_logger(), "loaded landmark model: %s", model_name_list[0]);
    RCLCPP_INFO(this->get_logger(), "output_count: %d", output_count_);
  }

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    frame_count_++;

    if (process_every_n_ > 1 && frame_count_ % process_every_n_ != 0) {
      return;
    }

    cv::Mat rgb;
    try {
      rgb = cv_bridge::toCvCopy(msg, "rgb8")->image;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge failed: %s", e.what());
      return;
    }

    if (rgb.empty()) {
      return;
    }

    cv::Mat vis = rgb.clone();

    RoiBox face_box;
    bool has_face = DetectLargestFace(rgb, face_box);

    std::vector<cv::Point2f> landmarks;
    bool has_landmarks = false;

    auto t0 = std::chrono::steady_clock::now();

    if (has_face) {
      has_landmarks = RunLandmark(rgb, face_box, landmarks);
    }

    auto t1 = std::chrono::steady_clock::now();
    double infer_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (has_face) {
      cv::rectangle(
          vis,
          cv::Point(face_box.left, face_box.top),
          cv::Point(face_box.right, face_box.bottom),
          cv::Scalar(180, 180, 180),
          2);
    }

    if (has_landmarks) {
      for (const auto &p : landmarks) {
        cv::circle(vis, p, 1, cv::Scalar(0, 255, 255), -1);
      }
      cv::putText(vis, "106 landmarks OK",
                  cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.9,
                  cv::Scalar(0, 255, 0),
                  2);
    } else {
      cv::putText(vis, has_face ? "face found, landmark failed" : "no face",
                  cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.9,
                  cv::Scalar(255, 128, 0),
                  2);
    }

    std::ostringstream info;
    info << "infer_ms=" << std::fixed << std::setprecision(2) << infer_ms
         << " points=" << landmarks.size();

    cv::putText(vis, info.str(),
                cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX,
                0.7,
                cv::Scalar(255, 255, 255),
                2);

    PublishResult(has_face, has_landmarks, face_box, landmarks, infer_ms);
    PublishVis(vis, msg->header);

    if (show_image_) {
      cv::Mat bgr;
      cv::cvtColor(vis, bgr, cv::COLOR_RGB2BGR);
      cv::imshow(window_name_, bgr);
      cv::waitKey(1);
    }
  }

  bool DetectLargestFace(const cv::Mat &rgb, RoiBox &box) {
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    cv::equalizeHist(gray, gray);

    std::vector<cv::Rect> faces;
    face_detector_.detectMultiScale(
        gray,
        faces,
        1.1,
        5,
        0,
        cv::Size(min_face_size_, min_face_size_));

    if (faces.empty()) {
      return false;
    }

    auto best = *std::max_element(
        faces.begin(), faces.end(),
        [](const cv::Rect &a, const cv::Rect &b) {
          return a.area() < b.area();
        });

    double cx = best.x + best.width * 0.5;
    double cy = best.y + best.height * 0.5;
    double side = std::max(best.width, best.height) * (1.0 + face_margin_);

    int left = static_cast<int>(std::round(cx - side * 0.5));
    int top = static_cast<int>(std::round(cy - side * 0.5));
    int right = static_cast<int>(std::round(cx + side * 0.5));
    int bottom = static_cast<int>(std::round(cy + side * 0.5));

    ClipBox(rgb.cols, rgb.rows, left, top, right, bottom);

    if (right <= left || bottom <= top) {
      return false;
    }

    box = {left, top, right, bottom};
    return true;
  }

  void ClipBox(int w, int h, int &left, int &top, int &right, int &bottom) {
    left = std::max(0, std::min(w - 1, left));
    top = std::max(0, std::min(h - 1, top));
    right = std::max(0, std::min(w, right));
    bottom = std::max(0, std::min(h, bottom));
  }

  bool RunLandmark(const cv::Mat &rgb, const RoiBox &box, std::vector<cv::Point2f> &landmarks) {
    hbDNNTensor input;
    std::memset(&input, 0, sizeof(input));

    if (PrepareNV12SeparateTensor(rgb, input) != 0) {
      return false;
    }

    std::vector<hbDNNTensor> output_tensors(output_count_);

    for (int i = 0; i < output_count_; ++i) {
      std::memset(&output_tensors[i], 0, sizeof(hbDNNTensor));

      int ret = hbDNNGetOutputTensorProperties(
          &output_tensors[i].properties,
          dnn_handle_,
          i);
      if (ret != 0) {
        FreeInputTensor(input);
        return false;
      }

      int mem_size = output_tensors[i].properties.alignedByteSize;
      ret = hbSysAllocCachedMem(&output_tensors[i].sysMem[0], mem_size);
      if (ret != 0) {
        FreeInputTensor(input);
        return false;
      }

      std::memset(output_tensors[i].sysMem[0].virAddr, 0, mem_size);
      hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

    hbDNNRoi roi;
    roi.left = box.left;
    roi.top = box.top;
    roi.right = box.right;
    roi.bottom = box.bottom;

    hbDNNInferCtrlParam ctrl_param;
    std::memset(&ctrl_param, 0, sizeof(ctrl_param));
    ctrl_param.bpuCoreId = 0;
    ctrl_param.dspCoreId = 0;
    ctrl_param.priority = HB_DNN_PRIORITY_LOWEST;

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNTensor *outputs = output_tensors.data();

    int ret = hbDNNRoiInfer(
        &task_handle,
        &outputs,
        &input,
        &roi,
        1,
        dnn_handle_,
        &ctrl_param);

    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "hbDNNRoiInfer failed, ret=%d",
          ret);
      FreeOutputTensors(output_tensors);
      FreeInputTensor(input);
      return false;
    }

    ret = hbDNNWaitTaskDone(task_handle, 5000);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "hbDNNWaitTaskDone failed, ret=%d",
          ret);
      hbDNNReleaseTask(task_handle);
      FreeOutputTensors(output_tensors);
      FreeInputTensor(input);
      return false;
    }

    DecodeLandmarks(output_tensors, box, landmarks);

    hbDNNReleaseTask(task_handle);
    FreeOutputTensors(output_tensors);
    FreeInputTensor(input);

    return landmarks.size() == 106;
  }

  int PrepareNV12SeparateTensor(const cv::Mat &rgb, hbDNNTensor &input) {
    if (rgb.empty()) {
      return -1;
    }

    cv::Mat img = rgb;

    if (img.cols % 2 != 0 || img.rows % 2 != 0) {
      cv::resize(img, img, cv::Size(img.cols - img.cols % 2, img.rows - img.rows % 2));
    }

    int width = img.cols;
    int height = img.rows;
    int y_size = width * height;
    int uv_size = width * height / 2;

    cv::Mat yuv420p;
    cv::cvtColor(img, yuv420p, cv::COLOR_RGB2YUV_I420);

    const uint8_t *src = yuv420p.data;
    const uint8_t *y_ptr = src;
    const uint8_t *u_ptr = src + y_size;
    const uint8_t *v_ptr = src + y_size + y_size / 4;

    std::vector<uint8_t> uv(uv_size);

    for (int i = 0; i < y_size / 4; ++i) {
      uv[2 * i] = u_ptr[i];
      uv[2 * i + 1] = v_ptr[i];
    }

    input.properties.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE;
    input.properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
    input.properties.quantiType = NONE;
    input.properties.alignedByteSize = y_size + uv_size;

    SetTensorShape(input.properties.validShape, 1, 3, height, width);
    SetTensorShape(input.properties.alignedShape, 1, 3, height, width);

    int ret = hbSysAllocCachedMem(&input.sysMem[0], y_size);
    if (ret != 0) return ret;

    ret = hbSysAllocCachedMem(&input.sysMem[1], uv_size);
    if (ret != 0) return ret;

    std::memcpy(input.sysMem[0].virAddr, y_ptr, y_size);
    std::memcpy(input.sysMem[1].virAddr, uv.data(), uv_size);

    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    hbSysFlushMem(&input.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);

    return 0;
  }

  void SetTensorShape(hbDNNTensorShape &shape, int n, int c, int h, int w) {
    shape.numDimensions = 4;
    shape.dimensionSize[0] = n;
    shape.dimensionSize[1] = c;
    shape.dimensionSize[2] = h;
    shape.dimensionSize[3] = w;
  }

  void FreeInputTensor(hbDNNTensor &input) {
    if (input.sysMem[0].virAddr) {
      hbSysFreeMem(&input.sysMem[0]);
    }
    if (input.sysMem[1].virAddr) {
      hbSysFreeMem(&input.sysMem[1]);
    }
  }

  void FreeOutputTensors(std::vector<hbDNNTensor> &outputs) {
    for (auto &t : outputs) {
      if (t.sysMem[0].virAddr) {
        hbSysFreeMem(&t.sysMem[0]);
      }
    }
  }

  void DecodeLandmarks(
      std::vector<hbDNNTensor> &outputs,
      const RoiBox &roi,
      std::vector<cv::Point2f> &landmarks) {
    landmarks.clear();

    hbSysFlushMem(&outputs[0].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&outputs[1].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

    const int32_t *x_data =
        reinterpret_cast<const int32_t *>(outputs[0].sysMem[0].virAddr);
    const int32_t *y_data =
        reinterpret_cast<const int32_t *>(outputs[1].sysMem[0].virAddr);

    int x_aligned_c = outputs[0].properties.alignedShape.dimensionSize[3];
    int y_aligned_c = outputs[1].properties.alignedShape.dimensionSize[3];

    if (x_aligned_c < 106) x_aligned_c = 112;
    if (y_aligned_c < 106) y_aligned_c = 112;

    float roi_w = static_cast<float>(roi.right - roi.left);
    float roi_h = static_cast<float>(roi.bottom - roi.top);

    for (int p = 0; p < 106; ++p) {
      int best_x_bin = 0;
      int best_y_bin = 0;

      int32_t best_x_score = x_data[p];
      int32_t best_y_score = y_data[p];

      for (int b = 0; b < 32; ++b) {
        int32_t xs = x_data[b * x_aligned_c + p];
        int32_t ys = y_data[b * y_aligned_c + p];

        if (xs > best_x_score) {
          best_x_score = xs;
          best_x_bin = b;
        }

        if (ys > best_y_score) {
          best_y_score = ys;
          best_y_bin = b;
        }
      }

      float x_128 = (static_cast<float>(best_x_bin) + 0.5f) * 4.0f;
      float y_128 = (static_cast<float>(best_y_bin) + 0.5f) * 4.0f;

      float px = static_cast<float>(roi.left) + x_128 / 128.0f * roi_w;
      float py = static_cast<float>(roi.top) + y_128 / 128.0f * roi_h;

      landmarks.emplace_back(px, py);
    }
  }

  void PublishResult(
      bool has_face,
      bool has_landmarks,
      const RoiBox &box,
      const std::vector<cv::Point2f> &landmarks,
      double infer_ms) {
    std_msgs::msg::String msg;

    std::ostringstream ss;
    ss << "{";
    ss << "\"has_face\":" << (has_face ? "true" : "false") << ",";
    ss << "\"has_landmarks\":" << (has_landmarks ? "true" : "false") << ",";
    ss << "\"landmark_count\":" << landmarks.size() << ",";
    ss << "\"infer_ms\":" << std::fixed << std::setprecision(2) << infer_ms;

    if (has_face) {
      ss << ",\"box\":[" << box.left << "," << box.top << ","
         << box.right << "," << box.bottom << "]";
    }

    ss << "}";

    msg.data = ss.str();
    result_pub_->publish(msg);
  }

  void PublishVis(const cv::Mat &rgb, const std_msgs::msg::Header &header) {
    try {
      auto img_msg = cv_bridge::CvImage(header, "rgb8", rgb).toImageMsg();
      vis_pub_->publish(*img_msg);
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "publish vis failed: %s", e.what());
    }
  }

 private:
  std::string image_topic_;
  std::string result_topic_;
  std::string vis_topic_;
  std::string model_path_;
  bool show_image_;
  int process_every_n_;
  int min_face_size_;
  double face_margin_;

  std::string window_name_ = "RDK X5 106 Landmarks ROI Test";

  int frame_count_ = 0;

  cv::CascadeClassifier face_detector_;

  hbPackedDNNHandle_t packed_handle_ = nullptr;
  hbDNNHandle_t dnn_handle_ = nullptr;
  int output_count_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EmotionLandmarkCppNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
