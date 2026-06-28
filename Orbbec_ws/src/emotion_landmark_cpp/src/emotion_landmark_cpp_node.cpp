#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
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
    result_topic_ = this->declare_parameter<std::string>("result_topic", "/emotion/result");
    vis_topic_ = this->declare_parameter<std::string>("vis_topic", "/emotion/vis_image");

    landmark_model_path_ = this->declare_parameter<std::string>(
        "landmark_model_path",
        "/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm");

    emotion_model_path_ = this->declare_parameter<std::string>(
        "emotion_model_path",
        "/home/sunrise/robot/Orbbec_ws/src/emotion/emotion_resnet18_5cls_224.bin");

    label_path_ = this->declare_parameter<std::string>(
        "label_path",
        "/home/sunrise/robot/Orbbec_ws/src/emotion/emotion_labels.txt");

    show_image_ = this->declare_parameter<bool>("show_image", false);
    process_every_n_ = this->declare_parameter<int>("process_every_n", 3);
    min_face_size_ = this->declare_parameter<int>("min_face_size", 60);
    face_margin_ = this->declare_parameter<double>("face_margin", 0.20);
    landmark_crop_margin_ = this->declare_parameter<double>("landmark_crop_margin", 0.35);
    conf_threshold_ = this->declare_parameter<double>("conf_threshold", 0.35);

    low_mood_min_prob_ = this->declare_parameter<double>("low_mood_min_prob", 0.30);
    low_mood_neutral_margin_ = this->declare_parameter<double>("low_mood_neutral_margin", 0.12);
    low_mood_boost_ = this->declare_parameter<double>("low_mood_boost", 1.15);

    LoadLabels();
    LoadFaceDetector();

    LoadDNNModel(
        landmark_model_path_,
        &landmark_packed_handle_,
        &landmark_dnn_handle_,
        &landmark_output_count_,
        "landmark");

    LoadDNNModel(
        emotion_model_path_,
        &emotion_packed_handle_,
        &emotion_dnn_handle_,
        &emotion_output_count_,
        "emotion");

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
    RCLCPP_INFO(this->get_logger(), "labels count: %zu", labels_.size());
    RCLCPP_INFO(this->get_logger(), "low_mood_min_prob: %.2f", low_mood_min_prob_);
    RCLCPP_INFO(this->get_logger(), "low_mood_neutral_margin: %.2f", low_mood_neutral_margin_);
    RCLCPP_INFO(this->get_logger(), "low_mood_boost: %.2f", low_mood_boost_);
  }

  ~EmotionLandmarkCppNode() override {
    if (landmark_packed_handle_) {
      hbDNNRelease(landmark_packed_handle_);
      landmark_packed_handle_ = nullptr;
    }

    if (emotion_packed_handle_) {
      hbDNNRelease(emotion_packed_handle_);
      emotion_packed_handle_ = nullptr;
    }

    if (show_image_) {
      cv::destroyAllWindows();
    }
  }

 private:
  struct RoiBox {
    int left = 0;
    int top = 0;
    int right = 0;
    int bottom = 0;
  };

  struct EmotionResult {
    bool ok = false;
    int class_id = -1;
    std::string emotion = "unknown";
    float confidence = 0.0f;
    std::vector<float> probs;
    std::string raw_emotion = "unknown";
    int raw_class_id = -1;
    float raw_confidence = 0.0f;
    bool low_mood_rescue = false;
  };

  void LoadLabels() {
    std::ifstream ifs(label_path_);
    if (!ifs.is_open()) {
      throw std::runtime_error("cannot open label file: " + label_path_);
    }

    labels_.clear();

    std::string line;
    while (std::getline(ifs, line)) {
      if (!line.empty()) {
        labels_.push_back(line);
      }
    }

    if (labels_.empty()) {
      throw std::runtime_error("label file is empty");
    }

    RCLCPP_INFO(this->get_logger(), "loaded labels:");
    for (size_t i = 0; i < labels_.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %zu: %s", i, labels_[i].c_str());
    }
  }

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

  void LoadDNNModel(
      const std::string &model_path,
      hbPackedDNNHandle_t *packed_handle,
      hbDNNHandle_t *dnn_handle,
      int *output_count,
      const std::string &tag) {
    const char *model_files[] = {model_path.c_str()};

    int ret = hbDNNInitializeFromFiles(packed_handle, model_files, 1);
    if (ret != 0) {
      throw std::runtime_error(tag + " hbDNNInitializeFromFiles failed");
    }

    const char **model_name_list = nullptr;
    int model_count = 0;
    ret = hbDNNGetModelNameList(&model_name_list, &model_count, *packed_handle);
    if (ret != 0 || model_count <= 0) {
      throw std::runtime_error(tag + " hbDNNGetModelNameList failed");
    }

    ret = hbDNNGetModelHandle(dnn_handle, *packed_handle, model_name_list[0]);
    if (ret != 0) {
      throw std::runtime_error(tag + " hbDNNGetModelHandle failed");
    }

    ret = hbDNNGetOutputCount(output_count, *dnn_handle);
    if (ret != 0) {
      throw std::runtime_error(tag + " hbDNNGetOutputCount failed");
    }

    RCLCPP_INFO(
        this->get_logger(),
        "loaded %s model: %s, output_count=%d",
        tag.c_str(),
        model_name_list[0],
        *output_count);
  }

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    frame_count_++;

    if (process_every_n_ > 1 && frame_count_ % process_every_n_ != 0) {
      return;
    }

    auto total_t0 = std::chrono::steady_clock::now();

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

    RoiBox haar_box;
    bool has_face = DetectLargestFace(rgb, haar_box);

    std::vector<cv::Point2f> landmarks;
    bool has_landmarks = false;

    RoiBox crop_box = haar_box;

    if (has_face) {
      has_landmarks = RunLandmark(rgb, haar_box, landmarks);

      RoiBox landmark_box;
      if (has_landmarks && RefineBoxByLandmarks(rgb, landmarks, landmark_box)) {
        crop_box = landmark_box;
      }
    }

    EmotionResult emotion_result;

    if (has_face) {
      emotion_result = RunEmotion(rgb, crop_box);
    }

    auto total_t1 = std::chrono::steady_clock::now();
    double total_ms =
        std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();

    DrawVisualization(
        vis,
        has_face,
        haar_box,
        crop_box,
        has_landmarks,
        landmarks,
        emotion_result,
        total_ms);

    PublishResult(
        has_face,
        has_landmarks,
        haar_box,
        crop_box,
        landmarks,
        emotion_result,
        total_ms);

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

  bool RefineBoxByLandmarks(
      const cv::Mat &rgb,
      const std::vector<cv::Point2f> &landmarks,
      RoiBox &box) {
    if (landmarks.empty()) {
      return false;
    }

    float x_min = landmarks[0].x;
    float y_min = landmarks[0].y;
    float x_max = landmarks[0].x;
    float y_max = landmarks[0].y;

    for (const auto &p : landmarks) {
      x_min = std::min(x_min, p.x);
      y_min = std::min(y_min, p.y);
      x_max = std::max(x_max, p.x);
      y_max = std::max(y_max, p.y);
    }

    float bw = x_max - x_min;
    float bh = y_max - y_min;

    if (bw < 10.0f || bh < 10.0f) {
      return false;
    }

    float cx = (x_min + x_max) * 0.5f;
    float cy = (y_min + y_max) * 0.5f;
    float side = std::max(bw, bh) * (1.0f + static_cast<float>(landmark_crop_margin_));

    int left = static_cast<int>(std::round(cx - side * 0.5f));
    int top = static_cast<int>(std::round(cy - side * 0.5f));
    int right = static_cast<int>(std::round(cx + side * 0.5f));
    int bottom = static_cast<int>(std::round(cy + side * 0.5f));

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

  bool RunLandmark(
      const cv::Mat &rgb,
      const RoiBox &box,
      std::vector<cv::Point2f> &landmarks) {
    hbDNNTensor input;
    std::memset(&input, 0, sizeof(input));

    if (PrepareNV12SeparateTensor(rgb, input) != 0) {
      return false;
    }

    std::vector<hbDNNTensor> output_tensors(landmark_output_count_);

    if (!PrepareOutputTensors(landmark_dnn_handle_, landmark_output_count_, output_tensors)) {
      FreeInputTensor(input);
      return false;
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
        landmark_dnn_handle_,
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
          "hbDNNWaitTaskDone landmark failed, ret=%d",
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

  EmotionResult RunEmotion(const cv::Mat &rgb, const RoiBox &box) {
    EmotionResult result;

    cv::Mat crop = rgb(cv::Range(box.top, box.bottom), cv::Range(box.left, box.right)).clone();

    if (crop.empty()) {
      return result;
    }

    cv::Mat resized;
    cv::resize(crop, resized, cv::Size(emotion_input_size_, emotion_input_size_), 0, 0, cv::INTER_AREA);

    hbDNNTensor input;
    std::memset(&input, 0, sizeof(input));

    if (PrepareNV12Tensor(resized, input) != 0) {
      return result;
    }

    std::vector<hbDNNTensor> output_tensors(emotion_output_count_);

    if (!PrepareOutputTensors(emotion_dnn_handle_, emotion_output_count_, output_tensors)) {
      FreeInputTensor(input);
      return result;
    }

    hbDNNInferCtrlParam ctrl_param;
    std::memset(&ctrl_param, 0, sizeof(ctrl_param));
    ctrl_param.bpuCoreId = 0;
    ctrl_param.dspCoreId = 0;
    ctrl_param.priority = HB_DNN_PRIORITY_LOWEST;

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNTensor *outputs = output_tensors.data();

    int ret = hbDNNInfer(
        &task_handle,
        &outputs,
        &input,
        emotion_dnn_handle_,
        &ctrl_param);

    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "hbDNNInfer emotion failed, ret=%d",
          ret);
      FreeOutputTensors(output_tensors);
      FreeInputTensor(input);
      return result;
    }

    ret = hbDNNWaitTaskDone(task_handle, 5000);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "hbDNNWaitTaskDone emotion failed, ret=%d",
          ret);
      hbDNNReleaseTask(task_handle);
      FreeOutputTensors(output_tensors);
      FreeInputTensor(input);
      return result;
    }

    result = DecodeEmotion(output_tensors[0]);

    hbDNNReleaseTask(task_handle);
    FreeOutputTensors(output_tensors);
    FreeInputTensor(input);

    return result;
  }

  bool PrepareOutputTensors(
      hbDNNHandle_t dnn_handle,
      int output_count,
      std::vector<hbDNNTensor> &output_tensors) {
    for (int i = 0; i < output_count; ++i) {
      std::memset(&output_tensors[i], 0, sizeof(hbDNNTensor));

      int ret = hbDNNGetOutputTensorProperties(
          &output_tensors[i].properties,
          dnn_handle,
          i);

      if (ret != 0) {
        return false;
      }

      int mem_size = output_tensors[i].properties.alignedByteSize;

      ret = hbSysAllocCachedMem(&output_tensors[i].sysMem[0], mem_size);
      if (ret != 0) {
        return false;
      }

      std::memset(output_tensors[i].sysMem[0].virAddr, 0, mem_size);
      hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

    return true;
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

  int PrepareNV12Tensor(const cv::Mat &rgb, hbDNNTensor &input) {
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
    int total_size = y_size + uv_size;

    cv::Mat yuv420p;
    cv::cvtColor(img, yuv420p, cv::COLOR_RGB2YUV_I420);

    const uint8_t *src = yuv420p.data;
    const uint8_t *y_ptr = src;
    const uint8_t *u_ptr = src + y_size;
    const uint8_t *v_ptr = src + y_size + y_size / 4;

    std::vector<uint8_t> nv12(total_size);
    std::memcpy(nv12.data(), y_ptr, y_size);

    uint8_t *uv = nv12.data() + y_size;
    for (int i = 0; i < y_size / 4; ++i) {
      uv[2 * i] = u_ptr[i];
      uv[2 * i + 1] = v_ptr[i];
    }

    input.properties.tensorType = HB_DNN_IMG_TYPE_NV12;
    input.properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
    input.properties.quantiType = NONE;
    input.properties.alignedByteSize = total_size;

    SetTensorShape(input.properties.validShape, 1, 3, height, width);
    SetTensorShape(input.properties.alignedShape, 1, 3, height, width);

    int ret = hbSysAllocCachedMem(&input.sysMem[0], total_size);
    if (ret != 0) return ret;

    std::memcpy(input.sysMem[0].virAddr, nv12.data(), total_size);
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

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

  EmotionResult DecodeEmotion(hbDNNTensor &output) {
    EmotionResult result;

    hbSysFlushMem(&output.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

    int num = 1;
    for (int i = 0; i < output.properties.validShape.numDimensions; ++i) {
      num *= output.properties.validShape.dimensionSize[i];
    }

    num = std::min(num, static_cast<int>(labels_.size()));

    const float *data = reinterpret_cast<const float *>(output.sysMem[0].virAddr);

    std::vector<float> logits(num);
    for (int i = 0; i < num; ++i) {
      logits[i] = data[i];
    }

    std::vector<float> probs = Softmax(logits);

    int best = 0;
    for (int i = 1; i < num; ++i) {
      if (probs[i] > probs[best]) {
        best = i;
      }
    }

    int final_cls = best;
    float final_conf = probs[best];
    std::string final_emotion = labels_[best];
    bool low_mood_rescue = false;

    if (num >= 5) {
      float low_prob = probs[1];
      float neutral_prob = probs[3];

      bool neutral_close_to_low =
          (best == 3) &&
          (low_prob >= static_cast<float>(low_mood_min_prob_)) &&
          ((neutral_prob - low_prob) <= static_cast<float>(low_mood_neutral_margin_));

      bool mild_low_boost =
          (best != 1) &&
          (low_prob >= static_cast<float>(low_mood_min_prob_)) &&
          ((low_prob * static_cast<float>(low_mood_boost_)) >= probs[best]);

      if (neutral_close_to_low || mild_low_boost) {
        final_cls = 1;
        final_conf = low_prob;
        final_emotion = "possible_low_mood";
        low_mood_rescue = true;
      }
    }

    result.ok = true;
    result.class_id = final_cls;
    result.emotion = final_emotion;
    result.confidence = final_conf;
    result.probs = probs;
    result.raw_class_id = best;
    result.raw_emotion = labels_[best];
    result.raw_confidence = probs[best];
    result.low_mood_rescue = low_mood_rescue;

    return result;
  }

  std::vector<float> Softmax(const std::vector<float> &x) {
    std::vector<float> y(x.size());

    if (x.empty()) {
      return y;
    }

    float max_v = *std::max_element(x.begin(), x.end());
    float sum = 0.0f;

    for (size_t i = 0; i < x.size(); ++i) {
      y[i] = std::exp(x[i] - max_v);
      sum += y[i];
    }

    if (sum <= 0.0f) {
      return y;
    }

    for (auto &v : y) {
      v /= sum;
    }

    return y;
  }

  cv::Scalar GetEmotionColor(const std::string &emotion) {
    if (emotion == "happy") return cv::Scalar(0, 255, 0);
    if (emotion == "low_mood") return cv::Scalar(255, 128, 0);
    if (emotion == "possible_low_mood") return cv::Scalar(255, 128, 0);
    if (emotion == "negative_distress") return cv::Scalar(255, 0, 0);
    if (emotion == "neutral") return cv::Scalar(0, 180, 255);
    if (emotion == "surprise") return cv::Scalar(255, 255, 0);
    return cv::Scalar(255, 255, 255);
  }

  void DrawVisualization(
      cv::Mat &vis,
      bool has_face,
      const RoiBox &haar_box,
      const RoiBox &crop_box,
      bool has_landmarks,
      const std::vector<cv::Point2f> &landmarks,
      const EmotionResult &emotion_result,
      double total_ms) {
    if (!has_face) {
      cv::putText(
          vis,
          "No face detected",
          cv::Point(20, 40),
          cv::FONT_HERSHEY_SIMPLEX,
          1.0,
          cv::Scalar(255, 255, 0),
          2);
      return;
    }

    cv::rectangle(
        vis,
        cv::Point(haar_box.left, haar_box.top),
        cv::Point(haar_box.right, haar_box.bottom),
        cv::Scalar(180, 180, 180),
        1);

    cv::Scalar color = cv::Scalar(255, 255, 255);

    if (emotion_result.ok) {
      color = GetEmotionColor(emotion_result.emotion);
    }

    cv::rectangle(
        vis,
        cv::Point(crop_box.left, crop_box.top),
        cv::Point(crop_box.right, crop_box.bottom),
        color,
        2);

    if (has_landmarks) {
      for (const auto &p : landmarks) {
        cv::circle(vis, p, 1, cv::Scalar(0, 255, 255), -1);
      }
    }

    std::string emotion_text = "emotion: unknown";
    if (emotion_result.ok) {
      std::ostringstream es;
      es << emotion_result.emotion << " conf=" << std::fixed << std::setprecision(2)
         << emotion_result.confidence;
      emotion_text = es.str();
    }

    std::ostringstream raw_text;
    if (emotion_result.ok) {
      raw_text << "raw=" << emotion_result.raw_emotion
               << " " << std::fixed << std::setprecision(2)
               << emotion_result.raw_confidence;
      if (emotion_result.low_mood_rescue) {
        raw_text << "  low_mood_rescue";
      }
    } else {
      raw_text << "raw=unknown";
    }

    std::ostringstream ms_text;
    ms_text << "total=" << std::fixed << std::setprecision(1) << total_ms << " ms"
            << " landmarks=" << (has_landmarks ? "true" : "false");

    int y0 = std::max(30, crop_box.top - 70);
    int y1 = std::max(60, crop_box.top - 45);
    int y2 = std::max(90, crop_box.top - 15);

    cv::putText(
        vis,
        emotion_text,
        cv::Point(crop_box.left, y0),
        cv::FONT_HERSHEY_SIMPLEX,
        0.75,
        color,
        2);

    cv::putText(
        vis,
        raw_text.str(),
        cv::Point(crop_box.left, y1),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2);

    cv::putText(
        vis,
        ms_text.str(),
        cv::Point(crop_box.left, y2),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2);

    cv::putText(
        vis,
        "gray=Haar, color=landmark emotion crop, yellow=106 pts",
        cv::Point(20, vis.rows - 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        2);
  }

  void PublishResult(
      bool has_face,
      bool has_landmarks,
      const RoiBox &haar_box,
      const RoiBox &crop_box,
      const std::vector<cv::Point2f> &landmarks,
      const EmotionResult &emotion_result,
      double total_ms) {
    std_msgs::msg::String msg;

    std::ostringstream ss;
    ss << "{";
    ss << "\"has_face\":" << (has_face ? "true" : "false") << ",";
    ss << "\"has_landmarks\":" << (has_landmarks ? "true" : "false") << ",";
    ss << "\"landmark_count\":" << landmarks.size() << ",";
    ss << "\"total_ms\":" << std::fixed << std::setprecision(2) << total_ms;

    if (has_face) {
      ss << ",\"haar_box\":[" << haar_box.left << "," << haar_box.top << ","
         << haar_box.right << "," << haar_box.bottom << "]";
      ss << ",\"crop_box\":[" << crop_box.left << "," << crop_box.top << ","
         << crop_box.right << "," << crop_box.bottom << "]";
    }

    if (emotion_result.ok) {
      ss << ",\"emotion\":\"" << emotion_result.emotion << "\"";
      ss << ",\"class_id\":" << emotion_result.class_id;
      ss << ",\"confidence\":" << std::fixed << std::setprecision(4)
         << emotion_result.confidence;
      ss << ",\"raw_emotion\":\"" << emotion_result.raw_emotion << "\"";
      ss << ",\"raw_class_id\":" << emotion_result.raw_class_id;
      ss << ",\"raw_confidence\":" << std::fixed << std::setprecision(4)
         << emotion_result.raw_confidence;
      ss << ",\"low_mood_rescue\":"
         << (emotion_result.low_mood_rescue ? "true" : "false");

      ss << ",\"status\":\""
         << (emotion_result.confidence >= conf_threshold_ ? "ok" : "low_confidence")
         << "\"";
    } else {
      ss << ",\"emotion\":\"unknown\"";
      ss << ",\"class_id\":-1";
      ss << ",\"confidence\":0.0";
      ss << ",\"raw_emotion\":\"unknown\"";
      ss << ",\"raw_class_id\":-1";
      ss << ",\"raw_confidence\":0.0";
      ss << ",\"low_mood_rescue\":false";
      ss << ",\"status\":\"no_result\"";
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
  std::string landmark_model_path_;
  std::string emotion_model_path_;
  std::string label_path_;

  bool show_image_;
  int process_every_n_;
  int min_face_size_;
  double face_margin_;
  double landmark_crop_margin_;
  double conf_threshold_;
  double low_mood_min_prob_;
  double low_mood_neutral_margin_;
  double low_mood_boost_;

  int emotion_input_size_ = 224;
  int frame_count_ = 0;

  std::string window_name_ = "RDK X5 Emotion + 106 Landmarks";

  std::vector<std::string> labels_;
  cv::CascadeClassifier face_detector_;

  hbPackedDNNHandle_t landmark_packed_handle_ = nullptr;
  hbDNNHandle_t landmark_dnn_handle_ = nullptr;
  int landmark_output_count_ = 0;

  hbPackedDNNHandle_t emotion_packed_handle_ = nullptr;
  hbDNNHandle_t emotion_dnn_handle_ = nullptr;
  int emotion_output_count_ = 0;

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
