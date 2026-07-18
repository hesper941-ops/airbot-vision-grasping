#include <dnn/hb_dnn.h>
#include <dnn/hb_sys.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#define CHECK_RET(ret, msg)                                      \
  do {                                                           \
    if ((ret) != 0) {                                            \
      std::cerr << "[ERROR] " << msg << " failed, ret=" << ret  \
                << std::endl;                                    \
      return -1;                                                 \
    }                                                            \
  } while (0)

static void SetTensorShape(hbDNNTensorShape &shape, int n, int c, int h, int w) {
  shape.numDimensions = 4;
  shape.dimensionSize[0] = n;
  shape.dimensionSize[1] = c;
  shape.dimensionSize[2] = h;
  shape.dimensionSize[3] = w;
}

static void PrintShape(const hbDNNTensorShape &shape) {
  std::cout << "(";
  for (int i = 0; i < shape.numDimensions; ++i) {
    std::cout << shape.dimensionSize[i];
    if (i + 1 < shape.numDimensions) std::cout << ",";
  }
  std::cout << ")";
}

static int PrepareNV12SeparateTensor(const cv::Mat &bgr, hbDNNTensor &input) {
  if (bgr.empty()) {
    std::cerr << "[ERROR] empty image" << std::endl;
    return -1;
  }

  cv::Mat img = bgr;
  if (img.cols % 2 != 0 || img.rows % 2 != 0) {
    cv::resize(img, img, cv::Size(img.cols - img.cols % 2, img.rows - img.rows % 2));
  }

  const int width = img.cols;
  const int height = img.rows;
  const int y_size = width * height;
  const int uv_size = width * height / 2;

  cv::Mat yuv420p;
  cv::cvtColor(img, yuv420p, cv::COLOR_BGR2YUV_I420);

  const uint8_t *src = yuv420p.data;
  const uint8_t *y_ptr = src;
  const uint8_t *u_ptr = src + y_size;
  const uint8_t *v_ptr = src + y_size + y_size / 4;

  std::vector<uint8_t> uv(uv_size);
  for (int i = 0; i < y_size / 4; ++i) {
    uv[2 * i] = u_ptr[i];
    uv[2 * i + 1] = v_ptr[i];
  }

  std::memset(&input, 0, sizeof(hbDNNTensor));

  input.properties.tensorType = HB_DNN_IMG_TYPE_NV12_SEPARATE;
  input.properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
  input.properties.quantiType = NONE;
  input.properties.alignedByteSize = y_size + uv_size;

  SetTensorShape(input.properties.validShape, 1, 3, height, width);
  SetTensorShape(input.properties.alignedShape, 1, 3, height, width);

  int ret = hbSysAllocCachedMem(&input.sysMem[0], y_size);
  CHECK_RET(ret, "hbSysAllocCachedMem Y");

  ret = hbSysAllocCachedMem(&input.sysMem[1], uv_size);
  CHECK_RET(ret, "hbSysAllocCachedMem UV");

  std::memcpy(input.sysMem[0].virAddr, y_ptr, y_size);
  std::memcpy(input.sysMem[1].virAddr, uv.data(), uv_size);

  ret = hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
  CHECK_RET(ret, "hbSysFlushMem Y clean");

  ret = hbSysFlushMem(&input.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
  CHECK_RET(ret, "hbSysFlushMem UV clean");

  std::cout << "[INFO] prepared NV12_SEPARATE input: "
            << width << "x" << height
            << ", y=" << y_size
            << ", uv=" << uv_size << std::endl;

  return 0;
}

static void FreeInputTensor(hbDNNTensor &input) {
  if (input.sysMem[0].virAddr) {
    hbSysFreeMem(&input.sysMem[0]);
  }
  if (input.sysMem[1].virAddr) {
    hbSysFreeMem(&input.sysMem[1]);
  }
}

static void PrintOutputInfo(hbDNNTensor *outputs, int output_count) {
  for (int i = 0; i < output_count; ++i) {
    std::cout << "[INFO] output[" << i << "] validShape=";
    PrintShape(outputs[i].properties.validShape);
    std::cout << ", alignedShape=";
    PrintShape(outputs[i].properties.alignedShape);
    std::cout << ", tensorType=" << outputs[i].properties.tensorType
              << ", quantiType=" << outputs[i].properties.quantiType
              << std::endl;
  }
}

static void DecodeAndPrintLandmarkBins(hbDNNTensor *outputs) {
  hbSysFlushMem(&outputs[0].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
  hbSysFlushMem(&outputs[1].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

  const int32_t *x_data = reinterpret_cast<const int32_t *>(outputs[0].sysMem[0].virAddr);
  const int32_t *y_data = reinterpret_cast<const int32_t *>(outputs[1].sysMem[0].virAddr);

  int x_aligned_c = outputs[0].properties.alignedShape.dimensionSize[3];
  int y_aligned_c = outputs[1].properties.alignedShape.dimensionSize[3];

  if (x_aligned_c < 106) x_aligned_c = 112;
  if (y_aligned_c < 106) y_aligned_c = 112;

  std::cout << "[INFO] first 10 landmark bin results:" << std::endl;

  for (int p = 0; p < 10; ++p) {
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

    float x_128 = (best_x_bin + 0.5f) * 4.0f;
    float y_128 = (best_y_bin + 0.5f) * 4.0f;

    std::cout << "  point " << p
              << ": x_bin=" << best_x_bin
              << ", y_bin=" << best_y_bin
              << ", approx_in_128=(" << x_128 << "," << y_128 << ")"
              << std::endl;
  }
}

int main(int argc, char **argv) {
  std::string model_file =
      "/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm";

  std::string image_file = "";

  if (argc >= 2) {
    model_file = argv[1];
  }
  if (argc >= 3) {
    image_file = argv[2];
  }

  cv::Mat bgr;
  if (!image_file.empty()) {
    bgr = cv::imread(image_file, cv::IMREAD_COLOR);
    if (bgr.empty()) {
      std::cerr << "[WARN] failed to read image, use gray dummy image: "
                << image_file << std::endl;
    }
  }

  if (bgr.empty()) {
    bgr = cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::rectangle(bgr, cv::Point(220, 100), cv::Point(420, 320), cv::Scalar(180, 180, 180), -1);
    std::cout << "[INFO] use dummy 640x480 gray image" << std::endl;
  }

  int img_w = bgr.cols;
  int img_h = bgr.rows;

  std::cout << "[INFO] model_file: " << model_file << std::endl;
  std::cout << "[INFO] image size: " << img_w << "x" << img_h << std::endl;

  hbPackedDNNHandle_t packed_handle = nullptr;
  const char *model_files[] = {model_file.c_str()};

  int ret = hbDNNInitializeFromFiles(&packed_handle, model_files, 1);
  CHECK_RET(ret, "hbDNNInitializeFromFiles");

  const char **model_name_list = nullptr;
  int model_count = 0;
  ret = hbDNNGetModelNameList(&model_name_list, &model_count, packed_handle);
  CHECK_RET(ret, "hbDNNGetModelNameList");

  std::cout << "[INFO] model count: " << model_count << std::endl;
  for (int i = 0; i < model_count; ++i) {
    std::cout << "  model[" << i << "]: " << model_name_list[i] << std::endl;
  }

  hbDNNHandle_t dnn_handle = nullptr;
  ret = hbDNNGetModelHandle(&dnn_handle, packed_handle, model_name_list[0]);
  CHECK_RET(ret, "hbDNNGetModelHandle");

  int input_count = 0;
  int output_count = 0;
  ret = hbDNNGetInputCount(&input_count, dnn_handle);
  CHECK_RET(ret, "hbDNNGetInputCount");

  ret = hbDNNGetOutputCount(&output_count, dnn_handle);
  CHECK_RET(ret, "hbDNNGetOutputCount");

  std::cout << "[INFO] input_count=" << input_count
            << ", output_count=" << output_count << std::endl;

  hbDNNTensor input;
  ret = PrepareNV12SeparateTensor(bgr, input);
  if (ret != 0) {
    hbDNNRelease(packed_handle);
    return -1;
  }

  hbDNNRoi roi;
  roi.left = img_w / 4;
  roi.top = img_h / 5;
  roi.right = img_w * 3 / 4;
  roi.bottom = img_h * 4 / 5;

  std::cout << "[INFO] roi: left=" << roi.left
            << ", top=" << roi.top
            << ", right=" << roi.right
            << ", bottom=" << roi.bottom << std::endl;

  hbDNNInferCtrlParam ctrl_param;
  std::memset(&ctrl_param, 0, sizeof(ctrl_param));
  ctrl_param.bpuCoreId = 0;
  ctrl_param.dspCoreId = 0;
  ctrl_param.priority = HB_DNN_PRIORITY_LOWEST;  
  
  std::vector<hbDNNTensor> output_tensors(output_count);

  for (int i = 0; i < output_count; ++i) {
    std::memset(&output_tensors[i], 0, sizeof(hbDNNTensor));
  
    ret = hbDNNGetOutputTensorProperties(
        &output_tensors[i].properties,
        dnn_handle,
        i
    );
    CHECK_RET(ret, "hbDNNGetOutputTensorProperties");
  
    int output_mem_size = output_tensors[i].properties.alignedByteSize;
  
    ret = hbSysAllocCachedMem(&output_tensors[i].sysMem[0], output_mem_size);
    CHECK_RET(ret, "hbSysAllocCachedMem output");
  
    std::memset(output_tensors[i].sysMem[0].virAddr, 0, output_mem_size);
  
    ret = hbSysFlushMem(&output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    CHECK_RET(ret, "hbSysFlushMem output clean");
  
    std::cout << "[INFO] prepare output[" << i
              << "], mem_size=" << output_mem_size << std::endl;
  }
  
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *outputs = output_tensors.data();
  
  ret = hbDNNRoiInfer(
      &task_handle,
      &outputs,
      &input,
      &roi,
      1,
      dnn_handle,
      &ctrl_param
  );  

  CHECK_RET(ret, "hbDNNRoiInfer");

  std::cout << "[INFO] hbDNNRoiInfer success" << std::endl;

  ret = hbDNNWaitTaskDone(task_handle, 5000);
  CHECK_RET(ret, "hbDNNWaitTaskDone");

  std::cout << "[INFO] task done" << std::endl;

  PrintOutputInfo(outputs, output_count);
  DecodeAndPrintLandmarkBins(outputs);  
  
  hbDNNReleaseTask(task_handle);

  for (int i = 0; i < output_count; ++i) {
    if (output_tensors[i].sysMem[0].virAddr) {
      hbSysFreeMem(&output_tensors[i].sysMem[0]);
    }
  }
  
  FreeInputTensor(input);
  hbDNNRelease(packed_handle);
  
  std::cout << "[INFO] done" << std::endl;
  return 0;
}
