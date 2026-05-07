#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <algorithm>
#include <string>
#include <mutex>
#include <cmath>
#include <cstdio>

using std::placeholders::_1;

class BoxDetector : public rclcpp::Node
{
public:
    BoxDetector() : Node("box_detector"),
                    window_name_("Box Detector"),
                    mask_window_name_("Box Mask")
    {
        show_image_ = this->declare_parameter<bool>("show_image", false);

        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&BoxDetector::colorCallback, this, _1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&BoxDetector::depthCallback, this, _1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10,
            std::bind(&BoxDetector::cameraInfoCallback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/box_position", 10);

        if (show_image_) {
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_name_, 960, 720);
            cv::namedWindow(mask_window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(mask_window_name_, 640, 480);
        }

        RCLCPP_INFO(this->get_logger(), "Box detector node started, show_image=%s",
                    show_image_ ? "true" : "false");
    }

    ~BoxDetector()
    {
        if (show_image_) {
            cv::destroyWindow(window_name_);
            cv::destroyWindow(mask_window_name_);
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

    cv::Mat latest_depth_;
    std::mutex depth_mutex_;

    bool has_camera_info_ = false;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    std::string camera_frame_id_ = "camera_color_optical_frame";
    std::mutex cam_info_mutex_;

    bool show_image_ = false;
    std::string window_name_;
    std::string mask_window_name_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_frame_id_ = msg->header.frame_id;
        has_camera_info_ = true;

        RCLCPP_INFO_ONCE(
            this->get_logger(),
            "Camera info received: fx=%.3f fy=%.3f cx=%.3f cy=%.3f frame=%s",
            fx_, fy_, cx_, cy_, camera_frame_id_.c_str());
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat depth_img;

            if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                msg->encoding == sensor_msgs::image_encodings::MONO16) {
                depth_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
            }
            else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
                const cv::Mat &depth_float = cv_ptr->image;

                depth_img = cv::Mat(depth_float.size(), CV_16UC1, cv::Scalar(0));
                for (int y = 0; y < depth_float.rows; ++y) {
                    for (int x = 0; x < depth_float.cols; ++x) {
                        float d_m = depth_float.at<float>(y, x);
                        if (std::isfinite(d_m) && d_m > 0.0f) {
                            int d_mm = static_cast<int>(d_m * 1000.0f);
                            if (d_mm > 0 && d_mm < 65535) {
                                depth_img.at<uint16_t>(y, x) = static_cast<uint16_t>(d_mm);
                            }
                        }
                    }
                }
            }
            else {
                depth_img = cv_bridge::toCvCopy(
                    msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
            }

            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_ = depth_img.clone();
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Depth convert failed: %s", e.what());
        }
    }

    int getWindowDepthMedian(const cv::Mat &depth_img, int cx, int cy, int radius)
    {
        if (depth_img.empty()) return -1;

        std::vector<int> vals;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int x = cx + dx;
                int y = cy + dy;
                if (x < 0 || x >= depth_img.cols || y < 0 || y >= depth_img.rows) continue;

                int d = static_cast<int>(depth_img.at<uint16_t>(y, x));
                if (d > 80 && d < 10000) {
                    vals.push_back(d);
                }
            }
        }

        if (vals.size() < 5) return -1;
        std::sort(vals.begin(), vals.end());
        return vals[vals.size() / 2];
    }

    int getRobustDepthFromMask(const cv::Mat &depth_img,
                               const cv::Mat &obj_mask,
                               const cv::Rect &obj_rect,
                               int u, int v)
    {
        if (depth_img.empty() || obj_mask.empty()) return -1;

        cv::Rect roi = obj_rect & cv::Rect(0, 0, depth_img.cols, depth_img.rows);
        if (roi.width <= 0 || roi.height <= 0) {
            return getWindowDepthMedian(depth_img, u, v, 5);
        }

        cv::Mat use_mask = obj_mask.clone();

        cv::Mat eroded;
        cv::erode(obj_mask, eroded,
                  cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
        if (cv::countNonZero(eroded) > 30) {
            use_mask = eroded;
        }

        std::vector<int> depths;
        depths.reserve(roi.width * roi.height);

        for (int y = roi.y; y < roi.y + roi.height; ++y) {
            const uchar *m_ptr = use_mask.ptr<uchar>(y);
            const uint16_t *d_ptr = depth_img.ptr<uint16_t>(y);

            for (int x = roi.x; x < roi.x + roi.width; ++x) {
                if (m_ptr[x] == 0) continue;

                int d = static_cast<int>(d_ptr[x]);
                if (d > 80 && d < 10000) {
                    depths.push_back(d);
                }
            }
        }

        if (depths.size() >= 12) {
            std::sort(depths.begin(), depths.end());
            int median = depths[depths.size() / 2];
            int tol = std::max(80, median / 10);

            std::vector<int> filtered;
            filtered.reserve(depths.size());
            for (int d : depths) {
                if (std::abs(d - median) <= tol) {
                    filtered.push_back(d);
                }
            }

            if (filtered.size() >= 8) {
                std::sort(filtered.begin(), filtered.end());
                return filtered[filtered.size() / 2];
            }
            return median;
        }

        int d = getWindowDepthMedian(depth_img, u, v, 3);
        if (d > 0) return d;
        d = getWindowDepthMedian(depth_img, u, v, 5);
        if (d > 0) return d;
        d = getWindowDepthMedian(depth_img, u, v, 7);
        return d;
    }

    bool pixelToCamera3D(int u, int v, int depth_mm, double &X, double &Y, double &Z)
    {
        std::lock_guard<std::mutex> lock(cam_info_mutex_);

        if (!has_camera_info_ || fx_ <= 1e-6 || fy_ <= 1e-6 || depth_mm <= 0) {
            return false;
        }

        Z = depth_mm / 1000.0;
        X = (u - cx_) * Z / fx_;
        Y = (v - cy_) * Z / fy_;
        return true;
    }

    void drawTextLine(cv::Mat &img, const std::string &text, int x, int y)
    {
        cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 0, 0), 3);
        cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 255, 255), 2);
    }

    void showFrame(const cv::Mat &img, const cv::Mat &mask)
    {
        if (!show_image_) return;

        cv::imshow(window_name_, img);
        cv::imshow(mask_window_name_, mask);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            RCLCPP_INFO(this->get_logger(), "Pressed q, shutting down...");
            rclcpp::shutdown();
        }
    }

    cv::Mat removeSmallComponents(const cv::Mat &binary, int min_area)
    {
        cv::Mat labels, stats, centroids;
        int num = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

        cv::Mat clean = cv::Mat::zeros(binary.size(), CV_8UC1);
        for (int i = 1; i < num; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area >= min_area) {
                clean.setTo(255, labels == i);
            }
        }
        return clean;
    }

    void colorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image;

        try {
            image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Color convert failed: %s", e.what());
            return;
        }

        if (image.empty()) return;

        cv::Mat display = image.clone();

        cv::Mat depth_img;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (!latest_depth_.empty()) {
                depth_img = latest_depth_.clone();
            }
        }

        // =========================
        // 1) 更窄的绿色阈值 + BGR约束 + ROI限制
        // =========================
        cv::Mat blur, hsv, mask_hsv, mask;
        cv::GaussianBlur(image, blur, cv::Size(5, 5), 0);
        cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

        // 绿色阈值收紧，不要太大
        cv::inRange(hsv, cv::Scalar(50, 80, 70), cv::Scalar(78, 255, 255), mask_hsv);

        // 再加一层 BGR 约束：G 要明显大于 R 和 B
        std::vector<cv::Mat> bgr;
        cv::split(blur, bgr);

        cv::Mat g_r_diff, g_b_diff, cond_gr, cond_gb;
        cv::subtract(bgr[1], bgr[2], g_r_diff);   // G - R
        cv::subtract(bgr[1], bgr[0], g_b_diff);   // G - B

        cv::compare(g_r_diff, 18, cond_gr, cv::CMP_GT);
        cv::compare(g_b_diff, 6, cond_gb, cv::CMP_GT);

        cv::bitwise_and(mask_hsv, cond_gr, mask);
        cv::bitwise_and(mask, cond_gb, mask);

        // 只在图像下半区域找盒子
        cv::Mat roi_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        int roi_y = static_cast<int>(image.rows * 0.52);
        cv::rectangle(
            roi_mask,
            cv::Rect(0, roi_y, image.cols, image.rows - roi_y),
            cv::Scalar(255),
            cv::FILLED
        );
        cv::bitwise_and(mask, roi_mask, mask);

        // 形态学清理
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // 去掉小连通域
        mask = removeSmallComponents(mask, 250);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_score = -1.0;
        int best = -1;
        cv::Rect best_rect;
        int best_depth_mm = -1;
        int best_u = -1;
        int best_v = -1;

        // =========================
        // 2) 更严格的候选筛选
        // =========================
        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 1000) continue;

            cv::Rect rect = cv::boundingRect(contours[i]);
            if (rect.width < 35 || rect.height < 35) continue;

            // 过滤贴边
            if (rect.x <= 2 || rect.y <= 2 ||
                rect.x + rect.width >= image.cols - 2 ||
                rect.y + rect.height >= image.rows - 2) {
                continue;
            }

            // 盒子中心必须更靠下
            double cy = rect.y + rect.height * 0.5;
            if (cy < image.rows * 0.58) {
                continue;
            }

            double rect_area = static_cast<double>(rect.width) * rect.height;
            if (rect_area <= 1.0) continue;

            double extent = area / rect_area;
            if (extent < 0.35) continue;

            double wh_ratio = static_cast<double>(rect.width) / rect.height;
            if (wh_ratio < 0.65 || wh_ratio > 1.45) continue;

            std::vector<cv::Point> hull;
            cv::convexHull(contours[i], hull);
            double hull_area = cv::contourArea(hull);
            double solidity = (hull_area > 1e-6) ? (area / hull_area) : 0.0;
            if (solidity < 0.78) continue;

            cv::RotatedRect rr = cv::minAreaRect(contours[i]);
            cv::Point2f center = rr.center;

            int u = static_cast<int>(std::round(center.x));
            int v = static_cast<int>(std::round(center.y));

            u = std::max(0, std::min(u, image.cols - 1));
            v = std::max(0, std::min(v, image.rows - 1));

            cv::Mat obj_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
            cv::drawContours(obj_mask, contours, i, cv::Scalar(255), cv::FILLED);

            int depth_mm = getRobustDepthFromMask(depth_img, obj_mask, rect, u, v);

            double depth_bonus = 0.0;
            if (depth_mm > 0) {
                double depth_m = depth_mm / 1000.0;
                depth_bonus = 1000.0 / std::max(0.18, depth_m);
            }

            double score = area * 2.5
                         + extent * 800.0
                         + solidity * 500.0
                         + depth_bonus;

            if (score > best_score) {
                best_score = score;
                best = i;
                best_rect = rect;
                best_depth_mm = depth_mm;
                best_u = u;
                best_v = v;
            }
        }

        if (best < 0) {
            drawTextLine(display, "No box detected", 20, 35);
            showFrame(display, mask);
            return;
        }

        cv::Mat best_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(best_mask, contours, best, cv::Scalar(255), cv::FILLED);

        cv::RotatedRect best_rr = cv::minAreaRect(contours[best]);
        cv::Point2f pts[4];
        best_rr.points(pts);

        int u = best_u;
        int v = best_v;
        int depth_mm = best_depth_mm;

        if (depth_mm <= 0) {
            depth_mm = getRobustDepthFromMask(depth_img, best_mask, best_rect, u, v);
        }

        for (int i = 0; i < 4; ++i) {
            cv::line(display, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(display, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1);

        if (depth_mm <= 0) {
            drawTextLine(display, "Box depth invalid", 20, 35);
            char buf[128];
            std::snprintf(buf, sizeof(buf), "pixel=(%d,%d)", u, v);
            drawTextLine(display, buf, 20, 65);
            showFrame(display, mask);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Box detected at (%d, %d), but depth is invalid", u, v);
            return;
        }

        double X = 0.0, Y = 0.0, Z = 0.0;
        if (!pixelToCamera3D(u, v, depth_mm, X, Y, Z)) {
            drawTextLine(display, "Box 3D convert failed", 20, 35);
            showFrame(display, mask);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Failed to convert box pixel to camera 3D point");
            return;
        }

        geometry_msgs::msg::PointStamped msg_out;
        msg_out.header.stamp = msg->header.stamp;
        {
            std::lock_guard<std::mutex> lock(cam_info_mutex_);
            msg_out.header.frame_id = camera_frame_id_;
        }
        msg_out.point.x = X;
        msg_out.point.y = Y;
        msg_out.point.z = Z;
        publisher_->publish(msg_out);

        drawTextLine(display, "Box detected", 20, 35);
        char buf1[128], buf2[128];
        std::snprintf(buf1, sizeof(buf1), "pixel=(%d,%d) depth=%d mm", u, v, depth_mm);
        std::snprintf(buf2, sizeof(buf2), "X=%.3f Y=%.3f Z=%.3f m", X, Y, Z);
        drawTextLine(display, buf1, 20, 65);
        drawTextLine(display, buf2, 20, 95);

        showFrame(display, mask);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Publish box_position: X=%.3f m, Y=%.3f m, Z=%.3f m (pixel u=%d, v=%d, depth=%d mm)",
            X, Y, Z, u, v, depth_mm);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BoxDetector>());
    rclcpp::shutdown();
    return 0;
}