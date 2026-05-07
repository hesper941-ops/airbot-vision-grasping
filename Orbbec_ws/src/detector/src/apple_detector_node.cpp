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

class AppleDetector : public rclcpp::Node
{
public:
    AppleDetector() : Node("apple_detector"),
                      window_name_("Apple Detector"),
                      mask_window_name_("Apple Mask")
    {
        show_image_ = this->declare_parameter<bool>("show_image", false);

        color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&AppleDetector::colorCallback, this, _1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&AppleDetector::depthCallback, this, _1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 10,
            std::bind(&AppleDetector::cameraInfoCallback, this, _1));

        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/apple_position", 10);

        if (show_image_) {
            cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(window_name_, 960, 720);
            cv::namedWindow(mask_window_name_, cv::WINDOW_NORMAL);
            cv::resizeWindow(mask_window_name_, 640, 480);
        }

        RCLCPP_INFO(this->get_logger(), "Apple detector node started, show_image=%s",
                    show_image_ ? "true" : "false");
    }

    ~AppleDetector()
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
            } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
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
            } else {
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
                if (d > 80 && d < 10000) vals.push_back(d);
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
                  cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));
        if (cv::countNonZero(eroded) > 20) {
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

        cv::Mat blur, hsv, mask1, mask2, mask;
        cv::GaussianBlur(image, blur, cv::Size(5, 5), 0);
        cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

        // ·Å¿íÆ»¹ûºìÉ«·¶Î§£¬¼æÈÝ·Ûºì¡¢Ç³ºìºÍ¸ß¹â
        cv::inRange(hsv, cv::Scalar(0, 45, 45), cv::Scalar(20, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(155, 40, 40), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_score = -1.0;
        int best = -1;

        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 300) continue;

            cv::Rect rect = cv::boundingRect(contours[i]);
            if (rect.x <= 2 || rect.y <= 2 ||
                rect.x + rect.width >= image.cols - 2 ||
                rect.y + rect.height >= image.rows - 2) {
                continue;
            }

            double peri = cv::arcLength(contours[i], true);
            double circularity = (peri > 1e-6) ? (4.0 * CV_PI * area / (peri * peri)) : 0.0;
            if (circularity < 0.22) continue;

            double score = area + 1200.0 * circularity;
            if (score > best_score) {
                best_score = score;
                best = i;
            }
        }

        if (best < 0) {
            drawTextLine(display, "No apple detected", 20, 35);
            showFrame(display, mask);
            return;
        }

        cv::Mat obj_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::drawContours(obj_mask, contours, best, cv::Scalar(255), cv::FILLED);

        cv::Rect rect = cv::boundingRect(contours[best]);

        cv::Point2f circle_center;
        float radius = 0.0f;
        cv::minEnclosingCircle(contours[best], circle_center, radius);

        int u = static_cast<int>(std::round(circle_center.x));
        int v = static_cast<int>(std::round(circle_center.y));

        u = std::max(0, std::min(u, image.cols - 1));
        v = std::max(0, std::min(v, image.rows - 1));

        int depth_mm = getRobustDepthFromMask(depth_img, obj_mask, rect, u, v);

        cv::drawContours(display, contours, best, cv::Scalar(0, 255, 0), 2);
        cv::circle(display, cv::Point(u, v), 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(display, cv::Point(u, v), static_cast<int>(radius), cv::Scalar(255, 0, 0), 2);

        if (depth_mm <= 0) {
            drawTextLine(display, "Apple depth invalid", 20, 35);
            char buf[128];
            std::snprintf(buf, sizeof(buf), "pixel=(%d,%d)", u, v);
            drawTextLine(display, buf, 20, 65);
            showFrame(display, mask);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Apple detected at (%d, %d), but depth is invalid", u, v);
            return;
        }

        double X = 0.0, Y = 0.0, Z = 0.0;
        if (!pixelToCamera3D(u, v, depth_mm, X, Y, Z)) {
            drawTextLine(display, "Apple 3D convert failed", 20, 35);
            showFrame(display, mask);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Failed to convert apple pixel to camera 3D point");
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

        drawTextLine(display, "Apple detected", 20, 35);
        char buf1[128], buf2[128];
        std::snprintf(buf1, sizeof(buf1), "pixel=(%d,%d) depth=%d mm", u, v, depth_mm);
        std::snprintf(buf2, sizeof(buf2), "X=%.3f Y=%.3f Z=%.3f m", X, Y, Z);
        drawTextLine(display, buf1, 20, 65);
        drawTextLine(display, buf2, 20, 95);
        showFrame(display, mask);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Publish apple_position: X=%.3f m, Y=%.3f m, Z=%.3f m (pixel u=%d, v=%d, depth=%d mm)",
            X, Y, Z, u, v, depth_mm);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AppleDetector>());
    rclcpp::shutdown();
    return 0;
}