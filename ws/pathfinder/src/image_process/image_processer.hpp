#pragma once

#include "../include/monobehaviour.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/visibility_control.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace engineering_match::image {

struct LineType {
    float slope;
    float intercept;
    cv::Point2f start;
    cv::Point2f end;
    LineType(float s, float i, cv::Point2f st, cv::Point2f en)
        : slope(s)
        , intercept(i)
        , start(st)
        , end(en)
    {
    }
};

class ColorRangeHSV {
public:
    ColorRangeHSV() = default;

    enum class ColorType {
        CANTIDTIFY,
        WHITE,
        BLACK,
        RED,
        GREEN,
        BLUE,
        YELLOW,
        CYAN,
        MAGENTA
    };

    struct ColorRange {
        ColorType color;
        int h_min, s_min, v_min;
        int h_max, s_max, v_max;
    };

    const std::vector<ColorRange> color_ranges = {
        { ColorType::RED, 0, 100, 100, 10, 255, 255 },
        { ColorType::RED, 160, 100, 100, 180, 255, 255 },
        { ColorType::GREEN, 40, 100, 100, 80, 255, 255 },
        { ColorType::BLUE, 94, 80, 2, 120, 255, 255 },
        { ColorType::YELLOW, 20, 100, 100, 30, 255, 255 },
        { ColorType::CYAN, 80, 100, 100, 90, 255, 255 },
        { ColorType::MAGENTA, 140, 100, 100, 160, 255, 255 }
    };

    std::vector<ColorRange> get_bound(ColorType color) const
    {
        std::vector<ColorRange> ranges;
        for (const auto& range : color_ranges) {
            if (range.color == color) {
                ranges.push_back(range);
            }
        }
        return ranges;
    }

    cv::Mat get_mask(const cv::Mat& hsv_image, ColorType color)
    {
        cv::Mat mask;

        for (const auto& range : get_bound(color)) {
            cv::Mat temp_mask;
            cv::inRange(hsv_image, cv::Scalar(range.h_min, range.s_min, range.v_min),
                cv::Scalar(range.h_max, range.s_max, range.v_max), temp_mask);
            if (mask.empty()) {
                mask = temp_mask;
            } else {
                cv::bitwise_or(mask, temp_mask, mask);
            }
        }

        cv::threshold(mask, mask, 10, 255, cv::THRESH_BINARY);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(mask, mask, kernel);
        cv::dilate(mask, mask, kernel);

        return mask;
    }
};

class ImageProcesser : public base::IMonoBehaviour {
public:
    ImageProcesser(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    cv::VideoCapture cap {};

    void start() override
    {
        // image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/unity/camera/image_raw", 10,
        //     std::bind(&ImageProcesser::image_data_callback, this, std::placeholders::_1));

        // read video from file
        // cap = cv::VideoCapture("/workspaces/engineeringMatch/test.mp4");

        cap = cv::VideoCapture(0);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "started.");
    };

    void update() override {
        // cv::Mat frame;
        // if (!cap.read(frame)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to read frame from video.");
        //     return;
        // }

        // cv::Mat processed_image = post_process_image(frame);
        // auto color = get_color(processed_image, cv::Point2f(processed_image.cols / 2.0f, processed_image.rows / 2.0f), 200.0f);
    };

    // void image_data_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    // {
    //     RCLCPP_INFO(rclcpp::get_logger("ImageProcesser"), "Received image data with width: %d, height: %d",
    //         msg->width, msg->height);
    //     cv::Mat image_received = cv_bridge::toCvShare(msg, "bgr8")->image;
    //     cv::cvtColor(image_received, image_received, cv::COLOR_BGR2RGB);
    //     cv::flip(image_received, image_received, 0);

    //     post_process_image(image_received);
    // };

    ColorRangeHSV::ColorType read_and_get_color()
    {
        cv::Mat frame;
        if (!cap.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame from video.");
            return ColorRangeHSV::ColorType::CANTIDTIFY;
        }

        cv::Mat processed_image = post_process_image(frame);

        cv::imshow("image", processed_image);
        cv::waitKey(0);
        auto color = get_color(processed_image, cv::Point2f(processed_image.cols / 2.0f, processed_image.rows / 2.0f), 200.0f);
        return color;
    };

private:
    cv::Mat post_process_image(const cv::Mat& image)
    {
        cv::Mat adjusted_image;
        // alpha: the contrast control (<1.0: decrease contrast, >1.0: increase contrast)
        // beta: the brightness control
        cv::convertScaleAbs(image, adjusted_image, 1.5, 1);

        return adjusted_image;
    };

    ColorRangeHSV::ColorType get_color(cv::Mat& image, const cv::Point2f& point, const float& radius)
    {
        cv::Mat b_image, g_image, r_image;

        ColorRangeHSV color_range;
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        b_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::BLUE);
        g_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::GREEN);
        r_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::RED);

        cv::Rect roi = cv::Rect(point.x - radius, point.y - radius, radius * 2, radius * 2);
        cv::rectangle(image, roi, cv::Scalar(255, 0, 0), 1);

        b_image = b_image(roi);
        g_image = g_image(roi);
        r_image = r_image(roi);
        int b_count = cv::countNonZero(b_image);
        int g_count = cv::countNonZero(g_image);
        int r_count = cv::countNonZero(r_image);

        if (b_count > g_count && b_count > r_count) {
            return ColorRangeHSV::ColorType::BLUE;
        } else if (g_count > b_count && g_count > r_count) {
            return ColorRangeHSV::ColorType::GREEN;
        } else if (r_count > b_count && r_count > g_count) {
            return ColorRangeHSV::ColorType::RED;
        }
        return ColorRangeHSV::ColorType::CANTIDTIFY;
    };

    cv::Mat find_edges(const cv::Mat& image)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(image, image, kernel);
        cv::dilate(image, image, kernel);

        cv::Mat edges;
        cv::Canny(image, edges, 50, 150, 3);

        return edges;
    };

    std::vector<LineType> find_lines(const cv::Mat& edges)
    {
        std::vector<LineType> lines;
        std::vector<cv::Vec4i> lines_p;
        cv::HoughLinesP(edges, lines_p, 1, CV_PI / 180, 100, 20, 20);

        for (const auto& line : lines_p) {
            float x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            float slope = (y2 - y1) / (x2 - x1 + 1e-5);
            float intercept = y1 - slope * x1;
            lines.emplace_back(slope, intercept, cv::Point2f(x1, y1), cv::Point2f(x2, y2));
            // RCLCPP_INFO(this->get_logger(), "Line found: slope = %f, intercept = %f", slope, intercept);
        }
        // delete same slope lines

        // RCLCPP_INFO(this->get_logger(), "Found %zu unique lines", lines.size());
        return lines;
    };

    std::vector<cv::Point3f> find_circles(const cv::Mat& edges)
    {
        std::vector<cv::Point3f> circles;
        std::vector<cv::Vec3f> circles_p;
        cv::HoughCircles(edges, circles_p, cv::HOUGH_GRADIENT, 1, 20, 40, 20, 5, 60);

        for (const auto& circle : circles_p) {
            circles.emplace_back(circle[0], circle[1], circle[2]);
            RCLCPP_INFO(this->get_logger(), "Circle found: center = (%f, %f), radius = %f", circle[0], circle[1], circle[2]);
        }
        RCLCPP_INFO(this->get_logger(), "Found %zu circles", circles.size());
        return circles;
    };

    cv::Mat visualize(const cv::Mat& image, const std::vector<LineType>& lines, const std::vector<cv::Point3f>& circles)
    {
        cv::Mat output_image = image.clone();
        // for (const auto& line : lines) {
        //     cv::line(output_image, line.start, line.end, cv::Scalar(0, 255, 0), 2);
        // }

        for (const auto& circle : circles) {
            cv::circle(output_image, cv::Point(circle.x, circle.y), circle.z, cv::Scalar(255, 0, 0), 2);
        }

        return output_image;
    };
};

}