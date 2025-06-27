#pragma once

#include "../include/monobehaviour.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/visibility_control.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace engineering_match::image {

class ColorRangeHSV {
public:
    ColorRangeHSV() = default;

    enum class ColorType {
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
        { ColorType::BLUE, 100, 100, 100, 140, 255, 255 },
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

        return mask;
    }
};

class ImageProcesser : public base::IMonoBehaviour {
public:
    ImageProcesser(const std::string& name, float update_rate)
        : base::IMonoBehaviour(name, update_rate) {};

    void start() override
    {
        this->create_subscription<sensor_msgs::msg::Image>(
            "/unity/camera/image_raw", 10,
            std::bind(&ImageProcesser::image_data_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "started.");
    };

    void update() override {};

    void image_data_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(rclcpp::get_logger("ImageProcesser"), "Received image data with width: %d, height: %d",
            msg->width, msg->height);

        cv::Mat image_received = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(image_received, image_received, cv::COLOR_BGR2RGB);
        cv::flip(image_received, image_received, 0);

        cv::Mat adjusted_image = post_process_image(image_received);
        cv::Mat b_image, g_image, r_image, gray_image;

        ColorRangeHSV color_range;
        cv::Mat hsv_image;
        cv::cvtColor(adjusted_image, hsv_image, cv::COLOR_BGR2HSV);
        b_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::BLUE);
        g_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::GREEN);
        r_image = color_range.get_mask(hsv_image, ColorRangeHSV::ColorType::RED);

        cv::cvtColor(adjusted_image, gray_image, cv::COLOR_BGR2GRAY);

        cv::Mat edges = find_edges(gray_image);
        std::vector<cv::Point2f> lines = find_lines(gray_image);
        std::vector<cv::Point2f> circles = find_circles(gray_image);

        cv::Mat output_image = visualize(adjusted_image, lines, circles);

        cv::imshow("Processed Image", output_image);
        cv::imshow("Edges", edges);
        cv::imshow("Blue Channel", b_image);
        cv::imshow("Green Channel", g_image);
        cv::imshow("Red Channel", r_image);

        cv::waitKey(1);
    };

private:
    cv::Mat post_process_image(const cv::Mat& image)
    {
        cv::Mat blurred_image;
        cv::GaussianBlur(image, blurred_image, cv::Size(5, 5), 0);
        cv::Mat adjusted_image;
        cv::convertScaleAbs(blurred_image, adjusted_image, 1.5, 30);

        return adjusted_image;
    };

    cv::Mat find_edges(const cv::Mat& image)
    {
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150, 3);
        return edges;
    };

    std::vector<cv::Point2f> find_lines(const cv::Mat& image)
    {
        std::vector<cv::Point2f> lines;
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150, 3);
        std::vector<cv::Vec4i> lines_p;
        cv::HoughLinesP(edges, lines_p, 1, CV_PI / 180, 100, 10, 10);

        for (const auto& line : lines_p) {
            float x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
            float slope = (y2 - y1) / (x2 - x1);
            float intercept = y1 - slope * x1;
            lines.emplace_back(slope, intercept);
        }
        return lines;
    };

    std::vector<cv::Point2f> find_circles(const cv::Mat& image)
    {
        std::vector<cv::Point2f> circles;
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150, 3);
        std::vector<cv::Vec3f> circles_p;
        cv::HoughCircles(edges, circles_p, cv::HOUGH_GRADIENT, 1, 1.0, 100, 30, 5, 100);

        for (const auto& circle : circles_p) {
            circles.emplace_back(circle[0], circle[1]);
        }
        return circles;
    };

    cv::Mat visualize(const cv::Mat& image, const std::vector<cv::Point2f>& lines, const std::vector<cv::Point2f>& circles)
    {
        cv::Mat output_image = image.clone();
        for (const auto& line : lines) {
            float slope = line.x, intercept = line.y;
            cv::Point2f start(0, intercept);
            cv::Point2f end(output_image.cols, slope * output_image.cols + intercept);
            cv::line(output_image, start, end, cv::Scalar(0, 255, 0), 2);
        }

        for (const auto& circle : circles) {
            cv::circle(output_image, cv::Point(circle.x, circle.y), 5, cv::Scalar(255, 0, 0), -1);
        }

        return output_image;
    };
};

}