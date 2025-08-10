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

    void start() override
    {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
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
        cv::threshold(gray_image, gray_image, 150, 255, cv::THRESH_BINARY);

        cv::Mat edges = find_edges(r_image);
        std::vector<LineType> lines = find_lines(edges);
        std::vector<cv::Point3f> circles = find_circles(edges);

        cv::Mat output_image = visualize(adjusted_image, lines, circles);

        cv::imshow("Processed Image", output_image);
        cv::imshow("Edges", edges);
        cv::imshow("Blue Channel", b_image);
        // cv::imshow("Green Channel", g_image);
        // cv::imshow("Red Channel", r_image);

        cv::waitKey(1);
    };

private:
    cv::Mat post_process_image(const cv::Mat& image)
    {
        // cv::Mat blurred_image;
        // cv::GaussianBlur(image, blurred_image, cv::Size(5, 5), 0);
        cv::Mat adjusted_image;
        cv::convertScaleAbs(image, adjusted_image, 1.5, 30);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::erode(adjusted_image, adjusted_image, kernel);
        cv::dilate(adjusted_image, adjusted_image, kernel);

        return adjusted_image;
    };

    cv::Mat find_edges(const cv::Mat& image)
    {
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