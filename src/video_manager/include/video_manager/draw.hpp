#pragma once

#include <string>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <algorithm>
#include <common_msgs/msg/detection.hpp>


static inline std::vector<cv::Scalar> generateColors(const std::vector<std::string> &classNames, int seed = 42)
{
    // Static cache to store colors based on class names to avoid regenerating
    static std::unordered_map<size_t, std::vector<cv::Scalar>> colorCache;

    // Compute a hash key based on class names to identify unique class configurations
    size_t hashKey = 0;
    for (const auto &name : classNames)
    {
        hashKey ^= std::hash<std::string>{}(name) + 0x9e3779b9 + (hashKey << 6) + (hashKey >> 2);
    }

    // Check if colors for this class configuration are already cached
    auto it = colorCache.find(hashKey);
    if (it != colorCache.end())
    {
        return it->second;
    }

    // Generate unique random colors for each class
    std::vector<cv::Scalar> colors;
    colors.reserve(classNames.size());

    std::mt19937 rng(seed);                         // Initialize random number generator with fixed seed
    std::uniform_int_distribution<int> uni(0, 255); // Define distribution for color values

    for (size_t i = 0; i < classNames.size(); ++i)
    {
        colors.emplace_back(cv::Scalar(uni(rng), uni(rng), uni(rng))); // Generate random BGR color
    }

    // Cache the generated colors for future use
    colorCache.emplace(hashKey, colors);

    return colorCache[hashKey];
}

/**
 * @brief Draws bounding boxes and labels on the image based on detections.
 * @param image Image on which to draw.
 * @param detections Vector of detections.
 */
static inline cv::Mat drawBoundingBox(const cv::Mat &frame, const std::vector<common_msgs::msg::Detection> &detections, const float confidence_threshold = 0.0)
{
    cv::Mat image = frame.clone();
    // Iterate through each detection to draw bounding boxes and labels
    for (const auto &detection : detections)
    {
        // Skip detections below the confidence threshold
        if (detection.conf <= confidence_threshold)
            continue;

        // Select color based on object ID for consistent coloring
        cv::Scalar color(0, 0, 255); // B, G, R

        // Draw the bounding box rectangle
        cv::rectangle(image, cv::Point(detection.x, detection.y),
                        cv::Point(detection.x + detection.width, detection.y + detection.height),
                        color, 2, cv::LINE_AA);

    }
    return image;
}
