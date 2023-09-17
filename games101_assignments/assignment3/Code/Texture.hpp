//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    // read in the texture image
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    // given a coordination, return the color
    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;
        // here must be min and max,because there's sth wrong with the .obj file itself.
        // explicitly,the texture coordination is not completely correct recorded in .obj file
        int uLeft = std::max(0, (int)floor(u_img)), uRight = std::min(width - 1, (int)ceil(u_img));
        int vUp = std::max(0, (int)floor(v_img)), vDown = std::min(height - 1, (int)ceil(v_img));
        float rateRow = u_img - uLeft, rateCol = v_img - vUp;

        auto colorLD = image_data.at<cv::Vec3b>(vDown, uLeft);
        auto colorLU = image_data.at<cv::Vec3b>(vUp, uLeft);
        auto colorRD = image_data.at<cv::Vec3b>(vDown, uRight);
        auto colorRU = image_data.at<cv::Vec3b>(vUp, uRight);

        auto colorLRU = colorLU * (1 - rateRow) + colorRU * rateRow;
        auto colorLRD = colorLD * (1 - rateRow) + colorRD * rateRow;
        auto colorUP = colorLRU * (1 - rateCol) + colorLRD * rateCol;
        return Eigen::Vector3f(colorUP[0], colorUP[1], colorUP[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
