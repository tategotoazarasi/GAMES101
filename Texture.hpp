//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
	cv::Mat image_data;

public:
	Texture(const std::string &name) {
		image_data = cv::imread(name);
		cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
		width  = image_data.cols;
		height = image_data.rows;
	}

	int width, height;

	Eigen::Vector3f getColor(float u, float v) {
		auto u_img = u * width;
		auto v_img = (1 - v) * height;
		auto color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	/**
	 * 使用双线性插值进行纹理采样
	 */
	Eigen::Vector3f getColorBilinear(float u, float v) {
		float u_img = u * width;
		float v_img = (1 - v) * height;

		int x0 = static_cast<int>(floor(u_img));
		int x1 = x0 + 1;
		int y0 = static_cast<int>(floor(v_img));
		int y1 = y0 + 1;

		x1 = std::min(x1, width - 1);
		y1 = std::min(y1, height - 1);

		float s = u_img - x0;
		float t = v_img - y0;

		auto color00 = image_data.at<cv::Vec3b>(y0, x0);
		auto color01 = image_data.at<cv::Vec3b>(y1, x0);
		auto color10 = image_data.at<cv::Vec3b>(y0, x1);
		auto color11 = image_data.at<cv::Vec3b>(y1, x1);

		auto color0 = color00 * (1 - s) + color10 * s;
		auto color1 = color01 * (1 - s) + color11 * s;
		auto color  = color0 * (1 - t) + color1 * t;

		return Eigen::Vector3f(color[0], color[1], color[2]);
	}
};
#endif//RASTERIZER_TEXTURE_H
