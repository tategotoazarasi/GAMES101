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
		float du    = u_img - floor(u_img);
		float dv    = v_img - floor(v_img);
		cv::Vec2i u00, u01, u10, u11;
		if(du < 0.5 && dv < 0.5) {// left bottom
			u00 = cv::Vec2i(floor(u_img - 0.5), floor(v_img - 0.5));
			u01 = cv::Vec2i(floor(u_img - 0.5), floor(v_img));
			u10 = cv::Vec2i(floor(u_img), floor(v_img - 0.5));
			u11 = cv::Vec2i(floor(u_img), floor(v_img));
		} else if(du < 0.5 && dv >= 0.5) {// left top
			u00 = cv::Vec2i(floor(u_img - 0.5), floor(v_img));
			u01 = cv::Vec2i(floor(u_img - 0.5), floor(v_img + 0.5));
			u10 = cv::Vec2i(floor(u_img), floor(v_img));
			u11 = cv::Vec2i(floor(u_img), floor(v_img + 0.5));
		} else if(du >= 0.5 && dv < 0.5) {// right bottom
			u00 = cv::Vec2i(floor(u_img), floor(v_img - 0.5));
			u01 = cv::Vec2i(floor(u_img), floor(v_img));
			u10 = cv::Vec2i(floor(u_img + 0.5), floor(v_img - 0.5));
			u11 = cv::Vec2i(floor(u_img + 0.5), floor(v_img));
		} else {//right top
			u00 = cv::Vec2i(floor(u_img), floor(v_img));
			u01 = cv::Vec2i(floor(u_img), floor(v_img + 0.5));
			u10 = cv::Vec2i(floor(u_img + 0.5), floor(v_img));
			u11 = cv::Vec2i(floor(u_img + 0.5), floor(v_img + 0.5));
		}
		if((u00[0] < 0 || u00[0] >= height || u00[1] < 0 || u00[1] >= width) || (u01[0] < 0 || u01[0] >= height || u01[1] < 0 || u01[1] >= width) || (u10[0] < 0 || u10[0] >= height || u10[1] < 0 || u10[1] >= width) || (u11[0] < 0 || u11[0] >= height || u11[1] < 0 || u11[1] >= width)) {
			auto color = image_data.at<cv::Vec3b>(v_img, u_img);
			return Eigen::Vector3f(color[0], color[1], color[2]);
		}
		auto color00 = image_data.at<cv::Vec3b>(u00[1], u00[0]);
		auto color01 = image_data.at<cv::Vec3b>(u01[1], u01[0]);
		auto color10 = image_data.at<cv::Vec3b>(u10[1], u10[0]);
		auto color11 = image_data.at<cv::Vec3b>(u11[1], u11[0]);
		float s      = u_img - (static_cast<float>(u00[0]) + 0.5f);
		auto color0  = color00 * (1 - s) + color10 * s;
		auto color1  = color01 * (1 - s) + color11 * s;
		float t      = v_img - (static_cast<float>(u00[1]) + 0.5f);
		auto color   = color0 * (1 - t) + color1 * t;
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}
};
#endif//RASTERIZER_TEXTURE_H
