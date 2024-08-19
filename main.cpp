#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
	        -eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	model << cos(rotation_angle), -sin(rotation_angle), 0, 0,
	        sin(rotation_angle), cos(rotation_angle), 0, 0,
	        0, 0, 1, 0,
	        0, 0, 0, 1;
	return model;
}

/**
 * 逐个元素地构建模型变换矩阵并返回该矩阵。在此函数中,你只需要实现三维中绕 z 轴旋转的变换矩阵,而不用处理平移与缩放。
 * @return 变换矩阵
 */
Eigen::Matrix4f get_model_matrix(float angle_x, float angle_y, float angle_z) {
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.
	Matrix4f rotate_x = Matrix4f();
	Matrix4f rotate_y = Matrix4f();
	Matrix4f rotate_z = Matrix4f();
	angle_x           = angle_x / 180 * M_PIf;
	angle_y           = angle_y / 180 * M_PIf;
	angle_z           = angle_z / 180 * M_PIf;

	rotate_x << 1, 0, 0, 0,
	        0, cos(angle_x), -sin(angle_x), 0,
	        0, sin(angle_x), cos(angle_x), 0,
	        0, 0, 0, 1;
	rotate_y << cos(angle_y), 0, sin(angle_y), 0,
	        0, 1, 0, 0,
	        -sin(angle_y), 0, cos(angle_y), 0,
	        0, 0, 0, 1;
	rotate_z << cos(angle_z), -sin(angle_z), 0, 0,
	        sin(angle_z), cos(angle_z), 0, 0,
	        0, 0, 1, 0,
	        0, 0, 0, 1;
	return rotate_z * rotate_y * rotate_x;
}

/**
 * 使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵。
 * @param eye_fov 视野角度 (Field of View, FOV)，即观察者在垂直方向上可以看到的视角范围，单位为度。
 * @param aspect_ratio 视锥的宽高比 (Aspect Ratio)，通常是视口的宽度除以高度。
 * @param zNear 近裁剪面 (Near Clipping Plane) 的距离。距离从观察者到近裁剪面的距离应为正值。
 * @param zFar 远裁剪面 (Far Clipping Plane) 的距离。距离从观察者到远裁剪面的距离应为正值。
 * @return 透视投影矩阵
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
	// Create the projection matrix for the given parameters.
	// Then return it.
	float height              = 2 * zNear * tan(eye_fov / 2 / 180 * M_PIf);
	float width               = height * aspect_ratio;
	Matrix4f proj_persp2ortho = Matrix4f();
	proj_persp2ortho << zNear, 0, 0, 0,
	        0, zNear, 0, 0,
	        0, 0, zNear + zFar, -zNear * zFar,
	        0, 0, 1, 0;
	Matrix4f ortho = Matrix4f();
	ortho << 1 / (width / 2), 0, 0, 0,
	        0, 1 / (height / 2), 0, 0,
	        0, 0, -1 / ((zNear - zFar) / 2), 0,
	        0, 0, 0, 1;

	return ortho * proj_persp2ortho;
}

int main(int argc, const char **argv) {
	float angle          = 0;
	bool command_line    = false;
	std::string filename = "output.png";

	if(argc == 2) {
		command_line = true;
		filename     = std::string(argv[1]);
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = {0, 0, 5};


	std::vector<Eigen::Vector3f> pos{
	        {2, 0, -2},
	        {0, 2, -2},
	        {-2, 0, -2},
	        {3.5, -1, -5},
	        {2.5, 1.5, -5},
	        {-1, 0.5, -5}};

	std::vector<Eigen::Vector3i> ind{
	        {0, 1, 2},
	        {3, 4, 5}};

	std::vector<Eigen::Vector3f> cols{
	        {217.0, 238.0, 185.0},
	        {217.0, 238.0, 185.0},
	        {217.0, 238.0, 185.0},
	        {185.0, 217.0, 238.0},
	        {185.0, 217.0, 238.0},
	        {185.0, 217.0, 238.0}};

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);
	auto col_id = r.load_colors(cols);

	int key         = 0;
	int frame_count = 0;

	if(command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		cv::imwrite(filename, image);

		return 0;
	}

	while(key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';
	}

	return 0;
}