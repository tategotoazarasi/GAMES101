#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
	if(event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
		std::cout << "Left button of the mouse is clicked - position (" << x << ", "
		          << y << ")" << '\n';
		control_points.emplace_back(x, y);
	}
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
	auto &p_0 = points[0];
	auto &p_1 = points[1];
	auto &p_2 = points[2];
	auto &p_3 = points[3];

	for(double t = 0.0; t <= 1.0; t += 0.001) {
		auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
		             3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

		window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
	}
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) {
	// Implement de Casteljau's algorithm
	if(control_points.size() == 1) {
		return control_points[0];
	}
	std::vector<cv::Point2f> next_control_points = std::vector<cv::Point2f>(control_points.size() - 1);
	for(int i = 0; i < control_points.size() - 1; i++) {
		next_control_points[i] = control_points[i] + t * (control_points[i + 1] - control_points[i]);
	}

	return recursive_bezier(next_control_points, t);
}

float calcWeight(cv::Vec2i x, cv::Vec2f vxy) {
	float norm = cv::norm(cv::Vec2f(x) - vxy);
	return fmax(0, 1 - norm * norm);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
	// Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
	// recursive Bezier algorithm.
	const float STEP = 0.001f;
	for(float t = 0.f; t <= 1.f; t += STEP) {
		auto point = recursive_bezier(control_points, t);
		float x    = point.x;
		float y    = point.y;
		float dx   = x - floor(x);
		float dy   = y - floor(y);
		cv::Vec2i x00, x01, x10, x11;
		if(dx < 0.5 && dy < 0.5) {// left bottom
			x00 = cv::Vec2i(floor(x - 0.5), floor(y - 0.5));
			x01 = cv::Vec2i(floor(x - 0.5), floor(y));
			x10 = cv::Vec2i(floor(x), floor(y - 0.5));
			x11 = cv::Vec2i(floor(x), floor(y));
		} else if(dx < 0.5 && dy >= 0.5) {// left top
			x00 = cv::Vec2i(floor(x - 0.5), floor(y));
			x01 = cv::Vec2i(floor(x - 0.5), floor(y + 0.5));
			x10 = cv::Vec2i(floor(x), floor(y));
			x11 = cv::Vec2i(floor(x), floor(y + 0.5));
		} else if(dx >= 0.5 && dy < 0.5) {// right bottom
			x00 = cv::Vec2i(floor(x), floor(y - 0.5));
			x01 = cv::Vec2i(floor(x), floor(y));
			x10 = cv::Vec2i(floor(x + 0.5), floor(y - 0.5));
			x11 = cv::Vec2i(floor(x + 0.5), floor(y));
		} else {//right top
			x00 = cv::Vec2i(floor(x), floor(y));
			x01 = cv::Vec2i(floor(x), floor(y + 0.5));
			x10 = cv::Vec2i(floor(x + 0.5), floor(y));
			x11 = cv::Vec2i(floor(x + 0.5), floor(y + 0.5));
		}
		if((x00[0] < 0 || x00[0] >= window.rows || x00[1] < 0 || x00[1] >= window.cols) || (x01[0] < 0 || x01[0] >= window.rows || x01[1] < 0 || x01[1] >= window.cols) || (x10[0] < 0 || x10[0] >= window.rows || x10[1] < 0 || x10[1] >= window.cols) || (x11[0] < 0 || x11[0] >= window.rows || x11[1] < 0 || x11[1] >= window.cols)) {
			return;
		}
		window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
		auto vxy                                  = cv::Vec2f(x, y);
		window.at<cv::Vec3b>(x00[1], x00[0])[1]   = fmax(window.at<cv::Vec3b>(x00[1], x00[0])[1], calcWeight(x00, vxy) * 255);
		window.at<cv::Vec3b>(x01[1], x01[0])[1]   = fmax(window.at<cv::Vec3b>(x01[1], x01[0])[1], calcWeight(x01, vxy) * 255);
		window.at<cv::Vec3b>(x10[1], x10[0])[1]   = fmax(window.at<cv::Vec3b>(x10[1], x10[0])[1], calcWeight(x10, vxy) * 255);
		window.at<cv::Vec3b>(x11[1], x11[0])[1]   = fmax(window.at<cv::Vec3b>(x11[1], x11[0])[1], calcWeight(x00, vxy) * 255);
	}
}

int main() {
	cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
	cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
	cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

	cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

	int key = -1;
	while(key != 27) {
		for(auto &point: control_points) {
			cv::circle(window, point, 3, {255, 255, 255}, 3);
		}

		if(control_points.size() == 4) {
			naive_bezier(control_points, window);
			bezier(control_points, window);

			cv::imshow("Bezier Curve", window);
			cv::imwrite("my_bezier_curve.png", window);
			key = cv::waitKey(0);

			return 0;
		}

		cv::imshow("Bezier Curve", window);
		key = cv::waitKey(20);
	}

	return 0;
}
