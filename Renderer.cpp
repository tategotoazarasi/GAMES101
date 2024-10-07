//
// Created by goksu on 2/25/20.
//

#include "Renderer.hpp"
#include "Scene.hpp"
#include <fstream>
#include <mutex>
#include <thread>

// 添加一个互斥锁来保护进度更新的输出
std::mutex mutex;

float progress = 0;

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

/**
 * @brief 渲染指定范围的图像行。
 *
 * 该函数用于在多线程环境下渲染图像的一部分行。它计算指定行范围内每个像素的颜色并更新帧缓冲区。
 *
 * @param startY 开始渲染的行索引（包含）。
 * @param endY 结束渲染的行索引（不包含）。
 * @param width 图像的宽度（像素）。
 * @param height 图像的高度（像素）。
 * @param scene 要渲染的场景对象。
 * @param framebuffer 存储渲染结果的帧缓冲区。
 * @param spp 每像素的采样次数，用于抗锯齿。
 * @param m 像素计数器的引用（用于跟踪渲染进度）。
 */
void renderRows(int startY, int endY, int width, int height, const Scene &scene, std::vector<Vector3f> &framebuffer, int spp, int &m) {
	float scale            = tan(deg2rad(scene.fov * 0.5));
	float imageAspectRatio = width / (float) height;
	Vector3f eye_pos(278, 273, -800);

	for(int j = startY; j < endY; ++j) {
		for(int i = 0; i < width; ++i) {
			float x = (2 * (i + 0.5) / (float) width - 1) * imageAspectRatio * scale;
			float y = (1 - 2 * (j + 0.5) / (float) height) * scale;

			Vector3f dir = normalize(Vector3f(-x, y, 1));
			for(int k = 0; k < spp; k++) {
				framebuffer[j * width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
			}
		}

		// 使用互斥锁来保护进度更新
		{
			std::lock_guard<std::mutex> lock(mutex);
			progress += 1 / (float) scene.height;
			UpdateProgress(progress);
		}
	}
}


// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene) {
	std::vector<Vector3f> framebuffer(scene.width * scene.height);
	int spp = 16;
	std::cout << "SPP: " << spp << "\n";

	int numThreads = std::thread::hardware_concurrency();
	std::vector<std::thread> threads;
	int rowsPerThread = scene.height / numThreads;

	int m = 0;

	// 创建线程
	for(int t = 0; t < numThreads; ++t) {
		int startY = t * rowsPerThread;
		int endY   = (t == numThreads - 1) ? scene.height : startY + rowsPerThread;
		threads.emplace_back(renderRows, startY, endY, scene.width, scene.height, std::ref(scene), std::ref(framebuffer), spp, std::ref(m));
	}

	// 等待所有线程完成
	for(auto &thread: threads) {
		thread.join();
	}

	UpdateProgress(1.f);

	// 保存帧缓冲区到文件
	FILE *fp = fopen("binary.ppm", "wb");
	(void) fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
	for(auto i = 0; i < scene.height * scene.width; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
		color[1] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
		color[2] = (unsigned char) (255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);
}