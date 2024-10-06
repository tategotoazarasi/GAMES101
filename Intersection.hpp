//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Material.hpp"
#include "Vector.hpp"
class Object;
class Sphere;

/**
 * @brief 表示光线与场景中物体交点的结构体。
 *
 * 该结构体用于存储光线与物体相交时的相关信息，包含交点坐标、法线、材质等，
 * 并可用于后续的光线追踪计算，如着色、阴影判断等。
 */
struct Intersection {
	/**
     * @brief 默认构造函数，初始化一个无交点的结果。
     *
     * 构造函数将 `happened` 初始化为 `false`，并将其他成员设置为默认值。
     */
	Intersection() {
		happened = false;
		coords   = Vector3f();
		normal   = Vector3f();
		distance = std::numeric_limits<double>::max();// 初始化为最大值表示没有有效的交点
		obj      = nullptr;
		m        = nullptr;
	}
	bool happened;   ///< 表示是否发生了交点
	Vector3f coords; ///< 交点的坐标
	Vector3f tcoords;///< 交点的纹理坐标
	Vector3f normal; ///< 交点处的法向量
	Vector3f emit;   ///< 交点处的自发光颜色（发射光）
	double distance; ///< 光线从发射点到交点的距离
	Object *obj;     ///< 与交点相交的物体
	Material *m;     ///< 交点处物体的材质
};
#endif//RAYTRACING_INTERSECTION_H
