//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <array>
#include <limits>

class Bounds3 {
public:
	Vector3f pMin, pMax;// two points to specify the bounding box
	Bounds3() {
		double minNum = std::numeric_limits<double>::lowest();
		double maxNum = std::numeric_limits<double>::max();
		pMax          = Vector3f(minNum, minNum, minNum);
		pMin          = Vector3f(maxNum, maxNum, maxNum);
	}
	Bounds3(const Vector3f p): pMin(p), pMax(p) {}
	Bounds3(const Vector3f p1, const Vector3f p2) {
		pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
		pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
	}

	Vector3f Diagonal() const { return pMax - pMin; }
	int maxExtent() const {
		Vector3f d = Diagonal();
		if(d.x > d.y && d.x > d.z)
			return 0;
		else if(d.y > d.z)
			return 1;
		else
			return 2;
	}

	double SurfaceArea() const {
		Vector3f d = Diagonal();
		return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
	}

	Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
	Bounds3 Intersect(const Bounds3 &b) {
		return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
		                        fmax(pMin.z, b.pMin.z)),
		               Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
		                        fmin(pMax.z, b.pMax.z)));
	}

	Vector3f Offset(const Vector3f &p) const {
		Vector3f o = p - pMin;
		if(pMax.x > pMin.x)
			o.x /= pMax.x - pMin.x;
		if(pMax.y > pMin.y)
			o.y /= pMax.y - pMin.y;
		if(pMax.z > pMin.z)
			o.z /= pMax.z - pMin.z;
		return o;
	}

	bool Overlaps(const Bounds3 &b1, const Bounds3 &b2) {
		bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
		bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
		bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
		return (x && y && z);
	}

	bool Inside(const Vector3f &p, const Bounds3 &b) {
		return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
		        p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
	}
	inline const Vector3f &operator[](int i) const {
		return (i == 0) ? pMin : pMax;
	}

	inline bool IntersectP(const Ray &ray, const Vector3f &invDir,
	                       const std::array<int, 3> &dirisNeg) const;
};


inline bool Bounds3::IntersectP(const Ray &ray, const Vector3f &invDir,
                                const std::array<int, 3> &dirIsNeg) const {
	// invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
	// dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
	// test if ray bound intersects
	auto t_xmin = (pMin[0] - ray.origin[0]) * invDir[0];
	auto y_xmin = ray.origin[1] + t_xmin * ray.direction[1];
	auto z_xmin = ray.origin[2] + t_xmin * ray.direction[2];
	if((t_xmin > 0 == dirIsNeg[0]) && ((y_xmin >= pMin[1] && y_xmin <= pMax[1]) || (z_xmin >= pMin[2] && z_xmin <= pMax[2])))
		return true;
	auto t_xmax = (pMax[0] - ray.origin[0]) * invDir[0];
	auto y_xmax = ray.origin[1] + t_xmax * ray.direction[1];
	auto z_xmax = ray.origin[2] + t_xmax * ray.direction[2];
	if((t_xmax > 0 == dirIsNeg[0]) && ((y_xmax >= pMin[1] && y_xmax <= pMax[1]) || (z_xmax >= pMin[2] && z_xmax <= pMax[2])))
		return true;
	auto t_ymin = (pMin[1] - ray.origin[1]) * invDir[1];
	auto x_ymin = ray.origin[0] + t_ymin * ray.direction[0];
	auto z_ymin = ray.origin[2] + t_ymin * ray.direction[2];
	if((t_ymin > 0 == dirIsNeg[1]) && ((x_ymin >= pMin[0] && x_ymin <= pMax[0]) || (z_ymin >= pMin[2] && z_ymin <= pMax[2])))
		return true;
	auto t_ymax = (pMax[1] - ray.origin[1]) * invDir[1];
	auto x_ymax = ray.origin[0] + t_ymax * ray.direction[0];
	auto z_ymax = ray.origin[2] + t_ymax * ray.direction[2];
	if((t_ymax > 0 == dirIsNeg[1]) && ((x_ymax >= pMin[0] && x_ymax <= pMax[0]) || (z_ymax >= pMin[2] && z_ymax <= pMax[2])))
		return true;
	auto t_zmin = (pMin[2] - ray.origin[2]) * invDir[2];
	auto x_zmin = ray.origin[0] + t_zmin * ray.direction[0];
	auto y_zmin = ray.origin[1] + t_zmin * ray.direction[1];
	if((t_zmin > 0 == dirIsNeg[2]) && ((x_zmin >= pMin[0] && x_zmin <= pMax[0]) || (y_zmin >= pMin[1] && y_zmin <= pMax[1])))
		return true;
	auto t_zmax = (pMax[2] - ray.origin[2]) * invDir[2];
	auto x_zmax = ray.origin[0] + t_zmax * ray.direction[0];
	auto y_zmax = ray.origin[1] + t_zmax * ray.direction[1];
	if((t_zmax > 0 == dirIsNeg[2]) && ((x_zmax >= pMin[0] && x_zmax <= pMax[0]) || (y_zmax >= pMin[1] && y_zmax <= pMax[1])))
		return true;
	return false;
}

inline Bounds3 Union(const Bounds3 &b1, const Bounds3 &b2) {
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
	ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
	return ret;
}

inline Bounds3 Union(const Bounds3 &b, const Vector3f &p) {
	Bounds3 ret;
	ret.pMin = Vector3f::Min(b.pMin, p);
	ret.pMax = Vector3f::Max(b.pMax, p);
	return ret;
}

#endif// RAYTRACING_BOUNDS3_H
