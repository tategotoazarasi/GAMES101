//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
	float emit_area_sum = 0;
	for(uint32_t k = 0; k < objects.size(); ++k) {
		if(objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p       = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for(uint32_t k = 0; k < objects.size(); ++k) {
		if(objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if(p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object *> &objects,
        float &tNear, uint32_t &index, Object **hitObject) {
	*hitObject = nullptr;
	for(uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if(objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear      = tNearK;
			index      = indexK;
		}
	}


	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const {
	// Implement Path Tracing Algorithm here
	// sampleLight (inter, pdf_light)
	auto intersection = intersect(ray);
	if(!intersection.happened || depth > maxDepth || intersection.m == nullptr) {
		return this->backgroundColor;
	}
	auto pdf_light = intersection.m->pdf(ray.direction, intersection.m->sample(ray.direction, intersection.normal), intersection.normal);
	sampleLight(intersection, pdf_light);

	// Get x, ws, NN, emit from inter
	auto x    = intersection.coords;                        ///< 光源上采样到的交点坐标，即光线与光源相交时的位置
	auto ws   = normalize(intersection.coords - ray.origin);///< 光源上交点处的入射方向
	auto nn   = normalize(intersection.normal);             ///< 光源上采样点的法向量
	auto emit = intersection.emit;                          ///< 光源在采样点处的自发光强度或颜色

	// Shoot a ray from p to x
	auto new_ray              = Ray(x, ws);
	auto new_ray_intersection = intersect(new_ray);
	Vector3f L_dir            = {0, 0, 0};
	if(!new_ray_intersection.happened && new_ray_intersection.m != nullptr) {
		L_dir = new_ray_intersection.emit * new_ray_intersection.m->eval(ws, new_ray_intersection.m->sample(ws, nn), nn) * dotProduct(ws, nn) * dotProduct(ws, nn) / powf((x - ray.origin).norm(), 2) / pdf_light;
	}

	Vector3f L_indir = {0, 0, 0};
	if(get_random_float() < RussianRoulette) {
		auto wi               = normalize(intersection.m->sample(ray.direction, intersection.normal));
		auto pdf              = intersection.m->pdf(ray.direction, wi, intersection.normal);
		auto f                = intersection.m->eval(ray.direction, wi, intersection.normal);
		auto new_ray2         = Ray(intersection.coords, wi);
		auto new_intersection = intersect(new_ray2);
		if(new_intersection.happened) {
			L_indir = castRay(new_ray2, depth + 1) * f * dotProduct(wi, intersection.normal) * dotProduct(-wi, new_intersection.normal) / pdf / RussianRoulette;
		}
	}
	return L_dir + L_indir;
}