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
	// Russian Roulette termination
	if(depth > maxDepth) {
		return Vector3f(0.0f);
	}

	// Find intersection with the scene
	Intersection intersection = intersect(ray);
	if(!intersection.happened) {
		return this->backgroundColor;
	}

	// If the intersected object is a light source, return its emission
	if(intersection.m->hasEmission()) {
		return intersection.m->getEmission();
	}

	Vector3f L_dir(0.0f);  // Direct lighting
	Vector3f L_indir(0.0f);// Indirect lighting

	// ----- Direct Lighting -----
	// Sample a point on the light source
	Intersection light_inter;
	float pdf_light = 0.0f;
	sampleLight(light_inter, pdf_light);

	// Compute the direction from the intersection point to the light sample
	Vector3f p  = intersection.coords;
	Vector3f x  = light_inter.coords;
	Vector3f ws = normalize(x - p);

	// Check if the light is visible from the intersection point
	Ray shadowRay(p, ws);
	Intersection shadow_inter = intersect(shadowRay);
	// If the shadow ray hits the light source directly
	if(shadow_inter.happened && (shadow_inter.coords - x).norm() < EPSILON) {
		Vector3f N    = intersection.normal;
		Vector3f NN   = light_inter.normal;
		Vector3f emit = light_inter.emit;

		// Compute BRDF, cosine terms, and the squared distance
		Vector3f f             = intersection.m->eval(ray.direction, ws, N);
		float cosTheta         = dotProduct(ws, N);
		float cosTheta_x       = dotProduct(-ws, NN);
		float distance_squared = (x - p).norm();
		distance_squared *= distance_squared;

		// Accumulate direct lighting
		L_dir = emit * f * cosTheta * cosTheta_x / (distance_squared * pdf_light);
	}

	// ----- Indirect Lighting -----
	if(get_random_float() < RussianRoulette) {
		Vector3f N  = intersection.normal;
		Vector3f wi = intersection.m->sample(ray.direction, N);
		float pdf   = intersection.m->pdf(ray.direction, wi, N);

		if(pdf > EPSILON) {
			Ray newRay(p, wi);
			Intersection new_intersection = intersect(newRay);

			// Only consider non-emitting surfaces for indirect lighting
			if(new_intersection.happened && !new_intersection.m->hasEmission()) {
				Vector3f f     = intersection.m->eval(ray.direction, wi, N);
				float cosTheta = dotProduct(wi, N);

				// Recursively compute indirect lighting
				L_indir = castRay(newRay, depth + 1) * f * cosTheta / (pdf * RussianRoulette);
			}
		}
	}

	return L_dir + L_indir;
}