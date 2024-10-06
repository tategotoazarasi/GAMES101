//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"

enum MaterialType { DIFFUSE };

class Material {
private:
	// Compute reflection direction
	Vector3f reflect(const Vector3f &I, const Vector3f &N) const {
		return I - 2 * dotProduct(I, N) * N;
	}

	// Compute refraction direction using Snell's law
	//
	// We need to handle with care the two possible situations:
	//
	//    - When the ray is inside the object
	//
	//    - When the ray is outside.
	//
	// If the ray is outside, you need to make cosi positive cosi = -N.I
	//
	// If the ray is inside, you need to invert the refractive indices and negate the normal N
	Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const {
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		Vector3f n = N;
		if(cosi < 0) {
			cosi = -cosi;
		} else {
			std::swap(etai, etat);
			n = -N;
		}
		float eta = etai / etat;
		float k   = 1 - eta * eta * (1 - cosi * cosi);
		return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
	}

	// Compute Fresnel equation
	//
	// \param I is the incident view direction
	//
	// \param N is the normal at the intersection point
	//
	// \param ior is the material refractive index
	//
	// \param[out] kr is the amount of light reflected
	void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const {
		float cosi = clamp(-1, 1, dotProduct(I, N));
		float etai = 1, etat = ior;
		if(cosi > 0) {
			std::swap(etai, etat);
		}
		// Compute sini using Snell's law
		float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
		// Total internal reflection
		if(sint >= 1) {
			kr = 1;
		} else {
			float cost = sqrtf(std::max(0.f, 1 - sint * sint));
			cosi       = fabsf(cosi);
			float Rs   = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
			float Rp   = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
			kr         = (Rs * Rs + Rp * Rp) / 2;
		}
		// As a consequence of the conservation of energy, transmittance is given by:
		// kt = 1 - kr;
	}

	Vector3f toWorld(const Vector3f &a, const Vector3f &N) {
		Vector3f B, C;
		if(std::fabs(N.x) > std::fabs(N.y)) {
			float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
			C            = Vector3f(N.z * invLen, 0.0f, -N.x * invLen);
		} else {
			float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
			C            = Vector3f(0.0f, N.z * invLen, -N.y * invLen);
		}
		B = crossProduct(C, N);
		return a.x * B + a.y * C + a.z * N;
	}

public:
	MaterialType m_type;
	//Vector3f m_color;
	Vector3f m_emission;
	float ior;
	Vector3f Kd, Ks;
	float specularExponent;
	//Texture tex;

	inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0));
	inline MaterialType getType();
	//inline Vector3f getColor();
	inline Vector3f getColorAt(double u, double v);
	inline Vector3f getEmission();
	inline bool hasEmission();

	// sample a ray by Material properties
	inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
	// given a ray, calculate the PdF of this ray
	inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
	// given a ray, calculate the contribution of this ray
	inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
};

Material::Material(MaterialType t, Vector3f e) {
	m_type = t;
	//m_color = c;
	m_emission = e;
}

MaterialType Material::getType() { return m_type; }
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() { return m_emission; }
bool Material::hasEmission() {
	if(m_emission.norm() > EPSILON)
		return true;
	else
		return false;
}

Vector3f Material::getColorAt(double u, double v) {
	return Vector3f();
}

/**
 * 按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
 */
Vector3f Material::sample(const Vector3f &wi, const Vector3f &N) {
	switch(m_type) {
		case DIFFUSE: {
			// uniform sample on the hemisphere
			float x_1 = get_random_float(), x_2 = get_random_float();
			float z = std::fabs(1.0f - 2.0f * x_1);
			float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
			Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
			return toWorld(localRay, N);

			break;
		}
	}
}

/**
 * @brief 计算材质的概率密度函数（PDF）。
 *
 * @param wi 入射方向，通常表示光线从光源进入表面的方向，使用局部坐标系（物体表面）表示。
 * @param wo 出射方向，表示光线从表面反射或透射的方向，使用局部坐标系表示。
 * @param N  表面的法向量，通常为单位向量，表示表面的朝向，用于计算光线和表面的角度关系。
 * @return float 返回的概率密度函数值，代表在特定方向（wo）上采样的概率。
 */
float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
	switch(m_type) {
		case DIFFUSE: {
			// uniform sample probability 1 / (2 * PI)
			if(dotProduct(wo, N) > 0.0f)
				return 0.5f / M_PI;
			else
				return 0.0f;
			break;
		}
	}
}

/**
 * @brief 评估材质的BRDF（双向反射分布函数）。
 *
 * @param wi 入射方向，表示光线进入物体表面的方向，通常在局部坐标系中表示。
 * @param wo 出射方向，表示光线从表面反射的方向，也在局部坐标系中表示。
 * @param N  法向量，表示交点处的物体表面法线方向，通常是单位向量。
 * @return Vector3f 返回BRDF的值，用于计算反射光的强度。
 */
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
	switch(m_type) {
		case DIFFUSE: {
			// calculate the contribution of diffuse   model
			float cosalpha = dotProduct(N, wo);
			if(cosalpha > 0.0f) {
				Vector3f diffuse = Kd / M_PI;
				return diffuse;
			} else
				return Vector3f(0.0f);
			break;
		}
	}
}

#endif//RAYTRACING_MATERIAL_H
