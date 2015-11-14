#ifndef OBSTACLE_3D_H
#define OBSTACLE_3D_H

#include <Eigen/Dense>
#include <vector>

class Obstacle3d {
public:
	Obstacle3d(void);
	Obstacle3d(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
						 const Eigen::Vector3f& v2, const Eigen::Vector3f& normal,
						 float radius);

	// Accessors
	Eigen::Vector3f true_vertices(int index);
	Eigen::Vector3f translated_vertices(int index);
	Eigen::Vector3f noise_vertices(int index);
	Eigen::Vector3f normal(void);
	
	bool IsTrueIntersecting(const Eigen::Vector3f& current_position,
													const Eigen::Vector3f& desired_position);
	bool IsTranslatedIntersecting(const Eigen::Vector3f& current_position,
																const Eigen::Vector3f& desired_position);
	bool IsNoiseIntersecting(const Eigen::Vector3f& current_position,
													 const Eigen::Vector3f& desired_position);

	Eigen::Vector3f TrueIntersectionPoint(const Eigen::Vector3f& current_position,
																				const Eigen::Vector3f& desired_position);
	Eigen::Vector3f TranslatedIntersectionPoint(const Eigen::Vector3f& current_position,
																							const Eigen::Vector3f& desired_position);
	Eigen::Vector3f NoiseIntersectionPoint(const Eigen::Vector3f& current_position,
																				 const Eigen::Vector3f& desired_position);

	bool IsTrueSeeable(const Eigen::Vector3f& position);

	bool IsTranslatedSeeable(const Eigen::Vector3f& position);

private:
	bool true_intersection_calculated_;
	bool translated_intersection_calculated_;
	bool noise_intersection_calculated_;
	std::vector<Eigen::Vector3f> true_vertices_;
	std::vector<Eigen::Vector3f> translated_vertices_;
	std::vector<Eigen::Vector3f> noise_vertices_;
	Eigen::Vector3f normal_;
	
	float true_t_, true_u_, true_v_;  // Noise intersection defining variables
	float translated_t_, translated_u_, translated_v_;
	float noise_t_, noise_u_, noise_v_;

	void SolveForTrueIntersection(const Eigen::Vector3f& current_position,
																const Eigen::Vector3f& desired_position);
	void SolveForTranslatedIntersection(const Eigen::Vector3f& current_position,
																			const Eigen::Vector3f& desired_position);
	void SolveForNoiseIntersection(const Eigen::Vector3f& current_position,
																 const Eigen::Vector3f& desired_position);
};
#endif
