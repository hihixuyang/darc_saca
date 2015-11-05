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
	
	bool IsIntersecting(const Eigen::Vector3f& current_position,
											const Eigen::Vector3f& desired_position);

	Eigen::Vector3f IntersectionPoint(const Eigen::Vector3f& current_position,
																		const Eigen::Vector3f& desired_position);

	bool IsTrueSeeable(const Eigen::Vector3f& position);

	bool IsTranslatedSeeable(const Eigen::Vector3f& position);

private:
	std::vector<Eigen::Vector3f> true_vertices_;
	std::vector<Eigen::Vector3f> translated_vertices_;
	std::vector<Eigen::Vector3f> noise_vertices_;
	Eigen::Vector3f normal_;
	
	float t_, u_, v_;  // Intersection defining variables

	void SolveForIntersection(const Eigen::Vector3f& current_position,
														const Eigen::Vector3f& desired_position);
};

#endif
