#ifndef OBSTACLE_3D_H
#define OBSTACLE_3D_H

#include "Obstacle.h"
#include <Eigen/Dense>

class Obstacle3d : public Obstacle {
public:
	Obstacle3d(void);
	Obstacle3d(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
						 const Eigen::Vector3f& v2, const Eigen::Vector3f& normal,
						 float radius);
	
	bool IsIntersecting(const Eigen::VectorXf& current_position,
											const Eigen::VectorXf& desired_position);

	Eigen::VectorXf IntersectionPoint(const Eigen::VectorXf& current_position,
																		const Eigen::VectorXf& desired_position);

	bool IsTrueSeeable(const Eigen::VectorXf& position);

	bool IsTranslatedSeeable(const Eigen::VectorXf& position);

protected:
	float t_, u_, v_;  // Intersection defining variables

	void SolveForIntersection(const Eigen::VectorXf& current_position,
														const Eigen::VectorXf& desired_position);
};

#endif
