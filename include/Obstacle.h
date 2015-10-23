#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include <Eigen/Dense>

class Obstacle {
public:
	// Return the normal of a plane
	Eigen::VectorXf normal(void);

	// Check if a line intersects the plane
	virtual bool IsIntersecting(const Eigen::VectorXf& current_position,
															const Eigen::VectorXf& desired_position) = 0;

	// Solve for the intersection point of a line-plane if one exists
	virtual Eigen::VectorXf IntersectionPoint(const Eigen::VectorXf& current_position,
																						const Eigen::VectorXf& desired_position) = 0;
	
protected:
	std::vector<Eigen::VectorXf> true_vertices_;
	std::vector<Eigen::VectorXf> translated_vertices_;  // Minkowski sum result
	std::vector<Eigen::VectorXf> noise_vertices_;  // Vertices with offset for noise
	Eigen::VectorXf normal_;

  // Add noise to definition of obstacle
	void MakeNoisy(float noise);

  // Check dot product of position and normal of true vertices
	virtual bool IsTrueSeeable(const Eigen::VectorXf& position) = 0;

	// Check dot product of position and normal of translated vertices
	virtual bool IsTranslatedSeeable(const Eigen::VectorXf& position) = 0;

	// Solve for the intersection varaibles
	virtual void SolveForIntersection(const Eigen::VectorXf& current_position,
																		const Eigen::VectorXf& desired_position) = 0;

};

#endif
