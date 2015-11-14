#ifndef OBSTACLE_2D_H
#define OBSTACLE_2D_H

#include <Eigen/Dense>
#include <vector>

class Obstacle2d {
public:
	Obstacle2d(void);
	Obstacle2d(const Eigen::Vector2f& v0, const Eigen::Vector2f& v1,
						 const Eigen::Vector2f& normal, float radius);

	// Accessors
	Eigen::Vector2f true_vertices(int index);
	Eigen::Vector2f translated_vertices(int index);
	Eigen::Vector2f noise_vertices(int index);
	Eigen::Vector2f normal(void);

	bool IsTrueIntersecting(const Eigen::Vector2f& current_position,
													const Eigen::Vector2f& desired_position);
	bool IsTranslatedIntersecting(const Eigen::Vector2f& current_position,
																const Eigen::Vector2f& desired_position);
	bool IsNoiseIntersecting(const Eigen::Vector2f& current_position,
													 const Eigen::Vector2f& desired_position);

	Eigen::Vector2f TrueIntersectionPoint(const Eigen::Vector2f& current_position,
																				const Eigen::Vector2f& desired_position);
	Eigen::Vector2f TranslatedIntersectionPoint(const Eigen::Vector2f& current_position,
																							const Eigen::Vector2f& desired_position);
	Eigen::Vector2f NoiseIntersectionPoint(const Eigen::Vector2f& current_position,
																				 const Eigen::Vector2f& desired_position);

	bool IsTrueSeeable(const Eigen::Vector2f& position);
	bool IsTranslatedSeeable(const Eigen::Vector2f& position);

private:
	bool true_intersection_calculated_;
	bool translated_intersection_calculated_;
	bool noise_intersection_calculated_;
	std::vector<Eigen::Vector2f> true_vertices_;
	std::vector<Eigen::Vector2f> translated_vertices_;
	std::vector<Eigen::Vector2f> noise_vertices_;
	Eigen::Vector2f normal_;

  float true_a_, true_b_, true_c_;
	float translated_a_, translated_b_, translated_c_;
	float noise_a_, noise_b_, noise_c_;

	void SolveForTrueIntersection(const Eigen::Vector2f& current_position,
																const Eigen::Vector2f& desired_position);
	void SolveForTranslatedIntersection(const Eigen::Vector2f& current_position,
																			const Eigen::Vector2f& desired_position);
	void SolveForNoiseIntersection(const Eigen::Vector2f& current_position,
																 const Eigen::Vector2f& desired_position);
};
#endif
