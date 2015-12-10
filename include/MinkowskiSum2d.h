#ifndef MINKOWSKI_SUM_2D_H_
#define MINKOWSKI_SUM_2D_H_

#include <vector>
#include <Eigen/Dense>

class MinkowskiSum2d {
public:
	MinkowskiSum2d(void);
	MinkowskiSum2d(const std::vector<Eigen::Vector2f>& points_in, float radius);
	std::vector<Eigen::Vector2f> CalculateMinkowskiSum(void);
	
private:
	std::vector<Eigen::Vector2f> original_points_;
	std::vector<Eigen::Vector2f> minkowski_points_;
	float radius_;

	Eigen::Vector2f FindIntersection(const Eigen::Vector2f&,
																	 const Eigen::Vector2f&,
																	 const Eigen::Vector2f&);
	std::vector<Eigen::Vector2f> FindCircle(const Eigen::Vector2f&,
																					const Eigen::Vector2f&,
																					const Eigen::Vector2f&);
	void RemoveOutliers(void);
};

#endif
