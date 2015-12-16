#include "MinkowskiSum2d.h"
#include <iostream>

MinkowskiSum2d::MinkowskiSum2d(void) {
	;
}  // MinkowskiSum2d

MinkowskiSum2d::MinkowskiSum2d(const std::vector<Eigen::Vector2f>& points_in,
															 float radius) {
	original_points_ = points_in;
	radius_ = radius;
}  // MinkowskiSum2d

std::vector<Eigen::Vector2f> MinkowskiSum2d::ReturnMinkowskiSum(int flag) {
	RemoveOutliers();
	CalculateMinkowskiSum();
	if (flag)
		RemoveIntersections();
	return minkowski_points_;
}  // ReturnMinkowskiSum

void MinkowskiSum2d::CalculateMinkowskiSum(void) {
	if (original_points_.size() > 0) {
		std::vector<Eigen::Vector3f> normals(original_points_.size() - 1);
		for (int i = 1; i < original_points_.size(); ++i) {
			Eigen::Vector3f normal;
			normal << -(original_points_[i-1][1] - original_points_[i][1]),
				original_points_[i-1][0] - original_points_[i][0], 0;
			normal.normalize();
			normals[i-1] = normal;
		}
		
		// First check between first point and last point
		if (normals.back().cross(normals.front())[2] > 0.0 &&
				normals.back().dot(normals.front()) < 0.7071) { // Find Circle
			std::vector<Eigen::Vector2f> minkowski_points =
				FindCircle(original_points_[original_points_.size()-2],
									 original_points_[0],
									 original_points_[1]);
			for (int i = 0; i < minkowski_points.size(); ++i) {
				minkowski_points_.push_back(minkowski_points[i]);
			}
		} else {  // Find Intersection
			Eigen::Vector2f minkowski_point =
				FindIntersection(original_points_[original_points_.size()-2],
												 original_points_[0], original_points_[1]);
			minkowski_points_.push_back(minkowski_point);
		}
		for (int i = 1; i < original_points_.size()-1; ++i) {
			if (normals[i-1].cross(normals[i])[2] > 0.0 &&
					normals[i-1].dot(normals[i]) < 0.7071) { // Find Circle
				std::vector<Eigen::Vector2f> minkowski_points =
					FindCircle(original_points_[i-1],
										 original_points_[i],
										 original_points_[i+1]);
				for (int i = 0; i < minkowski_points.size(); ++i) {
					minkowski_points_.push_back(minkowski_points[i]);
				}
			} else {  // Find Intersection
				Eigen::Vector2f minkowski_point = FindIntersection(original_points_[i-1],
																													original_points_[i],
																													original_points_[i+1]);
				minkowski_points_.push_back(minkowski_point);
			}
		}
		minkowski_points_.push_back(minkowski_points_.front());
	}
}  // CalculateMinkowskiSum

Eigen::Vector2f MinkowskiSum2d::FindIntersection(const Eigen::Vector2f& A,
																								 const Eigen::Vector2f& B,
																								 const Eigen::Vector2f& C) {
	Eigen::Vector2f normal_AB; normal_AB << -((A-B)/(A-B).norm())[1],
															 ((A-B)/(A-B).norm())[0];
	Eigen::Vector2f normal_BC; normal_BC << -((B-C)/(B-C).norm())[1],
															 ((B-C)/(B-C).norm())[0];

	Eigen::Vector2f a = A + radius_*normal_AB;
	Eigen::Vector2f b = B + radius_*normal_AB;
	Eigen::Vector2f c = B + radius_*normal_BC;
	Eigen::Vector2f d = C + radius_*normal_BC;

	float a_num = (d[0] - c[0]) * (a[1] - c[1]) - (d[1] - c[1]) * (a[0] - c[0]);
	float b_num = (b[0] - a[0]) * (a[1] - c[1]) - (b[1] - a[1]) * (a[0] - c[0]);
	float denom = (d[1] - c[1]) * (b[0] - a[0]) - (d[0] - c[0]) * (b[1] - a[1]);

	return a + a_num/denom*(b-a);
}  // FindIntersection

std::vector<Eigen::Vector2f> MinkowskiSum2d::FindCircle(const Eigen::Vector2f& A,
																												const Eigen::Vector2f& B,
																												const Eigen::Vector2f& C) {
	Eigen::Vector2f normal_AB; normal_AB << -(A-B)[1], (A-B)[0];
	normal_AB.normalize();
	Eigen::Vector2f normal_BC; normal_BC << -(B-C)[1], (B-C)[0];
	normal_BC.normalize();
	
	Eigen::Vector2f a2 = radius_*normal_AB;
	Eigen::Vector2f b2 = radius_*normal_BC;

	Eigen::Vector3f a; a << a2[0], a2[1], 0.0;
	Eigen::Vector3f b; b << b2[0], b2[1], 0.0;

	std::vector<Eigen::Vector2f> points;
	points.reserve(1000);
	points.push_back(B + a.head(2));
	points.push_back(B + b.head(2));
	
	std::vector<Eigen::Vector2f>::iterator it = points.begin();
	int i = 1;
	while (i < points.size()) {
		a << points[i-1][0] - B[0], points[i-1][1] - B[1], 0.0;
		b << points[i][0] - B[0], points[i][1] - B[1], 0.0;
		float theta = atan2(((a.normalized()).cross(b.normalized())).norm(),
											(a.normalized()).dot(b.normalized()));
		if (theta > 45.0*M_PI/180.0) { // Split the line
			Eigen::Vector2f c;
			c = radius_*(a + 0.5*(b-a)).normalized().head(2);
			points.insert(it+i, B + c);
		} else {
			i++;
		}
	}
	return points;
}  // FindCircle

// Go through the original point list to remove any points that are the
// end of a narrow channel where the intersection point
// Jumps to the other side away from the point at the end of the narrow
// channel. When the channel is narrower than the value of 2r then
// the point at the end should be an outlier.
void MinkowskiSum2d::RemoveOutliers(void) {
	if (original_points_.size() > 0) {
		original_points_.pop_back();
		std::vector<Eigen::Vector2f>::iterator it = original_points_.begin();
		Eigen::Vector3f A,B,C;
		Eigen::Vector3f n_ab, n_bc;
		for (int i = 0; i < original_points_.size(); ++i) {
			// If first point, compare to end
			if (i == 0) {
				A << original_points_.back()[0], original_points_.back()[1], 0;
			} else {
				A << original_points_[i - 1][0], original_points_[i - 1][1], 0;
			}
			B << original_points_[i][0], original_points_[i][1], 0;
			if (i == original_points_.size() - 1) {
				C << original_points_.front()[0], original_points_.front()[1], 0;
			} else {
				C << original_points_[i + 1][0], original_points_[i + 1][1], 0;
			}
			n_ab << -(A-B)[1], (A-B)[0], 0;
			n_ab.normalize();
			n_bc << -(B-C)[1], (B-C)[0], 0;
			n_bc.normalize();
			if (n_ab[0]*n_bc[1] - n_ab[1]*n_bc[0] < 0.0 &&
					((A + radius_*n_ab - B).cross(C-B))[2] < 0.0 ) {
				original_points_.erase(it + i);
				i = 0;
			}
		}
		original_points_.push_back(original_points_.front());
	}
}  // RemoveOutliers

// Takes the solved Minkowski difference (without outliers) and removes
// any intersections that may have occurred between lines
void MinkowskiSum2d::RemoveIntersections(void) {
	if (minkowski_points_.size() > 0) {
		std::vector<Eigen::Vector2f>::iterator it = minkowski_points_.begin();
		Eigen::Vector2f intersection_point;
		for (int i = 1; i < minkowski_points_.size() - 1; ++i) {
			for (int j = i + 2;
					 ((i == 1 && j < minkowski_points_.size() - 1) ||
						(i != 1 && j < minkowski_points_.size()));
					 ++j) {

				// DEBUGGING
				std::cout << "i: " << i << ", j: " << j << std::endl;
				// \DEBUGGING

				if (FindIntersection(minkowski_points_[i - 1], minkowski_points_[i],
														 minkowski_points_[j - 1], minkowski_points_[j],
														 intersection_point) ) {
					// DEBUGGING
					std::cout << std::endl << "Intersection Found" << std::endl;
					std::cout << "line a: (" << minkowski_points_[i-1].transpose()
										<< "), (" << minkowski_points_[i].transpose() << ")" << std::endl;
					std::cout << "line b: (" << minkowski_points_[j-1].transpose()
										<< "), (" << minkowski_points_[j].transpose() << ")" << std::endl;
					std::cout << "intersection: " << intersection_point.transpose() << std::endl;
					for (int k = 0; k < minkowski_points_.size(); ++k) {
						std::cout << minkowski_points_[k].transpose() << std::endl;
					}
					// \DEBUGGING

					if (j == minkowski_points_.size() - 1) {
						minkowski_points_.erase(it + i - 1);
						minkowski_points_.erase(it);
						minkowski_points_.pop_back();
						minkowski_points_.insert(it, intersection_point);
						minkowski_points_.push_back(intersection_point);
					} else {
						minkowski_points_.erase(it + i, it + j);
						minkowski_points_.insert(it + i, intersection_point);
					}
					
					std::cout << std::endl;
					for (int k = 0; k < minkowski_points_.size(); ++k) {
						std::cout << minkowski_points_[k].transpose() << std::endl;
					}
					int k;
					std::cin >> k;
					// \DEBUGGING
					std::cout << "Removed Intersection" << std::endl;
					i = 0;
					break;
				}
			}
			//std::cout << std::endl;
		}
	}		
}  // RemoveIntersections

// Returns true if an intersection between lines AB and CD occurs
bool MinkowskiSum2d::FindIntersection(const Eigen::Vector2f& A,
																			const Eigen::Vector2f& B,
																			const Eigen::Vector2f& C,
																			const Eigen::Vector2f& D,
																			Eigen::Vector2f& P) {
	float t_num = (A[0]-C[0])*(D[1]-C[1]) - (A[1]-C[1])*(D[0]-C[0]);
	float u_num = (A[0]-C[0])*(B[1]-A[1]) - (A[1]-C[1])*(B[0]-A[0]);
	float denom = (D[0]-C[0])*(B[1]-A[1]) - (D[1]-C[1])*(B[0]-A[0]);

	float t = t_num/denom;
	float u = u_num/denom;
	float eps = 0.0;
	if ((eps < t && t < 1-eps) && (eps < u && u < 1-eps)) {
		P << A + t*(B-A);
		return true;
	}
	return false;
}  // TestIntersection
 
	
