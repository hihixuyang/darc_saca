#include "MinkowskiSum2d.h"
#include <iterator>
#include <algorithm>
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
// the point at the end should be an outlier. To do this, start at a point
// with a concave corner, then loop through the list until returning to the
// current point and skipping any entries that are a distance less than
// 2r from the current point
void MinkowskiSum2d::RemoveOutliers(void) {
	typedef Eigen::Vector2f vec2;
	typedef Eigen::Vector3f vec3;

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
		/*
		// Loop over each point in the list
		for (std::vector<vec2>::iterator it = original_points_.begin();
				 it != original_points_.end(); ++it) {
			vec3 a;
			if (it != original_points_.begin()) {
				a << (*std::prev(it))[0], (*std::prev(it))[1], 0.0;
			} else {
				a << (*std::prev(original_points_.end()))[0],
					(*std::prev(original_points_.end()))[1], 0.0;
			}
			vec3 b; b << (*it)[0], (*it)[1], 0.0;
			vec3 c;
			if (it != std::prev(original_points_.end())) {
				c << (*std::next(it))[0], (*std::next(it))[1], 0.0;
			} else {
				c << (*original_points_.begin())[0], (*original_points_.begin())[1], 0.0;
			}
			
			if ((c - a).cross(c - b)[2] > 0.0) {  // is corner concave?
				// If the corner is concave, loop to find the nearest point from
				// the first point on the corner that is concave (a)
				if (it != original_points_.end() &&
						std::next(it) != original_points_.end() &&
						std::next(it,2) != original_points_.end()) {
					std::vector<vec2>::iterator nearest_it =
						std::min_element(std::next(it,2), original_points_.end(),
														 [it](const vec2& one, const vec2& two) {
															 return (*it - one).norm() < (*it - two).norm();
														 });
					if (nearest_it != original_points_.end()) {
						// Check if the projection of that point onto the line between
						// it and next(it) is less than 2r
						vec2 la = *it - *std::next(it);
						vec2 lb = *nearest_it - *std::next(it);
						double d = (lb - la.normalized() * (lb.transpose()*la.normalized())).norm();
						if (d <= 2.0 * radius_ ) {
							// First check the forward loop;
							vec2 p1 = *it;
							int counter = 0;
							for (auto next_it = it; next_it != std::next(nearest_it); ++next_it) {
								vec2 p2 = *next_it;
								if ((std::min(p1[1], p2[1]) < 0.0) &&
										(std::max(p1[1], p2[1]) >= 0.0) &&
										(std::max(p1[0], p2[0]) >= 0.0) && (p1[1] != p2[1])) {
									double xinterst = -p1[1] * (p2[0] - p1[0]) / (p2[1] - p1[1]) + p1[0];
									if (p1[0] == p2[0] || 0.0 <= xinterst)
										counter++;
								}
								p1 = p2;
							}
							if (counter % 2 != 0) {  // point is inside, use this circle
								original_points_.erase(std::next(nearest_it), original_points_.end());
								original_points_.erase(original_points_.begin(), it);
								//it = original_points_.begin();
							} else {
								original_points_.erase(std::next(it), nearest_it);
								//it = original_points_.begin();
							}
						}
					}
				}
			}			
		}
		*/
		original_points_.push_back(original_points_[0]);
	}

}  // RemoveOutliers
