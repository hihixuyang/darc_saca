#include "LidarSegment.h"
#include <iostream>

// Default Constructor
LidarSegment::LidarSegment(void) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.25;
}  // LidarSegment

// Custom constructor
LidarSegment::LidarSegment(std::vector<Eigen::Vector2f>& point_list) {
	distance_threshold_ = 0.05;
	radius_threshold_ = 0.25;
	for (int point_index = 0; point_index < point_list.size(); ++point_index) {
		full_points_.push_back(point_list[point_index]);
	}
}  // LidarSegment

// Custom constructor
LidarSegment::LidarSegment(std::vector<Eigen::Vector2f>& point_list,
													 float thresh) {
	distance_threshold_ = thresh;
	radius_threshold_ = 0.25;
	for (int point_index = 0; point_index < point_list.size(); ++point_index) {
		full_points_.push_back(point_list[point_index]);
	}
}  // LidarSegment

// Calculate then return the list of segmented points
std::vector<Eigen::Vector2f> LidarSegment::segmented_points(void) {
	// Pre-process points to do clustering
	if (full_points_.size() > 0) {
		std::vector< std::vector<int> > segments(2);
		std::vector<int> low_segment(full_points_.size());
		std::vector<int> high_segment(2);
		for (int index = 0; index < full_points_.size(); ++index) {
			low_segment[index] = index;
		}
		high_segment[0] = full_points_.size()-1;
		high_segment[1] = 0;
		segments[0] = low_segment;
		segments[1] = high_segment;
		segmented_indices_.clear();

		// Cluster based on radii difference
		PerformClustering(segments);
		// Segment using split and merge
		PerformSplitAndMerge(segments, 0);
		
		for (int i = 0; i < segments.size(); ++i) {
			segmented_points_.push_back(full_points_[segmented_indices_[i]]);
		}
		segmented_points_.push_back(full_points_[segmented_indices_[0]]);
	}
	return segmented_points_;
} // segmented_points

// Perform an initial clustering from radius criterion
void LidarSegment::PerformClustering(std::vector< std::vector<int> >& segments) {
	for (int segment_index = 0; segment_index < segments.size(); ++segment_index) {
		if (segments[segment_index].size() > 2) {  // skip groups of 2
			for (int point_index = 1; point_index < segments[segment_index].size();
					 ++point_index) {
				if(abs(full_points_[segments[segment_index][point_index]].norm() -
							 full_points_[segments[segment_index][point_index - 1]].norm()) >
					 radius_threshold_) {  // radial distance between two is too big, split
					std::vector< std::vector<int> > split_segments =
						SplitSegment(segments[segment_index], point_index);
					segments[segment_index].swap(split_segments[1]);
					segments.insert(segments.begin() + segment_index, split_segments[0]);
				}
			}
		}
	}
}  // PerformClustering

// Perform computation of segmentation into lines
void LidarSegment::PerformSplitAndMerge(std::vector<std::vector<int> >& segments,
																				int segment_index) {
 	for (; segment_index < segments.size(); ++segment_index) {
		if (segments[segment_index].size() > 2) {  // line is already finished
			std::vector< std::vector<int> > split_segments =
				SplitAtFurthestPoint(segments[segment_index]);
			if(split_segments.size() > 1) {  // line was split
				segments[segment_index].swap(split_segments[1]);
				segments.insert(segments.begin() + segment_index, split_segments[0]);
			} else {  // line was not split form distance criterion
				segments[segment_index].swap(split_segments[0]);
			}
			PerformSplitAndMerge(segments, segment_index);
		} else {
			segmented_indices_.push_back(segments[segment_index][0]);
		}
	}
}  // PerformSegmentation

// Split a line segment at the furthest point
std::vector< std::vector<int> >
LidarSegment::SplitAtFurthestPoint(const std::vector<int>& segment) {
	Point break_point = PointFarthestFromLine(segment);
	std::vector< std::vector<int> >segments;
	if (break_point.distance_ > distance_threshold_) {
		segments = SplitSegment(segment, break_point.index_);
	} else {
		std::vector<int> full_segment(2);
		full_segment[0] = segment.front();
		full_segment[1] = segment.back();
		segments.push_back(full_segment);
	}
	return segments;
}  // SplitLineSegment

// Split a line segment at a given index
std::vector< std::vector<int> >
LidarSegment::SplitSegment(const std::vector<int>& segment, int split_index) {
	std::vector<int> low_segment;
	for (int low_index = 0; low_index < split_index + 1; ++low_index) {
		low_segment.push_back(segment[low_index]);
	}
	std::vector<int> high_segment;
	for (int high_index = split_index; high_index < segment.size(); ++high_index) {
		high_segment.push_back(segment[high_index]);
	}
	std::vector< std::vector<int> > segments;
	segments.push_back(low_segment);
	segments.push_back(high_segment);
	return segments;
}  // SplitSegment
							 
LidarSegment::Point
LidarSegment::PointFarthestFromLine(const std::vector<int>& point_list) {
	float max_distance = FindDistance(full_points_[point_list.front()],
																		full_points_[point_list.back()],
																		full_points_[point_list[1]]);
	int max_index = 1;
	for (int point_index = 2; point_index < point_list.size()-1; ++point_index) {
		float distance = FindDistance(full_points_[point_list.front()],
																	full_points_[point_list.back()],
																	full_points_[point_list[point_index]]);
		if (distance > max_distance) {
			max_distance = distance;
			max_index = point_index;
		}
	}
	Point farthest_point;
	farthest_point.index_ = max_index;
	farthest_point.distance_ = max_distance;
	return farthest_point;
}  // PointFarthestFromLine

// Find a the distance between point C and its projection onto the segment
// between points a and b
float LidarSegment::FindDistance(const Eigen::Vector2f& a,
																 const Eigen::Vector2f& b,
																 const Eigen::Vector2f& c) {
	Eigen::Vector2f ba_hat = (b-a).normalized();
	float t = (c-a).transpose()*ba_hat;
	if (t < 0 ) {  // outside a
		return (c-a).norm();
	} else if (t > (b-a).norm()) {  // outside b
		return (c-b).norm();
	}
	return (c-a-t*ba_hat).norm();
}  // FindDistance


