#include "Obstacle.h"

Eigen::VectorXf Obstacle::true_vertices(int index) {
	return true_vertices_[index];
}

Eigen::VectorXf Obstacle::translated_vertices(int index) {
	return translated_vertices_[index];
}

Eigen::VectorXf Obstacle::noise_vertices(int index) {
	return noise_vertices_[index];
}

Eigen::VectorXf Obstacle::normal(void) {
	return normal_;
}

void Obstacle::MakeNoisy(float noise) {
	for (int vertex; vertex < noise_vertices_.size(); vertex++) {
		noise_vertices_[vertex] += noise*normal_;
	}
}




