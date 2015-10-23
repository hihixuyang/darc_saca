#include "Obstacle.h"

void Obstacle::MakeNoisy(float noise) {
	for (int vertex; vertex < noise_vertices_.size(); vertex++) {
		noise_vertices_[vertex] += noise*normal_;
	}
}

Eigen::VectorXf Obstacle::normal(void) {
	return normal_;
}


