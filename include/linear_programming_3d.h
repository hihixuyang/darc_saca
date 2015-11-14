#ifndef LINEAR_PROGRAMMING_3D_H
#define LINEAR_PROGRAMMING_3D_H

#include "Vector3.h"
#include <vector>

static const float RVO_EPSILON = 0.00001f;
using namespace std;

struct Plane {
public:
    Vector3 point;
    Vector3 normal;
};

struct Line {
public:
    Vector3 direction;
    Vector3 point;
};

bool linearProgram1(const std::vector<Plane>& planes,	size_t planeNo,	Line& line,
													 float radius,	const Vector3& optVelocity,	bool directionOpt,
													 Vector3& result);

bool linearProgram2(const std::vector<Plane>& planes,	size_t planeNo,
													 float radius,	const Vector3& optVelocity,	bool directionOpt,
													 Vector3& result);

size_t linearProgram3(const std::vector<Plane>& planes,	double radius,
														 const Vector3& optVelocity,	bool directionOpt,
														 Vector3& result);

void linearProgram4(const std::vector<Plane>& planes,	size_t beginPlane,
													 float radius,	Vector3& result);
#endif
