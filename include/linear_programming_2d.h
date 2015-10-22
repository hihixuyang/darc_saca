#ifndef LINEAR_PROGRAMMING_2D_H
#define LINEAR_PROGRMAMING_2D_H

#include "Vector2.h"
#include <vector>

// Error value for ORCA library functions
static const float RVO_EPSILON = 0.00001f;

// Line struct for 2-D half-planes
struct Line {
	Vector2 point;//@brief		A point on the directed line.
	Vector2 direction;//@brief		The direction of the directed line.
};

// From ORCA2D
inline bool linearProgram1(const std::vector<Line> &lines, size_t lineNo,
													 float radius, const Vector2 &optVelocity,
													 bool directionOpt, Vector2 &result);

// From ORCA2D
inline size_t linearProgram2(const std::vector<Line> &lines, float radius,
														 const Vector2 &optVelocity, bool directionOpt,
														 Vector2 &result);

// From ORCA2D - Used when others fail
inline void linearProgram3(const std::vector<Line> &lines, size_t numObstLines,
													 size_t beginLine, float radius, Vector2 &result);

#endif
