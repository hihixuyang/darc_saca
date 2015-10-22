#include "linear_programming_3d.h"
#include <cmath>
#include <iostream>

// From ORCA_3D
bool linearProgram1(const std::vector<Plane>& planes,	size_t planeNo,	Line& line,
										float radius,	const Vector3& optVelocity,	bool directionOpt,
										Vector3& result)  {
  const float dotProduct = line.point * line.direction;
  const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(line.point);

  if (discriminant < 0.0f) {
    // Max speed sphere fully invalidates line. 
    return false;
  }

  const float sqrtDiscriminant = std::sqrt(discriminant);
  float tLeft = -dotProduct - sqrtDiscriminant;
  float tRight = -dotProduct + sqrtDiscriminant;

  for (size_t i = 0; i < planeNo; ++i) {
    const float numerator = (planes[i].point - line.point) * planes[i].normal;
    const float denominator = line.direction * planes[i].normal;
      
    if (sqr(denominator) <= RVO_EPSILON) {
      // Lines line is (almost) parallel to plane i. 
      if (numerator > 0.0f) {
        return false;
      } else {
        continue;
      }
    }

    const float t = numerator / denominator;

    if (denominator >= 0.0f) {
      // Plane i bounds line on the left. 
      tLeft = max(tLeft, t);
    } else {
      // Plane i bounds line on the right. 
      tRight = min(tRight, t);
    }

    if (tLeft > tRight) {
      return false;
    }
  }

  if (directionOpt) {
    // Optimize direction. 
    if (optVelocity * line.direction > 0.0f) {
      // Take right extreme. 
      result = line.point + tRight * line.direction;
    } else {
      // Take left extreme. 
      result = line.point + tLeft * line.direction;
    }
  } else {
    // Optimize closest point. 
    const float t = line.direction * (optVelocity - line.point);

    if (t < tLeft) {
      result = line.point + tLeft * line.direction;
    } else if (t > tRight) {
      result = line.point + tRight * line.direction;
    } else {
      result = line.point + t * line.direction;
    }
  }
  return true;
}

// From ORCA_3D
bool linearProgram2(const std::vector<Plane>& planes,	size_t planeNo,
										float radius,	const Vector3& optVelocity,	bool directionOpt,
										Vector3& result) {
  const float planeDist = planes[planeNo].point * planes[planeNo].normal;
  const float planeDistSq = sqr(planeDist);
  const float radiusSq = sqr(radius);
  if (planeDistSq > radiusSq) {
    // Max speed sphere fully invalidates plane planeNo. 
    return false;
  }
    
  const float planeRadiusSq = radiusSq - planeDistSq;
  const Vector3 planeCenter = planeDist * planes[planeNo].normal;

  if (directionOpt) {
    // Project direction optVelocity on plane planeNo 
    const Vector3 planeOptVelocity = optVelocity
			-	(optVelocity * planes[planeNo].normal) * planes[planeNo].normal;
    const float planeOptVelocityLengthSq = absSq(planeOptVelocity); 
    if (planeOptVelocityLengthSq <= RVO_EPSILON) {
      result = planeCenter;
    } else {
      result = planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq)
				*	planeOptVelocity; 
    }
  } else {
    // Project point optVelocity on plane planeNo 
    result = optVelocity
			+ ((planes[planeNo].point - optVelocity) * planes[planeNo].normal)
			  * planes[planeNo].normal;
    // If outside planeCircle, project on planeCircle 
    if (absSq(result) > radiusSq) {
      const Vector3 planeResult = result - planeCenter;
      const float planeResultLengthSq = absSq(planeResult);
      result = planeCenter + std::sqrt(planeRadiusSq / planeResultLengthSq)
				* planeResult;
    }
  }

  for (size_t i = 0; i < planeNo; ++i) {
    if (planes[i].normal * (planes[i].point - result) > 0.0f) {
      // Result does not satisfy constraint i. Compute new optimal result.
      // Compute intersection line of plane i and plane planeNo. 
      Vector3 crossProduct = cross(planes[i].normal, planes[planeNo].normal);
      
      if (absSq(crossProduct) <= RVO_EPSILON) {
        // Planes planeNo and i are (almost) parallel, and
        // plane i fully invalidates plane planeNo.   
        return false;
      }

      Line line;
      line.direction = normalize(crossProduct);
      const Vector3 lineNormal = cross(line.direction, planes[planeNo].normal);
      line.point = planes[planeNo].point	+ (
				((planes[i].point - planes[planeNo].point) * planes[i].normal) /
				(lineNormal * planes[i].normal)) * lineNormal;

      if (!linearProgram1(planes, i, line, radius, optVelocity,
													directionOpt, result)) {
        return false;
      }
    }
  }  
  return true;
}
    
// From ORCA_3D
size_t linearProgram3(const std::vector<Plane>& planes,	double radius,
											const Vector3& optVelocity,	bool directionOpt,
											Vector3& result)  {
  if (directionOpt) {
    // Optimize direction. Note that the optimization velocity is of unit
    // length_ in this case.   
    result = optVelocity * radius;
  } else if (absSq(optVelocity) > sqr(radius)) {
    // Optimize closest point and outside circle. 
    result = normalize(optVelocity) * radius;
  } else {
    // Optimize closest point and inside circle. 
    result = optVelocity;
  }

  for (size_t i = 0; i < planes.size(); ++i) {
    if (planes[i].normal * (planes[i].point - result) > 0.0f) {
      // Result does not satisfy constraint i. Compute new optimal result. 
      const Vector3 tempResult = result;
        
      if (!linearProgram2(planes, i, radius, optVelocity,
													directionOpt, result)) {
        result = tempResult;
        return i;
      }
    }
  }
  return planes.size();
}
  
// From ORCA_3D
void linearProgram4(const std::vector<Plane>& planes,	size_t beginPlane,
										float radius,	Vector3& result) {
  float distance = 0.0f;

  for (size_t i = beginPlane; i < planes.size(); ++i) {
    if (planes[i].normal * (planes[i].point - result) > distance) {
      // Result does not satisfy constraint of plane i.
      std::vector<Plane> projPlanes;

      for (size_t j = 0; j < i; ++j) {
        Plane plane;

        const Vector3 crossProduct = cross(planes[j].normal, planes[i].normal);

        if (absSq(crossProduct) <= RVO_EPSILON) {
          // Plane i and plane j are (almost) parallel.
          if (planes[i].normal * planes[j].normal > 0.0f) {
            // Plane i and plane j point in the same direction.
            continue;
          } else {
            // Plane i and plane j point in opposite direction.
            plane.point = 0.5f * (planes[i].point + planes[j].point);
          }
        } else {
          // Plane.point is point on line of intersection between plane i and plane j
          const Vector3 lineNormal = cross(crossProduct, planes[i].normal);
          plane.point = planes[i].point + (((planes[j].point - planes[i].point)
					  * planes[j].normal) / (lineNormal * planes[j].normal)) * lineNormal;
        }

        plane.normal = normalize(planes[j].normal - planes[i].normal);
        projPlanes.push_back(plane);
      }

      const Vector3 tempResult = result;

      if (linearProgram3(projPlanes, radius, planes[i].normal, true, result)
					< projPlanes.size()) {
        // This should in principle not happen.  The result is by definition
        // already in the feasible region of this linear program. If it fails,
        // it is due to small floating point error, and the current result is
        // kept.
        result = tempResult;
      }

      distance = planes[i].normal * (planes[i].point - result);
    }
  }
}
