#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <vector>
#include <glm/glm.hpp>

class OrientedBoundingBox {
public:
	OrientedBoundingBox(glm::mat4 m);

	std::vector<glm::vec3> getTranslatedVertices();

	bool isCollidingWith(OrientedBoundingBox other);

private:
	glm::mat4 model;
    std::vector<float> vertices = {};
};


float intersectPlane(glm::vec3 n, glm::vec3 p0, glm::vec3 l0, glm::vec3 l);

#endif