#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>
#include "glad.h"


class CollisionInfo {
public:
  glm::vec3 normal;
  glm::vec3 position;
  float depth;
  CollisionInfo();
};


class BCube {
public:
    unsigned int shaderProgram;
    unsigned int VBO, VAO;

    GLuint queryFront;
    GLuint queryBack;
    std::vector<glm::vec3> _vertices = {};
    std::vector<glm::vec3> translated_vertices = {};
    std::vector<float> vertices = {};
    glm::vec3 position;
    glm::vec3 velocity;

    glm::mat4 model;

    BCube();

    int getTranslatedVertices(glm::mat4 modelMatrix);

    void setup(glm::vec3 min, glm::vec3 max);

    void setMatrix(std::string name, glm::mat4 mat);
    void draw();
};

glm::vec3 getPosition(glm::mat4 mat);

float intersectPlane(glm::vec3 n, glm::vec3 p0, glm::vec3 l0, glm::vec3 l);
CollisionInfo rayBoxIntersect(glm::vec3 rayOrigin, glm::vec3 rayDirection, BCube &boundingBox);
float rayTriangleIntersectNegative(
    const glm::vec3 &rayOrigin, const glm::vec3 &rayVector,
    const glm::vec3 &vertex0, const glm::vec3 &vertex1, const glm::vec3 &vertex2);

CollisionInfo check_collision(BCube &boundingCube1, BCube &boundingCube2);
glm::vec3 reflect(glm::vec3 I, glm::vec3 N);

#endif