#ifndef LINE_H
#define LINE_H

#include "glad.h"
#include <glm/glm.hpp>
#include <vector>

class Line {
    int shaderProgram;
    unsigned int VBO, VAO;
    std::vector<float> vertices;
    glm::vec3 startPoint;
    glm::vec3 endPoint;
    glm::mat4 MVP;
    glm::vec3 lineColor;
public:
    Line(glm::vec3 start, glm::vec3 end);

    int setMVP(glm::mat4 mvp);

    int setColor(glm::vec3 color);
    int draw();

    int setEndpoints(glm::vec3 start, glm::vec3 end);

    ~Line();
};

#endif