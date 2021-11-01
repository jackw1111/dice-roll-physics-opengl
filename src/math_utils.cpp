#include "math_utils.h"

glm::vec3 reflect(glm::vec3 I, glm::vec3 N){
    return I  + N  * -2.0f * glm::dot(N, I);
}

CollisionInfo::CollisionInfo() {
  position = glm::vec3(0,0,0);
  normal = glm::vec3(0,0,0);
  depth = 0.0f;
}

float rayTriangleIntersectNegative(
    const glm::vec3 &rayOrigin, const glm::vec3 &rayVector,
    const glm::vec3 &vertex0, const glm::vec3 &vertex1, const glm::vec3 &vertex2)
{
  float t;
  // Moller-trumbore intersection algorithm
  const float EPSILON = 0.0000001f;
  glm::vec3 edge1, edge2, h, s, q;
  float a, f, u, v;
  edge1 = vertex1 - vertex0;
  edge2 = vertex2 - vertex0;
  h = glm::cross(rayVector, edge2);
  a = glm::dot(edge1, h);
  if (a > -EPSILON && a < EPSILON)
    return false; // this ray is parallel to this triangle plane
  f = 1.0f/a;
  s = rayOrigin - vertex0;
  u = f * glm::dot(s, h);
  if (u < 0.0f || u > 1.0f)
    return false;
  q = glm::cross(s, edge1);
  v = f * glm::dot(rayVector, q);
  if (v < 0.0f || u + v > 1.0f)
    return false;
  // at this stage we can compute t to find out where the intersection point is on the line
  t = f * glm::dot(edge2, q);
  if (t < 1/EPSILON) // ray intersection
  {
    return t;
  }
  else // line intersection but no ray intersection
    return 0.0f;
}

glm::vec3 getPosition(glm::mat4 mat){
  glm::vec4 v4 = mat[3];
  return glm::vec3(v4.x, v4.y, v4.z);
}

CollisionInfo rayBoxIntersect(glm::vec3 rayOrigin, glm::vec3 rayDirection, BCube &boundingBox)
{   
  CollisionInfo c;
  std::vector<float> intersect_values = {};
  std::vector<glm::vec3> normals = {};
  std::vector<glm::vec3> positions = {};

  for (unsigned int i = 0; i < boundingBox.translated_vertices.size(); i+=3) {
      glm::vec3 v1 = boundingBox.translated_vertices.at(i + 0);
      glm::vec3 v2 = boundingBox.translated_vertices.at(i + 1);
      glm::vec3 v3 = boundingBox.translated_vertices.at(i + 2);
      float t = rayTriangleIntersectNegative(rayOrigin, rayDirection, v1, v2, v3);
      if (t) {
        intersect_values.push_back(t);
        glm::vec3 intersectionPosition = rayOrigin + rayDirection * t;
        positions.push_back(intersectionPosition);
        glm::vec3 normal = glm::normalize(glm::cross(v2-v1, v3-v1));
        //std::cout << "1. " << glm::to_string(normal) << std::endl;

        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(boundingBox.model)));
        glm::vec3 _v1 = boundingBox._vertices.at(i + 0);
        glm::vec3 _v2 = boundingBox._vertices.at(i + 1);
        glm::vec3 _v3 = boundingBox._vertices.at(i + 2);
        normal = glm::normalize(glm::cross(_v2-_v1, _v3-_v1));
        normal = glm::normalize(normalMatrix * normal);

        //std::cout << "2. " << glm::to_string(normal) << std::endl;
        // recalculate outward normals
        // if (dot(intersectionPosition - boundingBox.position, normal) < 0.0f) {
        //   normal *= -1.0f;
        // }
        normals.push_back(normal);
      }
  }

  if (intersect_values.size()) {
    float shortest_distance = FLT_MAX;
    glm::vec3 collidingNormal(0.0f, 0.0f, 0.0f);
    for (unsigned int i = 0; i < intersect_values.size(); i++) {
        if (intersect_values.at(i) < shortest_distance) {
          shortest_distance = intersect_values.at(i);
          c.depth = intersect_values.at(i);
          c.normal = normals.at(i);
          c.position = positions.at(i);
        }
    }
    //std::cout << "smallest depth: " << c.depth << std::endl;
    return c;
  } else {
    return c;
  }

}

BCube::BCube(){};

int BCube::getTranslatedVertices(glm::mat4 modelMatrix) {
    position = getPosition(modelMatrix);
    model = modelMatrix;
    this->translated_vertices = {};
    for (unsigned int i = 0; i < vertices.size(); i+=3) {
        glm::vec3 v = glm::vec3(vertices.at(i), vertices.at(i+1), vertices.at(i+2));
        glm::vec4 _v = glm::vec4(v.x, v.y, v.z, 1.0f);
        glm::vec4 tv = modelMatrix * _v;
        this->translated_vertices.push_back(glm::vec3(tv.x, tv.y, tv.z));
        this->_vertices.push_back(v);

    }
    return 0;
}

void BCube::setup(glm::vec3 min, glm::vec3 max) {

    glGenQueries(1, &queryFront);
    glGenQueries(1, &queryBack);

    float verts[] = {
        max[0], min[1], min[2], // 1
        min[0], min[1], min[2], // 0
        max[0], max[1], min[2], // 2
        min[0], min[1], min[2], // 0
        min[0], max[1], min[2], // 3
        max[0], max[1], min[2], // 2

        min[0], min[1], max[2], // 0
        max[0], min[1], max[2], // 1
        max[0], max[1], max[2], // 2
        max[0], max[1], max[2], // 2
        min[0], max[1], max[2], // 3
        min[0], min[1], max[2], // 0

        min[0], max[1], max[2],
        min[0], max[1], min[2],
        min[0], min[1], min[2],
        min[0], min[1], min[2],
        min[0], min[1], max[2],
        min[0], max[1], max[2],

        max[0], max[1], min[2], // 1
        max[0], max[1], max[2], // 0
        max[0], min[1], min[2], // 2
        max[0], max[1], max[2], // 0
        max[0], min[1], max[2], // 3
        max[0], min[1], min[2], // 2

        min[0], min[1], min[2], //0
        max[0], min[1], min[2], //1
        max[0], min[1], max[2], //2
        max[0], min[1], max[2], //2
        min[0], min[1], max[2], //3
        min[0], min[1], min[2], //0

        max[0], max[1], min[2], //1
        min[0], max[1], min[2], //0
        max[0], max[1], max[2], //2
        min[0], max[1], min[2], //0
        min[0], max[1], max[2], //3
        max[0], max[1], max[2], //2

    };

    vertices = std::vector<float>(verts, verts+108);

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    const char *vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out vec4 outColor;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = projection * view * model * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
        "   outColor = vec4(aPos,1.0f);\n"
        "}\0";
    const char *fragmentShaderSource = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "in vec4 outColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = outColor;\n"
        "}\n\0";

    // vertex shader
    int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    // check for shader compile errors
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    // fragment shader
    int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    // check for shader compile errors
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    // link shaders
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    // check for linking errors
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void BCube::setMatrix(std::string name, glm::mat4 mat) {
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, name.c_str()), 1, GL_FALSE, &mat[0][0]);
}

void BCube::draw() {
    glUseProgram(shaderProgram);

    // render box
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

float intersectPlane(glm::vec3 n, glm::vec3 p0, glm::vec3 l0, glm::vec3 l)
{
  // scratch a pixel
    // assuming vectors are all normalized
    float denom = glm::dot(l,n);
    glm::vec3 p0l0 = p0 - l0;
    float t = glm::dot(p0l0, n) / denom;


    return t;
}

CollisionInfo check_collision(BCube &boundingCube1, BCube &boundingCube2) {

  glm::vec3 displacement(0.0f, 0.0f, 0.0f);
  float minOverlapXY = FLT_MAX;
  glm::vec3 smallestXY;
  float minOverlapXZ = FLT_MAX;
  glm::vec3 smallestXZ;
  float minOverlapYZ = FLT_MAX;
  glm::vec3 smallestYZ;
  float smallest = 0.0f;

  glm::vec3 collidingNormal;
  glm::vec3 collidingPosition;

  BCube *poly1 = &boundingCube1;
  BCube *poly2 = &boundingCube2;

  CollisionInfo c;


  float axisProjx, axisProjy, axisProjz;

  for (unsigned int shape = 0; shape < 2; shape++) {
    if (shape == 1) {
      poly1 = &boundingCube2;
      poly2 = &boundingCube1;
    }
    for (unsigned int a = 0; a < poly1->translated_vertices.size(); a++) {
      float b = (a + 1) % poly1->translated_vertices.size();
      axisProjx = -(poly1->translated_vertices[b].y - poly1->translated_vertices[a].y);
      axisProjy = poly1->translated_vertices[b].x - poly1->translated_vertices[a].x;

      float minR1_xy = FLT_MAX;
      float maxR1_xy = -FLT_MAX;
      for (unsigned int p = 0; p < poly1->translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1->translated_vertices[p].x, poly1->translated_vertices[p].y);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjy));
        minR1_xy = std::min(minR1_xy, q);
        maxR1_xy = std::max(maxR1_xy, q);
      }

      float minR2_xy = FLT_MAX;
      float maxR2_xy = -FLT_MAX;
      for (unsigned int p = 0; p < poly2->translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly2->translated_vertices[p].x, poly2->translated_vertices[p].y);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjy));
        minR2_xy = std::min(minR2_xy, q);
        maxR2_xy = std::max(maxR2_xy, q);
      }
 
      axisProjx = -(poly1->translated_vertices[b].z - poly1->translated_vertices[a].z);
      axisProjz = poly1->translated_vertices[b].x - poly1->translated_vertices[a].x;
      float minR1_xz = FLT_MAX;
      float maxR1_xz = -FLT_MAX;
      for (unsigned int p = 0; p < poly1->translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1->translated_vertices[p].x, poly1->translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjz));
        minR1_xz = std::min(minR1_xz, q);
        maxR1_xz = std::max(maxR1_xz, q);
      }

      float minR2_xz = FLT_MAX;
      float maxR2_xz = -FLT_MAX;
      for (unsigned int p = 0; p < poly2->translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly2->translated_vertices[p].x, poly2->translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjz));
        minR2_xz = std::min(minR2_xz, q);
        maxR2_xz = std::max(maxR2_xz, q);
      }
 
      axisProjy = -(poly1->translated_vertices[b].z - poly1->translated_vertices[a].z);
      axisProjz = poly1->translated_vertices[b].y - poly1->translated_vertices[a].y;
 
      float minR1_yz = FLT_MAX;
      float maxR1_yz = -FLT_MAX;
      for (unsigned int p = 0; p < poly1->translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1->translated_vertices[p].y, poly1->translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjy, axisProjz));
        minR1_yz = std::min(minR1_yz, q);
        maxR1_yz = std::max(maxR1_yz, q);
      }
 
      float minR2_yz = FLT_MAX;
      float maxR2_yz = -FLT_MAX;
      for (unsigned int p = 0; p < poly2->translated_vertices.size(); p++) {
          glm::vec2 translated_vertex = glm::vec2(poly2->translated_vertices[p].y, poly2->translated_vertices[p].z);
          float q = glm::dot(translated_vertex, glm::vec2(axisProjy, axisProjz));
          minR2_yz = std::min(minR2_yz, q);
          maxR2_yz = std::max(maxR2_yz, q);
      }

      // seperation between shadows of objects found => no collision possible, exit early


      if (!(maxR2_xy >= minR1_xy && maxR1_xy >= minR2_xy 
        && maxR2_xz >= minR1_xz && maxR1_xz >= minR2_xz 
        && maxR2_yz >= minR1_yz && maxR1_yz >= minR2_yz)) {

          return c;
      } else {
        for (unsigned int i = 0; i < boundingCube1.translated_vertices.size(); i++) {
          glm::vec3 rayOrigin = boundingCube1.translated_vertices.at(i);
          glm::vec3 rayDirection = boundingCube1.velocity;
          CollisionInfo boxCollisionInfo = rayBoxIntersect(rayOrigin, rayDirection, boundingCube2);
          float t = boxCollisionInfo.depth;

          if ((t < smallest && t < 0.0f) || smallest == 0.0f) {
            smallest = t;
            displacement = boundingCube1.velocity;
            collidingNormal = boxCollisionInfo.normal;
            collidingPosition = boxCollisionInfo.position;

          }
        }
        for (unsigned int i = 0; i < boundingCube2.translated_vertices.size(); i++) {
          glm::vec3 rayOrigin = boundingCube2.translated_vertices.at(i);
          glm::vec3 rayDirection = -boundingCube1.velocity;
          CollisionInfo boxCollisionInfo = rayBoxIntersect(rayOrigin, rayDirection, boundingCube1);
          float t = boxCollisionInfo.depth;
          if ((t < smallest && t < 0.0f) || smallest == 0.0f) {
            smallest = t;
            displacement = boundingCube1.velocity * -1.0f;
            collidingNormal = boxCollisionInfo.normal;
            collidingPosition = boxCollisionInfo.position;

          }
        }

          glm::vec3 rayOrigin = boundingCube1.position;
          glm::vec3 rayDirection = boundingCube1.velocity;
          CollisionInfo boxCollisionInfo = rayBoxIntersect(rayOrigin, rayDirection, boundingCube2);
          CollisionInfo boxCollisionInfo2 = rayBoxIntersect(rayOrigin, rayDirection, boundingCube1);
          float t = boxCollisionInfo.depth + boxCollisionInfo2.depth;
          float _t = glm::length(boxCollisionInfo.position - boundingCube1.position);
          if ((t < smallest && t < 0.0f) || smallest == 0.0f) {
            //std::cout << boxCollisionInfo2.depth << std::endl;

            smallest = t;
            displacement = boundingCube1.velocity;
            collidingNormal = boxCollisionInfo.normal;
            collidingPosition = boxCollisionInfo.position;

          }

          rayOrigin = boundingCube2.position;
          rayDirection = -boundingCube1.velocity;
          boxCollisionInfo = rayBoxIntersect(rayOrigin, rayDirection, boundingCube1);
          boxCollisionInfo2 = rayBoxIntersect(rayOrigin, rayDirection, boundingCube2);

          _t = glm::length(boxCollisionInfo.position - boundingCube2.position);
          t = boxCollisionInfo.depth + boxCollisionInfo2.depth;
          if ((t < smallest && t < 0.0f) || smallest == 0.0f) {
            //std::cout << boxCollisionInfo2.depth << std::endl;

            smallest = t;
            displacement = boundingCube1.velocity * -1.0f;
            collidingNormal = boxCollisionInfo2.normal;
            collidingPosition = boxCollisionInfo.position;

          }

      }
      
    }
  }
  c.normal = collidingNormal;
  //std::cout << "3. " << glm::to_string(c.normal) << std::endl;

  c.depth = smallest;
  c.position = collidingPosition;


  return c;
}
