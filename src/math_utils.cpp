#include "math_utils.h"

OrientedBoundingBox::OrientedBoundingBox(glm::mat4 m) {
  model = m;
  glm::vec3 min(-1.0f, -1.0f, -1.0f);
  glm::vec3 max(1.0f, 1.0f, 1.0f);

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
}


std::vector<glm::vec3> OrientedBoundingBox::getTranslatedVertices() {
    std::vector<glm::vec3> translated_vertices = {};
    for (unsigned int i = 0; i < vertices.size(); i+=3) {
        glm::vec3 v = glm::vec3(vertices.at(i), vertices.at(i+1), vertices.at(i+2));
        glm::vec4 _v = glm::vec4(v.x, v.y, v.z, 1.0);
        glm::vec4 tv = this->model * _v;
        translated_vertices.push_back(glm::vec3(tv.x, tv.y, tv.z));
    }
    return translated_vertices;
}

bool OrientedBoundingBox::isCollidingWith(OrientedBoundingBox other) { 
  OrientedBoundingBox *poly1 = this;
  OrientedBoundingBox *poly2 = &other;
  std::vector<glm::vec3> poly1_translated_vertices = poly1->getTranslatedVertices();
  std::vector<glm::vec3> poly2_translated_vertices = poly2->getTranslatedVertices();

  for (unsigned int shape = 0; shape < 2; shape++) {
    if (shape == 1) {
      poly1 = &other;
      poly2 = this;
    }
    for (unsigned int a = 0; a < poly1_translated_vertices.size(); a++) {

      float b = (((a + 1) % poly1_translated_vertices.size()) + poly1_translated_vertices.size()) % poly1_translated_vertices.size();
      float axisProjx = -(poly1_translated_vertices[b].y - poly1_translated_vertices[a].y);
      float axisProjy = poly1_translated_vertices[b].x - poly1_translated_vertices[a].x;
 
      float minR1_xy = FLT_MAX;
      float maxR1_xy = -FLT_MAX;
      for (unsigned int p = 0; p < poly1_translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1_translated_vertices[p].x, poly1_translated_vertices[p].y);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjy));
        minR1_xy = std::min(minR1_xy, q);
        maxR1_xy = std::max(maxR1_xy, q);
      }

      float minR2_xy = FLT_MAX;
      float maxR2_xy = -FLT_MAX;
      for (unsigned int p = 0; p < poly2_translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly2_translated_vertices[p].x, poly2_translated_vertices[p].y);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjy));
        minR2_xy = std::min(minR2_xy, q);
        maxR2_xy = std::max(maxR2_xy, q);
      }
 
      axisProjx = -(poly1_translated_vertices[b].z - poly1_translated_vertices[a].z);
      float axisProjz =   poly1_translated_vertices[b].x - poly1_translated_vertices[a].x;

      float minR1_xz = FLT_MAX;
      float maxR1_xz = -FLT_MAX;
      for (unsigned int p = 0; p < poly1_translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1_translated_vertices[p].x, poly1_translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjz));
        minR1_xz = std::min(minR1_xz, q);
        maxR1_xz = std::max(maxR1_xz, q);
      }

      float minR2_xz = FLT_MAX;
      float maxR2_xz = -FLT_MAX;
      for (unsigned int p = 0; p < poly2_translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly2_translated_vertices[p].x, poly2_translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjx, axisProjz));
        minR2_xz = std::min(minR2_xz, q);
        maxR2_xz = std::max(maxR2_xz, q);
      }
 
      axisProjy = -(poly1_translated_vertices[b].z - poly1_translated_vertices[a].z);
      axisProjz = poly1_translated_vertices[b].y - poly1_translated_vertices[a].y;
 
      float minR1_yz = FLT_MAX;
      float maxR1_yz = -FLT_MAX;
      for (unsigned int p = 0; p < poly1_translated_vertices.size(); p++) {
        glm::vec2 translated_vertex = glm::vec2(poly1_translated_vertices[p].y, poly1_translated_vertices[p].z);
        float q = glm::dot(translated_vertex, glm::vec2(axisProjy, axisProjz));
        minR1_yz = std::min(minR1_yz, q);
        maxR1_yz = std::max(maxR1_yz, q);
      }
 
      float minR2_yz = FLT_MAX;
      float maxR2_yz = -FLT_MAX;
      for (unsigned int p = 0; p < poly2_translated_vertices.size(); p++) {
          glm::vec2 translated_vertex = glm::vec2(poly2_translated_vertices[p].y, poly2_translated_vertices[p].z);
          float q = glm::dot(translated_vertex, glm::vec2(axisProjy, axisProjz));
          minR2_yz = std::min(minR2_yz, q);
          maxR2_yz = std::max(maxR2_yz, q);
      }
        // seperation between shadows of objects found => no collision possible, exit early
        if (!(maxR2_xy >= minR1_xy && maxR1_xy >= minR2_xy 
          && maxR2_xz >= minR1_xz && maxR1_xz >= minR2_xz 
          && maxR2_yz >= minR1_yz && maxR1_yz >= minR2_yz)) {
            return false;
        }
      
    }
  }

  return true;
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
