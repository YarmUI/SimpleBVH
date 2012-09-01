#include <iostream>
#include <vector>
#include "bvh.h"
using namespace std;

float frand() { return (float)(std::rand()) / RAND_MAX; }

struct Vec3 { float x, y, z; };
std::ostream& operator<<(std::ostream& os, const Vec3& vec) {
  os << '(' << vec.x << ',' << vec.y << ',' << vec.z << ')'; 
  return os;
}

struct AABB {
  union {
    struct { Vec3 min, max; };
    float aabb[2][3];
  };

  inline AABB& operator+=(const AABB& rhs) {
    min.x = std::min(min.x, rhs.min.x);
    min.y = std::min(min.y, rhs.min.y);
    min.z = std::min(min.z, rhs.min.z);
    max.x = std::max(max.x, rhs.max.x);
    max.y = std::max(max.y, rhs.max.y);
    max.z = std::max(max.z, rhs.max.z);
    return *this;
  }

  inline void center(float& x, float& y, float& z) const {
    x = (max.x - min.x) * 0.5f;
    y = (max.y - min.y) * 0.5f;
    z = (max.z - min.z) * 0.5f;
  }
};
std::ostream& operator<<(std::ostream& os, const AABB& aabb) {
  float x = aabb.max.x - aabb.min.x;
  float y = aabb.max.y - aabb.min.y;
  float z = aabb.max.z - aabb.min.z;
  os << "min=" << aabb.min << ", max=" << aabb.max << ", Area=" << x*y + y*z + z*x;
  return os;
}


template<class AABB_T>
void print_bvh(const SimpleBVH::BVH<AABB_T>& bvh, const SimpleBVH::BVHNode<AABB_T>& node, int space = 0) {
  int n = space;
  while(n--) std::cout << ' ';
  std::cout << node.aabb << std::endl;
  if(node.is_leaf) {
    int m = space + 2;
    while(m--) std::cout << ' ';
    for(int i = 0; i < node.leaf_size; i++)
      std::cout << bvh.indexes[node.leaf_index + i] << ", ";
    std::cout << endl;
  } else {
    print_bvh(bvh, bvh.nodes[node.children[0]], space + 2);
    print_bvh(bvh, bvh.nodes[node.children[1]], space + 2);
  }
}

int main(void) {
  SimpleBVH::BVH<AABB> bvh;
  vector<AABB> boxes;
  for(int i = 0; i < 32; i++) {
    AABB aabb;
    aabb.min.x = frand()*10;
    aabb.min.y = frand()*10;
    aabb.min.z = frand()*10;
    aabb.max.x = aabb.min.x + 0.5f;
    aabb.max.y = aabb.min.y + 0.5f;
    aabb.max.z = aabb.min.z + 0.5f;
    boxes.push_back(aabb);
  }
  bvh.buildBVH(boxes, 4);
  print_bvh<AABB>(bvh, bvh.nodes[0]);
  return 0;
}
