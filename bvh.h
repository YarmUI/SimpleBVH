#pragma once
#include <vector>
#include <algorithm>
#include <iostream>

namespace SimpleBVH {

template<class AABB_T>
struct BVHNode {
  AABB_T aabb;
  union {
    uint32_t children[2];
    struct { uint32_t leaf_index, leaf_size; };
  };
  bool is_leaf;
};

template<class AABB_T>
class BVH {
  private:
    struct BVHSort {
      float x, y, z;
      AABB_T aabb;
      uint32_t index;
      BVHSort(const AABB_T& _aabb, uint32_t _index) : aabb(_aabb), index(_index) { aabb.center(x, y, z); }
      static bool sort_x(const BVHSort& lhs, const BVHSort& rhs) { return lhs.x < rhs.x; }
      static bool sort_y(const BVHSort& lhs, const BVHSort& rhs) { return lhs.y < rhs.y; }
      static bool sort_z(const BVHSort& lhs, const BVHSort& rhs) { return lhs.z < rhs.z; }
    };
    enum SORT_AXIS { X, Y, Z };
    uint32_t build(
        typename std::vector<BVHSort>::iterator begin,
        typename std::vector<BVHSort>::iterator end,
        uint32_t leaf_size, SORT_AXIS sort_axis) {

      uint32_t size = end - begin;
      uint32_t res = nodes.size();
      nodes.push_back(BVHNode<AABB_T>());
      nodes[res].aabb = begin->aabb;
      for(typename std::vector<BVHSort>::iterator it = begin + 1; it != end; it++) {
        nodes[res].aabb += it->aabb;
      }
      if(size <= leaf_size) {
        nodes[res].is_leaf = true;
        nodes[res].leaf_size = size;
        nodes[res].leaf_index = indexes.size();
        for(typename std::vector<BVHSort>::iterator it = begin; it != end; it++) {
          indexes.push_back(it->index);
        }
      } else {
        SORT_AXIS next_sort_axis;
        switch(sort_axis) {
          case X:
            std::sort(begin, end, BVHSort::sort_x);
            next_sort_axis = Y;
            break;
          case Y:
            std::sort(begin, end, BVHSort::sort_y);
            next_sort_axis = Z;
            break;
          case Z:
            std::sort(begin, end, BVHSort::sort_z);
            next_sort_axis = X;
            break;
        }
        nodes[res].children[0] = build(begin, begin + size / 2, leaf_size, next_sort_axis);
        nodes[res].children[1] = build(begin + size / 2, end, leaf_size, next_sort_axis);
        nodes[res].is_leaf = false;
      }
      return res;
    }

  public:
    std::vector< BVHNode<AABB_T> > nodes;
    std::vector<uint32_t> indexes;
    void buildBVH(const std::vector<AABB_T>& boxes, uint32_t leaf_size) {
      std::vector<BVHSort> sn;
      for(uint32_t i = 0; i < boxes.size(); i++) {
        sn.push_back(BVHSort(boxes[i], i));
      }
      nodes.clear();
      nodes.reserve(boxes.size() * 2);
      indexes.clear();
      indexes.reserve(boxes.size());
      build(sn.begin(), sn.end(), leaf_size, X);
    }
};
}
