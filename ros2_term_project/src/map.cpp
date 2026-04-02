#include "map.h"

void Map::update(const OctomapMsg& octomap_msg,
                 const octomap::point3d& world_min,
                 const octomap::point3d& world_max) {
  // Octomap Msg to Octree conversion   
  Octree_ptr octree_ptr = nullptr;
  octree_ptr.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));
     // octomap_msg 받아서                OcTree로 변환 하는거

  // Octree -> DynamicEDTOctomap 변환
  dynamic_edt_map_ptr = std::make_shared<DynamicEDTOctomap>(
      world_maxdist, octree_ptr.get(), world_min, world_max, false);
  dynamic_edt_map_ptr->update();
  updated = true;
}

DynamicEDTMapPtr Map::get_dist_map_ptr() const {
  return dynamic_edt_map_ptr;
}

// search_point에서 가장 가까운 장매울의 위치와 거리 찾기
[[nodiscard]] void Map::get_distance_and_closest_obstacle(const octomap::point3d& search_point,
                                                          float& distance,
                                                          octomap::point3d& closest_obstacle) const {
  dynamic_edt_map_ptr->getDistanceAndClosestObstacle(search_point, distance, closest_obstacle);
}