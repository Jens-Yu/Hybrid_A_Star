#include "algorithm.h"

State getStartState() {
  // FutureWork: read from yml file
  return State(685, 320, 54);
}

State getTargetState() {
  // FutureWork: read from yml file
  return State(400, 750, 0);
}

int main() {
  std::vector<std::pair<int, int>> obstacle_points = {
    {200, 50},  // 第一个点
    {700, 50}, // 第二个点
    {700, 300},// 第三个点
    {670, 300},// 第四个点
    {670, 440},// 第五个点
    {200, 440}  // 第六个点
};
  std::vector<std::pair<int, int>> boundary_points = {
    {300, 150},
    {750, 150},
    {750, 800},
    {300, 800}
  };

  Map map(obstacle_points);
  State::setBoundaryPoints(boundary_points);
  State initial = getStartState();

  State goal = getTargetState();
  //Point edgeDirection;
  //double distanceThreshold = 40;
  //std::cout<<map.isParallelAndNearby(initial, map, distanceThreshold, edgeDirection)<< std::endl;

  Algorithm algorithm(map);

  algorithm.updateInitial(initial);

  algorithm.updateGoal(goal);

  algorithm.hybridAstarPlanning();
  
  return 0;
}