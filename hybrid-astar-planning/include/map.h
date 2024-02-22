#ifndef MAP_H
#define MAP_H

#include "utils.h"


struct Point {
    double x, y;
};

class State;

struct Node {
  int x;
  int y;
  int nearest_obstacle;
};

class Map {
public:
  /**
   * Constructor
   *
   */
  Map() {}

  /**
   * Constructor
   *
   */
  Map(std::vector<std::pair<int, int>> obstacle_points);

  void initCollisionChecker();

  bool checkCollision(State pos);

  void findNearObs();

  int nearestObstacleDistance(State pos);

  bool isBoundaryObstacle(int i, int j);
  
  bool isParallelAndNearby(const State& state, const Map& map, double distanceThreshold, Point& edgeDirection);


public:
  int **obs_map;
  int **acc_obs_map;
  int **nearest_obstacle;
  int obs_dist_max;

  std::vector<std::pair<int, int>> obstacle_points;  // 存储障碍物顶点
  std::vector<std::vector<Point>> obstacle_edges; 


};

#endif // MAP_H
