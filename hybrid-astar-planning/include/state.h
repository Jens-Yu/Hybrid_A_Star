#ifndef STATE_H
#define STATE_H

#include "utils.h"
#include "map.h"
#include <cmath>

class State {
public: // methods
  /**
   * Constructor
   */
  State();

  /**
   * Constructor
   */
  State(double x, double y, double theta);

  /**
   * get the states of the next move
   *
   */
  std::vector<State> getNextStates(Map& map);

  State generateState(Map& map, double alpha, double dist, double movement);
  static std::vector<std::pair<int, int>> boundary_points;
  static void setBoundaryPoints(const std::vector<std::pair<int, int>>& points){
    boundary_points = points;
  }

public: // variables
  // 3d state on the geometric map
  double x;
  double y;
  double theta;

  // gx, gy and gtheta are co-ordinates in the 80X80 grid
  int gx;
  int gy;
  int gtheta;
  int movementDirection;

  // for 2d planning
  int dx;
  int dy;

  double cost2d;
  double cost3d;

  double change;
  double velocity;

  State *prev;
  State *next;
};

#endif // STATE_H
