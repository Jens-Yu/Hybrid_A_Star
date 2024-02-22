#include "state.h"
#include "algorithm.h"
/**
 * Constructor
 */
State::State(){
  this->x = -1;
  this->y = -1;
  this->theta = -1;
}

/**
 * Constructor
 */
State::State(double x, double y, double theta) {
  // Initilize 3d state on the geometric map
  this->x = x;
  this->y = y;
  this->theta = theta;

  // Initialize 2d state on the grid map
  this->gx = x / Grid_Res;
  this->gy = y / Grid_Res;
  this->gtheta = theta + 0.01;
}

std::vector<std::pair<int, int>> State::boundary_points = {};

/**
 * Compute the possible next moves according to
 * the kinematic bicycle model constraints.
 *
 * Kinematic Bicycle Model centered the rear wheel.
 *
 */
std::vector<State> State::getNextStates(Map& map) {
  std::vector<State> next;
  State state;

  // Bicycle model parameters
  double alpha = 0;
  double d_theta = 0;
  double radius = 0; // rotation radius
  double dist = 15.0; // distance travelled within a unit time

  //int hasobstacle = 1;
  // Try out possible values of alpha
  // among 3 angles: [-VEH_M_ALPHA, 0, VEH_M_ALPHA].
  for (alpha = -VEH_M_ALPHA; alpha <= VEH_M_ALPHA + 0.001; alpha += VEH_M_ALPHA) {
    // 生成前进状态
        State forwardState = generateState(map, alpha, dist, 1.0); // 前进
        if (!map.checkCollision(forwardState)) {
            //if (state.x >= map.minX && state.x <= map.maxX && state.y >= map.minY && state.y <= map.maxY) {
            next.push_back(forwardState);
        //}
            
        }

        // 生成倒车状态
        State reverseState = generateState(map, alpha, dist, -1.0); // 倒车
        if (!map.checkCollision(reverseState)) {
            //if (state.x >= map.minX && state.x <= map.maxX && state.y >= map.minY && state.y <= map.maxY) {
            next.push_back(reverseState);
        //}
            
        }
    }

    return next;
}
 
State State::generateState(Map& map, double alpha, double dist, double movement) {
    State state;
    double d_theta = dist * tan(alpha * PI / 180) / VEH_LEN;
    double radius = VEH_LEN / tan(alpha * PI / 180);
    Point edgeDirection;
    state.movementDirection = (movement > 0) ? 1 : 0;
    if (map.isParallelAndNearby(*this, map, VEH_LEN, edgeDirection)) {
        Point toGoal = {Algorithm::goal.x - this->x, Algorithm::goal.y - this->y};
        double norm = sqrt(toGoal.x * toGoal.x + toGoal.y * toGoal.y);
        toGoal.x /= norm;
        toGoal.y /= norm;

        double dot = edgeDirection.x * toGoal.x + edgeDirection.y * toGoal.y;
        double len = sqrt(edgeDirection.x * edgeDirection.x + edgeDirection.y * edgeDirection.y);
        edgeDirection.x /= len;
        edgeDirection.y /= len;

        double sign = std::copysign(1.0, dot);
        state.x = this->x + sign * dist * edgeDirection.x;
        state.y = this->y + sign * dist * edgeDirection.y;
        state.theta = this->theta; // 保持原有角度
    } else {
        if (abs(d_theta) < 0.001) { // Straight movement
        state.x = x + movement * dist * cos(theta * 2.0 * PI / Theta);
        state.y = y + movement * dist * sin(theta * 2.0 * PI / Theta);
        state.theta = theta;
    } else { // Turning movement
        state.x = x + movement * (radius * sin(theta * 2.0 * PI / Theta + d_theta) - radius * sin(theta * 2.0 * PI / Theta));
        state.y = y + movement * (-radius * cos(theta * 2.0 * PI / Theta + d_theta) + radius * cos(theta * 2.0 * PI / Theta));
        state.theta = (theta + movement * d_theta * 180 / PI / Theta_Res) > 0 ? fmod(theta + movement * d_theta * 180 / PI / Theta_Res, Theta) : theta + movement * d_theta * 180 / PI / Theta_Res + Theta;
        }
    }
    
    // 检查新状态是否在边界内
    if (state.x < boundary_points[0].first || state.x > boundary_points[1].first ||
        state.y < boundary_points[0].second || state.y > boundary_points[3].second) {
      return State(); // 返回无效状态
    }
    
    state.gx = state.x / Grid_Res;
    state.gy = state.y / Grid_Res;
    state.gtheta = state.theta + 0.01;

    return state;
}

