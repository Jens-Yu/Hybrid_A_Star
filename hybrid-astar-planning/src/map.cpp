#include "map.h"
#include "state.h"
#include <algorithm>

/*
 * Constructor
 */
Map::Map(std::vector<std::pair<int, int>> obstacle_points) {
  // Initilize the geometric map.
  obs_map = new int *[MAPX];
  for (int i = 0; i < MAPX; i++) {
    obs_map[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) {
      obs_map[i][j] = 0;
    }
  this->obstacle_points = obstacle_points;
  for (size_t i = 0; i < obstacle_points.size(); i++) {
      Point start, end;
      start.x = obstacle_points[i].first;
      start.y = obstacle_points[i].second;
      end.x = obstacle_points[(i + 1) % obstacle_points.size()].first;
      end.y = obstacle_points[(i + 1) % obstacle_points.size()].second;
      
      std::vector<Point> edge = {start, end};
      obstacle_edges.push_back(edge);
      }
  }

  // 使用 OpenCV 创建一个空白图像
  cv::Mat obsmap(MAPX, MAPY, CV_8UC1, cv::Scalar(255)); // 白色背景

  
  // 绘制障碍物
  std::vector<cv::Point> cv_points;
  for (const auto& point : obstacle_points) {
    cv_points.push_back(cv::Point(point.second, point.first));
  }
  const cv::Point *pts = (const cv::Point*) cv::Mat(cv_points).data;
  int npts = cv::Mat(cv_points).rows;

  // 用黑色填充多边形
  cv::fillPoly(obsmap, &pts, &npts, 1, cv::Scalar(0), 8);

  // 更新 obs_map 数组
  for (int i = 0; i < MAPX; i++) {
    for (int j = 0; j < MAPY; j++) {
      if (obsmap.at<uchar>(i, j) == 0) { // 黑色像素表示障碍物
        obs_map[i][j] = 1;
      }
    }
  }

  std::cout << "Cost Map Initialized!" << std::endl;
}

/**
 * Collision Checker.
 *
 */
void Map::initCollisionChecker() {
  acc_obs_map = new int *[MAPX];
  for (int i = 0; i < MAPX; i++) {
    acc_obs_map[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) acc_obs_map[i][j] = obs_map[i][j];
  }

  for (int i = 0; i < MAPX; i++)
    for (int j = 1; j < MAPY; j++)
      acc_obs_map[i][j] = acc_obs_map[i][j - 1] + acc_obs_map[i][j];

  for (int j = 0; j < MAPY; j++)
    for (int i = 1; i < MAPX; i++)
      acc_obs_map[i][j] = acc_obs_map[i - 1][j] + acc_obs_map[i][j];

  std::cout << "Collision checker initialized!" << std::endl;

  return;
}

bool Map::isParallelAndNearby(const State& state, const Map& map, double distanceThreshold, Point& edgeDirection) {
    for (const auto& edge : map.obstacle_edges) {
        
          Point start = edge[0];
          Point end = edge[1];

          // 计算障碍物边缘的方向
          Point edgeVector = {end.x - start.x, end.y - start.y};
          Point carDirection = {cos(state.theta * 2.0 * PI / Theta), sin(state.theta * 2.0 * PI / Theta)};
          bool isParallel = false;
          bool isWithinDistance = false;
            // 计算点积
          double dotProduct = edgeVector.x * carDirection.x + edgeVector.y * carDirection.y;

          // 计算两个向量的长度
          double lenVec1 = sqrt(edgeVector.x * edgeVector.x + edgeVector.y * edgeVector.y);
          double lenVec2 = sqrt(carDirection.x * carDirection.x + carDirection.y * carDirection.y);
          // 检测平行性
          isParallel = fabs(fabs(dotProduct) - lenVec1 * lenVec2) < 1e-2;
          
          double line_x = end.x - start.x;
          double line_y = end.y - start.y;

          // 将状态点（小车位置）转换为向量
          double point_x = state.x - start.x;
          double point_y = state.y - start.y;

          // 计算点到线段的最近点
          double t = std::max(0.0, std::min(1.0, (point_x * line_x + point_y * line_y) / (line_x * line_x + line_y * line_y)));
          double nearest_x = start.x + t * line_x;
          double nearest_y = start.y + t * line_y;

          // 计算状态点到最近点的距离
          double dist_x = state.x - nearest_x;
          double dist_y = state.y - nearest_y;
          double distance = sqrt(dist_x * dist_x + dist_y * dist_y);
          isWithinDistance = distance <= distanceThreshold;
              // 检测距离
              if (isParallel && isWithinDistance) {
                  edgeDirection = edgeVector;
                  return true;
              }
            
        }
    return false;
}


/**
 * Collision Checker.
 *
 */
bool Map::checkCollision(State pos) {
  // std::cout << "Collision checking: " << std::endl;
  // std::cout << pos.x << "," << pos.y << "," << pos.theta << std::endl;
  // std::cout << pos.gx << "," << pos.gy << "," << pos.gtheta << std::endl;

  if (pos.x >= MAPX || pos.x < 0 || pos.y >= MAPY || pos.y < 0 ||
      pos.theta >= Theta || pos.theta < 0)
    return true;

  if (pos.gx >= GX || pos.gx < 0 || pos.gy >= GY || pos.gy < 0 ||
      pos.gtheta >= Theta || pos.gtheta < 0)
    return true;

  // std::cout << "Out of Bounds" << std::endl;

  // first use a bounding box around car to check for collision in O(1) time
  int max_x, min_x, max_y, min_y;
  max_x = pos.x + VEH_LEN * abs(cos(pos.theta * 2 * PI / Theta)) / 2
                + VEH_WID * abs(sin(pos.theta * 2 * PI / Theta)) / 2 + 1;
  min_x = pos.x - VEH_LEN * abs(cos(pos.theta * 2 * PI / Theta)) / 2
                - VEH_WID * abs(sin(pos.theta * 2 * PI / Theta)) / 2 - 1;
  max_y = pos.y + VEH_LEN * abs(sin(pos.theta * 2 * PI / Theta)) / 2
                + VEH_WID * abs(cos(pos.theta * 2 * PI / Theta)) / 2 + 1;
  min_y = pos.y - VEH_LEN * abs(sin(pos.theta * 2 * PI / Theta)) / 2
                - VEH_WID * abs(cos(pos.theta * 2 * PI / Theta)) / 2 - 1;

  if (max_x >= MAPX || min_x < 0 || max_y >= MAPY || min_y < 0) return true;

  if (acc_obs_map[max_x][max_y] + acc_obs_map[min_x][min_y] ==
      acc_obs_map[max_x][min_y] + acc_obs_map[min_x][max_y]) {
    return false;
  }

  std::cout << "Obstacle present inside box" << std::endl;

  // brute force check through the car

  for (double i = -VEH_LEN / 2.0; i <= VEH_LEN / 2.0 + 0.001; i += 1) {
    for (double j = -VEH_WID / 2.0; j <= VEH_WID / 2.0 + 0.001; j += 1) {
      int s = pos.x + i * cos(pos.theta * 2.0 * PI / Theta) +
              j * sin(pos.theta * 2.0 * PI / Theta) + 0.001;
      int t = pos.y + i * sin(pos.theta * 2.0 * PI / Theta) +
              j * cos(pos.theta * 2.0 * PI / Theta) + 0.001;
      if (obs_map[s][t] || obs_map[s + 1][t] || obs_map[s][t + 1] ||
          obs_map[s + 1][t + 1] || obs_map[s - 1][t - 1] || obs_map[s - 1][t] ||
          obs_map[s][t - 1] || obs_map[s - 1][t + 1] || obs_map[s + 1][t - 1]) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Check if the input coordinate is at the boundary of the obstacles.
 *
 * Check in four directions.
 *
 * @return true or false.
 */
bool Map::isBoundaryObstacle(int i, int j) {
  for (int k = i - 1; k <= i + 1; k++) {
    for (int l = j - 1; l <= j + 1; l++) {
      if (!(k >= 0 && k < MAPX && l >= 0 && l < MAPY)) continue;
      if (obs_map[k][l] == 0) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Find nearest obstacles.
 *
 * Use BFS Search.
 *
 */
void Map::findNearObs() {
  Node node_p, node_c;

  nearest_obstacle = new int *[MAPX];

  for (int i = 0; i < MAPX; i++) {
    nearest_obstacle[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) {
      nearest_obstacle[i][j] = 0;
    }
  }

  std::queue<Node> q;

  for (int i = 0; i < MAPX; i++) {
    for (int j = 0; j < MAPY; j++) {
      if (obs_map[i][j] == 1) {
        if (!isBoundaryObstacle(i, j)) {
          nearest_obstacle[i][j] = -1;
        } else {
          node_p.x = i;
          node_p.y = j;
          node_p.nearest_obstacle = 1;
          q.push(node_p);
          nearest_obstacle[i][j] = 1;
        }
      }
    }
  }

  while (!q.empty()) {
    node_p = q.front();
    q.pop();

    // std::cout << "nearest_obstacle size" << nearest_obstacle.size()
    //           << nearest_obstacle[0].size() << std::endl;
    // std::cout << "obs_map size" << obs_map.size() << obs_map[0].size() << std::endl;

    for (int i = node_p.x - 1; i <= node_p.x + 1; i++) {
      for (int j = node_p.y - 1; j <= node_p.y + 1; j++) {
        if (i >= 0 && i < MAPX && j >= 0 && j < MAPY &&
            nearest_obstacle[i][j] == 0 && obs_map[i][j] == 0) {
          node_c.x = i;
          node_c.y = j;
          node_c.nearest_obstacle = node_p.nearest_obstacle + 1;
          nearest_obstacle[i][j] = node_c.nearest_obstacle;
          q.push(node_c);
        }
      }
    }
  }

  obs_dist_max = node_p.nearest_obstacle;
}

/**
 * Measure the distance to the nearest obstacle.
 *
 */

int Map::nearestObstacleDistance(State pos) {
  return nearest_obstacle[(int)pos.x][(int)pos.y];
}
