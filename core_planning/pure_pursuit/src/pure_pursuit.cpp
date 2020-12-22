/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pure_pursuit/pure_pursuit.h>

namespace waypoint_follower
{
// Constructor
PurePursuit::PurePursuit()
  : RADIUS_MAX_(9e10)
  , KAPPA_MIN_(1 / RADIUS_MAX_)
  , is_linear_interpolation_(false)
  , next_waypoint_number_(-1)
  , lookahead_distance_(0)
  , minimum_lookahead_distance_(6)
  , current_linear_velocity_(0)
{
}

// Destructor
PurePursuit::~PurePursuit()
{
}

double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa;
  double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

  if (denominator != 0)
  {
    kappa = numerator / denominator;
  }
  else
  {
    if (numerator > 0)
    {
      kappa = KAPPA_MIN_;
    }
    else
    {
      kappa = -KAPPA_MIN_;
    }
  }
  ROS_INFO("kappa : %lf", kappa);
  return kappa;
}

// linear interpolation of next target
bool PurePursuit::interpolateNextTarget(
  int next_waypoint, geometry_msgs::Point* next_target) const
{
  constexpr double ERROR = pow(10, -5);  // 0.00001

  int path_size = static_cast<int>(current_waypoints_.size());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.at(next_waypoint).pose.pose.position;
    return true;
  }
  double search_radius = lookahead_distance_;
  geometry_msgs::Point zero_p;
  geometry_msgs::Point end =
    current_waypoints_.at(next_waypoint).pose.pose.position;
  geometry_msgs::Point start =
    current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // let the linear equation be "ax + by + c = 0"
  // if there are two points (x1,y1) , (x2,y2),
  // a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
  double a = 0;
  double b = 0;
  double c = 0;
  double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
  if (!get_linear_flag)
    return false;

  // let the center of circle be "(x0,y0)", in my code ,
  // the center of circle is vehicle position
  // the distance  "d" between the foot of
  // a perpendicular line and the center of circle is ...
  //    | a * x0 + b * y0 + c |
  // d = -------------------------------
  //          √( a~2 + b~2)
  double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

  // ROS_INFO("a : %lf ", a);
  // ROS_INFO("b : %lf ", b);
  // ROS_INFO("c : %lf ", c);
  // ROS_INFO("distance : %lf ", d);

  if (d > search_radius)
    return false;

  // unit vector of point 'start' to point 'end'
  tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
  tf::Vector3 unit_v = v.normalize();

  // normal unit vectors of v
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);
  // rotate to counter clockwise 90 degree
  tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);

  // the foot of a perpendicular line
  geometry_msgs::Point h1;
  h1.x = current_pose_.position.x + d * unit_w1.getX();
  h1.y = current_pose_.position.y + d * unit_w1.getY();
  h1.z = current_pose_.position.z;

  geometry_msgs::Point h2;
  h2.x = current_pose_.position.x + d * unit_w2.getX();
  h2.y = current_pose_.position.y + d * unit_w2.getY();
  h2.z = current_pose_.position.z;

  // ROS_INFO("error : %lf", error);
  // ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
  // ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

  // check which of two foot of a perpendicular line is on the line equation
  geometry_msgs::Point h;
  if (fabs(a * h1.x + b * h1.y + c) < ERROR)
  {
    h = h1;
    //   ROS_INFO("use h1");
  }
  else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
  {
    //   ROS_INFO("use h2");
    h = h2;
  }
  else
  {
    return false;
  }

  // get intersection[s]
  // if there is a intersection
  if (d == search_radius)
  {
    *next_target = h;
    return true;
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(d, 2));
    geometry_msgs::Point target1;
    target1.x = h.x + s * unit_v.getX();
    target1.y = h.y + s * unit_v.getY();
    target1.z = current_pose_.position.z;

    geometry_msgs::Point target2;
    target2.x = h.x - s * unit_v.getX();
    target2.y = h.y - s * unit_v.getY();
    target2.z = current_pose_.position.z;

    // ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
    // ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
    // displayLinePoint(a, b, c, target1, target2, h);  // debug tool

    // check intersection is between end and start
    double interval = getPlaneDistance(end, start);
    if (getPlaneDistance(target1, end) < interval)
    {
      // ROS_INFO("result : target1");
      *next_target = target1;
      return true;
    }
    else if (getPlaneDistance(target2, end) < interval)
    {
      // ROS_INFO("result : target2");
      *next_target = target2;
      return true;
    }
    else
    {
      // ROS_INFO("result : false ");
      return false;
    }
  }
}


int PurePursuit::getNextWaypointNumber(bool use_lookahead_distance)
{
  int next_waypoint_number = -1;
  
  int path_size = static_cast<int>(current_waypoints_.size());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    next_waypoint_number = -1;
    return next_waypoint_number;
  }
  double closest_distance = getPlaneDistance(current_waypoints_.at(0).pose.pose.position, current_pose_.position);
  // look for the next waypoint.
  for (int i = 1; i < path_size; i++)
  {
    bool min_lookahead_satisfied = false;
    bool in_front = false;
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO_STREAM(">> Search waypoint reached the last: x: " << current_waypoints_.at(i).pose.pose.position.x 
                                              << ", y: " << current_waypoints_.at(i).pose.pose.position.y << ", speed: " << current_waypoints_.at(i).twist.twist.linear.x * 2.23694 << "mph");
      next_waypoint_number = i;
      std::cerr << ">> Search waypoint reached the last: x: " << current_waypoints_.at(i).pose.pose.position.x 
                                              << ", y: " << current_waypoints_.at(i).pose.pose.position.y << ", speed: " << current_waypoints_.at(i).twist.twist.linear.x * 2.23694 << "mph" << std::endl;

      return next_waypoint_number;
    }

    double current_distance = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
    
    // due to the fact that waypoints represent small portion of the road, there is no suddent change in direction
    // also since waypoints are in increasing distance from old curr_pos (which may have passed),
    // waypoint distances first decrease and increase back again, which helps find the closest point   
    if (use_lookahead_distance)
    {
      // loop through until we hit the closest and effective point
      if (current_distance < closest_distance || closest_distance < lookahead_distance_)
      {
        closest_distance = current_distance;
        continue;
      }
    }
    else
    {
      // loop through until we hit the closest point
      if (current_distance < closest_distance )
      {
        closest_distance = current_distance;
        continue;
      }
    }
    // Else, by this point, we found that prev point is the closest point and is bigger than lookahead_distance if applicable

    // then check if the prev point is in front or back
    tf::Vector3 curr_vector(current_waypoints_.at(i - 1).pose.pose.position.x - current_pose_.position.x, 
                      current_waypoints_.at(i - 1).pose.pose.position.y - current_pose_.position.y, 
                      current_waypoints_.at(i - 1).pose.pose.position.z - current_pose_.position.z);
    curr_vector.setZ(0);
    tf::Vector3 traj_vector(current_waypoints_.at(i).pose.pose.position.x - current_waypoints_.at(i - 1).pose.pose.position.x, 
                      current_waypoints_.at(i).pose.pose.position.y - current_waypoints_.at(i - 1).pose.pose.position.y, 
                      current_waypoints_.at(i).pose.pose.position.z - current_waypoints_.at(i - 1).pose.pose.position.z);
    traj_vector.setZ(0);
    double angle_in_rad = std::abs(tf::tfAngle(curr_vector, traj_vector));
    std::cerr << "curr x:" << current_waypoints_.at(i).pose.pose.position.x << ",y:" << current_waypoints_.at(i).pose.pose.position.y << std::endl;
    std::cerr << ">> Angle degrees: "  << angle_in_rad / M_PI * 180 <<std::endl;
    // if degree between curr_vector and the direction of the trajectory is more than 90 degrees, we know last point is behind us and unsatisfactory
    if (std::isnan(angle_in_rad) || angle_in_rad > M_PI / 2)
    {
      std::cerr << ">> We are not passing: degrees: "  << angle_in_rad / M_PI * 180 <<std::endl;
      closest_distance = current_distance;
      continue;
    }
    std::cerr << ">> We are passing: degrees: "  << angle_in_rad / M_PI * 180 <<std::endl;
    next_waypoint_number = i - 1;
    return next_waypoint_number;
  }
  
  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number = -1;
  return next_waypoint_number;
}

void PurePursuit::setNextWaypoint(int next_waypoint_number)
{
  next_waypoint_number_ = next_waypoint_number;
}

bool PurePursuit::canGetCurvature(double* output_kappa)
{
  // search next waypoint
  next_waypoint_number_ = getNextWaypointNumber();
  if (next_waypoint_number_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }
  // check whether curvature is valid or not
  bool is_valid_curve = false;
  for (const auto& el : current_waypoints_)
  {
    if (getPlaneDistance(el.pose.pose.position, current_pose_.position)
      > minimum_lookahead_distance_)
    {
      is_valid_curve = true;
      break;
    }
  }
  if (!is_valid_curve)
  {
    return false;
  }
  // if is_linear_interpolation_ is false or next waypoint is first or last
  if (!is_linear_interpolation_ || next_waypoint_number_ == 0 ||
    next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1)))
  {
    next_target_position_ =
      current_waypoints_.at(next_waypoint_number_).pose.pose.position;
    *output_kappa = calcCurvature(next_target_position_);
    return true;
  }

  // linear interpolation and calculate angular velocity
  bool interpolation =
    interpolateNextTarget(next_waypoint_number_, &next_target_position_);

  if (!interpolation)
  {
    ROS_INFO_STREAM("lost target! ");
    return false;
  }

  // ROS_INFO("next_target : ( %lf , %lf , %lf)",
  //  next_target.x, next_target.y,next_target.z);

  *output_kappa = calcCurvature(next_target_position_);
  return true;
}

}  // namespace waypoint_follower
