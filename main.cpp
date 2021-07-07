#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
// for convenience
using nlohmann::json;
using std::string;
using std::vector;
#define LANE_WIDTH 4
#define OPTIMAL_VELOCITY 48.5
// MSelim: Define Functions in this area

int lane = 0;          //3 possible values. 0,1 and 2
double ref_vel = 48.5; //velocity of car, note that time is constant and distance changes.

//-----------------------------------------------------------------------------------------------------------

vector<double> make_prediction(int lane, int future_steps, double car_s, double end_path_s, double car_speed, vector<vector<double>> sensor_fusion)
{
  vector<double> vect;

  double new_car_s = end_path_s; //end path s is the s of the last point of ur previous path.
  if (end_path_s - car_s < 0)
  {
    new_car_s = car_s;
  }
  new_car_s += ((double)future_steps * 0.02 * car_speed);
  vect.push_back(new_car_s);

  for (auto &car_reading : sensor_fusion) //sensor_fusion is vector of vectors. looping on every vector
  {
    float d = car_reading[6];
    double other_car_s = car_reading[5];
    if (d < (LANE_WIDTH + LANE_WIDTH * lane) && d > (LANE_WIDTH * lane) && other_car_s - car_s < 30 && other_car_s - car_s > -5) // checking on the cars in same lane
    {
      double vx = car_reading[3];             //velocity in x coordinate
      double vy = car_reading[4];             //velocity in y coordinate
      double speed = sqrt(vx * vx + vy * vy); //speed is ecludian between them
      //number of steps ahead(prev_size) * distance of each step (0.02 *speed)
      other_car_s += ((double)future_steps * 0.02 * speed);
      vect.push_back(other_car_s);
    }
  }

  return vect;
}

//-----------------------------------------------------------------------------------------------------------

double distance_cost(vector<double> prediction)
{
  double future_car_s = prediction[0]; //our car's future s coordinate based on the make predictions function
  double minDist = sqrt((future_car_s - prediction[1]) * (future_car_s - prediction[1]));
  for (int i = 2; i < prediction.size(); i++)
  {
    double dist = sqrt((future_car_s - prediction[i]) * (future_car_s - prediction[i]));
    if (dist < minDist)
    {
      minDist = dist;
    }
  }
  double maxCost = (5 * ((30 / minDist) - 1));

  if (maxCost < 0)
    maxCost = 0;

  return maxCost; //returns the cost of the closest car near our car
}

//-----------------------------------------------------------------------------------------------------------

double velocity_cost(double velocity)
{
  double x = ((velocity - OPTIMAL_VELOCITY) * (velocity - OPTIMAL_VELOCITY)) / 2;
  return x;
}

//-----------------------------------------------------------------------------------------------------------

double absolute(double number)
{
  if (number < 0)
  {
    return -1 * number;
  }
  else
  {
    return number;
  }
}
//-------------------------------------------------------------------------
double turning_cost(double current_lane, double future_lane)
{
  double x = current_lane - future_lane;
  return absolute(x);
}

//-----------------------------------------------------------------------------------------------------------

double compute_cost(vector<double> prediction, double velocity, double current_lane, double future_lane)
{
  return distance_cost(prediction) + 0.1 * velocity_cost(velocity) + turning_cost(current_lane, future_lane);
}

//-----------------------------------------------------------------------------------------------------------

bool shoulditurn(int lane, int minLaneIndx, vector<vector<double>> sensor_fusion, double car_s)
{
  //check if the lane i am turning to has a car in the same s
  for (auto &car_reading : sensor_fusion)
  {
    double rival_s = car_reading[5];
    double rival_d = car_reading[6];
    //check for the lane
    if (rival_d < (LANE_WIDTH + LANE_WIDTH * minLaneIndx) && rival_d > (LANE_WIDTH * minLaneIndx))
    { //check if this car is beside me
      if (absolute(rival_s - car_s)<20)
      {
        if(rival_s < car_s && ((car_s - rival_s) >10)){
          continue;
        }
          return false;
      }
    }
  }
  if ((lane ==2 && minLaneIndx ==0)||(lane == 0 && minLaneIndx == 2)){
    return shoulditurn(lane, 1, sensor_fusion, car_s);
  } else{
      return true;
  }
}
//----------------------------------------------------------------------------------------------
void choose_next_state(int prev_size, double car_s, double end_path_s, vector<vector<double>> sensor_fusion)
{
  double x[3] = {};
  for (int i = 0; i < 3; i++)
  {
      double velocity = OPTIMAL_VELOCITY;
      vector<double> prediction = make_prediction(i, 50 - prev_size, car_s, end_path_s, velocity, sensor_fusion);
      x[i] = compute_cost(prediction, velocity, lane, i);
  }

  // if (lane == 0)
  // {
  //   for (int i = 0; i < 10; i++)
  //     x[2][i] += x[1][i];
  // }
  // else if (lane == 2)
  // {
  //   for (int i = 0; i < 10; i++)
  //     x[0][i] += x[1][i];
  // }

  double min = x[0];
  int minLaneIndx = 0;
  for (int i = 0; i < 3; i++)
  {
      if (x[i] < min)
      {
        min = x[i];
        minLaneIndx = i;
      }
  }
  if (lane == minLaneIndx)
  { //if the best lane is the same lane : do nothing
    lane = minLaneIndx;
    ref_vel = OPTIMAL_VELOCITY;
    //if the best lane is the same lane and the lane is stuck then decrease velocity
    if(x[lane] > 30)
        ref_vel = 32;
  }
  else
  { //check for cars to your side
    if (shoulditurn(lane /*current lane*/, minLaneIndx /*desired lane*/, sensor_fusion, car_s))
    {
      lane = minLaneIndx;
      ref_vel = OPTIMAL_VELOCITY;
    }
    else
    {
      //remain in the same lane and decrease velocity gradually
      ref_vel = 32;
    }
  }
}

//-----------------------------------------------------------------------------------------------------------

// MSelim: End custom functions definitions

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];

                      json msgJson;

                      vector<double> next_x_vals;
                      vector<double> next_y_vals;

                      // MSelim: Begin your code starting from this point
                      /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
                      int prev_size = previous_path_x.size(); // number of unvisited points that has been planned

                      choose_next_state(prev_size, car_s, end_path_s, sensor_fusion);
                      vector<double> ptsx; // points planned to be sent to the spline function
                      vector<double> ptsy;

                      double ref_x = car_x;
                      double ref_y = car_y;
                      double ref_yaw = deg2rad(car_yaw);

                      if (prev_size < 2) // the if and else are used to add 2 points, current and previous, so that to make the spline curve smoother.
                      {                  // get a point exactly behind the car just for the sake of filling the array
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                      }
                      else
                      {
                        ref_x = previous_path_x[prev_size - 1]; // change the origin to the last planned position of the car
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                      }

                      double ref_car_s = car_s;
                      if (end_path_s - car_s > 0)
                        ref_car_s = end_path_s;
                      vector<double> next_wp0 = getXY(ref_car_s + 30, LANE_WIDTH / 2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_wp1 = getXY(ref_car_s + 40, LANE_WIDTH / 2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      vector<double> next_wp2 = getXY(ref_car_s + 50, LANE_WIDTH / 2 + (LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                      ptsx.push_back(next_wp0[0]);
                      ptsx.push_back(next_wp1[0]);
                      ptsx.push_back(next_wp2[0]);

                      ptsy.push_back(next_wp0[1]);
                      ptsy.push_back(next_wp1[1]);
                      ptsy.push_back(next_wp2[1]);

                      for (size_t i = 0; i < ptsx.size(); i++)
                      { // make the origin based on refx and refy(car position), which is the last point planned.
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
                      }
                      //call spline and send the vectors (path)
                      tk::spline s;
                      s.set_points(ptsx, ptsy);

                      next_x_vals.clear();
                      next_y_vals.clear();

                      for (int i = 0; i < prev_size; i++)
                      {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                      }

                      double target_x = 30.0;
                      double target_y = s(target_x);
                      double target_dist = sqrt(target_x * target_x + target_y * target_y);

                      double x_add_on = 0;

                      for (int i = 1; i < 50 - prev_size; i++)
                      { // change the curve into points to be put into the vector.
                        double N = target_dist / (0.02 * ref_vel / 2.24);
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                      }

                      // double dist_inc = 0.5;
                      // for(int i=0; i<50; ++i)
                      // {
                      //   double next_s = car_s + (i+1)*dist_inc;
                      //   double next_d= 6;

                      //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      //   next_x_vals.push_back(xy[0]);
                      //   next_y_vals.push_back(xy[1]);
                      // }

                      // MSelim: End your code before this point

                      msgJson["next_x"] = next_x_vals;
                      msgJson["next_y"] = next_y_vals;

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                } // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}