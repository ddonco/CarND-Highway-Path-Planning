#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "fsm.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// target lane
int targetLane = 1;
// target velocity
double targetVelocity = 0.0;
// vehicle is ahead
bool vehicleAhead = false;
// vehicle is to the left
bool vehicleToLeft = false;
// vehicle is to the right
bool vehicleToRight = false;

enum FSM_States {KeepLane, FollowVehicle, PrepareLaneChangeLeft, PrepareLaneChangeRight, LaneChangeLeft, LaneChangeRight};
static const char * StateStrings[] = {"Keep Lane", "Follow Vehicle", "Prepare Lane Change Left", "Prepare Lane Change Right", "Lane Change Left", "Lane Change Right"};
enum FSM_Triggers {OpenRoad, VehicleAhead};

using FiniteStateMachine = FSM::Fsm<FSM_States, FSM_States::KeepLane, FSM_Triggers>;
FiniteStateMachine fsm;

// transitions format:
// from state, to state, trigger, guard, action
std::vector<FiniteStateMachine::Trans> transitions = {
  // keepToChangeLeft
  {FSM_States::KeepLane, FSM_States::LaneChangeLeft, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead && !vehicleToLeft && targetLane > LEFT_LANE_ID;}, [&]{targetLane--;}},
  // keepToChangeRight
  {FSM_States::KeepLane, FSM_States::LaneChangeRight, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead && !vehicleToRight && targetLane != RIGHT_LANE_ID;}, [&]{targetLane++;}},
  // keepToFollow
  {FSM_States::KeepLane, FSM_States::FollowVehicle, FSM_Triggers::VehicleAhead, [&]{return true;}, [&]{targetVelocity -= MAX_DECELERATION;}},
  // changeLeftToKeep
  {FSM_States::LaneChangeLeft, FSM_States::KeepLane, FSM_Triggers::OpenRoad, [&]{return !vehicleAhead;}, [&]{}},
  // changeLeftToFollow
  {FSM_States::LaneChangeLeft, FSM_States::FollowVehicle, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead;}, [&]{ targetVelocity -= MAX_DECELERATION;}},
  // changeRightToKeep
  {FSM_States::LaneChangeRight, FSM_States::KeepLane, FSM_Triggers::OpenRoad, [&]{return !vehicleAhead;}, [&]{}},
  // changeRightToFollow
  {FSM_States::LaneChangeRight, FSM_States::FollowVehicle, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead;}, [&]{ targetVelocity -= MAX_DECELERATION;}},
  // followToChangeLeft
  {FSM_States::FollowVehicle, FSM_States::LaneChangeLeft, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead && !vehicleToLeft && targetLane > LEFT_LANE_ID;}, [&]{targetLane--;}},
  // followToChangeRight
  {FSM_States::FollowVehicle, FSM_States::LaneChangeRight, FSM_Triggers::VehicleAhead, [&]{return vehicleAhead && !vehicleToRight && targetLane != RIGHT_LANE_ID;}, [&]{targetLane++;}},
  // followToFollow
  {FSM_States::FollowVehicle, FSM_States::FollowVehicle, FSM_Triggers::VehicleAhead, [&]{return true;}, [&]{targetVelocity -= MAX_DECELERATION;}},
  // followToKeep
  {FSM_States::FollowVehicle, FSM_States::KeepLane, FSM_Triggers::OpenRoad, [&]{return !vehicleAhead;}, [&]{targetVelocity += MAX_ACCELERATION;}},
  // keepToKeep
  {FSM_States::KeepLane, FSM_States::KeepLane, FSM_Triggers::OpenRoad, [&]{return !vehicleAhead;}, [&]{if (targetVelocity < MAX_VELOCITY) {targetVelocity += MAX_ACCELERATION;}}}
};

void debug_func(FSM_States previousState, FSM_States nextState, FSM_Triggers) {
  if (previousState != nextState) {
    std::cout << "State Transition: " << StateStrings[previousState] << " --> " << StateStrings[nextState] << std::endl;
  }
}

int main() {
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
  while (getline(in_map_, line)) {
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

  fsm.add_transitions(transitions);
  fsm.add_debug_fn(debug_func);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
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

          int previousPathSize = previous_path_x.size();
          if (previousPathSize > 0) {
            car_s = end_path_s;
          }

          // ########## Prediction Logic ##########
          vehicleAhead = false;
          vehicleToLeft = false;
          vehicleToRight = false;

          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            int occupiedLane = INVALID_LANE_ID;

            if ( d > 0 && d < LEFT_LANE_MAX ) {
                occupiedLane = LEFT_LANE_ID;
            }
            if ( d > LEFT_LANE_MAX && d < MIDDLE_LANE_MAX ) {
                occupiedLane = MIDDLE_LANE_ID;
            }
            if ( d > MIDDLE_LANE_MAX && d < RIGHT_LANE_MAX ) {
                occupiedLane = RIGHT_LANE_ID;
            }
            if (occupiedLane == INVALID_LANE_ID) {
                continue;
            }

            double trackedVehicleVX = sensor_fusion[i][3];
            double trackedVehicleVY = sensor_fusion[i][4];
            double trackedVehicleV = sqrt((trackedVehicleVX * trackedVehicleVX) + (trackedVehicleVY * trackedVehicleVY));
            double trackedVehicleS = sensor_fusion[i][5];

            trackedVehicleS += previousPathSize * 0.02 * trackedVehicleV;

            if (occupiedLane == targetLane) {
              vehicleAhead |= trackedVehicleS > car_s && trackedVehicleS - car_s < BUFFER_DISTANCE;
            }
            if (occupiedLane - targetLane == -1) {
              vehicleToLeft |= car_s - BUFFER_DISTANCE < trackedVehicleS && car_s  + BUFFER_DISTANCE > trackedVehicleS;
            }
            if (occupiedLane - targetLane == 1) {
              vehicleToRight |= car_s - BUFFER_DISTANCE < trackedVehicleS && car_s + BUFFER_DISTANCE > trackedVehicleS;
            }
          }

          // ########## Behavior Logic ##########
          if (vehicleAhead) {
            fsm.execute(FSM_Triggers::VehicleAhead);
          } else {
            fsm.execute(FSM_Triggers::OpenRoad);
          }

          // ########## Trajectory Logic ##########
          vector<double> trajectoryX;
          vector<double> trajectoryY;

          double targetX = car_x;
          double targetY = car_y;
          double targetYaw = deg2rad(car_yaw);

          if (previousPathSize < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            trajectoryX.push_back(prev_car_x);
            trajectoryX.push_back(car_x);
            trajectoryY.push_back(prev_car_y);
            trajectoryY.push_back(car_y);
          } else {
            targetX = previous_path_x[previousPathSize - 1];
            targetY = previous_path_y[previousPathSize - 1];

            double targetPreviousX = previous_path_x[previousPathSize - 2];
            double targetPreviousY = previous_path_y[previousPathSize - 2];
            targetYaw = atan2(targetY - targetPreviousY, targetX - targetPreviousX);

            trajectoryX.push_back(targetPreviousX);
            trajectoryX.push_back(targetX);
            trajectoryY.push_back(targetPreviousY);
            trajectoryY.push_back(targetY);
          }

          vector<double> nextWaypoint = getXY(car_s + BUFFER_DISTANCE, (2 + 4 * targetLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWaypoint_1 = getXY(car_s + (BUFFER_DISTANCE * 2), (2 + 4 * targetLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextWaypoint_2 = getXY(car_s + (BUFFER_DISTANCE * 3), (2 + 4 * targetLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          trajectoryX.push_back(nextWaypoint[0]);
          trajectoryX.push_back(nextWaypoint_1[0]);
          trajectoryX.push_back(nextWaypoint_2[0]);
          trajectoryY.push_back(nextWaypoint[1]);
          trajectoryY.push_back(nextWaypoint_1[1]);
          trajectoryY.push_back(nextWaypoint_2[1]);

          for (int i = 0; i < trajectoryX.size(); i++) {
            double xOffset = trajectoryX[i] - targetX;
            double yOffset = trajectoryY[i] - targetY;
            trajectoryX[i] = ((trajectoryX[i] - targetX) * cos(0 - targetYaw)) - ((trajectoryY[i] - targetY) * sin(0 - targetYaw));
            trajectoryY[i] = ((trajectoryX[i] - targetX) * sin(0 - targetYaw)) + ((trajectoryY[i] - targetY) * cos(0 - targetYaw));
          }

          // create a trajectory spline
          tk::spline s;

          s.set_points(trajectoryX, trajectoryY);

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double splineX = 30.0;
          double splineY = s(splineX);
          double splineDistance = sqrt((splineX * splineX) + (splineY * splineY));

          double x_add_on = 0;
          // double calcX, calcY = 0.0;
          for (int i = 0; i < 50 - previous_path_x.size(); i++) {
            if (targetVelocity > MAX_VELOCITY) {
              targetVelocity = MAX_VELOCITY;
            }
            if (targetVelocity < MAX_ACCELERATION) {
              targetVelocity = MAX_ACCELERATION;
            }

            double n = (splineDistance / (0.02 * targetVelocity / 2.24));
            double x_point = x_add_on + splineX / n;
            double y_point = s(x_point);
            // calcX = calcX + splineX / n;
            // calcY = s(calcX);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            // double x_ref = calcX;
            // double y_ref = calcY;

            x_point = (x_ref * cos(targetYaw)) - (y_ref * sin(targetYaw));
            y_point = (x_ref * sin(targetYaw)) + (y_ref * cos(targetYaw));
            // calcX = (x_ref * cos(targetYaw)) - (y_ref * sin(targetYaw));
            // calcY = (x_ref * sin(targetYaw)) + (y_ref * cos(targetYaw));

            x_point += targetX;
            y_point += targetY;
            // calcX += targetX;
            // calcY += targetY;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            // next_x_vals.push_back(calcX);
            // next_y_vals.push_back(calcY);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}