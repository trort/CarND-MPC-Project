#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "poly_utils.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

vector<double> mapCoordinate2Car(double x, double y, double car_x, double car_y, double car_psi){
  double x_prime = x - car_x;
  double y_prime = y - car_y;
  double transformed_x = cos(car_psi) * x_prime + sin(car_psi) * y_prime;
  double transformed_y = -sin(car_psi) * x_prime + cos(car_psi) * y_prime;
  return {transformed_x, transformed_y};
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double current_steer = -1 * double(j[1]["steering_angle"]);
          double current_throttle = j[1]["throttle"];

          // car perspective
          Eigen::VectorXd ptsx_transformed(ptsx.size());
          Eigen::VectorXd ptsy_transformed(ptsx.size());
          for(int i = 0; i < ptsx.size(); ++i){
            vector<double> transformed = mapCoordinate2Car(ptsx[i], ptsy[i], px, py, psi);
            ptsx_transformed[i] = transformed[0];
            ptsy_transformed[i] = transformed[1];
          }

          auto coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
          double cte = polyeval(coeffs, 0.0);
          double epsi = -atan(polyderivative(coeffs, 0.0));

          Eigen::VectorXd state(8);
          state << 0.0, 0.0, 0.0, v, cte, epsi, current_steer, current_throttle;
          std::cout << "Current steer: " << current_steer << "; throttle: " << current_throttle << endl;

          /*
          * Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          */
          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[2 * N];
          double throttle_value = vars[2 * N + 1];

          double delayed_x = vars[2 * N + 2];
          double delayed_y = vars[2 * N + 3];
          double delayed_psi = vars[2 * N + 4];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = - steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          std::cout << "Desired steer: " << steer_value << "; throttle: " << throttle_value << endl;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals(N);
          vector<double> mpc_y_vals(N);
          for(int i = 0; i < N; i++){
            vector<double> mpc_position = mapCoordinate2Car(
              vars[i], vars[i + N], delayed_x, delayed_y, delayed_psi
            );
            mpc_x_vals[i] = mpc_position[0];
            mpc_y_vals[i] = mpc_position[1];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(12);
          vector<double> next_y_vals(12);
          for(int i = 0; i < 12; i++){
            vector<double> next_position = mapCoordinate2Car(
              i * 5.0, polyeval(coeffs, i * 5.0), delayed_x, delayed_y, delayed_psi
            );
            next_x_vals[i] = next_position[0];
            next_y_vals[i] = next_position[1];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
