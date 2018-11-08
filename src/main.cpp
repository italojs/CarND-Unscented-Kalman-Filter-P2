#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checking if the socketio's event has json data.
std::string hasData(std::string json_string) {
  auto found_null = json_string.find("null");
  if (found_null != std::string::npos) 
  {
    return "";
  }

  auto b_1 = json_string.find_first_of("[");
  auto b_2 = json_string.find_first_of("]");
  if (b_1 != std::string::npos && b_2 != std::string::npos) 
  {
    return json_string.substr(b_1, b_2 - b_1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub hub;

  UKF ukf;

  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  hub.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // 4: signifies a websocket message
    // 2: signifies a websocket event

    if (length && (length > 2) && (data[0] == '4') && (data[1] == '2'))
    {
      auto str = hasData(std::string(data));
      if (str != "") 
      {
        auto json_ = json::parse(str);

        std::string event = json_[0].get<std::string>();

        if (event == "telemetry")
        {

          string sensor_measurment = json_[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0)
          {
                meas_package.sensor_type_ = MeasurementPackage::LASER;
                meas_package.raw_measurements_ = VectorXd(2);

                float px;
                float py;

                iss >> px;
                iss >> py;

                meas_package.raw_measurements_ << px, py;
                iss >> timestamp;
                meas_package.timestamp_ = timestamp;
          } 
          else if (sensor_type.compare("R") == 0)
          {
      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);

          		float ro;
      	  		float theta;
      	  		float ro_dot;

          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;

          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;

          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);

          ukf.ProcessMeasurement(meas_package);

          VectorXd estimate(4);

          double p_x = ukf.x_(0);
          double p_y = ukf.x_(1);
          double v  = ukf.x_(2);
          double yaw = ukf.x_(3);

          double v1 = cos(yaw)*v;
          double v2 = sin(yaw)*v;

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msg_json;
          msg_json["estimate_x"] = p_x;
          msg_json["estimate_y"] = p_y;

          msg_json["rmse_x"] =  RMSE(0);
          msg_json["rmse_y"] =  RMSE(1);
          msg_json["rmse_vx"] = RMSE(2);
          msg_json["rmse_vy"] = RMSE(3);

          auto msg = "42[\"estimate_marker\"," + msg_json.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } 
      else 
      {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // TODO: remove it from here because we don't using HTTP protocol
  hub.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      res->end(nullptr, 0);
    }
  });

  hub.onConnection([&hub](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (hub.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  hub.run();
}
