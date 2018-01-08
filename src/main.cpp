#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <stdio.h>

using namespace std;

// for convenience
using json = nlohmann::json;

int INFO = 10;
int DBG = 5;
int CRITICAL = 1;
int LOG_LEVEL = 3;

#define LOG(w, x)                                                       \
  {                                                                     \
    if (w <= LOG_LEVEL)                                                 \
    {                                                                   \
      cout << __FUNCTION__ << "::" << __LINE__ << "->" << x << endl;    \
    }                                                                   \
  }

// For converting back and forth between radians and degrees.
constexpr double
pi()
{
  return M_PI;
}

double
deg2rad(double x)
{
  return x * pi() / 180;
}

double
rad2deg(double x)
{
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string
hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }

  return "";
}

/*
 reset_simulator: picked from
 https://github.com/shehjar/Term2_P4_PID_Controller/blob/master/src/main.cpp
 */
void
reset_simulator(uWS::WebSocket<uWS::SERVER>& ws)
{
  std::string msg("42[\"reset\",{}]");
  ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

enum Phase
{
  Start,
  Increase,
  Decrease
};

class Twiddler
{
public:

  double kp;
  double ki;
  double kd;
  double throttle;

  bool if_twiddle;

  vector<double> p;
  vector<double> dp;
  double best_err = numeric_limits<double>::max();

  long param_twiddled = 0;
  double err = 0;
  long ts = 0;
  double min_sum_dp = 0.2;
  Phase phase = Start;

  long num_params = 3;
  long it = 0;

  bool ignore_update = true;

  PID steer_pid;

  Twiddler(int argc, char* argv[]);

  bool UpdateState(double cte);

  double GetThrottle();

  double GetSteering();

  void Reset();
};

Twiddler::Twiddler(int argc, char* argv[])
{
  kp = 0.2;
  ki = 0.004;
  kd = 3.0;
  throttle = 0.1;

  if_twiddle = false;

  if (argc > 2)
  {
    if (!strcmp(argv[1], "yes") || !strcmp(argv[1], "y"))
    {
      if_twiddle = true;
    }

    if (argc >= 5)
    {
      kp = atof(argv[2]);
      ki = atof(argv[3]);
      kd = atof(argv[4]);
    }

    if (argc == 6)
    {
      throttle = atof(argv[5]);
    }
  }

  if (if_twiddle)
  {
    for (int i = 0; i < num_params; i++)
    {
      p.push_back(atof(argv[i + 2]));
      dp.push_back(1.0);
    }

    steer_pid.Init(0, 0, 0);
  }
}

void
Twiddler::Reset()
{
  LOG(INFO, "Enter");

  if (!if_twiddle)
  {
    LOG(CRITICAL, "NO_TWIDDLE trying: kp, ki, kd, t=" + to_string(kp) + "," + to_string(ki) + "," + to_string(kd) + "," + to_string(throttle));
    steer_pid.Init(kp, ki, kd);
    ignore_update = true;
    return;
  }

  err = err / ts;
  LOG(CRITICAL, "Twiddle: it, kp, ki, kd, t, err=" + to_string(it) + "," + to_string(p[0]) + "," + to_string(p[1]) + "," + to_string(p[2]) + "," + to_string(throttle) + "," + to_string(err));

  switch (phase)
  {
  case Start:
    LOG(INFO, "");

    best_err = err;
    kp = p.at(0);
    ki = p.at(1);
    kd = p.at(2);

    p.at(param_twiddled) += dp.at(param_twiddled);
    param_twiddled = param_twiddled % num_params;

    phase = Increase;

    LOG(INFO, "");

    break;

  case Increase:
    LOG(INFO, "");

    if (err < best_err)
    {
      best_err = err;
      kp = p.at(0);
      ki = p.at(1);
      kd = p.at(2);

      dp.at(param_twiddled) *= 1.1;
      param_twiddled++;
      param_twiddled = param_twiddled % num_params;

      p.at(param_twiddled) += dp.at(param_twiddled);
      phase = Increase;
    }
    else
    {
      p.at(param_twiddled) -= 2 * dp.at(param_twiddled);
      phase = Decrease;
    }

    LOG(INFO, "");

    break;

  case Decrease:
    LOG(INFO, "");

    if (err < best_err)
    {
      best_err = err;
      kp = p.at(0);
      ki = p.at(1);
      kd = p.at(2);

      dp.at(param_twiddled) *= 1.1;

      param_twiddled++;
      param_twiddled = param_twiddled % num_params;

      p.at(param_twiddled) += dp.at(param_twiddled);
      param_twiddled = param_twiddled % num_params;

      phase = Increase;
    }
    else
    {
      p.at(param_twiddled) += dp.at(param_twiddled);
      dp.at(param_twiddled) *= 0.9;

      param_twiddled++;
      param_twiddled = param_twiddled % num_params;

      p.at(param_twiddled) += dp.at(param_twiddled);
      phase = Increase;
    }

    LOG(INFO, "");

    break;
  }

  double sum_dp = 0;
  for (int i = 0; i < num_params; i++)
  {
    sum_dp += dp.at(i);
  }

  if (sum_dp > min_sum_dp)
  {
    steer_pid.Init(p.at(0), p.at(1), p.at(2));
    it++;
  }
  else
  {
    LOG(CRITICAL, "Finished Twiddling");
    steer_pid.Init(kp, ki, kd);
  }

  ignore_update = true;

  LOG(INFO, "Exit");
}

bool
Twiddler::UpdateState(double cte)
{
  if (ignore_update)
  {
    LOG(CRITICAL, "Ignoring");

    return true;
  }

  LOG(INFO, "Enter");

  ts++;
  err = err + (cte * cte);
  steer_pid.UpdateError(cte);

  if (abs(cte) > 4.0)
  {
    Reset();

    LOG(INFO, "Exit");

    return false;
  }

  LOG(INFO, "Exit");

  return true;
}

double
Twiddler::GetSteering()
{
  LOG(INFO, "Enter");

  // < 0 => steer left
  // < -4.2 => out of track on left side
  // > 0 => steer right
  // > 4.0 => out of track on right side

  return steer_pid.TotalError();
}

double
Twiddler::GetThrottle()
{
  return throttle;
}

int
main(int argc, char* argv[])
{
  uWS::Hub h;
  Twiddler twiddler(argc, argv);

  h.onMessage(
    [&twiddler]
    (uWS::WebSocket<uWS::SERVER> ws,
     char *data,
     size_t length,
     uWS::OpCode opCode)
    {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data).substr(0, length));
        if (s != "")
        {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          if (event == "telemetry")
          {
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            // double speed = std::stod(j[1]["speed"].get<std::string>());
            // double angle = std::stod(j[1]["steering_angle"].get<std::string>());

            if (!twiddler.UpdateState(cte))
            {
              reset_simulator(ws);
              return;
            }

            double steer_value = twiddler.GetSteering();
            double throttle = twiddler.GetThrottle();

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            if (LOG_LEVEL > INFO)
            {
              LOG(INFO, "last cte = " + to_string(cte) + ", current-steer = " + to_string(steer_value));
              getchar();
            }
          }
        }
        else
        {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
    });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest(
    []
    (uWS::HttpResponse *res,
     uWS::HttpRequest req,
     char *data,
     size_t,
     size_t)
    {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1)
      {
        res->end(s.data(), s.length());
      }
      else
      {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
    });

  h.onConnection(
    [&h, &twiddler]
    (uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
      twiddler.ignore_update = false;
      LOG(CRITICAL, "Connected");
    });

  h.onDisconnection(
    [&h]
    (uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      LOG(CRITICAL, "Disconnected");
    });

  int port = 4567;
  if (h.listen(port))
  {
    LOG(CRITICAL, "Listening to port " + to_string(port));
  }
  else
  {
    LOG(CRITICAL, "Failed to listen to port");
    return -1;
  }

  h.run();
}
