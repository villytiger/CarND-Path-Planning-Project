#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <experimental/optional>
#include <thread>
#include <vector>

#include <uWS/uWS.h>

#include "Eigen/Core"
#include "Eigen/QR"

#include "json/json.hpp"

#include "spline/spline.h"

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

struct Cartesian {
  Cartesian() = default;

  Cartesian(double x, double y)
    : x(x)
    , y(y) {
  }
  
  double x;
  double y;
};

struct Frenet {
  double s;
  double d;
};

double Distance(Cartesian coords1, Cartesian coords2) {
  return sqrt(pow(coords2.x - coords1.x, 2) + pow(coords2.y - coords1.y, 2));
}

double Distance(Frenet coords1, Frenet coords2) {
  return sqrt(pow(coords2.s - coords1.s, 2) + pow(coords2.d - coords1.d, 2));
}

Cartesian Rotate(Cartesian cartesian, double angle) {
  return {
      cartesian.x * cos(angle) + cartesian.y * sin(angle),
      -cartesian.x *sin(angle) + cartesian.y * cos(angle)
  };
}

pair<vector<double>, vector<double>> SplitCoordinates(const vector<Cartesian>& cartesian) {
  vector<double> x;
  vector<double> y;

  for (const auto& c: cartesian) {
    x.emplace_back(c.x);
    y.emplace_back(c.y);
  }

  return {move(x), move(y)};
}

vector<Cartesian> MergeCoordinates(const vector<double>& x, const vector<double>& y) {
  assert(x.size() == y.size());
  
  vector<Cartesian> c;

  for (size_t i = 0; i != x.size(); ++i) {
    c.emplace_back(x[i], y[i]);
  }

  return c;
}

template<typename F, typename T, bool done = false, int... n>
struct Caller {
  static auto Call(F&& f, T&& t) {
    return Caller<F, T, tuple_size<T>::value == 1 + sizeof...(n), n..., sizeof...(n)>::Call(forward<F>(f), forward<T>(t));
  }
};

template<typename F, typename T, int... n>
struct Caller<F, T, true, n...> {
  static auto Call(F&& f, T&& t) {
    return f(get<n>(forward<T>(t))...);
  }
};

template<typename F, typename T>
auto Call(F&& f, T&& t) {
  return Caller<F, T, false>::Call(forward<F>(f), forward<T>(t));
}

class Map {
public:
  Map(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
    : maps_s_(move(maps_s))
    , maps_x_(move(maps_x))
    , maps_y_(move(maps_y)) {
  }
  
  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  Frenet ToFrenet(Cartesian cartesian, double theta) {
    int next_wp = NextWaypoint(cartesian, theta);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = maps_x_.size() - 1;
    }

    double n_x = maps_x_[next_wp] - maps_x_[prev_wp];
    double n_y = maps_y_[next_wp] - maps_y_[prev_wp];
    double x_x = cartesian.x - maps_x_[prev_wp];
    double x_y = cartesian.y - maps_y_[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = Distance(Cartesian(x_x, x_y), Cartesian(proj_x, proj_y));

    // see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x_[prev_wp];
    double center_y = 2000 - maps_y_[prev_wp];
    double centerToPos = Distance(Cartesian(center_x, center_y), Cartesian(x_x, x_y));
    double centerToRef = Distance(Cartesian(center_x, center_y), Cartesian(proj_x, proj_y));

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
      frenet_s += Distance(Cartesian(maps_x_[i], maps_y_[i]),
                           Cartesian(maps_x_[i + 1], maps_y_[i + 1]));
    }

    frenet_s += Distance(Cartesian(0, 0), Cartesian(proj_x, proj_y));

    return {frenet_s, frenet_d};
  }

  Cartesian ToCartesian(Frenet frenet) {
    int prev_wp = -1;

    while (frenet.s > maps_s_[prev_wp + 1] && (prev_wp < (int)(maps_s_.size() - 1))) {
      prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x_.size();

    double heading =
      atan2((maps_y_[wp2] - maps_y_[prev_wp]), (maps_x_[wp2] - maps_x_[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (frenet.s - maps_s_[prev_wp]);

    double seg_x = maps_x_[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y_[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + frenet.d * cos(perp_heading);
    double y = seg_y + frenet.d * sin(perp_heading);

    return {x, y};
  }

private:
  int ClosestWaypoint(Cartesian cartesian) {
    double closestLen = 100000;  // large number
    int closestWaypoint = 0;

    for (size_t i = 0; i < maps_x_.size(); ++i) {
      double map_x = maps_x_[i];
      double map_y = maps_y_[i];
      double dist = Distance(cartesian, {map_x, map_y});
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }
    }

    return closestWaypoint;
  }

  int NextWaypoint(Cartesian cartesian, double theta) {
    int closestWaypoint = ClosestWaypoint(cartesian);

    double map_x = maps_x_[closestWaypoint];
    double map_y = maps_y_[closestWaypoint];

    double heading = atan2((map_y - cartesian.y), (map_x - cartesian.x));

    double angle = abs(atan2(sin(heading - theta), cos(heading - theta)));

    if (angle > pi() / 2) {
      closestWaypoint++;
    }

    return closestWaypoint;
  }
  
  vector<double> maps_s_;
  vector<double> maps_x_;
  vector<double> maps_y_;
};

struct SensorData {
  Cartesian cartesian;
  double vx;
  double vy;
  Frenet frenet;
};

class Driver {
public:
  explicit Driver(Map map)
    : map_(map) {
  }

  void UpdateState(Cartesian car, double car_yaw, vector<Cartesian> pts,
                   vector<SensorData> vehicles) {
    if (pts.size() < 2) {
      last_pos_ = car;
      car_yaw_ = car_yaw;
      velocity_ = 0;
    } else {      
      Cartesian prev = *(pts.end() - 2);
      
      last_pos_ = pts.back();
      car_yaw_ = atan2(last_pos_.y - prev.y, last_pos_.x - prev.x);
      velocity_ = 50 * Distance(prev, last_pos_);
    }

    current_pos_ = car;
    current_frenet_ = map_.ToFrenet(car, car_yaw_);

    last_frenet_ = map_.ToFrenet(last_pos_, car_yaw_);

    pts_ = move(pts);
    vehicles_ = move(vehicles);

    ChooseLane();
    UpdatePath();
  }

  const vector<Cartesian>& Points() const {
    return pts_;
  }

private:
  enum class State {
    none,
    change
  };
  
  void UpdatePath() {
    vector<Cartesian> spline_pts;

    if (pts_.size() < 2) {
      spline_pts.emplace_back(last_pos_.x - cos(car_yaw_), last_pos_.y - sin(car_yaw_));
      spline_pts.emplace_back(last_pos_);
    } else {
      spline_pts.insert(spline_pts.end(), pts_.end() - 2, pts_.end());
    }

    double car_s = map_.ToFrenet(last_pos_, car_yaw_).s;
    double target_d = 2 + 4 * target_lane_;

    spline_pts.emplace_back(map_.ToCartesian({fmod(car_s + 30, kMaxS), target_d}));
    spline_pts.emplace_back(map_.ToCartesian({fmod(car_s + 60, kMaxS), target_d}));
    spline_pts.emplace_back(map_.ToCartesian({fmod(car_s + 90, kMaxS), target_d}));

    for (auto& pt: spline_pts)
      pt = Rotate(pt, car_yaw_);

    tk::spline spline;
    Call([&spline](const auto& x, const auto& y) {spline.set_points(x, y);},
         SplitCoordinates(spline_pts));

    for (size_t i = pts_.size(); i <= 50; ++i ) {
      const double max_velocity = MaxVelocity(last_frenet_, 25);
      if (IsIntersected() || max_velocity + 0.2 < velocity_)
        velocity_ -= 0.1;
      else if (max_velocity + 0.2 > velocity_)
        velocity_ += 0.1;
      
      const double step = velocity_ / 50;
      auto prev = Rotate(pts_.empty() ? last_pos_ : pts_.back(), car_yaw_);
      
      double x = prev.x + step;
      double y = spline(x);

      while (Distance(prev, {x, y}) > step) {
        x -= step / 100;
        y = spline(x);
      }

      auto next = Rotate({x, y}, -car_yaw_);
      pts_.emplace_back(next);
    }
  }

  void ChooseLane() {
    if (last_frenet_.s < 60 || last_frenet_.s + 60 > kMaxS)
      return;

    if (state_ == State::change) {
      if (Lane(last_frenet_) != target_lane_) return;
      if (fabs(fmod(last_frenet_.d, 4) - 2) > 0.4) return;
      state_ = State::none;
    }

    if (state_ != State::none || ForwardDistance(last_frenet_) > 40) return;
    
    double current_lane = Lane(last_frenet_);
    double max_velocity = MaxVelocity(last_frenet_) + 1;
    double best_lane = current_lane;

    if (current_lane < 2) {
      int l = current_lane + 1;
      if (IsDrivable(l)) {
        double v = MaxVelocity({last_frenet_.s , last_frenet_.d + 4});
        if (v > max_velocity) {
          best_lane = l;
          max_velocity = v;
          state_ = State::change;
        }
      }
    }

    if (current_lane > 0) {
      int l = current_lane - 1;
      if (IsDrivable(l)) {
        double v = MaxVelocity({last_frenet_.s , last_frenet_.d - 4});
        if (v > max_velocity) {
          best_lane = l;
          max_velocity = v;
          state_ = State::change;
        }
      }
    }
      
    target_lane_ = best_lane;
  }

  double MaxVelocity(Frenet frenet, experimental::optional<double> forward = experimental::nullopt) {
    const auto lane = Lane(frenet);
    double velocity = kVelocity;
    
    for (const auto& vehicle: vehicles_) {
      if (Lane(vehicle.frenet) != lane) continue;
      if (frenet.s > vehicle.frenet.s) continue;
      if (forward && (vehicle.frenet.s - frenet.s) > *forward) continue;

      auto v = sqrt(pow(vehicle.vx, 2) + pow(vehicle.vy, 2));
      if (v < velocity) velocity = v;
    }

    return velocity;
  }

  double ForwardDistance(Frenet frenet) {
    const auto lane = Lane(frenet);
    double diff = std::numeric_limits<double>::max();
    
    for (const auto& vehicle: vehicles_) {
      if (Lane(vehicle.frenet) != lane) continue;
      if (frenet.s > vehicle.frenet.s) continue;

      auto new_diff = vehicle.frenet.s - frenet.s;
      if (new_diff < diff) diff = new_diff;
    }

    return diff;
  }

  bool IsDrivable(int lane) {
    for (const auto& vehicle: vehicles_) {
      if (Lane(vehicle.frenet) != lane) continue;

      const auto s = vehicle.frenet.s;
      if (s < current_frenet_.s) {
        if (s + 20 > current_frenet_.s) return false;
        if (s + 40 > last_frenet_.s && Velocity(vehicle) - velocity_ > 5) return false;
      } else if (s < last_frenet_.s + 20) {
        return false;
      }
    }

    return true;
  }

  bool IsIntersected() {
    const auto current_lane = Lane(current_frenet_);
    const auto last_lane = Lane(last_frenet_);
    
    for (const auto& vehicle: vehicles_) {
      const auto lane = Lane(vehicle.frenet);
      if (lane != current_lane && lane != last_lane) continue;

      const auto s = vehicle.frenet.s;
      if (s > current_frenet_.s && s < last_frenet_.s + 20) return true;
    }

    return false;
  }

  int Lane(Frenet frenet) {
    return static_cast<int>(frenet.d / 4);
  }

  double Velocity(SensorData vehicle) {
    return sqrt(pow(vehicle.vx, 2) + pow(vehicle.vy, 2));
  }
  
  static constexpr const double kVelocity = 49.5 * 0.44703924652;
  static constexpr const double kMaxS = 6945.554;

  Map map_;
  State state_ = State::none;
  int target_lane_ = 1;
  double car_yaw_;
  Cartesian current_pos_;
  Frenet current_frenet_;
  Cartesian last_pos_;
  Frenet last_frenet_;
  double velocity_;
  vector<Cartesian> pts_;
  vector<SensorData> vehicles_;
};

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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

  Driver driver(Map(map_waypoints_s, map_waypoints_x, map_waypoints_y));

  h.onMessage([&driver](uWS::WebSocket<uWS::SERVER> ws, char *data,
                        size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          vector<SensorData> vehicles;
          for (const vector<double>& data: j[1]["sensor_fusion"]) {
            assert(data.size() == 7);
            vehicles.emplace_back(SensorData{{data[1], data[2]}, data[3], data[4], {data[5], data[6]}});
          }

          json msgJson;

          auto pts = MergeCoordinates(previous_path_x, previous_path_y);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          driver.UpdateState({car_x, car_y}, car_yaw, move(pts), move(vehicles));
          tie(next_x_vals, next_y_vals) = SplitCoordinates(driver.Points());

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
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
