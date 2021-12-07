#ifndef SOCKET_LINK_H
#define SOCKET_LINK_H

#define ASIO_STANDALONE
#include "asio.hpp"
#include "command.h"
#include "json.hpp"
#include "state.h"
#include <array>
#include <memory>
#include <queue>
#include <spdlog/spdlog.h>
#include <string>
#include <thread>

using json = nlohmann::json;

struct Position {
  double x;
  double y;
  double z;
};

struct Orientation {
  double yaw;
};

struct DroneInitialConfig {
  Position position;
  Orientation orientation;
};

struct Status {
  double kalman_state_x;
  double kalman_state_y;
  double kalman_state_z;
  int drone_battery_level;
  double range_front;
  double range_back;
  double range_left;
  double range_right;
  double yaw;
  int drone_state;
};

class SocketLink {

public:
  SocketLink(unsigned int port);
  void Start();
  void Stop();
  void WaitClientConnection();
  void StartReadCommand();
  Command GetCommand();
  DroneInitialConfig GetInitialConfig();
  void SendStatus(Status status);
  std::queue<Command> CommandQueue_;
  std::queue<DroneInitialConfig> ConfigQueue_;

private:
  asio::error_code ec;
  asio::io_context context_;
  asio::ip::tcp::acceptor acceptor_;
  asio::ip::tcp::socket socket_;
  std::thread ThreadContext_;
  char data_[1024];
  void SerializeStatus(Status status);
  Status DeserializeStatus(std::string json_status);
};

#endif