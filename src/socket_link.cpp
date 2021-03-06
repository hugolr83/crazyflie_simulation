#include "socket_link.h"
#include <cstdint>
#include <iostream>
#include <string>

SocketLink::SocketLink(unsigned int port)
    : acceptor_(context_, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
      socket_(context_) {
  spdlog::info("Init socket");
}

void SocketLink::Start() {
  try {

    WaitClientConnection();
    ThreadContext_ = std::thread([this]() { context_.run(); });

  } catch (std::exception &e) {
    spdlog::error("[SERVER] Exception {}", e.what());
  }
  spdlog::info("[SERVER] Started drone socket");
}

void SocketLink::Stop() {

  context_.stop();

  if (ThreadContext_.joinable()) {
    ThreadContext_.join();
  }

  spdlog::info("[SERVER] Stopped drone socket");
}

void SocketLink::WaitClientConnection() {
  acceptor_.async_accept(
      [this](std::error_code ec, asio::ip::tcp::socket socket) {
        if (!ec) {

          spdlog::info("Client connected ");

          socket_ = std::move(socket);
          StartReadCommand();

        } else {
          spdlog::error("[SERVER] Wait for connection Error {}", ec.message());
        }

        WaitClientConnection();
      });
}

void SocketLink::StartReadCommand() {
  socket_.async_read_some(
      asio::buffer(data_, 100), [this](std::error_code ec, std::size_t length) {
        if (!ec) {

          CommandQueue_.push((Command)data_[0]);

          if ((Command)data_[0] == Command::SET_POSITION) {
            auto confStr = std::string(&data_[sizeof(int)]);

            auto posJson = json::parse(confStr);

            Position position = {posJson["position"]["x"],
                                 posJson["position"]["y"], 0.0};

            Orientation orientation = {posJson["orientation"]["yaw"]};

            DroneInitialConfig initial = {position, orientation};

            ConfigQueue_.push(initial);
          }

          StartReadCommand();
        }
      });
}

void SocketLink::SendStatus(Status status) {

  json json_status = {
      {"timestamp", std::time(nullptr)},
      {"kalman.stateX", status.kalman_state_x},
      {"kalman.stateY", status.kalman_state_y},
      {"kalman.stateZ", status.kalman_state_z},
      {"drone.batteryLevel", status.drone_battery_level},
      {"range.front", status.range_front},
      {"range.back", status.range_back},
      {"range.left", status.range_left},
      {"range.right", status.range_right},
      {"stateEstimate.yaw", status.yaw},
      {"drone.state", status.drone_state},
      {"range.up", 0},
      {"range.zrange", 0},

  };

  // A \n termination symbol is needed for the asyncio client socket to read
  // a line
  std::string data = json_status.dump() + "\n";

  socket_.async_send(asio::buffer(data.data(), data.size()),
                     [this](std::error_code ec, std::size_t length) {
                       if (ec) {
                         //  spdlog::error("Could not send state {} ",
                         //                ec.message());
                       }
                     });
}

Command SocketLink::GetCommand() {
  if (CommandQueue_.empty()) {
    return Command::UNKNOWN;
  }
  Command comm = CommandQueue_.front();
  CommandQueue_.pop();
  return comm;
}

DroneInitialConfig SocketLink::GetInitialConfig() {
  DroneInitialConfig init = ConfigQueue_.front();
  ConfigQueue_.pop();
  return init;
}
