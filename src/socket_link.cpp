#include "socket_link.h"
#include <cstdint>
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
  socket_.async_read_some(asio::buffer(data_, 1),
                          [this](std::error_code ec, std::size_t length) {
                            if (!ec) {
                              spdlog::info("Read command ok!");
                              CommandQueue_.push((Command)data_[0]);
                              StartReadCommand();
                            }
                          });
}

void SocketLink::SendStatus(Status status) {

  json json_status = {
      {"kalman_state_x", status.kalman_state_x},
      {"kalman_state_y", status.kalman_state_y},
      {"kalman_state_z", status.kalman_state_z},
      {"drone_battery_level", status.drone_battery_level},
      {"range_front", status.range_front},
      {"range_back", status.range_back},
      {"range_left", status.range_left},
      {"range_right", status.range_right},
      {"state_estimate_yaw", status.yaw},
      {"drone_state", status.drone_state},

  };

  // A \n termination symbol is needed for the asyncio client socket to read a
  // line
  std::string data = json_status.dump() + "\n";

  socket_.async_send(asio::buffer(data.data(), data.size()),
                     [this](std::error_code ec, std::size_t length) {
                       if (ec) {
                          spdlog::error("Could not send state {} ", ec.message());
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
