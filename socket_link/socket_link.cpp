#include "socket_link.h"
#include "spdlog/spdlog.h"

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
                              CommandQueue_.push((Command)data_[0]);
                              StartReadCommand();
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
