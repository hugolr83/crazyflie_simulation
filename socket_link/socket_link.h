#ifndef SOCKET_LINK_H
#define SOCKET_LINK_H

#define ASIO_STANDALONE
#include "asio.hpp"
#include "command.h"
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <thread>

class SocketLink {

public:
  SocketLink(unsigned int port);
  void Start();
  void Stop();
  void WaitClientConnection();
  void StartReadCommand();
  Command GetCommand();
  std::queue<Command> CommandQueue_;

private:
  asio::error_code ec;
  asio::io_context context_;
  asio::ip::tcp::acceptor acceptor_;
  asio::ip::tcp::socket socket_;
  std::thread ThreadContext_;
  char data_[1024];
};

#endif