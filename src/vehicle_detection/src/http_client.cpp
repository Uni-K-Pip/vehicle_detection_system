// Copyright 2026 kohei
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "vehicle_detection/http_client.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace vehicle_detection
{

namespace
{

class ScopedFd
{
public:
  explicit ScopedFd(int fd)
  : fd_(fd) {}
  ~ScopedFd() {if (fd_ >= 0) {::close(fd_);}}
  ScopedFd(const ScopedFd &) = delete;
  ScopedFd & operator=(const ScopedFd &) = delete;
  int get() const {return fd_;}
  int release() {const int r = fd_; fd_ = -1; return r;}

private:
  int fd_;
};

int connect_with_timeout(
  const struct addrinfo * ai,
  std::chrono::milliseconds timeout,
  std::string * error)
{
  int sock = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
  if (sock < 0) {
    *error = std::string("socket(): ") + std::strerror(errno);
    return -1;
  }

  int flags = ::fcntl(sock, F_GETFL, 0);
  if (flags < 0 || ::fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
    *error = std::string("fcntl(O_NONBLOCK): ") + std::strerror(errno);
    ::close(sock);
    return -1;
  }

  if (::connect(sock, ai->ai_addr, ai->ai_addrlen) < 0 && errno != EINPROGRESS) {
    *error = std::string("connect(): ") + std::strerror(errno);
    ::close(sock);
    return -1;
  }

  fd_set wfds;
  FD_ZERO(&wfds);
  FD_SET(sock, &wfds);
  struct timeval tv;
  tv.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
  const int sel = ::select(sock + 1, nullptr, &wfds, nullptr, &tv);
  if (sel <= 0) {
    *error = (sel == 0) ? "connect timed out" :
      std::string("select(): ") + std::strerror(errno);
    ::close(sock);
    return -1;
  }
  int so_error = 0;
  socklen_t len = sizeof(so_error);
  if (::getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0 ||
    so_error != 0)
  {
    *error = std::string("connect(): ") + std::strerror(so_error);
    ::close(sock);
    return -1;
  }

  if (::fcntl(sock, F_SETFL, flags) < 0) {
    *error = std::string("fcntl(restore): ") + std::strerror(errno);
    ::close(sock);
    return -1;
  }
  struct timeval rwt;
  rwt.tv_sec = static_cast<time_t>(timeout.count() / 1000);
  rwt.tv_usec = static_cast<suseconds_t>((timeout.count() % 1000) * 1000);
  ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &rwt, sizeof(rwt));
  ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rwt, sizeof(rwt));
  return sock;
}

bool send_all(int fd, const char * data, std::size_t size, std::string * error)
{
  // MSG_NOSIGNAL prevents SIGPIPE if the receiver closes the connection
  // mid-write. Without it, a closed peer would terminate the entire
  // detection_sender_node process instead of returning a failed
  // HttpPostResult.
  std::size_t sent = 0;
  while (sent < size) {
    const ssize_t n = ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      *error = std::string("send(): ") + std::strerror(errno);
      return false;
    }
    if (n == 0) {
      *error = "send(): peer closed connection";
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

int parse_status_line(const std::string & buffer)
{
  const auto eol = buffer.find("\r\n");
  if (eol == std::string::npos) {
    return -1;
  }
  const std::string line = buffer.substr(0, eol);
  if (line.rfind("HTTP/", 0) != 0) {
    return -1;
  }
  const auto first_space = line.find(' ');
  if (first_space == std::string::npos) {
    return -1;
  }
  const auto second_space = line.find(' ', first_space + 1);
  const std::string code = (second_space == std::string::npos) ?
    line.substr(first_space + 1) :
    line.substr(first_space + 1, second_space - first_space - 1);
  try {
    return std::stoi(code);
  } catch (...) {
    return -1;
  }
}

}  // namespace

bool parse_http_url(const std::string & url, HttpUrl * out)
{
  static const std::string kScheme = "http://";
  if (url.rfind(kScheme, 0) != 0) {
    return false;
  }
  const std::string rest = url.substr(kScheme.size());
  if (rest.empty()) {
    return false;
  }
  const auto slash = rest.find('/');
  std::string authority;
  std::string path;
  if (slash == std::string::npos) {
    authority = rest;
    path = "/";
  } else {
    authority = rest.substr(0, slash);
    path = rest.substr(slash);
  }
  if (authority.empty()) {
    return false;
  }
  std::string host;
  std::string port = "80";
  if (!authority.empty() && authority.front() == '[') {
    const auto end = authority.find(']');
    if (end == std::string::npos) {
      return false;
    }
    host = authority.substr(1, end - 1);
    if (end + 1 < authority.size()) {
      if (authority[end + 1] != ':') {
        return false;
      }
      port = authority.substr(end + 2);
    }
  } else {
    const auto colon = authority.rfind(':');
    if (colon == std::string::npos) {
      host = authority;
    } else {
      host = authority.substr(0, colon);
      port = authority.substr(colon + 1);
    }
  }
  if (host.empty() || port.empty()) {
    return false;
  }
  out->host = host;
  out->port = port;
  out->path = path;
  return true;
}

HttpPostResult http_post_json(
  const HttpUrl & url,
  const std::string & body,
  std::chrono::milliseconds timeout)
{
  HttpPostResult result{false, -1, {}};

  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * ai_list = nullptr;
  const int gai = ::getaddrinfo(url.host.c_str(), url.port.c_str(),
      &hints, &ai_list);
  if (gai != 0) {
    result.error = std::string("getaddrinfo(): ") + ::gai_strerror(gai);
    return result;
  }
  std::string last_error;
  int sock = -1;
  for (struct addrinfo * ai = ai_list; ai; ai = ai->ai_next) {
    sock = connect_with_timeout(ai, timeout, &last_error);
    if (sock >= 0) {
      break;
    }
  }
  ::freeaddrinfo(ai_list);
  if (sock < 0) {
    result.error = last_error.empty() ? "connect failed" : last_error;
    return result;
  }
  ScopedFd guard(sock);

  std::ostringstream req;
  req << "POST " << url.path << " HTTP/1.1\r\n"
      << "Host: " << url.host;
  if (url.port != "80") {
    req << ":" << url.port;
  }
  req << "\r\n"
      << "User-Agent: vehicle_detection/1.0.0\r\n"
      << "Content-Type: application/json\r\n"
      << "Content-Length: " << body.size() << "\r\n"
      << "Connection: close\r\n"
      << "Accept: */*\r\n\r\n"
      << body;
  const std::string req_str = req.str();

  std::string err;
  if (!send_all(guard.get(), req_str.data(), req_str.size(), &err)) {
    result.error = err;
    return result;
  }

  std::string response;
  char buf[1024];
  while (true) {
    const ssize_t n = ::recv(guard.get(), buf, sizeof(buf), 0);
    if (n > 0) {
      response.append(buf, static_cast<std::size_t>(n));
      if (response.size() > 64 * 1024) {
        break;
      }
      continue;
    }
    if (n == 0) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    result.error = std::string("recv(): ") + std::strerror(errno);
    return result;
  }

  result.status_code = parse_status_line(response);
  if (result.status_code < 0) {
    result.error = "invalid HTTP response";
    return result;
  }
  result.ok = result.status_code >= 200 && result.status_code < 300;
  if (!result.ok) {
    result.error = "non-2xx response";
  }
  return result;
}

}  // namespace vehicle_detection
