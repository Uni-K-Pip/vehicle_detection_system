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

#ifndef VEHICLE_DETECTION__HTTP_CLIENT_HPP_
#define VEHICLE_DETECTION__HTTP_CLIENT_HPP_

#include <chrono>
#include <string>

namespace vehicle_detection
{

struct HttpUrl
{
  std::string host;
  std::string port;
  std::string path;
};

struct HttpPostResult
{
  bool ok;
  int status_code;
  std::string error;
};

bool parse_http_url(const std::string & url, HttpUrl * out);

HttpPostResult http_post_json(
  const HttpUrl & url,
  const std::string & body,
  std::chrono::milliseconds timeout);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__HTTP_CLIENT_HPP_
