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

#include "vehicle_detection/pcd_playlist.hpp"

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <system_error>

namespace vehicle_detection
{

const char * to_string(PcdPlaylistSource source)
{
  switch (source) {
    case PcdPlaylistSource::kPcdFiles:
      return "pcd_files";
    case PcdPlaylistSource::kPcdDirectory:
      return "pcd_directory";
    case PcdPlaylistSource::kPcdFile:
      return "pcd_file";
  }
  return "unknown";
}

bool wildcard_match(std::string_view pattern, std::string_view text)
{
  // Iterative two-pointer matcher with backtracking on '*'.
  std::size_t p = 0;
  std::size_t t = 0;
  std::size_t star_p = std::string_view::npos;
  std::size_t star_t = 0;
  while (t < text.size()) {
    if (p < pattern.size() &&
      (pattern[p] == '?' || pattern[p] == text[t]))
    {
      ++p;
      ++t;
    } else if (p < pattern.size() && pattern[p] == '*') {
      star_p = p++;
      star_t = t;
    } else if (star_p != std::string_view::npos) {
      p = star_p + 1;
      t = ++star_t;
    } else {
      return false;
    }
  }
  while (p < pattern.size() && pattern[p] == '*') {
    ++p;
  }
  return p == pattern.size();
}

namespace
{

PcdPlaylistResolution fail(std::string reason)
{
  PcdPlaylistResolution r;
  r.ok = false;
  r.reason = std::move(reason);
  return r;
}

bool ensure_regular_file(const std::string & path, std::string & reason)
{
  std::error_code ec;
  const std::filesystem::path fs_path{path};
  if (!std::filesystem::exists(fs_path, ec) || ec) {
    std::ostringstream oss;
    oss << "PCD entry does not exist: " << path;
    reason = oss.str();
    return false;
  }
  if (std::filesystem::is_directory(fs_path, ec)) {
    std::ostringstream oss;
    oss << "PCD entry is a directory, expected file: " << path;
    reason = oss.str();
    return false;
  }
  return true;
}

}  // namespace

PcdPlaylistResolution resolve_pcd_playlist(const PcdPlaylistInputs & inputs)
{
  // 1. Explicit list takes priority.
  if (!inputs.pcd_files.empty()) {
    PcdPlaylistResolution r;
    r.source = PcdPlaylistSource::kPcdFiles;
    r.entries.reserve(inputs.pcd_files.size());
    for (std::size_t i = 0; i < inputs.pcd_files.size(); ++i) {
      const auto & p = inputs.pcd_files[i];
      if (p.empty()) {
        std::ostringstream oss;
        oss << "pcd_files[" << i << "] is empty";
        return fail(oss.str());
      }
      std::string reason;
      if (!ensure_regular_file(p, reason)) {
        return fail(std::move(reason));
      }
      r.entries.push_back(p);
    }
    r.ok = true;
    return r;
  }

  // 2. Directory + glob.
  if (!inputs.pcd_directory.empty()) {
    if (inputs.pcd_glob.empty()) {
      return fail("pcd_glob must not be empty when pcd_directory is set");
    }
    std::error_code ec;
    const std::filesystem::path dir{inputs.pcd_directory};
    if (!std::filesystem::exists(dir, ec) || ec) {
      std::ostringstream oss;
      oss << "pcd_directory does not exist: " << inputs.pcd_directory;
      return fail(oss.str());
    }
    if (!std::filesystem::is_directory(dir, ec)) {
      std::ostringstream oss;
      oss << "pcd_directory is not a directory: " << inputs.pcd_directory;
      return fail(oss.str());
    }

    PcdPlaylistResolution r;
    r.source = PcdPlaylistSource::kPcdDirectory;
    for (const auto & dirent : std::filesystem::directory_iterator(dir, ec)) {
      if (ec) {
        std::ostringstream oss;
        oss << "failed to scan pcd_directory '"
            << inputs.pcd_directory << "': " << ec.message();
        return fail(oss.str());
      }
      if (!dirent.is_regular_file()) {
        continue;
      }
      const auto filename = dirent.path().filename().string();
      if (!wildcard_match(inputs.pcd_glob, filename)) {
        continue;
      }
      r.entries.push_back(dirent.path().string());
    }
    if (r.entries.empty()) {
      std::ostringstream oss;
      oss << "no files matched pcd_glob '" << inputs.pcd_glob
          << "' under pcd_directory '" << inputs.pcd_directory << "'";
      return fail(oss.str());
    }
    std::sort(r.entries.begin(), r.entries.end());
    r.ok = true;
    return r;
  }

  // 3. Single-file fallback (MVP).
  if (!inputs.pcd_file.empty()) {
    std::string reason;
    if (!ensure_regular_file(inputs.pcd_file, reason)) {
      return fail(std::move(reason));
    }
    PcdPlaylistResolution r;
    r.source = PcdPlaylistSource::kPcdFile;
    r.entries.push_back(inputs.pcd_file);
    r.ok = true;
    return r;
  }

  return fail(
    "no PCD source configured: set pcd_file, pcd_files, or pcd_directory");
}

}  // namespace vehicle_detection
