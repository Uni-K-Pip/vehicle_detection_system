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

#ifndef VEHICLE_DETECTION__PCD_PLAYLIST_HPP_
#define VEHICLE_DETECTION__PCD_PLAYLIST_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace vehicle_detection
{

// Resolution source for the playlist. Indicates which input parameter
// provided the entries; useful for log messages.
enum class PcdPlaylistSource
{
  kPcdFiles,
  kPcdDirectory,
  kPcdFile,
};

const char * to_string(PcdPlaylistSource source);

struct PcdPlaylistInputs
{
  std::string pcd_file;
  std::vector<std::string> pcd_files;
  std::string pcd_directory;
  std::string pcd_glob;
};

struct PcdPlaylistResolution
{
  bool ok{false};
  std::string reason;
  std::vector<std::string> entries;
  PcdPlaylistSource source{PcdPlaylistSource::kPcdFile};
};

// Resolve a playback list from the PCD-related node parameters.
//
// Resolution order:
//   1. inputs.pcd_files (if non-empty) -> use that ordered list as-is.
//   2. inputs.pcd_directory (if non-empty) -> sorted scan of pcd_glob
//      under that directory.
//   3. inputs.pcd_file (if non-empty) -> single-element list (MVP).
//
// Each entry is verified to exist and not be a directory; if any entry
// is missing the call fails and the offending path is reported in
// `reason`. The returned `entries` keep the original (possibly relative)
// path strings so the caller can log them as the user typed them.
PcdPlaylistResolution resolve_pcd_playlist(const PcdPlaylistInputs & inputs);

// Lightweight `*.pcd`-style wildcard matcher. Supports `*` (any number
// of characters, including zero) and `?` (exactly one character). Used
// by `resolve_pcd_playlist` when expanding `pcd_directory`.
bool wildcard_match(std::string_view pattern, std::string_view text);

}  // namespace vehicle_detection

#endif  // VEHICLE_DETECTION__PCD_PLAYLIST_HPP_
