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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "vehicle_detection/pcd_playlist.hpp"

using vehicle_detection::PcdPlaylistInputs;
using vehicle_detection::PcdPlaylistSource;
using vehicle_detection::resolve_pcd_playlist;
using vehicle_detection::wildcard_match;

namespace
{

class TempDir
{
public:
  TempDir()
  {
    std::random_device rd;
    std::mt19937_64 gen{rd()};
    std::uniform_int_distribution<std::uint64_t> dist;
    std::ostringstream oss;
    oss << "vehicle_detection_pcd_playlist_test_" << dist(gen);
    path_ = std::filesystem::temp_directory_path() / oss.str();
    std::filesystem::create_directories(path_);
  }

  ~TempDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;

  std::filesystem::path touch(const std::string & name) const
  {
    const auto p = path_ / name;
    std::ofstream(p) << "stub";
    return p;
  }

  const std::filesystem::path & path() const {return path_;}

private:
  std::filesystem::path path_;
};

}  // namespace

TEST(WildcardMatch, MatchesAllPcdSuffix)
{
  EXPECT_TRUE(wildcard_match("*.pcd", "frame_000.pcd"));
  EXPECT_TRUE(wildcard_match("*.pcd", ".pcd"));
  EXPECT_FALSE(wildcard_match("*.pcd", "frame.bin"));
  EXPECT_FALSE(wildcard_match("*.pcd", "frame.pcd.bak"));
}

TEST(WildcardMatch, QuestionMarkMatchesSingleChar)
{
  EXPECT_TRUE(wildcard_match("frame_???.pcd", "frame_000.pcd"));
  EXPECT_FALSE(wildcard_match("frame_???.pcd", "frame_0000.pcd"));
  EXPECT_FALSE(wildcard_match("frame_???.pcd", "frame_00.pcd"));
}

TEST(WildcardMatch, ExactMatchWithoutWildcards)
{
  EXPECT_TRUE(wildcard_match("sample.pcd", "sample.pcd"));
  EXPECT_FALSE(wildcard_match("sample.pcd", "Sample.pcd"));
}

TEST(WildcardMatch, AllStarsAcceptEmpty)
{
  EXPECT_TRUE(wildcard_match("*", ""));
  EXPECT_TRUE(wildcard_match("**", ""));
  EXPECT_TRUE(wildcard_match("*", "anything"));
}

TEST(ResolvePcdPlaylist, FailsWhenAllInputsEmpty)
{
  PcdPlaylistInputs inputs;
  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("no PCD source"), std::string::npos);
}

TEST(ResolvePcdPlaylist, SingleFileFallback)
{
  TempDir tmp;
  const auto file = tmp.touch("sample.pcd");

  PcdPlaylistInputs inputs;
  inputs.pcd_file = file.string();

  const auto r = resolve_pcd_playlist(inputs);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.source, PcdPlaylistSource::kPcdFile);
  ASSERT_EQ(r.entries.size(), 1u);
  EXPECT_EQ(r.entries[0], file.string());
}

TEST(ResolvePcdPlaylist, RejectsMissingSingleFile)
{
  PcdPlaylistInputs inputs;
  inputs.pcd_file = "/this/path/should/not/exist/sample.pcd";

  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("does not exist"), std::string::npos);
}

TEST(ResolvePcdPlaylist, ExplicitListPreservesOrder)
{
  TempDir tmp;
  const auto a = tmp.touch("c.pcd");
  const auto b = tmp.touch("a.pcd");
  const auto c = tmp.touch("b.pcd");

  PcdPlaylistInputs inputs;
  inputs.pcd_files = {a.string(), b.string(), c.string()};

  const auto r = resolve_pcd_playlist(inputs);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.source, PcdPlaylistSource::kPcdFiles);
  ASSERT_EQ(r.entries.size(), 3u);
  EXPECT_EQ(r.entries[0], a.string());
  EXPECT_EQ(r.entries[1], b.string());
  EXPECT_EQ(r.entries[2], c.string());
}

TEST(ResolvePcdPlaylist, ExplicitListWinsOverDirectoryAndSingleFile)
{
  TempDir tmp;
  const auto in_list = tmp.touch("listed.pcd");
  tmp.touch("ignored_in_dir.pcd");
  const auto fallback = tmp.touch("fallback.pcd");

  PcdPlaylistInputs inputs;
  inputs.pcd_files = {in_list.string()};
  inputs.pcd_directory = tmp.path().string();
  inputs.pcd_glob = "*.pcd";
  inputs.pcd_file = fallback.string();

  const auto r = resolve_pcd_playlist(inputs);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.source, PcdPlaylistSource::kPcdFiles);
  ASSERT_EQ(r.entries.size(), 1u);
  EXPECT_EQ(r.entries[0], in_list.string());
}

TEST(ResolvePcdPlaylist, ExplicitListRejectsMissingEntry)
{
  TempDir tmp;
  const auto a = tmp.touch("a.pcd");

  PcdPlaylistInputs inputs;
  inputs.pcd_files = {a.string(), "/nope/missing.pcd"};

  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("missing.pcd"), std::string::npos);
}

TEST(ResolvePcdPlaylist, ExplicitListRejectsEmptyEntry)
{
  PcdPlaylistInputs inputs;
  inputs.pcd_files = {"", "/tmp/x.pcd"};

  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("pcd_files[0]"), std::string::npos);
}

TEST(ResolvePcdPlaylist, DirectoryGlobSortsLexicographically)
{
  TempDir tmp;
  const auto c = tmp.touch("c.pcd");
  const auto a = tmp.touch("a.pcd");
  const auto b = tmp.touch("b.pcd");
  tmp.touch("notes.txt");

  PcdPlaylistInputs inputs;
  inputs.pcd_directory = tmp.path().string();
  inputs.pcd_glob = "*.pcd";

  const auto r = resolve_pcd_playlist(inputs);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.source, PcdPlaylistSource::kPcdDirectory);
  ASSERT_EQ(r.entries.size(), 3u);
  EXPECT_EQ(r.entries[0], a.string());
  EXPECT_EQ(r.entries[1], b.string());
  EXPECT_EQ(r.entries[2], c.string());
}

TEST(ResolvePcdPlaylist, DirectoryGlobAppliesPattern)
{
  TempDir tmp;
  tmp.touch("frame_000.pcd");
  tmp.touch("frame_001.pcd");
  tmp.touch("metadata.txt");
  tmp.touch("preview.bin");

  PcdPlaylistInputs inputs;
  inputs.pcd_directory = tmp.path().string();
  inputs.pcd_glob = "frame_*.pcd";

  const auto r = resolve_pcd_playlist(inputs);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.entries.size(), 2u);
  for (const auto & e : r.entries) {
    EXPECT_NE(e.find("frame_"), std::string::npos);
  }
}

TEST(ResolvePcdPlaylist, DirectoryWithoutMatchesFails)
{
  TempDir tmp;
  tmp.touch("only.txt");

  PcdPlaylistInputs inputs;
  inputs.pcd_directory = tmp.path().string();
  inputs.pcd_glob = "*.pcd";

  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("no files matched"), std::string::npos);
}

TEST(ResolvePcdPlaylist, MissingDirectoryFails)
{
  PcdPlaylistInputs inputs;
  inputs.pcd_directory = "/this/directory/should/not/exist";
  inputs.pcd_glob = "*.pcd";

  const auto r = resolve_pcd_playlist(inputs);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.reason.find("pcd_directory"), std::string::npos);
}
