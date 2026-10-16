// Copyright 2026 John Cameron Furey
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef VELODYNE_POINTCLOUD__TESTS__TEST_UTILS_HPP_
#define VELODYNE_POINTCLOUD__TESTS__TEST_UTILS_HPP_

#include <filesystem>

#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace velodyne_test
{
inline std::filesystem::path packageSharePath()
{
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
  return ament_index_cpp::get_package_share_path("velodyne_pointcloud");
#else
  return ament_index_cpp::get_package_share_directory("velodyne_pointcloud");
#endif
}
}  // namespace velodyne_test

#endif  // VELODYNE_POINTCLOUD__TESTS__TEST_UTILS_HPP_
