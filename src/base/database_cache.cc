// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "base/database_cache.h"

#include <unordered_set>

#include "feature/utils.h"
#include "util/string.h"
#include "util/timer.h"
#include "util/misc.h"
namespace colmap {

DatabaseCache::DatabaseCache() {}

void DatabaseCache::AddCamera(const class Camera& camera) {
  CHECK(!ExistsCamera(camera.CameraId()));
  cameras_.emplace(camera.CameraId(), camera);
}


bool DatabaseCache::ReadKeyPointsDepthValue(const std::string &feature_prior_depth_path, const std::string &name,
                                            std::unordered_map<int, double> &KeyDepth)
{
    if (ExistsFile(JoinPaths(feature_prior_depth_path,name+".txt")))
    {
        std::vector<std::string> speed_string = ReadTextFileLines(JoinPaths(feature_prior_depth_path,name+".txt"));

        for (auto line : speed_string)
       {
            int feature_idx = std::stoi(StringSplit(line,",")[0]);
            double depth =std::stod(StringSplit(line,",")[1]);
//            int depth_confidence =std::stoi(StringSplit(line,",")[2]);
            KeyDepth[feature_idx] = depth;
//            DepthConfidence[feature_idx] = depth_confidence;
        }

        std::cout<<name<<"  use feature  prior depth , feature with depth number: "<<KeyDepth.size()<<std::endl;
        return true;
    }
   else
    {
//        std::cout<<" Do not use feature  prior depth!!!!!!!!!!!!!\n";
        return false;

    }

}
void DatabaseCache::AddImage(const class Image& image) {
  CHECK(!ExistsImage(image.ImageId()));
  images_.emplace(image.ImageId(), image);
  correspondence_graph_.AddImage(image.ImageId(), image.NumPoints2D());
}

void DatabaseCache::Load(const Database& database, const size_t min_num_matches,
                         const bool ignore_watermarks,
                         const std::unordered_set<std::string>& image_names,
                         std::string &speed_path,std::string &feature_prior_depth_path) {
  //////////////////////////////////////////////////////////////////////////////
  // Load cameras
  //////////////////////////////////////////////////////////////////////////////

  Timer timer;

  timer.Start();
  std::cout << "Loading cameras..." << std::flush;

  {
    const std::vector<class Camera> cameras = database.ReadAllCameras();
    cameras_.reserve(cameras.size());
    for (const auto& camera : cameras) {
      cameras_.emplace(camera.CameraId(), camera);
    }
  }

  std::cout << StringPrintf(" %d in %.3fs", cameras_.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  // Load matches
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading matches..." << std::flush;

  std::vector<image_pair_t> image_pair_ids;
  std::vector<TwoViewGeometry> two_view_geometries; // 几何验证模块
  database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

  std::cout << StringPrintf(" %d in %.3fs", image_pair_ids.size(),
                            timer.ElapsedSeconds())
            << std::endl;

  auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
                                   const TwoViewGeometry& two_view_geometry) {
    return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
               min_num_matches &&
           (!ignore_watermarks ||
            two_view_geometry.config != TwoViewGeometry::WATERMARK);
  };
  /////////////////////////////////////////////////////////////////////////
  // 加载速度文件
  ////////////////////////////////////////////////////////////////////////
  std::unordered_map<std::string, double>  speedx_pairs;
  std::unordered_map<std::string, double>  speedy_pairs;
  std::unordered_map<std::string, double>  speedz_pairs;

  if (ExistsFile(JoinPaths(speed_path,"speed.txt")))
  {
      std::vector<std::string> speed_string = ReadTextFileLines(JoinPaths(speed_path,"speed.txt"));

      for (auto line : speed_string)
     {
          std::string s1 = StringSplit(line,",")[0];
          // x 方向速度,y方向速度,z方向速度
          speedx_pairs[s1.c_str()] = std::stod(StringSplit(line,",")[1]);
          speedy_pairs[s1.c_str()] = std::stod(StringSplit(line,",")[2]);
          speedz_pairs[s1.c_str()] = std::stod(StringSplit(line,",")[3]);
      }
      std::cout<<"use speed to update time,speed number: "<<speedx_pairs.size()<<std::endl;

  }
 else
  {
      std::cout<<" Do not use speed to update time !!!!!!!!!!!!!\n";
  }

  //////////////////////////////////////////////////////////////////////////////
  // Load images
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Loading images..." << std::flush;

  std::unordered_set<image_t> image_ids;

  {
    const std::vector<class Image> images = database.ReadAllImages(); // 所有图像

    // Determines for which images data should be loaded.
    if (image_names.empty()) {
      for (const auto& image : images) {
        image_ids.insert(image.ImageId());
      }
    } else {
      for (const auto& image : images) {
        if (image_names.count(image.Name()) > 0) {
          image_ids.insert(image.ImageId());
        }
      }
    }

    // Collect all images that are connected in the correspondence graph.
    std::unordered_set<image_t> connected_image_ids;
    connected_image_ids.reserve(image_ids.size());
    for (size_t i = 0; i < image_pair_ids.size(); ++i) {
      if (UseInlierMatchesCheck(two_view_geometries[i])) {
        image_t image_id1;
        image_t image_id2;
        Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
        if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
          connected_image_ids.insert(image_id1);
          connected_image_ids.insert(image_id2);
        }
      }
    }

    // Load images with correspondences and discard images without
    // correspondences, as those images are useless for SfM.
   // 加载匹配点和丢弃那些没有匹配点的图像,因为没有匹配点的图像对sfm 是没有用
    // 添加feature scale
    images_.reserve(connected_image_ids.size()); // 新的images,丢弃没有匹配点的帧
    for (const auto& image : images) {
      if (image_ids.count(image.ImageId()) > 0 &&
          connected_image_ids.count(image.ImageId()) > 0) {
        images_.emplace(image.ImageId(), image);

        const FeatureKeypoints keypoints =
            database.ReadKeypoints(image.ImageId());
        const std::vector<Eigen::Vector2d> points =
            FeatureKeypointsToPointsVector(keypoints);
        const std::vector<Eigen::Vector2d> scales =
                FeatureKeypointsToFeatureScale(keypoints);
        CHECK(points.size()==scales.size());


        // 设置观测值的位置信息
        images_[image.ImageId()].SetPoints2D(points);

        std::unordered_map<int, double> KeyDepth;
        std::unordered_map<int, int> DepthConfidence; // 暂时不用confidence
        if (ReadKeyPointsDepthValue(feature_prior_depth_path,image.Name(),KeyDepth))
        {
            // 设置观测值的深度信息(如激光测高数据,或者tof获取的)
            images_[image.ImageId()].SetPointsDepth(points,KeyDepth);
        }
        // 设置观测值的尺度信息
        images_[image.ImageId()].SetPointsScale(scales);
        // 得到影像对应的速度,每一张影像都有速度
        if ( speedx_pairs.size() == images.size() )
        {
            const double speedx_value = speedx_pairs[image.Name()];
            const double speedy_value = speedy_pairs[image.Name()];
            const double speedz_value = speedz_pairs[image.Name()];
            images_[image.ImageId()].SetSpeedPrior(Eigen::Vector3d(speedx_value,speedy_value,speedz_value));
        }

      }
    }

    std::cout << StringPrintf(" %d in %.3fs (connected %d)", images.size(),
                              timer.ElapsedSeconds(),
                              connected_image_ids.size())
              << std::endl;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Build correspondence graph
  //////////////////////////////////////////////////////////////////////////////

  timer.Restart();
  std::cout << "Building correspondence graph..." << std::flush;

  for (const auto& image : images_) {
    correspondence_graph_.AddImage(image.first, image.second.NumPoints2D());
  }

 //  correspondece 修改
  size_t num_ignored_image_pairs = 0;
  for (size_t i = 0; i < image_pair_ids.size(); ++i) {
    if (UseInlierMatchesCheck(two_view_geometries[i])) {
      image_t image_id1;
      image_t image_id2;
      Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
      if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
        correspondence_graph_.AddCorrespondences(
            image_id1, image_id2, two_view_geometries[i].inlier_matches);
      } else {
        num_ignored_image_pairs += 1;
      }
    } else {
      num_ignored_image_pairs += 1;
    }
  }

  correspondence_graph_.Finalize();

  // Set number of observations and correspondences per image.
  for (auto& image : images_) {
    image.second.SetNumObservations(
        correspondence_graph_.NumObservationsForImage(image.first));
    image.second.SetNumCorrespondences(
        correspondence_graph_.NumCorrespondencesForImage(image.first));
  }

  std::cout << StringPrintf(" in %.3fs (ignored %d)", timer.ElapsedSeconds(),
                            num_ignored_image_pairs)
            << std::endl;
}

const class Image* DatabaseCache::FindImageWithName(
    const std::string& name) const {
  for (const auto& image : images_) {
    if (image.second.Name() == name) {
      return &image.second;
    }
  }
  return nullptr;
}

}  // namespace colmap
