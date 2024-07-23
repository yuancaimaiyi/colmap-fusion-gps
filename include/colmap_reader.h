#pragma once

#include <iostream>
#include <string>

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>

class ColmapReader {

public:
  void Read(const std::string& path);
  std::unordered_map<std::string, Sophus::SE3d> m_map_img_names_T_cam_world;
  void WriteCamerasText(std::vector<std::string> params,const std::string& path) const;
  void WriteImagesText(const std::vector<std::string> extrinsic,const std::string& path) const;
  void WritePoints3DText(const std::string& path) const;
private:
  void ReadText(const std::string& path);
  void ReadBinary(const std::string& path);
  void ReadImagesText(const std::string& path);
  void ReadImagesBinary(const std::string& path);
};
