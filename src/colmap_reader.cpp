#include "colmap_reader.h"

#include "files_utils.h"

void ColmapReader::Read(const std::string& path) {
  if (ExistsFile(JoinPaths(path, "images.bin"))) {
    ReadBinary(path);
  } else if (ExistsFile(JoinPaths(path, "images.txt"))) {
    ReadText(path);
  } else {
    std::cerr << "images files do not exist at sparse model " << path;
  }
}

void ColmapReader::ReadText(const std::string& path) {
  ReadImagesText(JoinPaths(path, "images.txt"));
}

void ColmapReader::ReadBinary(const std::string& path) {
  ReadImagesBinary(JoinPaths(path, "images.bin"));
}

void ColmapReader::ReadImagesText(const std::string& path) {
  // images_.clear();

  std::ifstream file(path);
  // CHECK(file.is_open()) << path;

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    // image.Qvec(0) = std::stold(item);
    const double qw = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(1) = std::stold(item);
    const double qx = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(2) = std::stold(item);
    const double qy = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Qvec(3) = std::stold(item);
    const double qz = std::stold(item);

    Eigen::Quaterniond qcw(qw, qx, qy, qz);
    qcw.normalize();

    // image.NormalizeQvec();

    // TVEC
    std::getline(line_stream1, item, ' ');
    // image.Tvec(0) = std::stold(item);
    const double tx = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Tvec(1) = std::stold(item);
    const double ty = std::stold(item);

    std::getline(line_stream1, item, ' ');
    // image.Tvec(2) = std::stold(item);
    const double tz = std::stold(item);

    const Eigen::Vector3d tcw(tx, ty, tz);

    // Transformation from world frame to cam frame
    const Sophus::SE3d T_cam_world(qcw, tcw);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    // image.SetCameraId(std::stoul(item));

    // NAME
    std::getline(line_stream1, item, ' ');
    // image.SetName(item);

    // We store the inverse transformation
    m_map_img_names_T_cam_world.emplace(item, T_cam_world);

    // // POINTS2D
    if (!std::getline(file, line)) {
      break;
    }
  }
}


void ColmapReader::ReadImagesBinary(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  // CHECK(file.is_open()) << path;

  const size_t num_reg_images = ReadBinaryLittleEndian<uint64_t>(&file);
  for (size_t i = 0; i < num_reg_images; ++i) {

    ReadBinaryLittleEndian<uint32_t>(&file);
    const double qw = ReadBinaryLittleEndian<double>(&file);
    const double qx = ReadBinaryLittleEndian<double>(&file);
    const double qy = ReadBinaryLittleEndian<double>(&file);
    const double qz = ReadBinaryLittleEndian<double>(&file);

    Eigen::Quaterniond qcw(qw, qx, qy, qz);
    qcw.normalize();

    const double tx = ReadBinaryLittleEndian<double>(&file);
    const double ty = ReadBinaryLittleEndian<double>(&file);
    const double tz = ReadBinaryLittleEndian<double>(&file);

    const Eigen::Vector3d tcw(tx, ty, tz);

    const Sophus::SE3d T_cam_world(qcw, tcw);
    ReadBinaryLittleEndian<uint32_t>(&file);

    std::string img_name;

    char name_char;
    do {
      file.read(&name_char, 1);
      if (name_char != '\0') {
        img_name += name_char;
      }
    } while (name_char != '\0');

    m_map_img_names_T_cam_world.emplace(img_name, T_cam_world);

    const size_t num_points2D = ReadBinaryLittleEndian<uint64_t>(&file);

    for (size_t j = 0; j < num_points2D; ++j) {
      ReadBinaryLittleEndian<double>(&file);
      ReadBinaryLittleEndian<double>(&file);
      ReadBinaryLittleEndian<uint64_t>(&file);
    }

  }
}

void ColmapReader::WriteCamerasText(std::vector<std::string> params,const std::string& path) const {
  std::ofstream file(path, std::ios::trunc);
  for(size_t i =0 ; i<params.size();++i)
  {
     file << params[i] <<std::endl;
  }
}
void ColmapReader::WritePoints3DText(const std::string& path) const {
  std::ofstream file(path, std::ios::trunc);
  file <<"";
}
void ColmapReader::WriteImagesText(std::vector<std::string> extrinsic,const std::string& path) const {
  std::ofstream file(path, std::ios::trunc);
  for(size_t i =0 ;i< extrinsic.size();++i){

    file << extrinsic[i] << std::endl;
    file << std::endl;

  }
}
