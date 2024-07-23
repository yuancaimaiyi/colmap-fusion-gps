#include "Camera_Spherical.hpp"
#include "Camera_Pinhole.hpp"

#include "image_io.hpp"
#include "sample.hpp"
#include "numeric.h"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/vectorGraphics/svgDrawer.hpp"
#include "colmap_reader.h"
#include "files_utils.h"

#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

using namespace std;
using namespace svg;


/// Compute a rectilinear camera focal from a given angular desired FoV
double focalFromPinholeHeight
(
  int h,
  double thetaMax = openMVG::D2R(60) // Camera FoV
)
{
  float f = 1.f;
  while ( thetaMax < atan2( h / (2 * f) , 1))
  {
    ++f;
  }
  return f;
}

// Convert spherical panorama to rectilinear images
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string
    s_directory_in,
    s_directory_out,
    ext ="jpg",
    sfm_path;
  int
    image_resolution = 640,
    nb_split = 5;


  // required
  cmd.add( make_option('i', s_directory_in, "input_dir") );
  cmd.add( make_option('o', s_directory_out, "output_dir") );
  // Optional
  cmd.add( make_option('r', image_resolution, "image_resolution") );
  cmd.add( make_option('n', nb_split, "nb_split") );
  cmd.add( make_switch('D', "demo_mode") );
  cmd.add(make_option('e',ext,"image extention"));
  cmd.add(make_option('s',sfm_path,"sfm path"));



  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch (const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_dir] the path where the spherical panoramic images are saved \n"
    << "[-o|--output_dir] the path where output rectilinear image will be saved \n"
    << " OPTIONAL:\n"
    << "[-r|--image_resolution] the rectilinear image size (default:" << image_resolution << ") \n"
    << "[-n|--nb_split] the number of rectilinear image along the X axis (default:" << nb_split << ") \n"
    << "[-D|--demo_mode] switch parameter, export a SVG file that simulate asked rectilinear\n"
    << "  frustum configuration on the spherical image.\n"
    << "[-s|--sfm_dir] the panorama sfm result \n"
    << "[-e|--extension] image extention \n"
    << std::endl;
  }

  // Input parameter checking

  if (image_resolution < 0)
  {
    std::cerr << "image_resolution must be larger than 0" << std::endl;
    return EXIT_FAILURE;
  }
  if (nb_split < 0)
  {
    std::cerr << "nb_split must be larger than 0" << std::endl;
    return EXIT_FAILURE;
  }
  if (s_directory_in.empty() || s_directory_out.empty())
  {
    std::cerr << "input_dir and output_dir option must not be empty" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::is_folder(s_directory_out))
  {
    if (!stlplus::folder_create(s_directory_out))
    {
      std::cerr << "Cannot create the output_dir directory" << std::endl;
      return EXIT_FAILURE;
    }
  }
  //
  // pose
  std::string clip_imgs_path = JoinPaths(s_directory_out,"clip_imgs");
  std::string clip_sparse = JoinPaths(s_directory_out,"clip_sparse");
  CreateDirIfNotExists(clip_sparse);
  CreateDirIfNotExists(clip_imgs_path);
  std::string cameras_txt = JoinPaths(clip_sparse,"cameras.txt");
  std::string images_txt = JoinPaths(clip_sparse,"images.txt");
  std::string points3D_txt = JoinPaths(clip_sparse,"points3D.txt");
  ColmapReader colmap_reader;
  colmap_reader.Read(sfm_path);
  cout << "pose size:" <<colmap_reader.m_map_img_names_T_cam_world.size()<<std::endl;

  // List images from the input directory
  const std::vector<std::string> vec_filenames
    = stlplus::folder_wildcard(s_directory_in, "*."+ext, false, true);

  if (vec_filenames.empty())
  {
    std::cerr << "Did not find any jpg image in the provided input_dir" << std::endl;
    return EXIT_FAILURE;
  }

  using namespace openMVG;

  //-----------------
  //-- Create N rectilinear cameras
  //     (according the number of asked split along the X axis)
  //-- For each camera
  //   -- Forward mapping
  //   -- Save resulting images to disk
  //-----------------


  //-- Simulate a camera with many rotations along the Y axis
//  const int pinhole_width = image_resolution, pinhole_height = image_resolution;
  const int pinhole_width = image_resolution, pinhole_height = image_resolution;
  const double focal = focalFromPinholeHeight(pinhole_height, openMVG::D2R(80.0/2.0));

  // write colmap format instrinsic
  std::stringstream camera1;
  camera1 << "1 PINHOLE " << pinhole_width << " " << pinhole_height <<" "<<focal << " "<<focal<<" "
          << pinhole_width/2. << " "<< pinhole_height/2.;
  std::vector<std::string> params{camera1.str()};
  colmap_reader.WriteCamerasText(params,cameras_txt);
  colmap_reader.WritePoints3DText(points3D_txt);

  const openMVG::cameras::Pinhole_Intrinsic pinhole_camera(
    //w, h, focal, ppx, ppy
    pinhole_width, pinhole_height, focal, pinhole_width/2., pinhole_height/2.);

  const double alpha = (M_PI * 2.0) / static_cast<double>(nb_split); // 360 / split_count
  std::vector<Mat3> camera_rotations;
  for (int i = 0; i < nb_split; ++i)
  {

    camera_rotations.emplace_back(openMVG::RotationAroundY(alpha * i));
  }

  if (cmd.used('D')) // Demo mode:
  {
    // Create a spherical camera:
    const int pano_width = 4096, pano_height = pano_width / 2;
    const openMVG::cameras::Intrinsic_Spherical sphere_camera(pano_width, pano_height);

    svgDrawer svgStream(pano_width, pano_height);
    svgStream.drawLine(0, 0, pano_width, pano_height, svgStyle());
    svgStream.drawLine(pano_width, 0, 0, pano_height, svgStyle());

    //--> For each cam, reproject the image borders onto the panoramic image

    for (const Mat3 & cam_rotation : camera_rotations)
    {
      // Draw the shot border with the givenStep:
      const int step = 10;
      // Store the location of the pinhole bearing vector projection in the spherical image
      Vec2 sphere_proj;

      // Vertical rectilinear image border:
      for (double j = 0; j <= image_resolution; j += image_resolution/(double)step)
      {
        // Project the pinhole bearing vector to the sphere
        sphere_proj = sphere_camera.project(cam_rotation * pinhole_camera(Vec2(0., j)));
        svgStream.drawCircle(sphere_proj.x(), sphere_proj.y(), 4, svgStyle().fill("green"));

        sphere_proj = sphere_camera.project(cam_rotation * pinhole_camera(Vec2(image_resolution, j)));
        svgStream.drawCircle(sphere_proj.x(), sphere_proj.y(), 4, svgStyle().fill("green"));
      }
      // Horizontal rectilinear image border:
      for (double j = 0; j <= image_resolution; j += image_resolution/(double)step)
      {
        sphere_proj = sphere_camera.project(cam_rotation * pinhole_camera(Vec2(j, 0.)));
        svgStream.drawCircle(sphere_proj.x(), sphere_proj.y(), 4, svgStyle().fill("yellow"));

        sphere_proj = sphere_camera.project(cam_rotation * pinhole_camera(Vec2(j, image_resolution)));
        svgStream.drawCircle(sphere_proj.x(), sphere_proj.y(), 4, svgStyle().fill("yellow"));
      }
    }

    std::ofstream svgFile( stlplus::create_filespec(s_directory_out, "test.svg" ));
    svgFile << svgStream.closeSvgFile().str();

    return EXIT_SUCCESS;
  }

  //-- For each input image extract multiple pinhole images
  std::vector<std::string> extrinsic_;  // clip extrinsic
  int image_id = 1;

  for (const std::string & filename_it : vec_filenames)
  {
    image::Image<image::RGBColor> spherical_image;
    if (!ReadImage(stlplus::create_filespec(s_directory_in, filename_it).c_str(), &spherical_image))
    {
      std::cerr << "Cannot read the image: " << stlplus::create_filespec(s_directory_in, filename_it) << std::endl;
      continue;
    }

    const openMVG::cameras::Intrinsic_Spherical sphere_camera(spherical_image.Width(), spherical_image.Height());

    const image::Sampler2d<image::SamplerLinear> sampler;
    image::Image<image::RGBColor> sampled_image(image_resolution, image_resolution, image::BLACK);

    size_t index = 0;
    for (const Mat3 & cam_rotation : camera_rotations)
    {
      sampled_image.fill(image::BLACK);

      // Backward mapping:
      // - Find for each pixels of the pinhole image where it comes from the panoramic image
      for (int j = 0; j < sampled_image.Height(); ++j)
      {
        for (int i = 0; i < sampled_image.Width(); ++i)
        {
          const Vec2 sphere_proj = sphere_camera.project(cam_rotation * pinhole_camera(Vec2(i, j)));
          sampled_image(j, i) = sampler(spherical_image, sphere_proj.y(), sphere_proj.x());
        }

      }
      //-- save image
      const std::string basename = stlplus::basename_part(filename_it);

      std::unordered_map<string,  Sophus::SE3d>::iterator it = colmap_reader.m_map_img_names_T_cam_world.find(filename_it);
      if(it != colmap_reader.m_map_img_names_T_cam_world.end()){

          Sophus::SE3d pose = it->second; // world -> camera
          Sophus::SE3d pose_inv = pose.inverse(); // camera -> world
          Eigen::Matrix3Xd postion = pose_inv.matrix().block(0,3,3,1); // camera pos
          Eigen::Matrix3d rotation = pose.rotationMatrix(); // world to camera rotation
          Eigen::Matrix3d clip_rot = cam_rotation * rotation; // Rwc' = Rtrans * Rwc
          Eigen::Matrix3Xd clip_trans = -1 * clip_rot * postion;
          Eigen::Quaterniond clip_qua(clip_rot);
          std::string clip_img_name = basename + "_" +std::to_string(index) +".jpg";
          std::string extri = std::to_string(image_id) + " " + std::to_string(clip_qua.w()) + " " + std::to_string(clip_qua.x())
                  + " " + std::to_string(clip_qua.y()) + " " + std::to_string(clip_qua.z()) + " " + std::to_string(clip_trans(0,0))
                  + " " + std::to_string(clip_trans(1,0)) + " " + std::to_string(clip_trans(2,0)) + " " + std::to_string(1)
                  + " " + clip_img_name;
          extrinsic_.push_back(extri);
          std::cout << basename << " cam index: " << index << std::endl;
          std::ostringstream os;
          os << clip_imgs_path << "/" << basename << "_" << index << ".jpg";
          WriteImage(os.str().c_str(), sampled_image);
          image_id++;
          ++index;

      }
    }
  }
  colmap_reader.WriteImagesText(extrinsic_,images_txt);


//  std::ofstream fileFocalOut(stlplus::create_filespec(s_directory_out, "focal.txt"));
//  fileFocalOut << focal;
//  fileFocalOut.close();

  return EXIT_SUCCESS;
}
