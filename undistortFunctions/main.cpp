/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"

#include <opencv2/opencv.hpp>
#include <exiv2/exiv2.hpp>
#include <iostream>
#include <glog/logging.h>
#include <filesystem>
#include <gflags/gflags.h>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <fstream>

DEFINE_string(input_path, "D:/ProjectX/project-3d/xsfm-data-converter/data/2025-06-26_07.51.22", "Path to Navvis result directory");

int get_orientation(const std::string &path)
{
  try
  {
    Exiv2::Image::UniquePtr image = Exiv2::ImageFactory::open(path);
    image->readMetadata();
    Exiv2::ExifData &exifData = image->exifData();
    if (exifData.empty())
      return 1;
    Exiv2::ExifKey key("Exif.Image.Orientation");
    auto pos = exifData.findKey(key);
    if (pos == exifData.end())
      return 1;
    return pos->toInt64();
  }
  catch (Exiv2::Error &e)
  {
    std::cerr << "Exiv2 error: " << e.what() << std::endl;
    return 1;
  }
}

cv::Mat apply_orientation(const cv::Mat &src, int orientation)
{
  cv::Mat dst;
  cv::rotate(src, dst, cv::ROTATE_90_COUNTERCLOCKWISE);
  return dst;
}

cv::Mat imread_with_orientation(const std::string &path)
{
  cv::Mat img = cv::imread(path);
  int orientation = get_orientation(path);
  CHECK_EQ(orientation, 6) << "Unsupported orientation: " << orientation << " in " << path << ", only 6 is supported.";
  return apply_orientation(img, orientation);
}

void generate_image_poses(const std::string &input_path, const std::string &output_file, int &count)
{
  count = 0;

  std::string info_dir = input_path + "/info";
  std::ofstream f(output_file);
  f << "camera_id image_name x y z rw rx ry rz\n";

  for (const auto &entry : std::filesystem::directory_iterator(info_dir))
  {
    if (entry.path().extension() == ".json" && entry.path().filename().string().find("-info.json") != std::string::npos)
    {
      count++;

      std::string filename = entry.path().filename().string();
      int frame_num = std::stoi(filename.substr(0, filename.find("-")));
      std::ifstream json_file(entry.path());
      nlohmann::json data;
      json_file >> data;

      for (int cam_idx = 0; cam_idx < 4; cam_idx++)
      {
        std::string cam_key = fmt::format("cam{}", cam_idx);
        if (data.contains(cam_key))
        {
          auto pos = data[cam_key]["position"];
          auto quat = data[cam_key]["quaternion"];
          std::string image_name = fmt::format("{:05d}-cam{}.jpg", frame_num, cam_idx);
          f << fmt::format("{} {} {} {} {} {} {} {} {}\n", cam_idx, image_name, pos[0].get<double>(), pos[1].get<double>(), pos[2].get<double>(), quat[0].get<double>(), quat[1].get<double>(), quat[2].get<double>(), quat[3].get<double>());
        }
      }
    }
  }
  f.close();
  LOG(INFO) << "Generated " << output_file << " with poses data.";
}

int main(int argc, char *argv[])
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_logtostderr = 1;

  if (!std::filesystem::exists(FLAGS_input_path + "/undistorted"))
  {
    LOG(INFO) << "Creating directory: " << FLAGS_input_path + "/undistorted";
    std::filesystem::create_directory(FLAGS_input_path + "/undistorted");
  }

  // Generate image-poses.txt
  int pose_count = 0;
  generate_image_poses(FLAGS_input_path, FLAGS_input_path + "/undistorted/image-poses.txt", pose_count);

  for (int camera_id = 0; camera_id < 4; camera_id++)
  {
    /* --------------------------------------------------------------------*/
    /* Read the parameters of the omnidirectional camera from the TXT file */
    /* --------------------------------------------------------------------*/
    struct ocam_model o; // our ocam_models for the fisheye and catadioptric cameras
    get_ocam_model_from_xml(o, camera_id, (FLAGS_input_path + "/sensor_frame.xml").c_str());

    int i;
    LOG(INFO) << "pol =";
    for (i = 0; i < o.length_pol; i++)
    {
      LOG(INFO) << "\t" << o.pol[i];
    }
    LOG(INFO) << "";
    LOG(INFO) << "invpol =";
    for (i = 0; i < o.length_invpol; i++)
    {
      LOG(INFO) << "\t" << o.invpol[i];
    }
    LOG(INFO) << fmt::format("xc = {}\nyc = {}\nwidth = {}\nheight = {}", o.xc, o.yc, o.width, o.height);

    cv::Mat mapx_persp(o.height, o.width, CV_32FC1);
    cv::Mat mapy_persp(o.height, o.width, CV_32FC1);
    float focal = 900;
    create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, o, focal);
    for (int i = 0; i < pose_count; i++)
    {
      std::string input_filename = fmt::format("{}/cam/{:05d}-cam{}.jpg", FLAGS_input_path, i, camera_id);
      std::string output_filename = fmt::format("{}/undistorted/{:05d}-cam{}.jpg", FLAGS_input_path, i, camera_id);

      CHECK(std::filesystem::exists(input_filename)) << "Input file does not exist: " << input_filename;

      cv::Mat src1 = imread_with_orientation(input_filename);
      if (src1.empty())
        continue;

      cv::Mat dst_persp(src1.size(), CV_8UC3);
      cv::remap(src1, dst_persp, mapx_persp, mapy_persp, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
      cv::imwrite(output_filename, dst_persp);
      LOG(INFO) << "Image " << output_filename << " saved";
    }
  }

  return 0;
}
