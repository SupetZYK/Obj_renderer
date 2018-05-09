//
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c), assimp OpenGL sample
// Copyright (c) 2013, Aldebaran Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#if CV_MAJOR_VERSION == 3
#include <opencv2/rgbd.hpp>
namespace cv {
using namespace cv::rgbd;
}
#else
#include <opencv2/objdetect/objdetect.hpp>
#endif

#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer3d.h>

#include <opencv2/highgui/highgui.hpp>


void drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst,
    cv::Point offset, int T) {
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0),
  CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m) {
// NOTE: Original demo recalculated max response for each feature in the TxT
// box around it and chose the display color based on that response. Here
// the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int) templates[m].features.size(); ++i) {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}
// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = cv::makePtr<cv::linemod::Detector>();
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<cv::String> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
}

bool visualize_=false;

int main(int argc, char **argv) {
  // Define the display
  size_t width = 640, height = 480;
  //render parameters
  size_t renderer_n_points=300;
  float render_near=0.1, render_far=2000.0;
  float renderer_angle_step = 15;
  float renderer_radius_min = 500;
  float renderer_radius_max = 1200;
  float renderer_radius_step = 1.1;
  float renderer_focal_length_x=572.41140;
  float renderer_focal_length_y=573.57043;

  // the model name can be specified on the command line.
  std::string file_name;
  std::string file_ext;
  if(argc>=2)
  {
    file_name=std::string(argv[1]);
    file_ext = file_name.substr(file_name.size() - 3, file_name.npos);
  }
  else
  {
    std::cout<<"No meshes!"<<std::endl;
    return -1;
  }
  if(argc>=3)
  {
    std::string vis(argv[2]);
    if(vis=="vis")
    {
      std::cout<<"visualize on!"<<std::endl;
      visualize_=true;
    }
  }
  cv::Rect rect;
  Renderer3d renderer = Renderer3d(file_name);
  renderer.set_parameters(width, height, renderer_focal_length_x, renderer_focal_length_y, width/2,height/2,render_near, render_far);
  cv::Ptr<cv::linemod::Detector> detector_ptr = cv::linemod::getDefaultLINEMOD();
  RendererIterator renderer_iterator = RendererIterator(&renderer, renderer_n_points);
  //set the RendererIterator parameters
  renderer_iterator.angle_min_=-170;
  renderer_iterator.angle_max_=180;
  renderer_iterator.angle_step_ = renderer_angle_step;
  renderer_iterator.radius_ = renderer_radius_min;
  renderer_iterator.radius_min_ = renderer_radius_min;
  renderer_iterator.radius_max_ = renderer_radius_max;
  renderer_iterator.radius_step_ = renderer_radius_step;
  renderer_iterator.absolute_radius_step=false;
  cv::Mat image, depth, mask;
  cv::Matx33d R;
  cv::Vec3d T;
  cv::Matx33f K;
  
  for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
  {
    std::stringstream status;
    status << "Loading images " << (i+1) << "/"
        << renderer_iterator.n_templates();
    std::cout << status.str();

    cv::Rect rect;
    renderer_iterator.render(image, depth, mask, rect);

    R = renderer_iterator.R_obj();
    T = renderer_iterator.T();
    float distance = renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f));
    K = cv::Matx33f(renderer_focal_length_x, 0.0f, float(rect.width)/2.0f, 0.0f, renderer_focal_length_y, float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

    std::vector<cv::Mat> sources(2);
    sources[0] = image;
    sources[1] = depth;
    //std::cout<<depth<<std::endl;
  //#if LINEMOD_VIZ_IMG
    // Display the rendered image
//    if (visualize_)
//    {
//      cv::namedWindow("Rendering");
//      if (!image.empty()) {
//        cv::imshow("Rendering", image);
//        //cv::imshow("mask",mask);
//        cv::waitKey(0);
//      }
//    }
  //#endif

    int template_in = detector_ptr->addTemplate(sources, "object1", mask);
    if (template_in == -1)
    {
      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
      continue;
    }

    // Also store the pose of each template
    // Rs_->push_back(cv::Mat(R));
    // Ts_->push_back(cv::Mat(T));
    // distances_->push_back(distance);
    // Ks_->push_back(cv::Mat(K));

    // Delete the status
    for (size_t j = 0; j < status.str().size(); ++j)
      std::cout << '\b';
  }
  //detector_ptr->writeClasses(file_name + std::string("_templates.yaml"));
    writeLinemod(detector_ptr,file_name + std::string("_templates.yaml"));






//   // Loop over a few views in front of the pattern
//   float xy_lim = 0.5;
//   for (float x = -xy_lim; x < xy_lim; x += 0.1)
//     for (float y = -xy_lim; y < xy_lim; y += 0.1)
//       for (float z = 0.6; z < 0.7; z += 0.1) {
//         cv::Vec3f up(0, z, -y);
//         up = up / norm(up);
//         // Rotate the vector
//         for (float theta = -10; theta < 20; theta += 10) {
//           cv::Vec3f Rvec(x, y, z);
//           Rvec = (theta * CV_PI / 180) * Rvec / norm(Rvec);
//           cv::Matx33f R;
//           cv::Rodrigues(Rvec, R);
//           cv::Vec3f up_rotated = R * up;
//           render.lookAt(0., y, z, up_rotated(0), up_rotated(1), up_rotated(2));
//           cv::Mat img, depth, mask;
//           render.render(img, depth, mask, rect);

//           std::vector<cv::Mat> sources(1);
//           sources[0] = img;
//           //sources[1] = depth;

//           detector_ptr->addTemplate(sources, "object1", mask);

// //          cv::imshow("img", img);
// //          cv::imshow("depth", depth);
// //          cv::imshow("mask", mask);
// //          cv::waitKey(0);
//         }
//       }

//   detector_ptr->writeClasses(file_name + std::string("_templates.yaml"));

//   cv::VideoCapture cap(0);
//   cv::Mat img;
//   int num_modalities = (int) detector_ptr->getModalities().size();
//   cv::namedWindow("result");
//   while (true) {
//     cap >> img;

//     std::vector<cv::Mat> sources(1, img);
//     std::vector<cv::linemod::Match> matches;
//     detector_ptr->match(sources, 93, matches);

//     for (size_t i = 0; i < matches.size(); ++i) {
//       const cv::linemod::Match & match = matches[i];
//       const std::vector<cv::linemod::Template>& templates = detector_ptr->getTemplates(match.class_id,
//           match.template_id);

//       drawResponse(templates, num_modalities, img, cv::Point(match.x, match.y), detector_ptr->getT(0));
//     };
//     cv::imshow("result", img);
//     cv::waitKey(5);
//   }

  return 0;
}
