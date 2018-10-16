
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <object_renderer/utils.h>
#include <object_renderer/renderer3d.h>

#include <opencv2/highgui/highgui.hpp>

bool visualize_=false;

int main(int argc, char **argv) {
  // Define the display
  size_t width = 640, height = 480;
  //render parameters
  size_t renderer_n_points=200;
  float render_near=0.1, render_far=2000.0;
  float renderer_angle_step = 10;
  float renderer_radius_min = 400;
  float renderer_radius_max = 400;
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
//  cv::Rect rect;
  Renderer3d renderer = Renderer3d(file_name);
  renderer.set_parameters(width, height, renderer_focal_length_x, renderer_focal_length_y, width/2,height/2,render_near, render_far);

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
  //set ele range
//  renderer_iterator.updir=cv::Vec3f(0.0,1.0,0.0);
//  renderer_iterator.frontdir=cv::Vec3f(1.0,0.0,0.0);
  renderer_iterator.ele_range=90;
  renderer_iterator.longtitude_min=0;
  renderer_iterator.longtitude_max=180;
  renderer_iterator.set_up_right_dir(cv::Vec3f(0.0,1.0,0.0), cv::Vec3f(1.0,0.0,0.0));
  cv::Mat image, depth, mask, image2,depth2,mask2;

//  cv::Matx33d R;
//  cv::Vec3d T;
  cv::Matx33f K;
  for (size_t i = 0; !renderer_iterator.isDone(); ++renderer_iterator)
  {
    if(!renderer_iterator.isValidRange())
      continue;
    std::stringstream status;
    status << "Loading images " << (++i) << "/"
        << renderer_iterator.n_templates();
    std::cout << status.str();
    cv::Rect rect;
    rect.width=width;
    rect.height=height;
    renderer_iterator.render(image, depth, mask, rect);

    cv::Matx44d rt = renderer_iterator.Rt_obj();
    cv::Rect rect2;
//    rect2.width=width;
//    rect2.height=height;

    renderer.setModelRt(cv::Mat(rt));
    renderer.renderDepthOnly(depth2, mask2, rect2);
    renderer.renderImageOnly(image2, rect2);



    float distance = renderer_iterator.D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f));
    K = cv::Matx33f(renderer_focal_length_x, 0.0f, float(rect.width)/2.0f, 0.0f, renderer_focal_length_y, float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

    std::vector<cv::Mat> sources(2);
    sources[0] = image;
    sources[1] = depth;
    //std::cout<<depth<<std::endl;
    //#if LINEMOD_VIZ_IMG
    // Display the rendered image
    if (visualize_)
    {
      cv::namedWindow("Rendering");
      if (!image.empty()) {
        cv::imshow("Rendering", image);
        cv::imshow("mask",mask);
        cv::imshow("reImage",image2);
        cv::imshow("reMask",mask2);
        cv::waitKey(0);
      }
    }
  //#endif
    // Delete the status
    for (size_t j = 0; j < status.str().size(); ++j)
      std::cout << '\b';
  }
  return 0;
}
