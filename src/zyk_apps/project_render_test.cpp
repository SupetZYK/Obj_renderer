#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <object_renderer/utils.h>
#include <object_renderer/renderer3d.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    // Define the display
    size_t width = 640, height = 480;
    //render parameters
//    size_t renderer_n_points=150;
//    float render_near=0.1, render_far=2000.0;
//    float renderer_angle_step = 10;
//    float renderer_radius_min = 0.6;
//    float renderer_radius_max = 1.1;
//    float renderer_radius_step = 0.4;
    float renderer_focal_length_x=572.41140;
    float renderer_focal_length_y=573.57043;
    float px=325.26110,py=242.04899;
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
    cv::Rect rect;
    rect.width=width;
    rect.height=height;
    Renderer3d renderer = Renderer3d(file_name);
    renderer.set_parameters(width, height, renderer_focal_length_x, renderer_focal_length_y, px, py);
    cv::Mat image, depth, mask;

    // cv::Matx33f K;
    // accv lamp 0 rt
//    cv::Mat Rt=(cv::Mat_<float>(4,4) << -0.00145487,-0.993196,0.116445,98.7207,
//                                0.89669, 0.0502498, 0.439798 ,-120.88,
//                              -0.442657, 0.105055, 0.890516, 1087.96,
//                                0,0,0,1);
//    // accv duck 0 rt
//    cv::Mat Rt=(cv::Mat_<float>(4,4) << 0.995554, 0.0246045, -0.0909228, 62.7016,
//                0.032046, 0.819225, 0.572576, 56.1318,
//                0.0885742, -0.572944, 0.814795, 854.07,
//                0,0,0,1);    // accv duck 0 rt
//    cv::Mat Rt=(cv::Mat_<float>(4,4) << 0.995554, 0.0246045, -0.0909228, 0,
//                0.032046, 0.819225, 0.572576, 0,
//                0.0885742, -0.572944, 0.814795, 200.07,
//                0,0,0,1);

    cv::Mat Rt=cv::Mat::eye(4,4,CV_32F);
    Rt.at<float>(2,3)=600;//d
    Rt.at<float>(1,3)=200;//y
    Rt.at<float>(0,3)=100; //x
//    cv::Mat R=(cv::Mat_<float>(3,3) << 1,0,0, 0,1,0,0,0,1);
//    cv::Mat t=(cv::Mat_<float>(3,1)<<98.7207,-120.88,1087.96);
//    cv::Mat t=(cv::Mat_<float>(3,1)<<0,0,400);
    //std::cout<<"dfdfd"<<newmat<<std::endl;
//    cv::Mat r;
//    cv::Rodrigues(R.t(),r);
//    cv::Mat t2=-R.t();
    //renderer.lookAt(100,100,400,0,0,1);
    //renderer.setEyeRt(Rt.inv());
    renderer.setModelRt(Rt);
    renderer.renderDepthOnly(depth);
    renderer.renderImageOnly(image);
    renderer.getMaskFromDepth(depth,mask,rect);
 // already write flip in functions so no need to!
//    cv::flip(image,image,0);
//    cv::flip(mask,mask,0);
    cv::imshow("a",image);
    cv::waitKey(0);
    cv::imshow("b",mask);
    cv::waitKey(0);
     cv::imshow("c",depth);
    cv::waitKey(0);
    return 0;
}
