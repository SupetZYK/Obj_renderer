/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Vincent Rabaud
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <object_recognition_renderer/renderer3d.h>

#include <iostream>
#include <stdlib.h>

#include <GL/gl.h>

#include "model.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <opencv2/highgui.hpp>
#if USE_GLUT
#include "renderer3d_impl_glut.h"
#else
#include "renderer3d_impl_osmesa.h"
#endif

#include <opencv2/calib3d.hpp>
Renderer3d::Renderer3d(const std::string & mesh_path)
    :
      renderer_(new Renderer3dImpl(mesh_path, 0, 0)),
      angle_(0),
      focal_length_x_(0),
      focal_length_y_(0),
      near_(0),
      far_(0),
      model_(new Model()),
      scene_list_(0)
{
  // get a handle to the predefined STDOUT log stream and attach
  // it to the logging system. It remains active for all further
  // calls to aiImportFile(Ex) and aiApplyPostProcessing.
  ai_stream_ = new aiLogStream(aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL));
  aiAttachLogStream(ai_stream_);
    model_->LoadModel(renderer_->mesh_path_);
}

Renderer3d::~Renderer3d()
{
  // We added a log stream to the library, it's our job to disable it
  // again. This will definitely release the last resources allocated
  // by Assimp.
  aiDetachAllLogStreams();
}

void
Renderer3d::set_parameters(size_t width, size_t height, double focal_length_x, double focal_length_y, double px, double py, double near,
                         double far)
{
  renderer_->width_ = width;
  renderer_->height_ = height;

  focal_length_x_ = focal_length_x;
  focal_length_y_ = focal_length_y;

  near_ = near;
  far_ = far;

  renderer_->clean_buffers();

  // Initialize the OpenGL context
  renderer_->set_parameters_low_level();

//{// initial render param
//  // Initialize the environment
//  glClearColor(0.f, 0.f, 0.f, 1.f);
//  GLfloat LightAmbient[]= { 0.3f, 0.3f, 0.3f, 1.0f };
//  GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
//  GLfloat LightPosition[]= { 0.0f, 0.0f, -100.0f, 0.0f };
//    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
//    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
//    glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
//    glEnable(GL_LIGHTING);
//    //glEnable(GL_LIGHT1);
//    glEnable(GL_LIGHT0); // Uses default lighting parameters

//    glEnable(GL_NORMALIZE);

//    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,LightAmbient);
//    glEnable(GL_COLOR_MATERIAL);
//    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);

//    glDepthFunc(GL_LEQUAL);
//  glEnable(GL_DEPTH_TEST);
// }

  {// param borrow from simple_opengl
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glPolygonOffset(1.0, 1.0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);  // try this!
    glEnable(GL_NORMALIZE);

    /* XXX docs say all polygons are emitted CCW, but tests show that some aren't. */
    if(getenv("MODEL_IS_BROKEN"))
        glFrontFace(GL_CW);
    /**light setting**/
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);    /* Uses default lighting parameters */
    GLfloat LightAmbient[]= { 0.3f, 0.3f, 0.3f, 1.0f };
    GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat LightPosition[]= { 0.0f, 0.0f, -100.0f, 0.0f };
    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
    glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
    glEnable(GL_LIGHT1);
    /**material setting**/
//    glLightModeli(GL_LIGHT_MODEL_AMBIENT, GL_TRUE);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,LightAmbient);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
  }

  // Initialize the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();


  // method 1, just create symmetrical view
//  double fx = Renderer3d::focal_length_x_;
//  double fy = Renderer3d::focal_length_y_;
//  double fovy = 2 * atan(0.5 * renderer_->height_ / fy) * 180 / CV_PI;
//  double aspect = (renderer_->width_ * fy) / (renderer_->height_ * fx);
//  // set perspective
//  gluPerspective(fovy, aspect, near, far);
  //method 2
  glFrustum(-px/focal_length_x*near, (width-px)/focal_length_x*near, (py-height)/focal_length_y*near, py/focal_length_y*near, near, far);

  glViewport(0, 0, renderer_->width_, renderer_->height_);
}

void
Renderer3d::lookAt(double x, double y, double z, double upx, double upy, double upz)
{
  renderer_->bind_buffers();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(x, y, z, 0, 0, 0, upx, upy, upz);

  //2018-2-24 zyk change: no need to center the model, making parameters absolute.
  /*{
      // scale the whole asset to fit into our view frustum
      aiVector3D scene_min, scene_max, scene_center;
      model_->get_bounding_box(&scene_min, &scene_max);
      scene_center.x = (scene_min.x + scene_max.x) / 2.0f;
      scene_center.y = (scene_min.y + scene_max.y) / 2.0f;
      scene_center.z = (scene_min.z + scene_max.z) / 2.0f;

      // center the model
      glTranslatef(-scene_center.x, -scene_center.y, -scene_center.z);
  }*/


  // if the display list has not been made yet, create a new one and
  // fill it with scene contents
  if (scene_list_ == 0)
  {
    scene_list_ = glGenLists(1);
    glNewList(scene_list_, GL_COMPILE);
    // now begin at the root node of the imported data and traverse
    // the scenegraph by multiplying subsequent local transforms
    // together on GL's matrix stack.
    model_->Draw();
    glEndList();
  }

  glCallList(scene_list_);
}
//void
//Renderer3d::setEyeRt(cv::Matx44d Rt)
//{
//    renderer_->bind_buffers();

//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//    GLdouble tx=Rt(0,3);
//    GLdouble ty=Rt(1,3);
//    GLdouble tz=Rt(2,3);
//    GLdouble lx=tx+Rt(0,2)*100;
//    GLdouble ly=ty+Rt(1,2)*100;
//    GLdouble lz=tz+Rt(2,2)*100;

//    gluLookAt(tx, ty, tz, lx, ly, lz, Rt(0,1), Rt(1,1), Rt(2,1));

//    // if the display list has not been made yet, create a new one and
//    // fill it with scene contents
//    if (scene_list_ == 0)
//    {
//      scene_list_ = glGenLists(1);
//      glNewList(scene_list_, GL_COMPILE);
//      // now begin at the root node of the imported data and traverse
//      // the scenegraph by multiplying subsequent local transforms
//      // together on GL's matrix stack.
//      model_->Draw();
//      glEndList();
//    }

//    glCallList(scene_list_);

//}

void
Renderer3d::setModelRt(cv::Matx44d Rt)
{
    renderer_->bind_buffers();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //gluLookAt(0,0,0,0,0,1,0,1,0);
    //float m[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,-800,1};

    //change standard opencv camera coord to opengl camera coord, just a rot
    cv::Matx44d rotx180=cv::Matx44d::eye();
    rotx180(1,1)=-1;
    rotx180(2,2)=-1;
    Rt=rotx180*Rt;

    Rt=Rt.t();
    glLoadMatrixd(Rt.val);
    //glTranslatef(0,0,-500);
    //glLoadMatrixf(m);
    float m[16];
    glGetFloatv(GL_MODELVIEW_MATRIX,m);
//    for(int i=0;i<16;++i)
//        printf("%f ", m[i]);
//    printf("\r\n");
    // if the display list has not been made yet, create a new one and
    // fill it with scene contents
    if (scene_list_ == 0)
    {
      scene_list_ = glGenLists(1);
      glNewList(scene_list_, GL_COMPILE);
      // now begin at the root node of the imported data and traverse
      // the scenegraph by multiplying subsequent local transforms
      // together on GL's matrix stack.
      model_->Draw();
      glEndList();
    }

    glCallList(scene_list_);

}
void
Renderer3d::render(cv::Mat &image_out, cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);
  cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
  cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

  glFlush();

  // Get data from the depth/image buffers
  renderer_->bind_buffers();

  // Get data from the depth/image buffers
  //renderer_->get_buffers(renderer_->width_, renderer_->height_, image.ptr(), depth.ptr());

  // Deal with the RGB image
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  // Deal with the depth image
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
  for (unsigned int j = 0; j < renderer_->height_; ++j)
    for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
      if (*it > max_allowed_z)
        *it = 0;
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
  cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
  depth.convertTo(depth_scale, CV_16UC1, 1e3);

  // Crop the images, just so that they are smaller to write/read
  if (i_min > 0)
    --i_min;
  if (i_max < renderer_->width_ - 1)
    ++i_max;
  if (j_min > 0)
    --j_min;
  if (j_max < renderer_->height_ - 1)
    ++j_max;
  rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

  if ((rect.width <=0) || (rect.height <= 0)) {
    depth_out = cv::Mat();
    image_out = cv::Mat();
    mask_out = cv::Mat();
  } else {
    depth_scale(rect).copyTo(depth_out);
    image(rect).copyTo(image_out);
    mask(rect).copyTo(mask_out);
  }
}

void
Renderer3d::renderDepthOnly(cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
  cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

  glFlush();

  // Get data from the OpenGL buffers
  renderer_->bind_buffers();

  // Deal with the depth image
  glReadBuffer(GL_DEPTH_ATTACHMENT);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

  float zNear = near_, zFar = far_;
  cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
  float max_allowed_z = zFar * 0.99;

  unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
  for (unsigned int j = 0; j < renderer_->height_; ++j)
    for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
    {
      //need to undo the depth buffer mapping
      //http://olivers.posterous.com/linear-depth-in-glsl-for-real
      *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
        float tmp=*it;
      if (*it > max_allowed_z)
        *it = 0;
      else
      {
        mask(j, i) = 255;
        // Figure the inclusive bounding box of the mask
        if (j > j_max)
          j_max = j;
        else if (j < j_min)
          j_min = j;
        if (i > i_max)
          i_max = i;
        else if (i < i_min)
          i_min = i;
      }
    }

  // Rescale the depth to be in millimeters
  cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);

  //2017-12-1 zyk changed
  //depth.convertTo(depth_scale, CV_16UC1, 1e3);
  depth.convertTo(depth_scale, CV_16UC1);

  //2018-2-24 zyk changed: only crop image when rect is not specified
  if(rect.width<=0||rect.height<=0){
      // Crop the images, just so that they are smaller to write/read
      if (i_min > 0)
        --i_min;
      if (i_max < renderer_->width_ - 1)
        ++i_max;
      if (j_min > 0)
        --j_min;
      if (j_max < renderer_->height_ - 1)
        ++j_max;
      rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);
  }
  if ((rect.width <=0) || (rect.height <= 0)) {
    depth_out = cv::Mat();
    mask_out = cv::Mat();
  } else {
    depth_scale(rect).copyTo(depth_out);
    mask(rect).copyTo(mask_out);
  }
}

//void Renderer3d::renderDepthSimple(cv::Mat &depth_out, cv::Mat &mask_out, cv::Rect &rect) const
//{
//    // Create images to copy the buffers to
//    cv::Mat_<float> depth(renderer_->height_, renderer_->width_);
//    cv::Mat_ < uchar > mask = cv::Mat_ < uchar > ::zeros(cv::Size(renderer_->width_, renderer_->height_));

//    glFlush();

//    // Get data from the OpenGL buffers
//    renderer_->bind_buffers();

//    // Deal with the depth image
//    glReadBuffer(GL_DEPTH_ATTACHMENT);
//    glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_DEPTH_COMPONENT, GL_FLOAT, depth.ptr());

//    float zNear = near_, zFar = far_;
//    cv::Mat_<float>::iterator it = depth.begin(), end = depth.end();
//    float max_allowed_z = zFar * 0.99;

//    unsigned int i_min = renderer_->width_, i_max = 0, j_min = renderer_->height_, j_max = 0;
//    for (unsigned int j = 0; j < renderer_->height_; ++j)
//      for (unsigned int i = 0; i < renderer_->width_; ++i, ++it)
//      {
//        //need to undo the depth buffer mapping
//        //http://olivers.posterous.com/linear-depth-in-glsl-for-real
//        *it = 2 * zFar * zNear / (zFar + zNear - (zFar - zNear) * (2 * (*it) - 1));
//         //float tp=*it;
//        if (*it > max_allowed_z)
//          *it = 0;
//        else
//        {
//          mask(j, i) = 255;
//          // Figure the inclusive bounding box of the mask
//          if (j > j_max)
//            j_max = j;
//          else if (j < j_min)
//            j_min = j;
//          if (i > i_max)
//            i_max = i;
//          else if (i < i_min)
//            i_min = i;
//        }
//      }

//    // Rescale the depth to be in millimeters
//    cv::Mat depth_scale(cv::Size(renderer_->width_, renderer_->height_), CV_16UC1);
//    depth.convertTo(depth_scale, CV_16UC1);


////    cv::imshow("d",depth_scale);
////    cv::waitKey(0);
////    // Crop the images, just so that they are smaller to write/read
////    if (i_min > 0)
////      --i_min;
////    if (i_max < renderer_->width_ - 1)
////      ++i_max;
////    if (j_min > 0)
////      --j_min;
////    if (j_max < renderer_->height_ - 1)
////      ++j_max;
////    rect = cv::Rect(i_min, j_min, i_max - i_min + 1, j_max - j_min + 1);

//    if ((rect.width <=0) || (rect.height <= 0)) {
//      depth_out = cv::Mat();
//      mask_out = cv::Mat();
//    } else {
//      depth_scale(rect).copyTo(depth_out);
//      mask(rect).copyTo(mask_out);
//    }
////    for(int i=0;i<depth_out.rows;++i)
////        for(int j=0;j<depth_out.cols;++j){
////            ushort test=depth_out.at<ushort>(i,j);
////            if(test>100)
////                test=0;
////        }
//}

void
Renderer3d::renderImageOnly(cv::Mat &image_out, const cv::Rect &rect) const
{
  // Create images to copy the buffers to
  cv::Mat_ < cv::Vec3b > image(renderer_->height_, renderer_->width_);

  glFlush();

  // Get data from the OpenGL buffers
  renderer_->bind_buffers();

  // Deal with the RGB image
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  if ((rect.width <=0) || (rect.height <= 0)) {
    image_out = cv::Mat();
  } else {
    image(rect).copyTo(image_out);
  }
}
