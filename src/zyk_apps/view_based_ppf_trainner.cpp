
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer3d.h>

#include <opencv2/highgui/highgui.hpp>

#include "util_pcl.h"
#include <util.h>
#include "Voxel_grid.h"
#include "PPFFeature.h"
#include "pose_cluster.h"
#include "SmartSampling.hpp"

#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

std::string model_filename_;
std::string cloud_filename_;
std::string save_filename_;

//Algorithm params
//bool use_cloud_resolution_  (false);
//bool use_ply_filetype_  (false);
bool use_existing_normal_data_  (false);
//bool x_centrosymmetric_  (false);
//bool y_centrosymmetric_  (false);
//bool z_centrosymmetric_  (false);
bool save_sampled_cloud_ (false);
bool normal_reorient_switch_ (false);
bool smart_sample_border_ (false);
bool show_original_model_ (false);
bool show_views_ (false);
//bool change_center_switch_(false);
bool use_mls_ (false);
float ang_degree_thresh (15);
float model_ds_ (0.05f);
float plane_ds_ (0.05f);
float curvature_radius_ (0.05f);
int angle_div_ (15);
int distance_div_ (20);

int number_render_points_ (168);
void showHelp(char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: "  << " ppf_train [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:              Show this help." << std::endl;
  std::cout << "     --mod val:       Path of the model CAD(ply/obj)." << std::endl;
  std::cout << "     --out val:       Path of the output .ppfs file(if not specified, same as model)" << std::endl;
  std::cout << "     --cld val:       Path of the cloud data(ply,txt), if not provided, use CAD to generate." << std::endl;
  //std::cout << "     -r:						Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "     -w:              write the sampled model" << std::endl;
  //std::cout << "     --ply:					Use .poly as input cloud. Default is .pcd" << std::endl;
  std::cout << "     --rn:            Reorient switch!" << std::endl;
  std::cout << "     --cc:            Change Center switch!" << std::endl;
  std::cout << "     --so:            show original model" << std::endl;
  std::cout << "     --sv:            Show views" << std::endl;
  std::cout << "     --mls:           Use moving least squares" << std::endl;
  std::cout << "     --sp val:        smart sampling, set angle_degree thresh" << std::endl;
  std::cout << "     --model_ds val:  Model down sampling radtia (default 0.05)" << std::endl;
  std::cout << "     --plane_ds val:  Model plane feature down sampling ratia, if not set, default same as model" << std::endl;
  std::cout << "     --curv_r val:  	curvature radius" << std::endl;
  std::cout << "     --a_div val: 		angle division" << std::endl;
  std::cout << "     --d_div val:			distance division" << std::endl;

}

void parseCommandLine(int argc, char *argv[])
{
  if (pcl::console::find_switch(argc, argv, "-h"))
  {
    showHelp(argv[0]);
    exit(0);
  }

  //Program behavior
  //if (pcl::console::find_switch(argc, argv, "-r"))
  //{
  //	use_cloud_resolution_ = true;
  //}
  if (pcl::console::find_switch(argc, argv, "-w"))
  {
    save_sampled_cloud_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--rn"))
  {
    normal_reorient_switch_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--sp"))
  {
    smart_sample_border_= true;
  }
  if (pcl::console::find_switch(argc, argv, "--so"))
  {
    show_original_model_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--sv"))
  {
    show_views_ = true;
  }
  if (pcl::console::find_switch(argc, argv, "--mls"))
  {
    use_mls_ = true;
  }
//	if (pcl::console::find_switch(argc, argv, "--cc"))
//	{
//		change_center_switch_ = true;
//	}
  //Model filename
  pcl::console::parse_argument(argc, argv, "--mod", model_filename_);
  pcl::console::parse_argument(argc, argv, "--out", save_filename_);
  if(save_filename_.empty()){
    int pos = model_filename_.find_last_of('.');
    save_filename_ = model_filename_.substr(0, pos);
    save_filename_ += ".ppfs";
  }
  else{
    if(save_filename_.find(".ppfs")==std::string::npos){
      pcl::console::print_error("invalid output file name, must be *.ppfs!");
      exit(-1);
    }
  }
  pcl::console::parse_argument(argc, argv, "--cld", cloud_filename_);
  //General parameters
  pcl::console::parse_argument(argc, argv, "--model_ds", model_ds_);
  plane_ds_ = model_ds_;
  pcl::console::parse_argument(argc, argv, "--plane_ds", plane_ds_);
  pcl::console::parse_argument(argc, argv, "--curv_r", curvature_radius_);
  pcl::console::parse_argument(argc, argv, "--sp", ang_degree_thresh);
  pcl::console::parse_argument(argc, argv, "--a_div", angle_div_);
  pcl::console::parse_argument(argc, argv, "--d_div", distance_div_);
  pcl::console::parse_argument(argc, argv, "--nv", number_render_points_);
}


void sample_multi_view_model_clouds(std::string cad_file_name,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints,
                                    int num_views,
                                    std::vector<std::vector<int> >& indexes_out,
                                    std::vector<cv::Matx44d>& Rt_out,
                                    std::vector<float>& weights);
int main(int argc, char **argv) {

  parseCommandLine(argc, argv);

  showHelp(argv[0]);
  pcl::PointCloud<pcl::PointNormal>::Ptr model(new pcl::PointCloud<pcl::PointNormal>());
  pcl::PointCloud<pcl::PointNormal>::Ptr keypoints(new pcl::PointCloud<pcl::PointNormal>());

  double min_coord[3],max_coord[3];
  if(cloud_filename_.empty()){
    if(!zyk::mesh_sampling(model_filename_,30000,*model,min_coord,max_coord)){
      PCL_ERROR("Samping mesh fail!");
      return -1;
    }
  }
  else{
    if(!zyk::readPointNormalCloud(cloud_filename_,model))
    {
      PCL_ERROR("Read cloud fail!");
      return -1;
    }
    pcl::PointCloud<PointType>::Ptr tmp (new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*model,*tmp);
    zyk::getBoundingBox(tmp, min_coord, max_coord);
  }


  //
  //  Set up resolution invariance
  //
  double model_length = max_coord[0] - min_coord[0];
  double model_width = max_coord[1] - min_coord[1];
  double model_height = max_coord[2] - min_coord[2];
  Eigen::Vector3f model_approximate_center;
  model_approximate_center(0) = (max_coord[0] + min_coord[0]) / 2;
  model_approximate_center(1) = (max_coord[1] + min_coord[1]) / 2;
  model_approximate_center(2) = (max_coord[2] + min_coord[2]) / 2;


  double d_max = sqrt(model_length*model_length + model_width*model_width + model_height*model_height);
  double model_ss_ = model_ds_*d_max;
  std::cout << "Model sampling distance step:    " << model_ss_ << std::endl;
  std::cout << "Model length: " << model_length << std::endl;
  std::cout << "Model width: " << model_width << std::endl;
  std::cout << "Model height: " << model_height << std::endl;

  //
  // show original model
  //
  if (show_original_model_)
  {

    pcl::visualization::PCLVisualizer org_visual("Original Viewr");
    org_visual.addCoordinateSystem(20);
    org_visual.addPointCloud(model, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(model, 0.0, 0.0, 255.0), "model");
    org_visual.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(model, model, 1, 10, "model_normal");
    org_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
    org_visual.spin();
  }

  if(smart_sample_border_)
  {
    std::cout<<"Use smart sampling, angle thresh(deg): "<<ang_degree_thresh<<std::endl;
    zyk::SmartDownSamplePointAndNormal(model,ang_degree_thresh,model_ss_,keypoints);
  }
  else
  {
    std::cout<<"Use uniform sampling"<<std::endl;
    zyk::uniformDownSamplePointAndNormal(model, model_ss_, keypoints);
  }



  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::Normal>::Ptr keypoints_normal(new pcl::PointCloud<pcl::Normal>());
  pcl::copyPointCloud(*keypoints,*keypoints_xyz);
  pcl::copyPointCloud(*keypoints,*keypoints_normal);
  std::cout << "Model total points: " << model->size() << std::endl;
  std::cout <<" Selected downsample: " << keypoints->size() << std::endl;

//  //
//  //visualize keypoints
//  //

//  pcl::visualization::PCLVisualizer key_visual("Key point Viewr");
//  key_visual.addCoordinateSystem(20);
//  key_visual.addPointCloud(keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(keypoints, 0.0, 0.0, 255.0), "keypoints");
//  key_visual.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(keypoints, keypoints, 1, 10, "keynormals");
//  key_visual.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
//  key_visual.spin();


  //
  //views
  //
  std::vector<std::vector<int> > indexes;
  std::vector<cv::Matx44d> Rt_vec;
  ////show some info
  std::cout<<">>>>>Number Views: "<<number_render_points_<<std::endl;
  std::vector<float>weights;
  sample_multi_view_model_clouds(model_filename_,keypoints_xyz, number_render_points_, indexes, Rt_vec,weights);

  //
  // if cloud are not provided, clean the keypoints by remove weights less than 0.3
  //
  if(cloud_filename_.empty()){
    pcl::PointCloud<pcl::PointNormal>::Ptr new_key(new pcl::PointCloud<pcl::PointNormal>());
    for(size_t i=0;i<keypoints->size();++i){
      if(weights[i]>0.3){
        new_key->push_back(keypoints->at(i));
      }
    }
    //sample again
    //keypoints->clear();
    //zyk::SmartDownSamplePointAndNormal(new_key,ang_degree_thresh,model_ss_,keypoints);
    keypoints=new_key;
    keypoints_xyz->clear();
    keypoints_normal->clear();
    pcl::copyPointCloud(*keypoints,*keypoints_xyz);
    pcl::copyPointCloud(*keypoints,*keypoints_normal);
    std::cout <<" Selected clean downsample: " << keypoints->size() << std::endl;
    //sample view again
    sample_multi_view_model_clouds(model_filename_,keypoints_xyz, number_render_points_, indexes, Rt_vec,weights);

  }

  //
  // show key
  //

  pcl::visualization::PCLVisualizer key_visual2("Key point Viewr");
  key_visual2.addCoordinateSystem(20);
  key_visual2.addPointCloud(keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(keypoints, 0.0, 0.0, 255.0), "keypoints");
  //key_visual2.addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(keypoints, keypoints, 1, 10, "keynormals");
  key_visual2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints");
  key_visual2.spin();
  if (save_sampled_cloud_)
  {
    pcl::io::savePLYFile(model_filename_ + "_changed", *keypoints);
    std::cout<<"save sample changed!"<<std::endl;
  }



  //
  // display view_indexes
  //
  if(show_views_)
  {
    float renderer_focal_length_x=572.41140;
    float renderer_focal_length_y=573.57043;
    cv::Matx33d K = cv::Matx33d(renderer_focal_length_x, 0.0f, 640.0/2.0f, 0.0f, renderer_focal_length_y, 480.0/2.0f, 0.0f, 0.0f, 1.0f);
    for(int i=0;i<indexes.size();++i){
      // create a temporary point cloud that can be visible from current view
      cv::Mat tmp_mask=cv::Mat::zeros(480,640,CV_8UC1);
      for(int j=0;j<indexes[i].size();++j){
        int idx=indexes[i][j];
        cv::Vec4d a = Rt_vec[i]*cv::Vec4d(keypoints_xyz->at(idx).x, keypoints_xyz->at(idx).y, keypoints_xyz->at(idx).z, 1);
        cv::Vec3d dir(a(0)/a(2),a(1)/a(2),1);
        cv::Vec3d uv=K*dir;
        int u = int(uv(0));
        int v = int(uv(1));
        tmp_mask.at<uchar>(v,u)=255;
      }
      std::cout<<"This view has point: "<<indexes[i].size()<<std::endl;
      cv::imshow("tmp view", tmp_mask);
      cv::waitKey(0);
    }
  }



  //model ppf space
  char tmp[100];
  _splitpath(model_filename_.c_str(), NULL, NULL, tmp, NULL);
  std::string objName(tmp);
  std::cout << "Trained object Name: " << objName << std::endl;
  zyk::PPF_Space model_feature_space;
  cout << "trained using angle_div , distance_div: " << angle_div_ << ", " << distance_div_ << endl;
  model_feature_space.init(objName, keypoints_xyz, keypoints_normal, indexes, angle_div_ , distance_div_,true);
  //model_feature_space.init(objName, keypoints_xyz, keypoints_normal, angle_div_ , distance_div_,true);
  model_feature_space.model_res = model_ss_;

  //
  // compute no empty ppf box nunber
  //

  int cnt = 0;
  for (int i = 0; i < model_feature_space.getBoxVector()->size(); ++i)
  {
    if (model_feature_space.getBoxVector()->at(i) != NULL)
      cnt++;
  }
  cout << "no empty box number is: " << cnt << endl;

  std::cout<<"save file name: "<<save_filename_<<std::endl;
  getchar();
  model_feature_space.save(save_filename_);
  return 0;

}

  /** sample a CAD model and generate visible context from multi view
   * @param cad_file_name the path of the model cad
   * @param num_views number of views on the sphere
   * @param keypoints
   * @param indexes_out each vector stores the index of sampled model points that are visible to the view
   * @param Rt_out vector to store the pose of object
   */
void sample_multi_view_model_clouds(std::string cad_file_name,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& keypoints,
                                    int num_views,
                                    std::vector<std::vector<int> >& indexes_out,
                                    std::vector<cv::Matx44d>& Rt_out,
                                    std::vector<float>& weights)
{
  // Define the display
  size_t width = 640, height = 480;
  //render parameters
  size_t renderer_n_points=num_views;
  float render_near=0.1, render_far=2000.0;
  float renderer_angle_step = 69;
  float renderer_radius_min = 500;
  float renderer_radius_max = 500;
  float renderer_radius_step = 100;
  float renderer_focal_length_x=572.41140;
  float renderer_focal_length_y=573.57043;

//  pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>());
//  pcl::io::loadPLYFile(cad_file_name, *model);

//  double max_coord[3],min_coord[3];
//  zyk::getBoundingBox(model,max_coord,min_coord);
//  double diameter = zyk::dist(max_coord,min_coord,3);
//  double res=sample_ratio*diameter;
//  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
//  uniform_sampling.setInputCloud(model);
//  uniform_sampling.setRadiusSearch(res);
//  uniform_sampling.filter(*sampled_cloud_out);
//  std::cout<<"key poitns size: "<<sampled_cloud_out->size()<<std::endl;

  Renderer3d renderer = Renderer3d(cad_file_name);
  renderer.set_parameters(width, height, renderer_focal_length_x, renderer_focal_length_y, width/2,height/2,render_near, render_far);

  RendererIterator renderer_iterator = RendererIterator(&renderer, renderer_n_points);
  //set the RendererIterator parameters
  renderer_iterator.angle_min_=0;
  //this must be fixed, when set angle_min angle_ are not changed
  renderer_iterator.angle_=0;
  renderer_iterator.angle_max_=50;
  renderer_iterator.angle_step_ = renderer_angle_step;
  renderer_iterator.radius_ = renderer_radius_min;
  renderer_iterator.radius_min_ = renderer_radius_min;
  renderer_iterator.radius_max_ = renderer_radius_max;
  renderer_iterator.radius_step_ = renderer_radius_step;
  renderer_iterator.absolute_radius_step=true;
  cv::Mat image, depth, mask;

  cv::Matx33d K = cv::Matx33d(renderer_focal_length_x, 0.0f, float(width)/2.0f, 0.0f, renderer_focal_length_y, float(height)/2.0f, 0.0f, 0.0f, 1.0f);

  indexes_out=std::vector<std::vector<int> >(num_views, std::vector<int>());
  weights=std::vector<float>(keypoints->size(),0.0);
  Rt_out.clear();
  for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
  {
    cv::Rect rect;
    rect.width=width;
    rect.height=height;
    renderer_iterator.render(image, depth, mask, rect);

    cv::Matx44d rt = renderer_iterator.Rt_obj();
    Rt_out.push_back(rt);
    //flip
    cv::flip(depth, depth,0);
    for(int j=0;j<keypoints->size();++j){
      //calculate current coord
      cv::Vec4d a = rt*cv::Vec4d(keypoints->at(j).x, keypoints->at(j).y, keypoints->at(j).z, 1);
      cv::Vec3d dir(a(0)/a(2),a(1)/a(2),1);
      cv::Vec3d uv=K*dir;
      int u = int(uv(0));
      int v = int(uv(1));
      float tmp = depth.at<ushort>(v,u);
      if(fabs(tmp-a(2))<3){
        weights[j]+=1.0;
        indexes_out[i].push_back(j);
      }
    }
  }
  for(size_t i=0;i<keypoints->size();++i){
    weights[i]/=renderer_n_points;
  }
}
