/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014- Centrum Wiskunde en Informatica
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
*   * Neither the name of copyright holder(s)  nor the names of its
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
* $Id$
*
*/
#include <iostream>
#include <fstream>
#include <vector>

#include <cmath>
#include <pcl/compression_eval/compression_eval.h>

// dependency on the reverie mesh, allows rendering in reverie framework
#include <pcl/compression_eval/reverie_mesh.h>
#include <pcl/compression_eval/impl/reverie_mesh_impl.hpp>

#include <pcl/compression_eval/compression_eval.h>
#include <pcl/compression_eval/impl/compression_eval_impl.hpp>

#include <pcl/quality/quality_metrics.h>
#include <pcl/quality/impl/quality_metrics_impl.hpp>

#include <boost/program_options.hpp>
#include<boost/program_options/parsers.hpp>

#include <assert.h>
#include <sstream>
#include <utility>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

using namespace std;
using namespace pcl;
using namespace pcl::quality;
using namespace pcl::io;
using namespace pcl::octree;
using namespace pcl::console;

namespace po = boost::program_options;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**! 
* \struct to store meshes metadata for official evaluation by MPEG committee
* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct compression_eval_mesh_meta_data
{
  string original_file_name;
  size_t original_file_size;
  bool has_coords;
  bool has_normals;
  bool has_colors;
  bool has_texts;
  bool has_conn;
  compression_eval_mesh_meta_data()
    : has_coords(true),  has_normals(false), has_colors(false), has_texts(false), has_conn(false)
  {}
};

//! explicit instantiation of the octree compression modules from pcl
//template class OctreePointCloudCodecV2<PointXYZRGB>;

template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>;
template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>;

typedef OctreePointCloudCompression<PointXYZ> pointsOnlyOctreeCodec;
typedef OctreePointCloudCompression<PointXYZRGB> colorOctreeCodec;
typedef OctreePointCloudCodecV2<PointXYZRGB> colorOctreeCodecV2;
///////////////  Bounding Box Logging /////////////////////////	
// log information on the bounding boxes, which is critical for alligning clouds in time
ofstream bb_out("bounding_box_pre_mesh.txt");
Eigen::Vector4f min_pt_bb;
Eigen::Vector4f max_pt_bb;
bool is_bb_init=false;
double bb_expand_factor = 0.10;
/////////////// END CODEC PARAMETER SETTINGS /////////////////////////

void
  printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input_dir1 input_dir2 ............ input_dirN\n put the parameter_config.txt", argv[0]);
}

//! function for loading a mesh file
bool
  loadPLYMesh (const string &filename, pcl::PolygonMesh &mesh)
{
  TicToc tt;
  print_highlight ("Loading "); 
  print_value ("%s ", filename.c_str ());
  pcl::PLYReader reader;
  tt.tic ();

  if (reader.read (filename, mesh) < 0)
    return (false);

  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); 
  print_info (" ms : "); 
  print_value ("%d", mesh.cloud.width * mesh.cloud.height); 
  print_info (" points]\n");
  print_info ("Available dimensions: "); 
  print_value ("%s\n", pcl::getFieldsList (mesh.cloud).c_str ());

  return (true);
}

//! function for loading a mesh files in a folder
bool
  loadPLYFolder(const string &folder_name, vector<pcl::PolygonMesh> &meshes, vector<compression_eval_mesh_meta_data> &meshes_meta_data){

    // check if folder is directory
    if(!boost::filesystem::is_directory(folder_name)){
      print_info("::LoadPLYFolder: not a directory!"); 
      print_value("%s\n", folder_name.c_str());
      return false;
    }

    // use boost file_system to load all the mesh files in the folder to the meshes array
    boost::filesystem::directory_iterator dir_iter(folder_name);
    boost::filesystem::directory_iterator end_iter;

    for(  ; dir_iter != end_iter ; ++dir_iter)
    {
      if(boost::filesystem::is_regular_file(dir_iter->path()))
      {
        string file_name = dir_iter->path().generic_string();
        string file_ext = dir_iter->path().extension().generic_string();

        //! we only support .ply meshes
        if(file_ext ==".ply"){

          //! load the mesh data
          print_info("::LoadFolder: found ply file, "); 
          print_value(" %s file_extension %s \n", file_name.c_str(),file_ext.c_str());
          meshes.push_back(pcl::PolygonMesh());
          loadPLYMesh(file_name,meshes.back());
          //~ done loading mesh

          // load the metadata 
          compression_eval_mesh_meta_data mdata;
          mdata.original_file_name = dir_iter->path().filename().generic_string();
          mdata.original_file_size =  file_size(dir_iter->path());

          //! check if the mesh is a point cloud or not
          if(meshes.back().polygons.size() > 0)
            mdata.has_conn = true;
          else
            mdata.has_conn = false;

          //! check the fields in the point cloud to detect properties of the mesh
#if __cplusplus >= 201103L
          for( auto it = meshes.back().cloud.fields.begin(); it != meshes.back().cloud.fields.end(); ++it)
#else
          for( std::vector<pcl::PCLPointField>::iterator it = meshes.back().cloud.fields.begin(); it != meshes.back().cloud.fields.end(); ++it)
#endif//__cplusplus >= 201103L
          {
            if( it->name == "rgb")
              mdata.has_colors = true;
            if( it->name == "normal_x")
              mdata.has_colors = true;
            if( it->name == "x")
              mdata.has_coords = true;
          }
          meshes_meta_data.push_back(mdata);
          //! done loading metadata
        }
      }
    }
    if(meshes.size())
      return true;
    else 
      return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**! 
* \brief 
script for point cloud codec evaluation by MPEG committee
\param  the input command line arguments dir1 , dir2, dir3 ..... 
\note clouds from dir1 , dir2 , dir3 .... will be fused in a single cloud if dir1,..dir2, contain the per-view clouds
* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
  main (int argc, char** argv)
{
  print_info ("Load a Folder of Point Clouds\n ", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  ////////////////// parse configuration settings from required file ..//parameter_config.txt ///////////////////
  boost::program_options::options_description desc;

  // default values for all options are the same as in the distributed 'parameter_config.txt '
  desc.add_options()
    ("help", " produce help message ")
    ("mesh_file_folders", po::value<vector<string> >(), " folder mesh files ")
    ("octree_bit_settings", po::value<vector<int> >()->default_value(std::vector<int>(),"{11}"), " quantization bit assignment octree ")
    ("color_bit_settings", po::value<vector<int> >()->default_value(std::vector<int>(),"{8}"), "color bit assignment octree or jpeg quality values ")
    ("enh_bit_settings", po::value<int>()->default_value(0), " bits to code the points towards the center ")
    ("color_coding_types", po::value<vector<int> >()->default_value(std::vector<int>(),"{0}"), "  pcl=0,jpeg=1 or graph transform ")
    ("keep_centroid", po::value<int>()->default_value(0), " for keeping centroid ")
    ("bb_expand_factor", po::value<double>()->default_value(0.20), " bounding box expansion to keep bounding box accross frames ")
    ("output_csv_file", po::value<string>()->default_value("intra_frame_quality.csv")," output .csv file ")
    ("write_output_ply", po::value<int>()->default_value(1)," write output as .ply files")
    ("do_delta_frame_coding", po::value<int>()->default_value(1)," do_delta_frame_coding ")
    ("icp_on_original", po::value<int>()->default_value(0)," icp_on_original ")
    ("pframe_quality_log",po::value<string>()->default_value("predictive_quality.csv"), " write the quality results of predictive coding of p frames")
    ("macroblocksize",po::value<int>()->default_value(16), " size of macroblocks used for predictive frame (has to be a power of 2)")
    ("testbbalign",po::value<int>()->default_value(0), " set this option to test allignements only ")
    ("code_color_off",po::value<int>()->default_value(0), " do color offset coding on predictive frames")
    ;
  // Check if required file 'parameter_config.txt' is present
  ifstream in_conf("..//parameter_config.txt");
    if (in_conf.fail()) {
      in_conf.open("parameter_config.txt");
      if (in_conf.fail()) {
        print_info (" Required file 'parameter_config.txt' not found in '%s' or its parent.\n", boost::filesystem::current_path().string().c_str());
        return (-1);
      }
  }
  po::variables_map vm;
  po::store(po::parse_config_file(in_conf, desc), vm);
  po::notify(vm);
  bb_expand_factor = vm["bb_expand_factor"].as<double>();
    
  ////////////////// ~end parse configuration file  /////////////////////////////////////


  ////////////////// LOADING CLOUDS INTO MEMORY /////////////////////////////////////
  // store folders to load in a vector
  vector<int> ply_folder_indices = parse_file_extension_argument (argc, argv, "");

  // store all loaded meshes in a vector and store all metadata separately (optional)
  vector<vector<pcl::PolygonMesh> > meshes;
  vector<vector<compression_eval_mesh_meta_data> > meshes_meta_data;

  // data structures for storing the fused meshes
  vector<boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > > fused_clouds;
  vector<compression_eval_mesh_meta_data> fused_clouds_meta_data;

  meshes.resize(ply_folder_indices.size());
  meshes_meta_data.resize(ply_folder_indices.size());

  // load all the meshes from the folder (point clouds)
  if(ply_folder_indices.size() > 0)
  {
    for(int i=0; i<ply_folder_indices.size();i++){
      if (!loadPLYFolder (argv[ply_folder_indices[i]], meshes[i], meshes_meta_data[i])) 
      {
        print_info (" Failed to Load Mesh File Folder"); print_value ("%s\n", (argv[ply_folder_indices[0]]));
        return (-1);
      }
    }
  }
  ////////////////// END LOADING CLOUDS //////////////////////////////////////////////////




  //////////////// FUSE CLOUDS ///////////////////////////////////////////////////////

  // in this cycle we create the fused point clouds when more than one folder is added
  if(ply_folder_indices.size() > 0)
  {
    // for each folder (we assume folder have the same number of files and the same ordering)
    for(int j =0; j<ply_folder_indices.size(); j++){
      // for each mesh in first folder create the fused cloud by loading from first folder and appending the rest
      for(int i=0; i <meshes[j].size(); i++){
        colorOctreeCodec::PointCloudPtr l_ptr= colorOctreeCodec::PointCloudPtr(new colorOctreeCodec::PointCloud()); 

        // convert to the point cloud 1 from the blob, with and without colors
        pcl::fromPCLPointCloud2(meshes.at(j).at(i).cloud,*l_ptr);

        // for the first folder, create the fused clouds, append the clouds from the next folders
        if(j == 0)
        {
          fused_clouds.push_back(l_ptr);
        }
        else
        {
          for(int k=0;k<l_ptr->size();k++)
            fused_clouds[i]->push_back((*l_ptr)[k]); // appends the points
        }
      }
    }
  }

  //////////////// END FUSE CLOUDS ///////////////////////////////////////////////////////

  //////////////// Logging of Prediction Performance /////////////////////////////////
  int bb_align_count=0;
  std::vector<bool> aligned_flags(fused_clouds.size()); // flags to check if a cloud is aligned 
  /////////////////////////////////////////////////////////////////////////////////////

  /////////////// NORMALIZE CLOUDS ///////////////////////////////////////////////////////

#if __cplusplus >= 201103L
  struct bounding_box
  {
    Eigen::Vector4f min_xyz;
    Eigen::Vector4f max_xyz;
  };
#endif//__cplusplus

  // initial bounding box
  min_pt_bb[0]= 1000;
  min_pt_bb[1]= 1000;
  min_pt_bb[2]= 1000;

  max_pt_bb[0]= -1000;
  max_pt_bb[1]= -1000;
  max_pt_bb[2]= -1000;

  vector<bounding_box> assigned_bbs(fused_clouds.size());

  for(int k=0;k<fused_clouds.size();k++){
    /*
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt,max_pt);

    bb_out << "[ " << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "]    [" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() <<"]" << endl;

    // check if min fits bounding box, otherwise adapt the bounding box
    if( !((min_pt.x() > min_pt_bb.x()) && (min_pt.y() > min_pt_bb.y()) && (min_pt.z() > min_pt_bb.z())))
    {
    is_bb_init = false;
    }

    // check if max fits bounding box, otherwise adapt the bounding box
    if(!((max_pt.x() < max_pt_bb.x()) && (max_pt.y() < max_pt_bb.y()) && (max_pt.z() < max_pt_bb.z())))
    {
    is_bb_init = false;
    }


    if(!is_bb_init)
    {
    // initialize the bounding box, with bb_expand_factor extra
    assigned_bbs[k].min_xyz[0] = min_pt[0] - bb_expand_factor*abs(max_pt[0] - min_pt[0]);
    assigned_bbs[k].min_xyz[1] = min_pt[1] - bb_expand_factor*abs(max_pt[1] - min_pt[1]);
    assigned_bbs[k].min_xyz[2] = min_pt[2] - bb_expand_factor*abs(max_pt[2] - min_pt[2]);

    min_pt_bb=assigned_bbs[k].min_xyz ;

    assigned_bbs[k].max_xyz[0] = max_pt[0] + bb_expand_factor*abs(max_pt[0] - min_pt[0]);
    assigned_bbs[k].max_xyz[1] = max_pt[1] + bb_expand_factor*abs(max_pt[1] - min_pt[1]);
    assigned_bbs[k].max_xyz[2] = max_pt[2] + bb_expand_factor*abs(max_pt[2] - min_pt[2]);

    max_pt_bb=assigned_bbs[k].max_xyz;

    is_bb_init = true;
    bb_align_count++;
    cout << "re-intialized bounding box !!! " << endl;
    aligned_flags[k] = false;
    }
    else
    {
    assigned_bbs[k].min_xyz= min_pt_bb;
    assigned_bbs[k].max_xyz= max_pt_bb;
    aligned_flags[k] = true;
    }

    Eigen::Vector4f dyn_range = assigned_bbs[k].max_xyz - assigned_bbs[k].min_xyz;

    for(int j=0; j < fused_clouds[k]->size();j++)
    {
    // offset the minimum value
    fused_clouds[k]->at(j).x-=assigned_bbs[k].min_xyz[0];
    fused_clouds[k]->at(j).y-=assigned_bbs[k].min_xyz[1];
    fused_clouds[k]->at(j).z-=assigned_bbs[k].min_xyz[2];

    // dynamic range
    fused_clouds[k]->at(j).x/=dyn_range[0];
    fused_clouds[k]->at(j).y/=dyn_range[1];
    fused_clouds[k]->at(j).z/=dyn_range[2];
    }

    // bounding box is expanded
    Eigen::Vector4f min_pt_res;
    Eigen::Vector4f max_pt_res;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt_res,max_pt_res);

    assert(min_pt_res[0] >= 0);
    assert(min_pt_res[1] >= 0);
    assert(min_pt_res[2] >= 0);

    assert(max_pt_res[0] <= 1);
    assert(max_pt_res[1] <= 1);
    assert(max_pt_res[2] <= 1);
    */
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt,max_pt);

    bb_out << "[ " << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "]    [" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() <<"]" << endl;

    // check if min fits bounding box, otherwise adapt the bounding box
    if( !( (min_pt.x() > min_pt_bb.x()) && (min_pt.y() > min_pt_bb.y()) && (min_pt.z() > min_pt_bb.z())))
    {
      is_bb_init = false;
    }

    // check if max fits bounding box, otherwise adapt the bounding box
    if(!((max_pt.x() < max_pt_bb.x()) && (max_pt.y() < max_pt_bb.y()) && (max_pt.z() < max_pt_bb.z())))
    {
      is_bb_init = false;
    }


    if(!is_bb_init)
    {
      aligned_flags[k] = false;
      bb_align_count++;
      // initialize the bounding box, with bb_expand_factor extra
      min_pt_bb[0] = min_pt[0] - bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      min_pt_bb[1] = min_pt[1] - bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      min_pt_bb[2] = min_pt[2] - bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      max_pt_bb[0] = max_pt[0] + bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      max_pt_bb[1] = max_pt[1] + bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      max_pt_bb[2] = max_pt[2] + bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      is_bb_init = true;

      cout << "re-intialized bounding box !!! " << endl;
    }
    else 
      aligned_flags[k] = true;

#if __cplusplus >= 201103L
    auto dyn_range = max_pt_bb - min_pt_bb;
#else
    Eigen::Vector4f  dyn_range = max_pt_bb - min_pt_bb;
#endif//__cplusplus >= 201103L

    assigned_bbs[k].max_xyz = max_pt_bb;
    assigned_bbs[k].min_xyz = min_pt_bb;

    for(int j=0; j < fused_clouds[k]->size();j++)
    {
      // offset the minimum value
      fused_clouds[k]->at(j).x-=min_pt_bb[0];
      fused_clouds[k]->at(j).y-=min_pt_bb[1];
      fused_clouds[k]->at(j).z-=min_pt_bb[2];

      // dynamic range
      fused_clouds[k]->at(j).x/=dyn_range[0];
      fused_clouds[k]->at(j).y/=dyn_range[1];
      fused_clouds[k]->at(j).z/=dyn_range[2];
    }


    // bounding box is expanded

    Eigen::Vector4f min_pt_res;
    Eigen::Vector4f max_pt_res;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt_res,max_pt_res);

    assert(min_pt_res[0] >= 0);
    assert(min_pt_res[1] >= 0);
    assert(min_pt_res[2] >= 0);

    assert(max_pt_res[0] <= 1);
    assert(max_pt_res[1] <= 1);
    assert(max_pt_res[2] <= 1);
  }

  /////////////// END NORMALIZE CLOUDS ///////////////////////////////////////////////////////

  /////////////// PREPARE OUTPUT CSV FILE AND CODEC PARAMTER SETTINGS /////////////////////////
  string o_log_csv = vm["output_csv_file"].as<string>();
#if __cplusplus >= 201103L
  ofstream res_base_ofstream(o_log_csv);
#else
  ofstream res_base_ofstream(o_log_csv.c_str());
#endif//__cplusplus >= 201103L
  string p_log_csv = vm["pframe_quality_log"].as<string>();
#if __cplusplus >= 201103L
  ofstream res_p_ofstream(p_log_csv);
#else
  ofstream res_p_ofstream(p_log_csv.c_str());
#endif//__cplusplus >= 201103L
  ofstream res_enh_ofstream("results_enh.csv");

  // print the headers
  QualityMetric::print_csv_header(res_base_ofstream);
  QualityMetric::print_csv_header(res_p_ofstream);

  /////////////// END PREPARE OUTPUT CSV FILE AND CODEC PARAMTER SETTINGS /////////////////////////

  ////////////// FOR EACH PARAMETER SETTING DO ASSESMENT //////////////////
  int enh_bit_settings = vm["enh_bit_settings"].as<int>();
  vector<int> octree_bit_settings = vm["octree_bit_settings"].as<vector<int> >();
  vector<int> color_bit_settings =  vm["color_bit_settings"].as<vector<int> >();
  vector<int> color_coding_types =  vm["color_coding_types"].as<vector<int> >();
  bool keep_centroid = vm["keep_centroid"].as<int>();
  int write_out_ply =  vm["write_output_ply"].as<int>();
  int do_delta_coding = vm["do_delta_frame_coding"].as<int>();
  int icp_on_original = vm["icp_on_original"].as<int>();
  int macroblocksize= vm["macroblocksize"].as<int>();
  int testbbalign = vm["testbbalign"].as<int>();  // testing the bounding box alignment algorithm
  bool do_icp_color_offset = vm["code_color_off"].as<int>(); 

  if(testbbalign){
    std::cout << " re-alligned " << bb_align_count << " frames out of " << fused_clouds.size() <<" frames" << std::endl;
    return true;
  }
  //////////////////////////////////////////////////////////////////////////

  // base layer resolution
  for(int ct=0; ct < color_coding_types.size();ct++ ){

    vector<float> icp_convergence_percentage(fused_clouds.size()); // field store the percentage of converged macroblocks
    vector<float> shared_macroblock_percentages(fused_clouds.size()); // field to store the percentage of macroblocks shared with the previous frame

    // enh layer resolution
    for(int ob=0; ob < octree_bit_settings.size(); ob++){

      // color resolution
      for(int cb=0; cb < color_bit_settings.size(); cb++){

        // store the parameters in a string to store them in the .csv file
        stringstream compression_arg_ss; 
        compression_arg_ss << octree_bit_settings[ob] << "_"  
          <<  color_bit_settings[cb] 
        << "_colort-" << color_coding_types[ct] << "_centroid-" << (keep_centroid ? "yes" : "no");

        ////////////// ASSESMENT: ENCODE, DECODE AND RECORD THE ACHIEVED QUALITY //////////////////

        // declare codecs outside the mesh iterator loop to test double buffering

        //! encode the fused cloud with and without colors
#if __cplusplus >= 201103L
        auto l_codec_encoder = generatePCLOctreeCodecV2<PointXYZRGB>(
#else
        boost::shared_ptr<OctreePointCloudCodecV2<PointXYZRGB> > l_codec_encoder = generatePCLOctreeCodecV2<PointXYZRGB>(
#endif//__cplusplus < 201103L
          octree_bit_settings[ob],
          enh_bit_settings,
          color_bit_settings[cb],
          0,
          color_coding_types[ct],
          keep_centroid
          );

        // set the macroblocksize for inter prediction
        l_codec_encoder->setMacroblockSize(macroblocksize);

        // initialize structures for decoding base and enhancement layers
#if __cplusplus >= 201103L
        auto l_codec_decoder_base = generatePCLOctreeCodecV2<PointXYZRGB>(
#else
        boost::shared_ptr<OctreePointCloudCodecV2<PointXYZRGB> > l_codec_decoder_base = generatePCLOctreeCodecV2<PointXYZRGB>(
#endif//__cplusplus >= 201103L
          octree_bit_settings[ob],
          enh_bit_settings,
          color_bit_settings[cb],
          0,
          color_coding_types[ct],
          keep_centroid
          );

        for(int i=0; i < fused_clouds.size(); i++)
        {
          // structs for storing the achieved quality
          TicToc tt;
          pcl::quality::QualityMetric achieved_quality;
          pcl::quality::QualityMetric pframe_quality;

          //! full compression into stringstreams, base and enhancement layers, with and without colors
          stringstream l_output_base;

          /////////////////////////////////////////////////////////////
          //! do the encoding
          tt.tic ();
          l_codec_encoder->encodePointCloud(fused_clouds[i] ,l_output_base);
          achieved_quality.encoding_time_ms = tt.toc();
          ////////////////////////////////////////////////////////////

          ////////////////////////////////////////////////////////////////
          // store and display the partial bytes sizes
          uint64_t *c_sizes = l_codec_encoder->getPerformanceMetrics();
          achieved_quality.byte_count_octree_layer = c_sizes[0];
          achieved_quality.byte_count_centroid_layer = c_sizes[1];
          achieved_quality.byte_count_color_layer= c_sizes[2];
          ////////////////////////////////////////////////////////////////

          cout << " octreeCoding " << (achieved_quality.compressed_size=l_output_base.tellp()) << " bytes  base layer  " << endl;
          //////////////////////////////////////////////////////////////

          // start decoding and computing quality metrics
          stringstream oc(l_output_base.str());
          colorOctreeCodec::PointCloudPtr decoded_cloud_base = colorOctreeCodec::PointCloudPtr(new colorOctreeCodec::PointCloud);

          // do the decoding base layer
          cout << "starting decoding the point cloud \n" << endl;
          tt.tic ();
          l_codec_decoder_base->decodePointCloud(oc,decoded_cloud_base);
          achieved_quality.decoding_time_ms = tt.toc ();
          cout << "finished decoding the point cloud \n" << endl;
          // end do the decoding base layer

          // compute quality metric
          computeQualityMetric<pcl::PointXYZRGB>(*fused_clouds[i],*decoded_cloud_base, achieved_quality);

          //////////////////// octree delta frame encoding /////////////////////
          // predicted frame, lossy prediction with artefacts that need to be assessed
          boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > out_d(new pcl::PointCloud<PointXYZRGB>());
          if(do_delta_coding){

            if(i+1 < aligned_flags.size())

              if(aligned_flags[i+1]){ // only do delta coding when frames are aligned
                cout << " delta coding frame nr " << i + 1 << endl;
                if(i < (fused_clouds.size() -1)) 
                {
                  // icp offset coding
                  l_codec_encoder->setDoICPColorOffset(do_icp_color_offset);

                  stringstream p_frame_pdat;
                  stringstream p_frame_idat;
                  // code a delta frame, either use original or simplified cloud for ICP
                  tt.tic();

                  // test function that encodes and generates the predicted frame, commented out as we will use encoder and decoder functions instead
                  //l_codec_encoder->generatePointCloudDeltaFrame(icp_on_original ? fused_clouds[i] : l_codec_encoder->getOutputCloud(),
                  // );
                  boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > out_n(new pcl::PointCloud<PointXYZRGB>());
                  l_codec_encoder->encodePointCloudDeltaFrame (icp_on_original ? fused_clouds[i] : l_codec_encoder->getOutputCloud(),
                    fused_clouds[i+1],out_n, p_frame_idat, p_frame_pdat, (int) icp_on_original,false);

                  pframe_quality.encoding_time_ms = tt.toc();
                  pframe_quality.byte_count_octree_layer = p_frame_idat.tellp();
                  pframe_quality.byte_count_centroid_layer = p_frame_pdat.tellp();
                  pframe_quality.compressed_size = p_frame_idat.tellp() + p_frame_pdat.tellp();
                  pframe_quality.byte_count_color_layer= 0;

                  cout << " encoded a predictive frame: coded " << p_frame_idat.tellp() << " bytes intra and " << p_frame_pdat.tellp()  << " inter frame encoded " <<endl;

                  shared_macroblock_percentages[i+1] = l_codec_encoder->getMacroBlockPercentage();
                  icp_convergence_percentage[i+1] = l_codec_encoder->getMacroBlockConvergencePercentage();

                  // decode the pframe
                  l_codec_encoder->decodePointCloudDeltaFrame(decoded_cloud_base, out_d,p_frame_idat, p_frame_pdat);

                  // compute the quality of the resulting predictive frame
                  computeQualityMetric<pcl::PointXYZRGB>(*fused_clouds[i+1],*out_d, pframe_quality);
                  pframe_quality.print_csv_line(compression_arg_ss.str(), res_p_ofstream);
                }
                // store the quality metrics for the p cloud
              }
          }
          ///////////////////////////////////////////////////////////////////////

          if(write_out_ply )
          {
            // we write output in the reverie mesh format, that allows us to render in reverie for subjective testing
            bool write_rev_format = true;
            if(!write_rev_format ){
              // write the .ply file by converting to point cloud2 and then to polygon mesh
              pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
              pcl::toPCLPointCloud2( *decoded_cloud_base, *cloud2);
              pcl::PLYWriter writer;
              writer.write("ct_" +
                boost::lexical_cast<string>(ct) + 
                "ob_" +
                boost::lexical_cast<string>(ob) + 
                "_cb_" +
                boost::lexical_cast<string>(cb) + 
                "_mesh_nr_" +
                boost::lexical_cast<string>(i) +
                "_out.ply", cloud2
                );
              // end writing .ply
            }
            else{
              ///////////////////////////////////////////////////////
              float *mdat=new float[9*decoded_cloud_base->size()];
              unsigned int *l_triang = new unsigned int[3];

              l_triang[0] = 0;
              l_triang[1] = 1;
              l_triang[2] = 2;

              Eigen::Vector4f l_range_scale =assigned_bbs[i].max_xyz - assigned_bbs[i].min_xyz;

              Nano3D::Mesh m((unsigned int) decoded_cloud_base->size(),mdat,1,l_triang);
              for(int l=0; l<decoded_cloud_base->size();l++)
              {
                mdat[9*l] = (*decoded_cloud_base)[l].x * l_range_scale[0]  + assigned_bbs[i].min_xyz[0];
                mdat[9*l+1] = (*decoded_cloud_base)[l].y * l_range_scale[1] + assigned_bbs[i].min_xyz[1];
                mdat[9*l+2] = (*decoded_cloud_base)[l].z * l_range_scale[2] + assigned_bbs[i].min_xyz[2];
                mdat[9*l+3] = 0;
                mdat[9*l+4] = 0;
                mdat[9*l+5] = 0;
                mdat[9*l+6] = (*decoded_cloud_base)[l].r;
                mdat[9*l+7] = (*decoded_cloud_base)[l].g;
                mdat[9*l+8] = (*decoded_cloud_base)[l].b; 
              }
              //////////////////////////////////////////////////////
              m.recomputeSmoothVertexNormals(.3);
              m.storePLY("rev_ct_" +
                boost::lexical_cast<string>(ct) + 
                "ob_" +
                boost::lexical_cast<string>(ob) + 
                "_cb_" +
                boost::lexical_cast<string>(cb) + 
                "_mesh_nr_" +
                boost::lexical_cast<string>(i) +
                "_out.ply",false);
            }
            // write predictively decoded frames
            if(i+1 < aligned_flags.size())
              if(do_delta_coding && aligned_flags[i+1])
              {
                if(!write_rev_format ){
                  pcl::PCLPointCloud2::Ptr cloud2d(new pcl::PCLPointCloud2());
                  pcl::toPCLPointCloud2( *out_d, *cloud2d);
                  pcl::PLYWriter writer;
                  writer.write("ct_" +
                    boost::lexical_cast<string>(ct) + 
                    "ob_" +
                    boost::lexical_cast<string>(ob) + 
                    "_cb_" +
                    boost::lexical_cast<string>(cb) + 
                    "_mesh_nr_" +
                    boost::lexical_cast<string>(i+1) +
                    "_out_predicted.ply", cloud2d
                    );
                }
                else
                {
                  ///////////////////////////////////////////////////////
                  float *mdat=new float[9*out_d->size()];
                  unsigned int *l_triang = new unsigned int[3];
                  l_triang[0] = 0;
                  l_triang[1] = 1;
                  l_triang[2] = 2;
				  Nano3D::Mesh m((unsigned int) out_d->size(), mdat, 1, l_triang);

                  Eigen::Vector4f l_range_scale =assigned_bbs[i].max_xyz - assigned_bbs[i].min_xyz;

                  for(int l=0; l<out_d->size();l++)
                  {
                    mdat[9*l] = (*out_d)[l].x * l_range_scale[0]  + assigned_bbs[i].min_xyz[0];
                    mdat[9*l+1] = (*out_d)[l].y * l_range_scale[1]  + assigned_bbs[i].min_xyz[1];
                    mdat[9*l+2] = (*out_d)[l].z* l_range_scale[2]  + assigned_bbs[i].min_xyz[2];
                    mdat[9*l+3] = 0;
                    mdat[9*l+4] = 0;
                    mdat[9*l+5] = 0;
                    mdat[9*l+6] = (*out_d)[l].r;
                    mdat[9*l+7] = (*out_d)[l].g;
                    mdat[9*l+8] = (*out_d)[l].b; 
                  }
                  //////////////////////////////////////////////////////
                  m.recomputeSmoothVertexNormals(.3);
                  m.storePLY("prev_ct_" +
                    boost::lexical_cast<string>(ct) + 
                    "ob_" +
                    boost::lexical_cast<string>(ob) + 
                    "_cb_" +
                    boost::lexical_cast<string>(cb) + 
                    "_mesh_nr_" +
                    boost::lexical_cast<string>(i + 1) +
                    "_out_predicted.ply",false);
                }
              }

          }
          // ~ write predictively encoded frames
          // print the evaluation results to the output .cs file
          achieved_quality.print_csv_line(compression_arg_ss.str(), res_base_ofstream);
        }
        ////////////// END ASSESMENT //////////////////
      }
      // report convergence and shared macroblock statistics
      double av_macroblock_sharing=0;
      double av_convergence_percentage=0;
      int p_frame_count =0;
      for(int i=0;i<aligned_flags.size();i++)
      {
        if(aligned_flags[i])
        {
          p_frame_count++;
          av_macroblock_sharing += shared_macroblock_percentages[i];
          av_convergence_percentage+= icp_convergence_percentage[i];
        }
      }
      cout << " overall shared macroblock percentage: " << av_macroblock_sharing / p_frame_count <<" total p frames: "  << p_frame_count <<endl;
      cout << " overall shared macroblock convergence percentage: " << av_convergence_percentage / p_frame_count <<" total p frames: "  << p_frame_count <<endl;
      cin.get();
    }
  }

  ////////////// END FOR //////////////////	
  return (-1);
}