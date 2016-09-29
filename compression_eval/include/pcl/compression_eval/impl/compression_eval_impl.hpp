/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014- Centrum Wiskunde en Informatica
*
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
#ifndef COMPRESSION_EVAL_IMPL_HPP
#define COMPRESSION_EVAL_IMPL_HPP

#include <pcl/compression_eval/compression_eval.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>

namespace pcl{

  namespace io{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**! 
    * function to generate octree codec with bit settings
    * @param nr_bits_base_layer an integer argument representing the number of bits for encoding the base layer (octree).
    * @param nr_bits_enh_layer an integer argument representing the number of bits for encoding the  enhancement layer (octree).
    * @param nr_bits_colors an integer argument representing the number of bits for encoding the colors per point.
	* @param i_frame_rate an integer controlling the framerate of iframes
	* @param color_coding_type an integer specifying the color coding type to be used (pcl=0,jpeg=1)
	* @param do_centroid_coding a boolean indicating whether or not centroid coding is toe be used
	* \note PointT typename of point used in point cloud
    * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> boost::shared_ptr<OctreePointCloudCodecV2<PointT> >
    generatePCLOctreeCodecV2(int nr_bits_base_layer, int nr_bits_enh_layer, int nr_bits_colors, int i_frame_rate, int color_coding_type, bool do_centroid_coding, bool scalable_arg, bool conn_arg, int jpeg_value,int num_threads)
    {
      return num_threads > 0 ? 
        boost::shared_ptr<OctreePointCloudCodecV2OMP<PointT> >(new OctreePointCloudCodecV2OMP<PointT>(
        MANUAL_CONFIGURATION,
        false,
        std::pow( 2.0, -1.0 *(nr_bits_base_layer + nr_bits_enh_layer) ),
        std::pow( 2.0, -1.0 *(nr_bits_base_layer)),
        true /* no intra voxel coding in this first version of the codec ok */,
        i_frame_rate,
        nr_bits_colors ? true : false,
        nr_bits_colors,
        color_coding_type,
        do_centroid_coding ,
        scalable_arg,
        conn_arg,
        jpeg_value,
        num_threads
        ))
        : boost::shared_ptr<OctreePointCloudCodecV2<PointT> >(new OctreePointCloudCodecV2<PointT>(
        MANUAL_CONFIGURATION,
        false,
        std::pow( 2.0, -1.0 *(nr_bits_base_layer + nr_bits_enh_layer) ),
        std::pow( 2.0, -1.0 *(nr_bits_base_layer)),
        true /* no intra voxel coding in this first version of the codec ok */,
        i_frame_rate,
        nr_bits_colors ? true : false,
        nr_bits_colors,
        color_coding_type,
        do_centroid_coding ,
        scalable_arg,
        conn_arg,
        jpeg_value
        ));
    }

    // function to log occupancy codes frequencies
    void
    logOccupancyCodesFrequencies(std::vector<std::vector<char> > & occupancy_codes,
    std::ostream &output_file)
    {
       // iterate each of the levels 
      for(int k=0; k < occupancy_codes.size(); k++)
      {
        // create the frequency table
        unsigned int freq_table[256]={};
        
        for(int l=0; l < occupancy_codes[k].size(); l++)
        {
          // increment the entry in the frequency table 
          freq_table[(unsigned char) occupancy_codes[k][l]]++;;
        }

        // write the frequency table to the .csv file
        output_file << k << ";";

        for(int l=0; l < 256; l++)
        {
          // 
          output_file << (unsigned int) freq_table[l] << ";";
        }

        output_file << std::endl;
      }
    }

	// class for running the MPEG Compression eval testbench
	class PCL_EXPORTS CompressionEval {
		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**!
		* \struct to store meshes metadata for official evaluation by MPEG committee
		* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
		*/
		////////////////////////////////////////////////////////////////////////////////////////////////////////////
	    public:
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
				: has_coords(true), has_normals(false), has_colors(false), has_texts(false), has_conn(false)
			{}
		};

		struct bounding_box
		{
			Eigen::Vector4f min_xyz;
			Eigen::Vector4f max_xyz;
		};


	    public:
		  ///////////////  Bounding Box Logging /////////////////////////	
		  // log information on the bounding boxes, which is critical for alligning clouds in time
		  ofstream bb_out;
		  Eigen::Vector4f min_pt_bb;
		  Eigen::Vector4f max_pt_bb;
		  bool is_bb_init;
		  double bb_expand_factor;
		// default constructor
		CompressionEval()
			  : bb_out("bounding_box_pre_mesh.txt"), is_bb_init(false), bb_expand_factor(0.10)
		{};

		int
		printHelp(int, char **argv);

		bool
		loadPLYMesh(const string &filename, pcl::PolygonMesh &mesh);

        bool
        loadPLYFolder(const string &folder_name,
          vector<pcl::PolygonMesh> &meshes,
          vector<compression_eval_mesh_meta_data> &meshes_meta_data);

        int 
		run(int argc, char** argv);

		int
	    loadConfig(void);

		int
		loadClouds(int argc, char** argv);

		int
	    fuseClouds();

		int
	    preFilterClouds();

		int
		allignBBClouds();

		int
		run_eval(int argc,char **argv);

	    protected: 

         ////////////// configuration  //////////////////
		 boost::program_options::variables_map vm;
		 boost::program_options::options_description desc;
		 int enh_bit_settings;
		 vector<int> octree_bit_settings;
		 vector<int> color_bit_settings;
         vector<int> color_coding_types;
		 bool keep_centroid;
		 int write_out_ply;
		 int do_delta_coding;
		 int icp_on_original;
		 int macroblocksize;
		 int testbbalign;  // testing the bounding box alignment algorithm
		 bool do_icp_color_offset;
		 int do_radius_align;
		 double rad_size;
		 bool create_scalable;
		 int jpeg_value;
		 int omp_cores;

		 ////////////// clouds //////////////////
		 vector<int> ply_folder_indices;

		 // store all loaded meshes in a vector and store all metadata separately (optional)
		 vector<vector<pcl::PolygonMesh> > meshes;
		 vector<vector<compression_eval_mesh_meta_data> > meshes_meta_data;

		 // data structures for storing the fused meshes
		 vector<boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > > fused_clouds;
		 vector<compression_eval_mesh_meta_data> fused_clouds_meta_data;

		 // bb allign
		 int bb_align_count;
		 std::vector<bool> aligned_flags;
		 std::vector<bounding_box> assigned_bbs;

		 // stream statistics
		 ofstream res_p_ofstream;
		 ofstream res_base_ofstream;
	     ofstream res_enh_ofstream;

		 // bounding box is expanded
		 Eigen::Vector4f min_pt_res;
		 Eigen::Vector4f max_pt_res;
	};
  }
}



#endif // COMPRESSION_EVAL_IMPL_HPP

