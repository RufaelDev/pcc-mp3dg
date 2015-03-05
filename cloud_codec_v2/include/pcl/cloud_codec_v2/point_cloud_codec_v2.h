/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014, Stichting Centrum Wiskunde en Informatica.
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
#ifndef POINT_CLOUD_DELTA_FRAME_H
#define POINT_CLOUD_DELTA_FRAME_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/impl/octree_iterator.hpp>
#include <pcl/cloud_codec_v2/point_coding_v2.h>

namespace pcl{

  namespace io{

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /**! 
  * class for computation and compression of time varying cloud frames
  * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
  */
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename PointT, typename LeafT = OctreeContainerPointIndices,
  typename BranchT = OctreeContainerEmpty,
  typename OctreeT = Octree2BufBase<LeafT, BranchT> >
  class OctreePointCloudCodecV2 : public OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT>
  {
  public:
			
	  /** \brief Constructor
	  * \param compressionProfile_arg:  define compression profile
	  * \param octreeResolution_arg:  octree resolution at lowest octree level
	  * \param pointResolution_arg:  precision of point coordinates
	  * \param doVoxelGridDownDownSampling_arg:  voxel grid filtering
	  * \param iFrameRate_arg:  i-frame encoding rate
	  * \param doColorEncoding_arg:  enable/disable color coding
	  * \param colorBitResolution_arg:  color bit depth
	  * \param showStatistics_arg:  output compression statistics
	  * \param colorCodingType_arg:  jpeg or pcl dpcm
	  * \param doVoxelGridCentroid_arg:  keep voxel grid positions or not 
	  */
      OctreePointCloudCodecV2 (compression_Profiles_e compressionProfile_arg = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
									bool showStatistics_arg = false,
									const double pointResolution_arg = 0.001,
									const double octreeResolution_arg = 0.01,
									bool doVoxelGridDownDownSampling_arg = false,
									const unsigned int iFrameRate_arg = 30,
									bool doColorEncoding_arg = true,
									const unsigned char colorBitResolution_arg = 6,
									const unsigned char colorCodingType_arg = 0,
									bool doVoxelGridCentroid_arg = true) :
	  OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT>(compressionProfile_arg,
									showStatistics_arg,
									pointResolution_arg,
									octreeResolution_arg,
									doVoxelGridDownDownSampling_arg,
									iFrameRate_arg,
									doColorEncoding_arg,
									colorBitResolution_arg )
	  {
	  }
		
	  /** \brief Encode point cloud Delta to output stream
	  * \param cloud_arg:  point cloud to be compressed
	  * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
	  */
	  virtual void
	  encodePointCloudDeltaFrame (const PointCloudConstPtr &dcloud_arg, std::ostream& compressed_tree_data_out_arg1, std::ostream& compressed_tree_data_out_arg2);

	  /** \brief Decode point cloud Delta from input stream
	  * \param compressed_tree_data_in_arg: binary input stream containing compressed data
	  * \param cloud_arg: reference to decoded point cloud
	  */
	  virtual void
	  decodePointCloudDeltaFrame (const PointCloudConstPtr &icloud_arg,std::istream& compressed_tree_data_in_arg1, std::istream& compressed_tree_data_out_arg2 , PointCloudPtr &cloud_arg);

	  /*!
	  \brief function to simplify the delta frame
	  \author Rufael Mekuria rufael.mekuria@cwi.nl
	  */
	  virtual void
	  simplifyPCloud(const PointCloudConstPtr &pcloud_arg_in,PointCloudPtr & out_cloud );

    protected: 
      
	  //! centroid encoding
      PointCodingV2<PointT> centroid_coder_;

	  //! write frame header
	  void
	  writeFrameHeader(std::ostream& compressed_tree_data_out_arg);

      //! read Frame header
	  void
	  readFrameHeader(std::istream& compressed_tree_data_in_arg);

	  /** \brief Encode leaf node information during serialization
      * \param leaf_arg: reference to new leaf node
      * \param key_arg: octree key of new leaf node
      */
      virtual void
      serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);

      /** \brief Decode leaf nodes information during deserialization
      * \param key_arg octree key of new leaf node
      */
      
	  // param leaf_arg reference to new leaf node
      virtual void 
      deserializeTreeCallback (LeafT&, const OctreeKey& key_arg);

      /** \brief Synchronize to frame header
      * \param compressed_tree_data_in_arg: binary input stream
      */
      void
      syncToHeader (std::istream& compressed_tree_data_in_arg);

	  /** \brief Apply entropy encoding to encoded information and output to binary stream
      * \param compressed_tree_data_out_arg: binary output stream: base layer
	  * \param compressed_tree_data_out_arg: binary output streams: enhancement layer
      */
      void
      entropyEncoding (std::ostream& compressed_tree_data_out_arg1, std::ostream& compressed_tree_data_out_arg2 =stringstream()  );

      /** \brief Entropy decoding of input binary stream and output to information vectors
      * \param compressed_tree_data_in_arg: binary input stream: base layer
	  * \param compressed_tree_data_in_arg: binary input stream: enhancement layer
      */
      void
      entropyDecoding (std::istream& compressed_tree_data_in_arg1, std::istream& compressed_tree_data_in_arg2=stringstream() );

      //! some new cool options in v2 of the codec 
      bool do_voxel_centroid_enDecoding_;  //! encode the centroid in addition

      bool do_jpeg_color_encoding_;     //! use a jpeg color coding scheme 

      bool do_graph_transform_encoding; //! encode colors based on graph transform

	  bool do_connectivity_encoding_;   //! encode the connectivity

	  bool create_scalable_bitstream_;  //! create a scalable bitstream

	  //! color coding with jpeg, graph transform, or differential encodings
      int color_coding_type_;

      //! octree_bytes, centroid_bytes, color_bytes, monitor the buildup of compressed data
      uint64_t compression_performance_metrics[3];

	  // frame header identifier
      static const char* frame_header_identifier_;
 	};
    
	
	// define frame identifier for cloud codec v2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
      const char* OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::frame_header_identifier_ = "<PCL-OCT-CODECV2-COMPRESSED>";
  }
}
#endif