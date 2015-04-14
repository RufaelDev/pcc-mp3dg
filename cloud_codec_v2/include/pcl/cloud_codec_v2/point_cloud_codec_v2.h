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
#ifndef POINT_CLOUD_CODECV2_H
#define POINT_CLOUD_CODECV2_H

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
#include <pcl/cloud_codec_v2/color_coding_jpeg.h>

namespace pcl{

  namespace io{

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**! 
    * \brief class for computation and compression of time varying cloud frames, extends original PCL cloud codec
    * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<typename PointT, typename LeafT = OctreeContainerPointIndices,
      typename BranchT = OctreeContainerEmpty,
      typename OctreeT = Octree2BufBase<LeafT, BranchT> >
    class OctreePointCloudCodecV2 : public OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT>
    {
      public:

        // public typedefs, copied from original implementation by Julius Kammerl
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
       typedef typename OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> > ConstPtr;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        typedef OctreePointCloudCodecV2<PointT, LeafT, BranchT, Octree2BufBase<LeafT, BranchT> > RealTimeStreamCompression;
        typedef OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeBase<LeafT, BranchT> > SinglePointCloudCompressionLowMemory;

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
          bool doVoxelGridCentroid_arg = true, 
          bool createScalebleStream_arg = true, 
          bool codeConnectivity_arg = false,
          int jpeg_quality_arg = 75) :
        OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT>(
          compressionProfile_arg,
          showStatistics_arg,
          pointResolution_arg,
          octreeResolution_arg,
          doVoxelGridDownDownSampling_arg,
          0 /* NO PCL P Frames in this version of the codec !! */,
          doColorEncoding_arg,
          colorBitResolution_arg), 
          color_coding_type_(colorCodingType_arg), 
          jp_color_coder_(jpeg_quality_arg,colorCodingType_arg),
          do_voxel_centroid_enDecoding_(doVoxelGridCentroid_arg),
          create_scalable_bitstream_(createScalebleStream_arg), 
          do_connectivity_encoding_(codeConnectivity_arg)
        {
          i_frame_rate_  = 0;
          macroblock_size = 16; // default macroblock size is 16x16x16
        }

        /** \brief Initialization using the parent
          */
        void initialization ()
        {
        }

        /** \brief function for setting the macroblocksize
          */
        void setMacroblockSize(int size)
        {
          macroblock_size = size;
        }
         /** \brief Encode point cloud to output stream
          * \param cloud_arg:  point cloud to be compressed
          * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
          */
        void
        encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);

        /** \brief Decode point cloud from input stream
          * \param compressed_tree_data_in_arg: binary input stream containing compressed data
          * \param cloud_arg: reference to decoded point cloud
          */
        void
        decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);

        /** \brief generate a point cloud Delta to output stream
        * \param icloud_arg:  point cloud to be used a I frame
        * \param pcloud_arg:  point cloud to be encoded
        * \param PointCloudPtr &out_cloud_arg [out] the predicted frame 
        * \param std::ostream& i_coded_data intra encoded data (not yet implemented)
        * \param std::ostream& p_coded_data inter encoded data (not yet implemented)
        * \param bool  icp_on_original 
        * \param bool  write_out_cloud  flag to write the output cloud to out_cloud_arg
        */
        virtual void
        generatePointCloudDeltaFrame (const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg, 
            std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original = false,bool write_out_cloud = true );

        /** \brief Encode point cloud Delta to output stream (not yet implemented)
        */
        virtual void
        encodePointCloudDeltaFrame (const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg, 
        std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original = false,bool write_out_cloud = true ){};

        /** \brief Decode point cloud Delta to output stream (not yet implemented)
        */
        virtual void
        decodePointCloudDeltaFrame(const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, 
        std::istream& i_coded_data, std::istream& p_coded_data);
        /*!
        \brief function to simplify the delta frame to do a prediction
        \author Rufael Mekuria rufael.mekuria@cwi.nl
        */
        virtual void
        simplifyPCloud(const PointCloudConstPtr &pcloud_arg_in, 
          OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> *octree_simplifier,
          PointCloudPtr &out_cloud );
      
        uint64_t *
        getPerformanceMetrics()
        {
          return compression_performance_metrics;
        }
      protected: 

        //! write frame header for CodecV2
        void
        writeFrameHeader(std::ostream& compressed_tree_data_out_arg);

        //! read Frame header for CodecV2
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
        entropyEncoding (std::ostream& compressed_tree_data_out_arg1, 
          std::ostream& compressed_tree_data_out_arg2=stringstream());

        /** \brief Entropy decoding of input binary stream and output to information vectors
        * \param compressed_tree_data_in_arg: binary input stream: base layer
        * \param compressed_tree_data_in_arg: binary input stream: enhancement layer
        */
        void
        entropyDecoding (std::istream& compressed_tree_data_in_arg1, 
          std::istream& compressed_tree_data_in_arg2=stringstream());

        //! some new cool options in v2 of the codec 
        uint32_t color_coding_type_; //! color coding with jpeg, graph transform, or differential encodings

        bool do_voxel_centroid_enDecoding_;  //! encode the centroid in addition

        bool create_scalable_bitstream_;  //! create a scalable bitstream

        bool do_connectivity_encoding_;   //! encode the connectivity

        PointCodingV2<PointT> centroid_coder_; //! centroid encoding

        ColorCodingJPEG<PointT> jp_color_coder_; //! new color coding

        uint64_t compression_performance_metrics[3]; //! octree_bytes, centroid_bytes, color_bytes, monitor the buildup of compressed data

        static const char* frame_header_identifier_; //! new frame header identifier

        int macroblock_size;  //! macroblock size for inter predictive frames
    };

    // define frame identifier for cloud codec v2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
    const char* OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::frame_header_identifier_ = "<PCL-OCT-CODECV2-COMPRESSED>";
  }
}
#endif