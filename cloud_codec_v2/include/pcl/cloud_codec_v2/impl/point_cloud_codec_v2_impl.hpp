/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2009-2012, Willow Garage, Inc.
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
#ifndef POINT_CLOUD_CODECV2_IMPL_HPP
#define POINT_CLOUD_CODECV2_IMPL_HPP

// point cloud compression from PCL
#include <pcl/compression/entropy_range_coder.h>
#include <pcl/compression/impl/entropy_range_coder.hpp>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
#include <pcl/compression/point_coding.h>
#include <pcl/compression/impl/octree_pointcloud_compression.hpp>

//includes to do the ICP procedures
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <Eigen/geometry>
#include <pcl/cloud_codec_v2/impl/rigid_transform_coding_impl.hpp>
#include <eigen/geometry>

namespace pcl{

  namespace io{

    /// encoding routine, based on the PCL octree codec written by Julius Kammerl
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void OctreePointCloudCodecV2<
        PointT, LeafT, BranchT, OctreeT>::encodePointCloud (
        const PointCloudConstPtr &cloud_arg,
        std::ostream& compressed_tree_data_out_arg)
    {
      unsigned char recent_tree_depth =
          static_cast<unsigned char> (getTreeDepth ());

      // CWI addition to prevent crashes as in original cloud codec
      deleteCurrentBuffer();
      deleteTree();

      // initialize octree
      setInputCloud (cloud_arg);

      // CWI added, when encoding hte output variable stores the (simplified icloud)
      output_ = PointCloudPtr(new PointCloud());

      // add point to octree
      addPointsFromInputCloud ();

      // make sure cloud contains points
      if (leaf_count_>0) {


        // color field analysis
        cloud_with_color_ = false;
        std::vector<pcl::PCLPointField> fields;
        int rgba_index = -1;
        rgba_index = pcl::getFieldIndex (*input_, "rgb", fields);
        if (rgba_index == -1)
        {
          rgba_index = pcl::getFieldIndex (*input_, "rgba", fields);
        }
        if (rgba_index >= 0)
        {
          point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
          cloud_with_color_ = true;
        }

        // apply encoding configuration
		cloud_with_color_ &= do_color_encoding_;

        // if octree depth changed, we enforce I-frame encoding
        i_frame_ |= (recent_tree_depth != getTreeDepth ());// | !(iFrameCounter%10);

        // enable I-frame rate
        if (i_frame_counter_++==i_frame_rate_)
        {
          i_frame_counter_ =0;
          i_frame_ = true;
        }

        // increase frameID
        frame_ID_++;

        // do octree encoding
        if (!do_voxel_grid_enDecoding_)
        {
          point_count_data_vector_.clear ();
          point_count_data_vector_.reserve (cloud_arg->points.size ());
        }

        // initialize color encoding
        if(!color_coding_type_){
          color_coder_.initializeEncoding ();
          color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
          color_coder_.setVoxelCount (static_cast<unsigned int> (leaf_count_));
        }else
        {
          jp_color_coder_.initializeEncoding ();
          jp_color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
          jp_color_coder_.setVoxelCount (static_cast<unsigned int> (leaf_count_));
        }
        // initialize point encoding
        point_coder_.initializeEncoding ();
        centroid_coder_.initializeEncoding ();
        point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
        centroid_coder_.initializeEncoding ();
        centroid_coder_.setPointCount( static_cast<unsigned int> (object_count_));

        // serialize octree
        if (i_frame_)
          // i-frame encoding - encode tree structure without referencing previous buffer
          serializeTree (binary_tree_data_vector_, false);
        else
          // p-frame encoding - XOR encoded tree structure
          serializeTree (binary_tree_data_vector_, true);


        // write frame header information to stream
        writeFrameHeader (compressed_tree_data_out_arg);

        // apply entropy coding to the content of all data vectors and send data to output stream
		entropyEncoding(compressed_tree_data_out_arg, compressed_tree_data_out_arg);

        // prepare for next frame
        switchBuffers ();
        i_frame_ = false;

        // reset object count
        object_count_ = 0;

        if (b_show_statistics_)
        {
          float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
          float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);

          PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
          PCL_INFO ("Frame ID: %d\n", frame_ID_);
          if (i_frame_)
            PCL_INFO ("Encoding Frame: Intra frame\n");
          else
            PCL_INFO ("Encoding Frame: Prediction frame\n");
          PCL_INFO ("Number of encoded points: %ld\n", point_count_);
          PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
          PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
          PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
          PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f  * sizeof (float)) / 1024);
          PCL_INFO ("Size of compressed point cloud: %d kBytes\n", (compressed_point_data_len_ + compressed_color_data_len_) / (1024));
          PCL_INFO ("Total bytes per point: %f\n", bytes_per_XYZ + bytes_per_color);
          PCL_INFO ("Total compression percentage: %f\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
        }
      } else {
        if (b_show_statistics_)
        PCL_INFO ("Info: Dropping empty point cloud\n");
        deleteTree();
        i_frame_counter_ = 0;
        i_frame_ = true;
      }
    }

     /// decoding routine, based on the PCL octree codec written by Julius Kammerl
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::decodePointCloud (
        std::istream& compressed_tree_data_in_arg,
        PointCloudPtr &cloud_arg)
    {

      // synchronize to frame header
      syncToHeader(compressed_tree_data_in_arg);

      // initialize octree
      switchBuffers ();
      
      // added to prevent crashes as happens with original cloud codec
	    deleteCurrentBuffer();
	    deleteTree();

      setOutputCloud (cloud_arg);

      // color field analysis
      cloud_with_color_ = false;
      std::vector<pcl::PCLPointField> fields;
      int rgba_index = -1;
      rgba_index = pcl::getFieldIndex (*output_, "rgb", fields);
      if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex (*output_, "rgba", fields);
      if (rgba_index >= 0)
      {
        point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
        cloud_with_color_ = true;
      }

      // read header from input stream
      readFrameHeader (compressed_tree_data_in_arg);

      // set the right grid pattern to the JPEG coder
      jp_color_coder_ = ColorCodingJPEG<PointT>(75,color_coding_type_);

      // decode data vectors from stream
	  entropyDecoding(compressed_tree_data_in_arg, compressed_tree_data_in_arg);

      // initialize color and point encoding
      if(!color_coding_type_)
        color_coder_.initializeDecoding ();
      else
        jp_color_coder_.initializeDecoding ();
      
      point_coder_.initializeDecoding ();
      centroid_coder_.initializeDecoding();

      // initialize output cloud
      output_->points.clear ();
      output_->points.reserve (static_cast<std::size_t> (point_count_));

      if (i_frame_)
        // i-frame decoding - decode tree structure without referencing previous buffer
        deserializeTree (binary_tree_data_vector_, false);
      else
        // p-frame decoding - decode XOR encoded tree structure
        deserializeTree (binary_tree_data_vector_, true);

      // assign point cloud properties
      output_->height = 1;
      output_->width = static_cast<uint32_t> (cloud_arg->points.size ());
      output_->is_dense = false;

      //! todo update fo cloud codecV2
      if (b_show_statistics_)
      {
        float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
        float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);

        PCL_INFO ("*** POINTCLOUDV2 DECODING ***\n");
        PCL_INFO ("Frame ID: %d\n", frame_ID_);
        if (i_frame_)
          PCL_INFO ("Encoding Frame: Intra frame\n");
        else
          PCL_INFO ("Encoding Frame: Prediction frame\n");
        PCL_INFO ("Number of encoded points: %ld\n", point_count_);
        PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
        PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
        PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
        PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
        PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
        PCL_INFO ("Total bytes per point: %d bytes\n", static_cast<int> (bytes_per_XYZ + bytes_per_color));
        PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
      }
    }

    /*!
    \brief  helper function to compute the delta frames
    \author Rufael Mekuria rufael.mekuria@cwi.nl
    \param  const PointCloudConstPtr &pcloud_arg_in  input argument, cloud to simplify
    \param  OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> *octree_simplifier, resulting octree datastructure that can be used for compression of the octree
    \param  out_cloud  output simplified point cloud
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::simplifyPCloud(const PointCloudConstPtr &pcloud_arg_in, 
                                                                            PointCloudPtr &out_cloud )
    {
      // this is the octree coding part of the predictive encoding
      OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> *octree_simplifier = new OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT>
      (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_,
        true,
        0,
        true,
        color_bit_resolution_ /*,0,do_voxel_centroid_enDecoding_*/
      );

      // use bounding box and obtain the octree grid
      octree_simplifier->defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      octree_simplifier->setInputCloud (pcloud_arg_in);
      octree_simplifier->addPointsFromInputCloud ();

      /////////////////////////////////////////////////////////////////////////
      //! initialize output cloud
      //PointCloudPtr out_cloud(new PointCloud( octree_simplifier.leaf_count_, 1));
	    out_cloud->width = (uint32_t) octree_simplifier->getLeafCount();
      out_cloud->height = 1;
      out_cloud->points.reserve(octree_simplifier->getLeafCount());  
      // cant get access to the number of leafs
      ////////////////////////////////////////////////////////////////////////////

      ///////////// compute the simplified cloud by iterating the octree ////////
      octree::OctreeLeafNodeIterator<OctreeT> it_ = octree_simplifier->leaf_begin();
      octree::OctreeLeafNodeIterator<OctreeT> it_end = octree_simplifier->leaf_end();

      for(int l_index =0;it_ !=it_end; it_++, l_index++)
      {
        // new point for the simplified cloud
        PointT l_new_point;
        
        //! centroid for storage
        std::vector<int>& point_indices = it_.getLeafContainer().getPointIndicesVector();
        
        // if centroid coding, store centroid, otherwise add 
        if(!do_voxel_centroid_enDecoding_)
        {
          octree_simplifier->genLeafNodeCenterFromOctreeKey(it_.getCurrentOctreeKey(),l_new_point);
        }
        else
        {
          Eigen::Vector4f cent;
          pcl::compute3DCentroid<PointT>(*pcloud_arg_in, point_indices, cent);

          l_new_point.x = cent[0];
          l_new_point.y = cent[1];
          l_new_point.z = cent[2];
        }
        long color_r=0;
        long color_g=0;
        long color_b=0;

        //! compute average color
        for(int i=0; i< point_indices.size();i++)
        {
          color_r+=pcloud_arg_in->points[point_indices[i]].r;
          color_g+=pcloud_arg_in->points[point_indices[i]].g;
          color_b+=pcloud_arg_in->points[point_indices[i]].b;
        }

        l_new_point.r = (char) (color_r / point_indices.size());
        l_new_point.g = (char) (color_g / point_indices.size());
        l_new_point.b = (char) (color_b / point_indices.size());
     
        out_cloud->points.push_back(l_new_point);
      }
      //////////////// done computing simplified cloud and octree structure //////////////////
      delete octree_simplifier;
    };

    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> OctreePointCloudCompression<PointT,LeafT,BranchT,OctreeT> *  
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::generate_macroblock_tree(PointCloudConstPtr  in_cloud)
    {
       MacroBlockTree *tree = new MacroBlockTree
      (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_ * macroblock_size_,
        true,
        0,
        true,
        color_bit_resolution_
        /*,0,do_voxel_centroid_enDecoding_*/
      );

      // I frame coder
      tree->defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      tree->setInputCloud (in_cloud); /* assume right argument was given, either the original p cloud or an i cloud */
      tree->addPointsFromInputCloud ();
      return tree;
    }

    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::do_icp_prediction(
      PointCloudPtr i_cloud,
      PointCloudPtr p_cloud,
      Eigen::Matrix4f &rigid_transform,
      bool & has_converged,
      char *rgb_offsets
      )
    {
      // check on the point count
      bool do_icp =  p_cloud->size() > 6 ? 
        (p_cloud->size() < i_cloud->size() * 2) 
        &&  (p_cloud->size() >= i_cloud->size() * 0.5) : false;

      if(!do_icp)
      {
        has_converged = false;
        return;
      }

      double in_av[3]={0,0,0};
      double out_av[3]={0,0,0};
      double in_var=0;
      double out_var=0;

      // compute the average colors
      if(do_icp)
      {
        // create references to ease acccess via [] operator in the cloud
        pcl::PointCloud<PointT> & rcloud_out = *p_cloud;
        pcl::PointCloud<PointT> & rcloud_in = *i_cloud;

        for(int i=0; i<rcloud_in.size();i++)
        {
          in_av[0]+= (double) rcloud_in[i].r;
          in_av[1]+= (double) rcloud_in[i].g;
          in_av[2]+= (double) rcloud_in[i].b;
        }

        in_av[0]/=rcloud_in.size();
        in_av[1]/=rcloud_in.size();
        in_av[2]/=rcloud_in.size();

        // variance
        for(int i=0; i<rcloud_in.size();i++)
        {
          double val= (rcloud_in[i].r - in_av[0]) * (rcloud_in[i].r - in_av[0]) + 
            (rcloud_in[i].g - in_av[1]) * (rcloud_in[i].g - in_av[1]) +
            (rcloud_in[i].b - in_av[2]) * (rcloud_in[i].b - in_av[2]);

          in_var+=val;
        }
        in_var/=(3*rcloud_in.size());

        //
        for(int i=0; i<rcloud_out.size();i++)
        {
          out_av[0]+= (double) rcloud_out[i].r;
          out_av[1]+= (double) rcloud_out[i].g;
          out_av[2]+= (double) rcloud_out[i].b;
        }
        out_av[0]/=rcloud_out.size();
        out_av[1]/=rcloud_out.size();
        out_av[2]/=rcloud_out.size();

        for(int i=0; i<rcloud_out.size();i++)
        {
          double val= (rcloud_out[i].r - out_av[0]) * (rcloud_out[i].r - out_av[0]) + 
            (rcloud_out[i].g - out_av[1]) * (rcloud_out[i].g - out_av[1]) +
            (rcloud_out[i].b - out_av[2]) * (rcloud_out[i].b - out_av[2]);

          out_var+=val;
        }
        out_var/=(3*rcloud_out.size());

        // for segments with large variance, skip the prediction
        if(in_var > icp_var_threshold_ || out_var > icp_var_threshold_)
          do_icp = false;
      }


      char &rgb_offset_r=rgb_offsets[0]; 
      char &rgb_offset_g=rgb_offsets[1]; 
      char &rgb_offset_b=rgb_offsets[2]; 

      if(do_icp_color_offset_){
        if(std::abs(out_av[0] - in_av[0]) < 32)
          rgb_offset_r = (char)(out_av[0] - in_av[0]);
        if(std::abs(out_av[1] - in_av[1]) < 32)
          rgb_offset_g = (char)(out_av[1] - in_av[1]);
        if(std::abs(out_av[2] - in_av[2]) < 32)
          rgb_offset_b = (char)(out_av[2] - in_av[2]);
      }

      if(!do_icp)
      {
        has_converged = false;
        return;
      }

      // do the actual icp transformation
      pcl::IterativeClosestPoint<PointT, PointT> icp;

      icp.setInputSource(i_cloud);
      icp.setInputTarget(p_cloud);

      icp.setMaximumIterations (icp_max_iterations_);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (icp_max_iterations_);
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (transformationepsilon_);

      pcl::PointCloud<PointT> Final;
      icp.align(Final);

      // compute the fitness for the colors

      if(icp.hasConverged() && icp.getFitnessScore() < point_resolution_ * 4 )
      {
        has_converged = true;
        rigid_transform = icp.getFinalTransformation();
        return;
      }
      has_converged = false;
    }

    /** \brief generate a point cloud Delta to output stream
        * \param icloud_arg:  point cloud to be used a I frame
        * \param pcloud_arg:  point cloud to be encoded as a pframe
        * \param PointCloudPtr &out_cloud_arg [out] the predicted frame 
        * \param std::ostream& i_coded_data intra encoded data 
        * \param std::ostream& p_coded_data inter encoded data
        * \param bool  icp_on_original 
        * \param bool  write_out_cloud  flag to write the output cloud to out_cloud_arg
        */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void 
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT>::generatePointCloudDeltaFrame( 
      const PointCloudConstPtr &icloud_arg /* icloud is already stored but can also be given as argument */,
      const PointCloudConstPtr &pcloud_arg,
      PointCloudPtr &out_cloud_arg, /* write the output cloud so we can assess the quality resulting from this algorithm */
      std::ostream& i_coded_data, 
      std::ostream& p_coded_data, 
      bool icp_on_original,
      bool write_out_cloud)
    {
      // intra coded points storage (points that cannot be predicted)
      typename pcl::PointCloud<PointT>::Ptr intra_coded_points(new pcl::PointCloud<PointT>());

      // initialize predictive cloud
      PointCloudPtr simp_pcloud(new PointCloud());
      if(!icp_on_original){
        simplifyPCloud(pcloud_arg, simp_pcloud);
      }

      // initialize output cloud
      out_cloud_arg->height=1;
      out_cloud_arg->width =0;

      // generate the octree for the macroblocks
      MacroBlockTree * i_block_tree = generate_macroblock_tree(icloud_arg);
      MacroBlockTree * p_block_tree = generate_macroblock_tree(icp_on_original ? pcloud_arg:simp_pcloud);
      
      // iterate the predictive frame and find common macro blocks /////////////
      octree::OctreeLeafNodeIterator<OctreeT> it_predictive = p_block_tree->leaf_begin();
      octree::OctreeLeafNodeIterator<OctreeT> it_predictive_end = p_block_tree->leaf_end();

      for(;it_predictive!=it_predictive_end;++it_predictive)
      {
        const octree::OctreeKey current_key = it_predictive.getCurrentOctreeKey();
        pcl::octree::OctreeContainerPointIndices *i_leaf;
        typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud , it_predictive.getLeafContainer().getPointIndicesVector()));

        if(i_leaf = i_block_tree->findLeaf(current_key.x,current_key.y,current_key.z))
        {
          typename pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector()));
          
          // shared block, do icp
          bool icp_success=false;
          Eigen::Matrix4f rt;
          char rbg_offsets[3]={0,0,0};

          do_icp_prediction(
            cloud_in,
            cloud_out,
            rt,
            icp_success,
            rbg_offsets
          );
          
          if(icp_success)
          {
            // icp success, encode the rigid transform
            std::vector<int16_t> comp_dat;
            Eigen::Quaternion<float> l_quat_out;
            RigidTransformCoding<float>::compressRigidTransform(rt,comp_dat,l_quat_out);
            
            // write octree key, write rigid transform
            int16_t l_key_dat[3]={0,0,0};
            l_key_dat[0] = (int) current_key.x; 
            l_key_dat[1] = (int) current_key.y; 
            l_key_dat[2] = (int) current_key.z;
            
            // write the p coded data (we can add entropy encoding later)
            p_coded_data.write((const char *) l_key_dat ,3*sizeof(int16_t));
            p_coded_data.write((const char *) &comp_dat[0] ,comp_dat.size()*sizeof(int16_t));
            
            if(do_icp_color_offset_)
              p_coded_data.write((const char *) &rbg_offsets[0] ,3*sizeof(char));
             
            // following code is for generation of predicted frame
            Eigen::Matrix4f mdec;
            Eigen::Quaternion<float> l_quat_out_dec;
            RigidTransformCoding<float>::deCompressRigidTransform(comp_dat, mdec,l_quat_out_dec);

            bool corrected_tf_matrix=false;
            for(int i=0; i<16;i++)
            {
              float diff=0;
               if((diff=std::abs(rt(i/4,i%4) - mdec(i/4,i%4))) > 0.01){
                //mdec(i/4,i%4) = rt(i/4,i%4);
                corrected_tf_matrix=true;
                std::cout << " error decoding rigid transform "
                  << comp_dat.size() << " index " << i/4 << "," 
                  << i%4 << std::endl;
                  std::cout << " original " << rt << std::endl;
                  std::cout << " decoded " <<  mdec << std::endl;
                std::cin.get();
               }
            }
            if(corrected_tf_matrix)
              std::cout << " matrix decoded not ok " <<std::endl;
            else
              std::cout << " matrix decoded ok " <<std::endl;

            pcl::PointCloud<PointT> manual_final;
            transformPointCloud<PointT, float>
            (  *cloud_in,
               manual_final, 
               mdec
            );

            // generate the output points
            for(int i=0; i < manual_final.size();i++){
              
              PointT &pt = manual_final[i];
              
              // color offset
              if(do_icp_color_offset_){
                pt.r+=rbg_offsets[0];
                pt.g+=rbg_offsets[1];
                pt.b+=rbg_offsets[2];
              }
              
              out_cloud_arg->push_back(pt);
            }
          }
          else
          {
            // icp failed
            // add to intra coded points
            for(int i=0; i < cloud_out->size();i++){
              out_cloud_arg->push_back((*cloud_out)[i]);
              intra_coded_points->push_back((*cloud_out)[i]);
            }
          }
        }
        else
        {
          // exclusive block
          // add to intra coded points
          for(int i=0; i < cloud_out->size();i++){
            out_cloud_arg->push_back((*cloud_out)[i]);
            intra_coded_points->push_back((*cloud_out)[i]);
          }
        }
      }  
      /* encode all the points that could not be predicted 
       from the previous coded frame in i frame manner with cluod_codec_v2 */
      OctreePointCloudCodecV2<PointT,LeafT,BranchT,OctreeT> intra_coder 
      (
        MANUAL_CONFIGURATION,
        false,
        point_resolution_,
        octree_resolution_,
        true,
        0,
        true,
        color_bit_resolution_,
        color_coding_type_, 
        do_voxel_centroid_enDecoding_
      );
      intra_coder.defineBoundingBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
      intra_coder.encodePointCloud(intra_coded_points, i_coded_data);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::decodePointCloudDeltaFrame( 
          const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, 
          std::istream& i_coded_data, std::istream& p_coded_data)
    {}

    /////////////////// CODE OVERWRITING CODEC V1 to become codec V2 ////////////////////////////////

    //! write frame header, extended for cloud codecV2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader(std::ostream& compressed_tree_data_out_arg)
    {
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier_), strlen (frame_header_identifier_));
      //! use the base class and write some extended information on codecV2
      OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader(compressed_tree_data_out_arg);

      //! write additional fields for cloud codec v2
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_voxel_centroid_enDecoding_), sizeof (do_voxel_centroid_enDecoding_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_connectivity_encoding_), sizeof (do_connectivity_encoding_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&create_scalable_bitstream_), sizeof (create_scalable_bitstream_));
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&color_coding_type_), sizeof (color_coding_type_));
    };

    //! read Frame header, extended for cloud codecV2
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::readFrameHeader(std::istream& compressed_tree_data_in_arg)
    {
      //! use the base class and read some extended information on codecV2
      OctreePointCloudCompression<PointT>::readFrameHeader(compressed_tree_data_in_arg);

      //! read additional fields for cloud codec v2
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_voxel_centroid_enDecoding_), sizeof (do_voxel_centroid_enDecoding_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_connectivity_encoding_), sizeof (do_connectivity_encoding_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&create_scalable_bitstream_), sizeof (create_scalable_bitstream_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&color_coding_type_), sizeof (color_coding_type_));
    };

    /** \brief Encode leaf node information during serialization
    * \param leaf_arg: reference to new leaf node
    * \param key_arg: octree key of new leaf node
    * \brief added centroids encoding compared to the original codec
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg)
    {
      // reference to point indices vector stored within octree leaf //
      const std::vector<int>& leafIdx = leaf_arg.getPointIndicesVector();
      double lowerVoxelCorner[3];
      PointT centroid;
      // //

      // calculate lower voxel corner based on octree key //
      lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
      lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
      lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;
      // //

      // points detail encoding
      if (!do_voxel_grid_enDecoding_)
      {
        // encode amount of points within voxel
        point_count_data_vector_.push_back (static_cast<int> (leafIdx.size ()));

        // differentially encode points to lower voxel corner
        point_coder_.encodePoints (leafIdx, lowerVoxelCorner, input_);

        if (cloud_with_color_) {
          // encode color of points
          if(!color_coding_type_) {
            color_coder_.encodePoints (leafIdx, point_color_offset_, input_);
          } else {
            jp_color_coder_.encodePoints (leafIdx, point_color_offset_, input_);
          }
        }
      }
      else // centroid or voxelgrid encoding
      {
        if (cloud_with_color_)
        {
          // encode average color of all points within voxel
          if(!color_coding_type_)
            color_coder_.encodeAverageOfPoints (leafIdx, point_color_offset_, input_);
          else
            jp_color_coder_.encodeAverageOfPoints (leafIdx, point_color_offset_, input_);
          std::vector<char> & l_colors = color_coding_type_ ? jp_color_coder_.getAverageDataVectorB() : color_coder_.getAverageDataVector();

          //! get the colors from the vector from the color coder, use to store to do temporal prediction
          centroid.r =  l_colors[l_colors.size() - 1];
          centroid.g =  l_colors[l_colors.size() - 2];
          centroid.b =  l_colors[l_colors.size() - 3];
        }
        if(!do_voxel_centroid_enDecoding_)
        {
          centroid.x = lowerVoxelCorner[0] + 0.5 * resolution_ ;
          centroid.y = lowerVoxelCorner[1] + 0.5 * resolution_ ;
          centroid.z = lowerVoxelCorner[2] + 0.5 * resolution_ ;
        }
        else
        {
          Eigen::Vector4f f_centroid;
          pcl::compute3DCentroid<PointT>(*(input_), leafIdx, f_centroid);

          centroid.x = f_centroid[0];
          centroid.y = f_centroid[1];
          centroid.z = f_centroid[2];

          centroid_coder_.encodePoint(lowerVoxelCorner, centroid);
        }
        // store the simplified cloud so that it can be used for predictive encoding
        output_->points.push_back(centroid);
      }
    }

    /** \brief Decode leaf nodes information during deserialization
    * \param key_arg octree key of new leaf node
    * \brief added centroids encoding compared to the original codec
    */
    // param leaf_arg reference to new leaf node
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::deserializeTreeCallback (LeafT&, const OctreeKey& key_arg)
    {
      double lowerVoxelCorner[3];
      std::size_t pointCount, i, cloudSize;
      PointT newPoint;
      pointCount = 1;

      if (!do_voxel_grid_enDecoding_)
      {
        // get current cloud size
		cloudSize = output_->points.size();

        // get amount of point to be decoded
        pointCount = *point_count_data_vector_iterator_;
        point_count_data_vector_iterator_++;

        // increase point cloud by amount of voxel points
        for (i = 0; i < pointCount; i++)
          output_->points.push_back (newPoint);

        // calculcate position of lower voxel corner
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;

        // decode differentially encoded points
        point_coder_.decodePoints (output_, lowerVoxelCorner, cloudSize, cloudSize + pointCount);
      }
      else
      {
        // decode the centroid or the voxel center
        if(do_voxel_centroid_enDecoding_)
        {
          PointT centroid_point;

          // calculcate position of lower voxel corner
          lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * resolution_ + min_x_;
          lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * resolution_ + min_y_;
          lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * resolution_ + min_z_;

          centroid_coder_.decodePoint(newPoint,lowerVoxelCorner);
        }
        else
        {
          // calculate center of lower voxel corner
          newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * resolution_ + min_x_);
          newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * resolution_ + min_y_);
          newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * resolution_ + min_z_);
        }

        // add point to point cloud
        output_->points.push_back (newPoint);
      }

      if (cloud_with_color_)
      {
        if (data_with_color_)
          // decode color information
          if(!color_coding_type_)
            color_coder_.decodePoints (output_, output_->points.size () - pointCount,
					     output_->points.size (), point_color_offset_);
          else
            jp_color_coder_.decodePoints (output_, output_->points.size () - pointCount, output_->points.size (), point_color_offset_);
        else
          // set default color information
          
		  color_coder_.setDefaultColor(output_, output_->points.size() - pointCount, output_->points.size(), point_color_offset_);
      }
    }

    /** \brief Synchronize to frame header
    * \param compressed_tree_data_in_arg: binary input stream
    * \brief use the new v2 header
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::syncToHeader (std::istream& compressed_tree_data_in_arg)
    {
      // sync to frame header
      unsigned int header_id_pos = 0;
      while (header_id_pos < strlen (frame_header_identifier_))
      {
        char readChar;
        compressed_tree_data_in_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
        if (readChar != frame_header_identifier_[header_id_pos++])
          header_id_pos = (frame_header_identifier_[0]==readChar)?1:0;
      }
      //! read the original octree header
      OctreePointCloudCompression<PointT>::syncToHeader (compressed_tree_data_in_arg);
    };

    /** \brief Apply entropy encoding to encoded information and output to binary stream, added bitstream scalability and centroid encoding compared to  V1
    * \param compressed_tree_data_out_arg: binary output stream
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::entropyEncoding (std::ostream& compressed_tree_data_out_arg1, std::ostream& compressed_tree_data_out_arg2 )
    {
      uint64_t binary_tree_data_vector_size;
      uint64_t point_avg_color_data_vector_size;

      compressed_point_data_len_ = 0;
      compressed_color_data_len_ = 0;

      // encode binary octree structure
      binary_tree_data_vector_size = binary_tree_data_vector_.size ();
      compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	    compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(binary_tree_data_vector_, compressed_tree_data_out_arg1);

      // store the number of bytes used for voxel encoding
      compression_performance_metrics[0] = compressed_point_data_len_ ;

      // encode centroids
      if(do_voxel_centroid_enDecoding_)
      {
        // encode differential centroid information
        std::vector<char>& point_diff_data_vector = centroid_coder_.getDifferentialDataVector ();
		uint32_t point_diff_data_vector_size = (uint32_t) point_diff_data_vector.size();
        compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&point_diff_data_vector_size), sizeof (uint32_t));
		compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(point_diff_data_vector, compressed_tree_data_out_arg1);
      }

      // store the number of bytes used for voxel centroid encoding
      compression_performance_metrics[1] = compressed_point_data_len_ - compression_performance_metrics[0];

      // encode colors
      if (cloud_with_color_)
      {
        // encode averaged voxel color information
        std::vector<char>& pointAvgColorDataVector = color_coding_type_ ?  jp_color_coder_.getAverageDataVector () : color_coder_.getAverageDataVector ();
        point_avg_color_data_vector_size = pointAvgColorDataVector.size ();
        compressed_tree_data_out_arg1.write (reinterpret_cast<const char*> (&point_avg_color_data_vector_size), sizeof (point_avg_color_data_vector_size));
		compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream(pointAvgColorDataVector, compressed_tree_data_out_arg1);
      }

      // store the number of bytes used for the color stream
      compression_performance_metrics[2] = compressed_color_data_len_ ;

      // flush output stream
      compressed_tree_data_out_arg1.flush ();

      if (!do_voxel_grid_enDecoding_)
      {
        uint64_t pointCountDataVector_size;
        uint64_t point_diff_data_vector_size;
        uint64_t point_diff_color_data_vector_size;

        // encode amount of points per voxel
		pointCountDataVector_size = point_count_data_vector_.size();
        compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&pointCountDataVector_size), 
					     sizeof (pointCountDataVector_size));
		compressed_point_data_len_ += entropy_coder_.encodeIntVectorToStream(point_count_data_vector_,
									      compressed_tree_data_out_arg2);

        // encode differential point information
		std::vector<char>& point_diff_data_vector = point_coder_.getDifferentialDataVector();
        point_diff_data_vector_size = point_diff_data_vector.size ();
        compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
		compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream(point_diff_data_vector, compressed_tree_data_out_arg2);

        if (cloud_with_color_)
        {
          // encode differential color information
          std::vector<char>& point_diff_color_data_vector = color_coding_type_ ?  jp_color_coder_.getDifferentialDataVector () : color_coder_.getDifferentialDataVector ();
          point_diff_color_data_vector_size = point_diff_color_data_vector.size ();
          compressed_tree_data_out_arg2.write (reinterpret_cast<const char*> (&point_diff_color_data_vector_size),
					       sizeof (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream (point_diff_color_data_vector,
										 compressed_tree_data_out_arg2);
        }
      }
      // flush output stream
      compressed_tree_data_out_arg2.flush ();
    };

    /** \brief Entropy decoding of input binary stream and output to information vectors, added bitstream scalability and centroid encoding compared to  V1
    * \param compressed_tree_data_in_arg: binary input stream1, binary input stream1,
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
      OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::entropyDecoding (std::istream& compressed_tree_data_in_arg1, std::istream& compressed_tree_data_in_arg2)
    {
      uint64_t binary_tree_data_vector_size;
      uint64_t point_avg_color_data_vector_size;

	  compressed_point_data_len_ = 0;
	  compressed_color_data_len_ = 0;

      // decode binary octree structure
      compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	  binary_tree_data_vector_.resize(static_cast<std::size_t> (binary_tree_data_vector_size));
	  compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1, binary_tree_data_vector_);

      //! new option for encoding centroids
      if(do_voxel_centroid_enDecoding_)
      {
        uint32_t l_count;

        // decode differential point information
        std::vector<char>& pointDiffDataVector = centroid_coder_.getDifferentialDataVector ();
        compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&l_count), sizeof (uint32_t));
        pointDiffDataVector.resize (static_cast<std::size_t> (l_count));
		compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1, pointDiffDataVector);
      }

      if (data_with_color_)
      {
        // decode averaged voxel color information
		  std::vector<char>& point_avg_color_data_vector = color_coding_type_ ? jp_color_coder_.getAverageDataVector() : color_coder_.getAverageDataVector();
        compressed_tree_data_in_arg1.read (reinterpret_cast<char*> (&point_avg_color_data_vector_size), sizeof (point_avg_color_data_vector_size));
        point_avg_color_data_vector.resize (static_cast<std::size_t> (point_avg_color_data_vector_size));
		compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector(compressed_tree_data_in_arg1,	point_avg_color_data_vector);
      }

      //! check if the enhancement layer has been received
      compressed_tree_data_in_arg2.peek();
      if(compressed_tree_data_in_arg2.good() && (! compressed_tree_data_in_arg2.eof()) )
        do_voxel_grid_enDecoding_ = false;
      else
        do_voxel_grid_enDecoding_ = true;

      if (!do_voxel_grid_enDecoding_)
      {
        uint64_t point_count_data_vector_size;
        uint64_t point_diff_data_vector_size;
        uint64_t point_diff_color_data_vector_size;

        // decode amount of points per voxel
        compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_count_data_vector_size), sizeof (point_count_data_vector_size));
        point_count_data_vector_.resize (static_cast<std::size_t> (point_count_data_vector_size));
		compressed_point_data_len_ += entropy_coder_.decodeStreamToIntVector(compressed_tree_data_in_arg2, point_count_data_vector_);
        point_count_data_vector_iterator_ = point_count_data_vector_.begin ();

        // decode differential point information
		std::vector<char>& pointDiffDataVector = point_coder_.getDifferentialDataVector();
        compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
        pointDiffDataVector.resize (static_cast<std::size_t> (point_diff_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg2, pointDiffDataVector);
        if (data_with_color_)
        {
          // decode differential color information
          std::vector<char>& pointDiffColorDataVector = color_coder_.getDifferentialDataVector ();
          compressed_tree_data_in_arg2.read (reinterpret_cast<char*> (&point_diff_color_data_vector_size), sizeof (point_diff_color_data_vector_size));
          pointDiffColorDataVector.resize (static_cast<std::size_t> (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg2, pointDiffColorDataVector);
        }
      }
    };
  }
}
#endif
