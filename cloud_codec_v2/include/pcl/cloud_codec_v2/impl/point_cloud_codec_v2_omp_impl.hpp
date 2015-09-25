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
#ifndef POINT_CLOUD_CODEC_V2_OMP_IMPL_HPP
#define POINT_CLOUD_CODEC_V2_OMP_IMPL_HPP
// point cloud compression from PCL, parallel version using OpenMP
#include <pcl/cloud_codec_v2/pcl_point_cloud_codec_v2.h>

#if defined (_OPENMP)
#include <omp.h>
#endif

namespace pcl{

  namespace io{
    /** \brief routine to encode a delta frame (predictive coding)
    * \author Rufael Mekuria rufael.mekuria@cwi.nl
    * \param icloud_arg  point cloud to be used a I frame
    * \param pcloud_arg  point cloud to be encoded as a pframe
    * \param out_cloud_arg [out] the predicted frame 
    * \param i_coded_data intra encoded data 
    * \param p_coded_data inter encoded data
    * \param icp_on_original  (option to do icp on original or simplified clouds)
    * \param write_out_cloud  (flag to write the output cloud to out_cloud_arg)
    */
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
		OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT>::encodePointCloudDeltaFrame(
		const PointCloudConstPtr &icloud_arg, const PointCloudConstPtr &pcloud_arg, PointCloudPtr &out_cloud_arg,
		std::ostream& i_coded_data, std::ostream& p_coded_data, bool icp_on_original, bool write_out_cloud)
	{
#if defined (_OPENMP)
#pragma omp parallel
		{ 
#endif //_OPENMP
			// intra coded points storage (points that cannot be predicted)
			typename pcl::PointCloud<PointT>::Ptr intra_coded_points(new pcl::PointCloud<PointT>());

			// keep track of the prediction statistics
			long macro_block_count = 0;
			long shared_macroblock_count = 0;
			long convergence_count = 0;

			// initialize predictive cloud
			PointCloudPtr simp_pcloud(new PointCloud());

			if (!icp_on_original){
				simplifyPCloud(pcloud_arg, simp_pcloud);
			}

			// initialize output cloud
			out_cloud_arg->height = 1;
			out_cloud_arg->width = 0;

			////////////// generate the octree for the macroblocks //////////////////////////////
			MacroBlockTree * i_block_tree = generate_macroblock_tree(icloud_arg);
			MacroBlockTree * p_block_tree = generate_macroblock_tree(icp_on_original ? pcloud_arg : simp_pcloud);

			//////////// iterate the predictive frame and find common macro blocks /////////////
			octree::OctreeLeafNodeIterator<OctreeT> it_predictive = p_block_tree->leaf_begin();
			octree::OctreeLeafNodeIterator<OctreeT> it_predictive_end = p_block_tree->leaf_end();
#if defined (_OPENMP)
			typedef struct { pcl::PointCloud<PointT> in_cloud; octree::OctreeKey current_key; std::vector<int> &indices; } cloudInfoT;
			typedef struct { pcl::PointCloud<PointT> out_cloud;  Eigen::Matrix4f rt; bool success; int rgb[3]; } cloudResultT;
			vector<cloudInfoT> cloud_info;
			vector<cloudResultT> cloud_result;
			int i = -1;
			std::vector<cloudInfofT> p_info_list;
			for (; it_predictive != it_predictive_end; ++it_predictive) {
				p_info_list[++i].in_cloud = it_predictive;
				p_info_list[i].current_key = it_predictive.getCurrentOctreeKey();
				p_info_list[i].indices = it_predictive.getLeafContainer().getPointIndicesVector();
			}
#pragma omp for
			for (i = 0; i < p_block_list.size(); i++) {
				const octree::OctreeKey current_key = p_block_list[i].getCurrentOctreeKey();
				typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud , indices);
#else	//_OPENMP
			for (; it_predictive != it_predictive_end; ++it_predictive)
			{
				const octree::OctreeKey current_key = it_predictive.getCurrentOctreeKey();
				typename pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>(icp_on_original ? *pcloud_arg : *simp_pcloud, it_predictive.getLeafContainer().getPointIndicesVector()));
#endif //_OPENMP
				pcl::octree::OctreeContainerPointIndices *i_leaf;
				macro_block_count++;

				if ((i_leaf = i_block_tree->findLeaf(current_key.x, current_key.y, current_key.z)) != NULL)
				{
					shared_macroblock_count++;
					typename pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>(*icloud_arg, i_leaf->getPointIndicesVector()));

					// shared block, do icp
					bool icp_success = false;
					Eigen::Matrix4f rt;
					char rgb_offsets[3] = { 0, 0, 0 };

					do_icp_prediction(
						cloud_in,
						cloud_out,
						rt,
						icp_success,
						rgb_offsets
						);

					if (icp_success)
					{
						convergence_count++;
						// icp success, encode the rigid transform
						std::vector<int16_t> comp_dat;
						Eigen::Quaternion<float> l_quat_out;
						RigidTransformCoding<float>::compressRigidTransform(rt, comp_dat, l_quat_out);

						// write octree key, write rigid transform
						int16_t l_key_dat[3] = { 0, 0, 0 };

						l_key_dat[0] = (int)current_key.x;
						l_key_dat[1] = (int)current_key.y;
						l_key_dat[2] = (int)current_key.z;

						// write the p coded data (we can add entropy encoding later)
						uint8_t chunk_size = (uint8_t)(3 * sizeof(int16_t)+comp_dat.size()*sizeof(int16_t)+(do_icp_color_offset_ ? 3 : 0)); // size of the chunk
						p_coded_data.write((const char *)&chunk_size, sizeof(chunk_size));
						p_coded_data.write((const char *)l_key_dat, 3 * sizeof(int16_t));
						p_coded_data.write((const char *)&comp_dat[0], comp_dat.size()*sizeof(int16_t));

						if (do_icp_color_offset_)
							p_coded_data.write((const char *)rgb_offsets, 3 * sizeof(char));

						// following code is for generation of predicted frame
						Eigen::Matrix4f mdec;
						Eigen::Quaternion<float> l_quat_out_dec;
						RigidTransformCoding<float>::deCompressRigidTransform(comp_dat, mdec, l_quat_out_dec);

						// predicted point cloud
						pcl::PointCloud<PointT> manual_final;
						if (write_out_cloud){
							transformPointCloud<PointT, float>
								(*cloud_in,
								manual_final,
								mdec
								);
						}

						// generate the output points
						if (write_out_cloud){
							for (int i = 0; i < manual_final.size(); i++){

								PointT &pt = manual_final[i];

								// color offset
								if (do_icp_color_offset_){
									pt.r += rgb_offsets[0];
									pt.g += rgb_offsets[1];
									pt.b += rgb_offsets[2];
								}

								out_cloud_arg->push_back(pt);
							}
						}
					}
					else
					{
						// icp failed
						// add to intra coded points
						for (int i = 0; i < cloud_out->size(); i++){
							if (write_out_cloud)
								out_cloud_arg->push_back((*cloud_out)[i]);
							intra_coded_points->push_back((*cloud_out)[i]);
						}
					}
				}
				else
				{
					// exclusive block
					// add to intra coded points
					for (int i = 0; i < cloud_out->size(); i++)
					{
						if (write_out_cloud)
							out_cloud_arg->push_back((*cloud_out)[i]);
						intra_coded_points->push_back((*cloud_out)[i]);
					}
				}
			}

			/* encode all the points that could not be predicted
			from the previous coded frame in i frame manner with cluod_codec_v2 */
			OctreePointCloudCodecV2<PointT, LeafT, BranchT, OctreeT> intra_coder
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

			//compute the convergence statistics
			shared_macroblock_percentage_ = (float)shared_macroblock_count / (float)macro_block_count;
			shared_macroblock_convergence_percentage_ = (float)convergence_count / (float)shared_macroblock_count;

			// clean up
			delete i_block_tree;
			delete p_block_tree;
		}
#if defined (_OPENMP)
		}
#endif //_OPENMP
  }
}
#endif // POINT_CLOUD_CODEC_V2_OMP_IMPL_HPP
