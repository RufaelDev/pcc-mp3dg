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
#ifndef COMPRESSION_EVAL_HPP
#define COMPRESSION_EVAL_HPP

#include <pcl/compression_eval/compression_eval.h>

namespace pcl{

  namespace io{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**! 
    * function to generate octree codec with bit settings
    * @param nr_bits_base_layer, an integer argument representing the number of bits for encoding the base layer (octree).
    * @param nr_bits_enh_layer, an integer argument representing the number of bits for encoding the  enhancement layer (octree).
    * @param nr_bits_colors, an integer argument representing the number of bits for encoding the colors per point.
    * @param i_frame_rate, an integer controlling the framerate of iframes
    * \note PointT typename of point used in point cloud
    * \author Rufael Mekuria (rufael.mekuria@cwi.nl)
    */
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> boost::shared_ptr<OctreePointCloudCodecV2<PointT>>
    generatePCLOctreeCodecV2(int nr_bits_base_layer, int nr_bits_enh_layer, int nr_bits_colors, int i_frame_rate=0, int color_coding_type=0, bool do_centroid_coding = true )
    {
      return boost::shared_ptr<OctreePointCloudCodecV2<PointT>>(new OctreePointCloudCodecV2<PointT>(
        MANUAL_CONFIGURATION,
        false,
        std::pow( 2.0, -1.0 *(nr_bits_base_layer + nr_bits_enh_layer) ),
        std::pow( 2.0, -1.0 *(nr_bits_base_layer)),
        true /* no intra voxel coding in this first version of the codec ok */,
        i_frame_rate,
        nr_bits_colors ? true : false,
        nr_bits_colors,
        color_coding_type,
        do_centroid_coding
        ));
    }
  }
}
#endif

