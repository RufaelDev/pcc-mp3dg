/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011-2012, Willow Garage, Inc.
 *  Copyright (c) 2016, CodeShop B.V
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

#ifndef PLANE_PROJ_COMPRESSION_H
#define PLANE_PROJ_COMPRESSION_H

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

namespace pcl
{
  namespace octree
  {
    /** \brief @b Plane projection PointCoding class
      * \note This class encodes point information for octree-based point cloud compression leaves via a plane projection method.
      * \note typename: PointT: type of point used in pointcloud
      * \author Rufael Mekuria rufael@unified-streaming.com
      */
	// value to signal plane projection coding in the color types
	const int PLANE_PROJ_CODING = 5;
	 
    template<typename PointT>
    class PlaneProjCoding
    {
        // public typedefs
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      public:
        /** \brief Constructor. */
        PlaneProjCoding () :
          output_ (), planeDataVector_ (), planeDataVectorIterator_ (),planeDataChunkSizes_(), 
          pointCompressionResolution_ (0.001f) // 1mm
        {
        }
        
        /** \brief Empty class constructor. */
        virtual
        ~PlaneProjCoding ()
        {
        }

        /** \brief Define precision of point information
          * \param precision_arg: precision
          */
        inline void
        setPrecision (float precision_arg)
        {
          pointCompressionResolution_ = precision_arg;
        }

        /** \brief Retrieve precision of point information
          * \return precision
          */
        inline float
        getPrecision ()
        {
          return (pointCompressionResolution_);
        }

        /** \brief Set amount of points within point cloud to be encoded and reserve memory
          * \param pointCount_arg: amounts of points within point cloud
          */
        inline void
        setPointCount (unsigned int pointCount_arg)
        {
          planeDataVector_.reserve (pointCount_arg * 3);
        }

        /** \brief Initialize encoding of differential point */
        void
        initializeEncoding ()
        {
          planeDataVector_.clear ();
        }

        /** \brief Initialize decoding of differential point */
        void
        initializeDecoding ()
        {
          planeDataVectorIterator_ = planeDataVector_.begin ();
        }

        /** \brief Get reference to vector containing differential color data */
        std::vector<char>&
        getPlaneDataVector ()
        {
          return (planeDataVector_);
        }

        /** \brief Encode point information for a subset of points from a point cloud based on differentials from a plane approximation
          * \param indexVector_arg indices defining a subset of points from points cloud
          * \param referencePoint_arg coordinates of reference point (voxel center)
          * \param inputCloud_arg input point cloud
          */
        void
        encodePoints (const typename std::vector<int>& indexVector_arg, const double* referencePoint_arg,
                      PointCloudConstPtr inputCloud_arg)
        {
          std::size_t i, len;

          len = indexVector_arg.size ();

          // iterate over points within current voxel
          for (i = 0; i < len; i++)
          {
            unsigned char diffX, diffY, diffZ;

            // retrieve point from cloud
            const int& idx = indexVector_arg[i];
            const PointT& idxPoint = inputCloud_arg->points[idx];

            // differentially encode point coordinates and truncate overflow
            diffX = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.x - referencePoint_arg[0])  / pointCompressionResolution_))));
            diffY = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.y - referencePoint_arg[1])  / pointCompressionResolution_))));
            diffZ = static_cast<unsigned char> (max (-127, min<int>(127, static_cast<int> ((idxPoint.z - referencePoint_arg[2])  / pointCompressionResolution_))));

            // store information in differential point vector
            planeDataVector_.push_back (diffX);
            planeDataVector_.push_back (diffY);
            planeDataVector_.push_back (diffZ);
          }
        }

        /** \brief Decode differential point information
          * \param outputCloud_arg output point cloud
          * \param referencePoint_arg coordinates of reference point
          * \param beginIdx_arg index indicating first point to be assiged with color information
          * \param endIdx_arg index indicating last point to be assiged with color information
          */
        void
        decodePoints (PointCloudPtr outputCloud_arg, const double* referencePoint_arg, std::size_t beginIdx_arg,
                      std::size_t endIdx_arg)
        {
          std::size_t i;
          unsigned int pointCount;

          assert (beginIdx_arg <= endIdx_arg);

          pointCount = static_cast<unsigned int> (endIdx_arg - beginIdx_arg);

          // iterate over points within current voxel
          for (i = 0; i < pointCount; i++)
          {
            // retrieve differential point information
            const unsigned char& diffX = static_cast<unsigned char> (*(planeDataVectorIterator_++));
            const unsigned char& diffY = static_cast<unsigned char> (*(planeDataVectorIterator_++));
            const unsigned char& diffZ = static_cast<unsigned char> (*(planeDataVectorIterator_++));

            // retrieve point from point cloud
            PointT& point = outputCloud_arg->points[beginIdx_arg + i];

            // decode point position
            point.x = static_cast<float> (referencePoint_arg[0] + diffX * pointCompressionResolution_);
            point.y = static_cast<float> (referencePoint_arg[1] + diffY * pointCompressionResolution_);
            point.z = static_cast<float> (referencePoint_arg[2] + diffZ * pointCompressionResolution_);
          }
        }

      protected:
        /** \brief Pointer to output point cloud dataset. */
        PointCloudPtr output_;

        /** \brief Vector for storing differential point information  */
        std::vector<char> planeDataVector_;
		
		/** \brief Vector for storing chunk sizes of PCA related data for plane projection */
		std::vector<uint16_t> planeDataChunkSizes_;

        /** \brief Iterator on differential point information vector */
        std::vector<char>::const_iterator planeDataVectorIterator_;

        /** \brief Precision of point coding*/
        float pointCompressionResolution_;
    };
  }
}

#define PCL_INSTANTIATE_ColorCoding(T) template class PCL_EXPORTS pcl::octree::ColorCoding<T>;

#endif
