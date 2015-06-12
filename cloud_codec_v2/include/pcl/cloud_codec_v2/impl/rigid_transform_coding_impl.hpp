/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014- Centrum Wiskunde Informatica
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
*   * Neither the name of its copyright holders nor the names of its
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
#ifndef RIGID_TRANSFORM_CODING_CCV2_HPP
#define RIGID_TRANSFORM_CODING_CCV2_HPP

// class for quaternion encoding
#include <eigen/geometry>
#include <stdint.h>

// use quaternions
#include <pcl/cloud_codec_v2/rigid_transform_coding.h>
#include <pcl/cloud_codec_v2/impl/quaternion_coding_impl.hpp>
#include <limits>

namespace pcl{

  namespace io
  {
    

    /*
      \brief Class for Rigid transform coding, assumes normalized [0,1][0,1] space
    */

    template <typename Scalar> bool 
      RigidTransformCoding<Scalar>::compressRigidTransform(
        const Eigen::Matrix<Scalar, 4, 4> &tr_in, 
        std::vector<int16_t> &comp_dat_out)
    {
      // 2 is the maximum 
      const float scaling_factor = (float) std::numeric_limits<int16_t>::max()/2.5;
      
      float t1 = (float) tr_in(0,3);
      if(t1>2.5)
        t1=2.5;
      if(t1<-2.5)
        t1=-2.5;

      float t2 = (float) tr_in(1,3);
      if(t2>2.5)
        t2=2.5;
      if(t2<-2.5)
        t2=-2.5;

      float t3 = (float) tr_in(2,3);
      if(t3>2.5)
        t3=2.5;
      if(t3<-2.5)
        t3=-2.5;
      
      // convert the rotation to a quaternion
      Eigen::Matrix<Scalar,3,3> rotm;
      rotm << tr_in.block<3,3>(0,0);
      Eigen::Quaternion<Scalar> rot_q(rotm);

      // sometimes the quaternion results in incorrect matrix check this before encoding 
      // not yet sure what is causing this
      // check if the quaternion is stable first
      Eigen::Matrix<Scalar, 3, 3> res_rot = rot_q.toRotationMatrix();
      bool quaternion_is_stable=true;
      for(int i=0; i<9;i++)
      {
        float diff=0;
          if((diff=std::abs(res_rot(i/3,i%3) - rotm(i/3,i%3))) > 0.001){
            quaternion_is_stable=false;
          }
      }
      
      // compress the rotation matrix as two vectors or quaternion
      if(quaternion_is_stable)
      {
        comp_dat_out.resize(3);
        QuaternionCoding::compressQuaternion(rot_q,(int16_t *) &comp_dat_out[0]);
      }
      else{
         comp_dat_out.resize(6);
         // only store two vectors of the rotation matrix
         for(int l=0;l<3;l++){
           comp_dat_out[l] =  (int16_t) int(rotm(0,l) * (std::numeric_limits<int16_t>::max() -1));
           comp_dat_out[l+3] =  (int16_t) int(rotm(1,l) * (std::numeric_limits<int16_t>::max() -1));
         }
      }

      // compress the translation
      comp_dat_out.push_back( (int16_t) (int) (t1 * (scaling_factor-1)));
      comp_dat_out.push_back( (int16_t) (int) (t2 * (scaling_factor-1)));
      comp_dat_out.push_back( (int16_t)  (int)(t3 * (scaling_factor-1)));

      return true;
    }

    template <typename Scalar> bool 
      RigidTransformCoding<Scalar>::deCompressRigidTransform(
        const std::vector<int16_t> comp_dat_in, 
        Eigen::Matrix<Scalar, 4, 4> &tr_out)
    {
      // 2 is the maximum 
      const float scaling_factor = (float) std::numeric_limits<int16_t>::max()/2.5;
      
      if(comp_dat_in.size() == 6){
        // decode rotation offset
        Eigen::Quaternion<Scalar> rot_q;
        QuaternionCoding::deCompressQuaternion((int16_t *) &comp_dat_in[0], rot_q);
        tr_out.block<3,3>(0,0) = rot_q.toRotationMatrix();
      }
      else{
        //decode the rotation matrix from two vectors
         for(int l=0;l<3;l++){
           tr_out(0,l) = (float) comp_dat_in[l] / (std::numeric_limits<int16_t>::max() -1);
           tr_out(1,l) = (float) comp_dat_in[l+3] / (std::numeric_limits<int16_t>::max() -1);
           tr_out(2,l) =  std::sqrt(1 - tr_out(0,l) * tr_out(0,l) - tr_out(1,l) * tr_out(1,l));
         }
      }
      // decode offset 
      tr_out(0,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-3]) / ((Scalar) (scaling_factor -1)); 
      tr_out(1,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-2]) / ((Scalar) (scaling_factor -1));
      tr_out(2,3) = ((Scalar) comp_dat_in[comp_dat_in.size()-1]) / ((Scalar) (scaling_factor -1));
      
      // construct transform matrix
      tr_out(3,0) = 0;
      tr_out(3,1) = 0;
      tr_out(3,2) = 0;
      tr_out(3,3) = 1;

      return true;
    }
  }
}
#endif

/*
bool is_normalized = true;
            for(int k=0;k<3;k++)
            {
              float norm=(rt(0,k)*rt(0,k) + rt(1,k)*rt(1,k) + rt(2,k)*rt(2,k));
              if( norm > 1.01 || norm < 0.99 ){
                is_normalized = false;
                 std::cout << "matrix not normalized! " << norm  << std::endl;
                 std::cout << rt;
                 std::cin.get();
                break;
              }
            }

            // code for generation
            Eigen::Quaternion<float> test_quat(rt.block<3,3>(0,0));
            if(! (test_quat.squaredNorm()) == 1)
            {
              std::cout << "quaternion not normalized" << std::endl;
              std::cin.get();
            }
            
            bool corrected_tf_matrix=false;
            for(int i=0; i<16;i++)
            {
              float diff=0;
               if((diff=std::abs(rt(i/4,i%4) - mdec(i/4,i%4))) > 0.001){
                mdec(i/4,i%4) = rt(i/4,i%4);
                corrected_tf_matrix=true;
               }
            }
            if(corrected_tf_matrix)
              std::cout << " corrected transform matrix! " << std::endl;
            else
              std::cout << " . ";
*/