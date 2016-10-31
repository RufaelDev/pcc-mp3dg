/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) CodeShop B.V. 2016
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/compression_eval/compression_eval.h>
#include <pcl/compression_eval/impl/compression_eval_impl.hpp>
#include <pcl/compression_eval/impl/compression_eval_impl.hpp>

#include <pcl/quality/quality_metrics.h>
#include <pcl/quality/impl/quality_metrics_impl.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

/* ---[ 
    Simple Tool to encode static Point Cloud frames for generation of Anchors in MPEG 
	use pcl_mpeg_pcc_frame_coder.exe -e iframe.ply compstream to encode 
	use pcl_mpeg_pcc_frame_coder.exe -d outfile.ply compstream to decode 
	uses separate parameter_config.txt file
*/
int
main (int argc, char** argv)
{
  print_info ("MPEG Point Cloud Frame Coder Tool, use: %s -h\n", argv[0]);

  if (std::string(argv[1]).compare("-h") == 0)
  {
	  std::cout << "3 input arguments usage app.exe mode icloudfile targetoutput file" << std::endl;
	  std::cout << std::endl;
	  std::cout << "example (encode):  mpeg_pcc_standalone.exe -e ifile.ply y out_stream" << std::endl;
	  std::cout << "example (decode):  mpeg_pcc_standalone.exe -d ifile_out.ply  compressed_stream_file" << std::endl;
	  return 1;
  }

  if (argc != 4) 
  {
	  std::cout << "3 input arguments usage app.exe mode icloudfile pcloudfile targetoutput file" << std::endl;
	  std::cout << std::endl;
	  std::cout << "example (encode):  mpeg_pcc_standalone.exe -e ifile.ply  out_stream" << std::endl;
	  std::cout << "example (decode):  mpeg_pcc_standalone.exe -d ifile_out.ply  compressed_stream_file" << std::endl;
	  return 1;
  }

  std::string coding_mode(argv[1]);
  std::string icloudfile = std::string(argv[2]);
  std::string target_out = std::string(argv[3]);
  std::string pcloud = std::string();
  
  pcl::io::CompressionEval eval;

  if(coding_mode.compare("-e") == 0)
     eval.encodeGOP(icloudfile, pcloud , target_out);

  else if(coding_mode.compare("-d") == 0)
	 eval.decodeGOP(target_out, true, icloudfile, pcloud);

  return(0);
}

