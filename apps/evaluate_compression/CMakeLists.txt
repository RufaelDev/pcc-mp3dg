cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(SUBSUBSYS_NAME evaluate_compression)
set(SUBSUBSYS_DESC "Evaluate PCL Compression Properties")
set(SUBSUBSYS_DEPS common sample_consensus search kdtree octree filters features io geometry cloud_codec_v2 registration cloud_codec_v2 visualization apps)

PCL_SUBSUBSYS_OPTION(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" ${DEFAULT} "${REASON}")
PCL_SUBSUBSYS_DEPEND(BUILD "${SUBSYS_NAME}" "${SUBSUBSYS_NAME}" DEPS ${SUBSUBSYS_DEPS} EXT_DEPS vtk)

#if (${BUILD})
  PCL_ADD_DOC("${SUBSUBSYS_NAME}")

  set(pcl_libraries pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree pcl_filters pcl_features pcl_io  pcl_registration pcl_cloud_codec_v2 pcl_visualization pcl_apps)
  set(srcs "src/evaluate_compression.cpp"
	   "src/quality_metrics.cpp")
  set(incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/evaluate_compression.h"
	   "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/quality_metrics.h")
  set(impl_incs "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/evaluate_compression_impl.hpp"
	        "include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl/quality_metrics_impl.hpp")
  set(excs "pcl_${SUBSUBSYS_NAME}")
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include/pcl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/"
                      "${CMAKE_CURRENT_SOURCE_DIR}/include/PCl/${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl")

  foreach (exc ${excs})
#   PCL_ADD_EXECUTABLE_OPT_BUNDLE(${exc} "${SUBSUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    PCL_ADD_EXECUTABLE(${exc} "${SUBSUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    set_target_properties(${exc} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${exc} ${Boost_LIBRARIES} ${pcl_libraries})
    PCL_MAKE_PKGCONFIG("${exc}" "${SUBSUBSYS_NAME}" "${SUBSUBSYS_DESC}" "" "" "" "" "")
  endforeach (exc ${excs})

  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}" ${incs})
  PCL_ADD_INCLUDES("${SUBSUBSYS_NAME}" "${SUBSYS_NAME}/${SUBSUBSYS_NAME}/impl" ${impl_incs})
#endif(${BUILD})
