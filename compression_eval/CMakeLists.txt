set(SUBSYS_NAME compression_eval)
set(SUBSYS_DESC "Compression Evaluation Suite for Evaluation of Point Cloud Codec")
set(SUBSYS_DEPS common sample_consensus search kdtree octree filters features io geometry visualization cloud_codec_v2 registration quality cloud_codec_v2)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs
        src/compression_eval.cpp
        )

    set(incs
        "include/pcl/${SUBSYS_NAME}/compression_eval.h"
        )

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/compression_eval_impl.hpp"
        )

	if(WITH_JPEG)
		include_directories(${LIBJPEG_INCLUDE_DIR})
		link_directories(${LIBJPEG_STATIC_LIB})
		link_directories(${LIBJPEG_TURBO_STATIC_LIB})
	endif(WITH_JPEG)

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_EXECUTABLE("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree pcl_io pcl_io_ply pcl_visualization)
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

endif(build)
