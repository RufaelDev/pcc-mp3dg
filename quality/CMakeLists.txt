set(SUBSYS_NAME quality)
set(SUBSYS_DESC "Compression Quality Evaluation Suite for Evaluation of Point Cloud Codecs")
set(SUBSYS_DEPS common sample_consensus search kdtree octree filters features io geometry)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS})

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
    set(srcs
		"src/quality_metrics.cpp"
        )

    set(incs
		"include/pcl/${SUBSYS_NAME}/quality_metrics.h"
        )

    set(impl_incs
		"include/pcl/${SUBSYS_NAME}/impl/quality_metrics_impl.hpp"
        )

	if(WITH_JPEG)
		include_directories(${LIBJPEG_INCLUDE_DIR})
		link_directories(${LIBJPEG_STATIC_LIB})
		link_directories(${LIBJPEG_TURBO_STATIC_LIB})
	endif(WITH_JPEG)

    set(LIB_NAME "pcl_${SUBSYS_NAME}")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${impl_incs})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_sample_consensus pcl_search pcl_kdtree pcl_octree pcl_io pcl_io_ply )
    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}" "${SUBSYS_DEPS}" "" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

endif(build)
