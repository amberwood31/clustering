#set(CMAKE_BUILD_TYPE Release)

# Set required variables.
SET(SLAM_EXAMPLE_BINARY_NAME "slam_incre_clustering")

SET(CMAKE_CXX_FLAGS -pg)

SET(SLAM_EXAMPLE_INCLUDE_DIRS
        ${SLAM_P_P_SOURCE_DIR}/include/slam_incre_clustering
        ${SLAM_P_P_SOURCE_DIR}/utilite/include)

IF(SLAM_P_P_SHORT_CXSPARSE)
    ADD_DEFINITIONS(-D__CXSPARSE_SHORT)
ENDIF(SLAM_P_P_SHORT_CXSPARSE)

IF(SLAM_P_P_SHORT_CHOLMOD)
    ADD_DEFINITIONS(-D__CHOLMOD_SHORT)
ENDIF(SLAM_P_P_SHORT_CHOLMOD)

IF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)
    ADD_DEFINITIONS(-D__FLAT_SYSTEM_ALIGNED_MEMORY)
ENDIF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)

# support for GPU BLAS
IF(SLAM_P_P_GPU_BLAS)
    ADD_DEFINITIONS(
            -DGPU_BLAS
            -D__CHANNEL_DESCRIPTOR_H__
            -D__CUDA_RUNTIME_H__)
ENDIF(SLAM_P_P_GPU_BLAS)

ADD_DEFINITIONS(-DCERES_NO_PROTOCOL_BUFFERS)

SET(SLAM_EXAMPLE_SOURCES
        Main.cpp)

IF(SLAM_P_P_LINEAR_SOLVER_TYPE)
    ADD_DEFINITIONS("-D__LINEAR_SOLVER_OVERRIDE=${SLAM_P_P_LINEAR_SOLVER_TYPE}")
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE)

# Build and install SLAM_plus_plus executable.
INCLUDE_DIRECTORIES(${SLAM_EXAMPLE_INCLUDE_DIRS})

ADD_EXECUTABLE(${SLAM_EXAMPLE_BINARY_NAME}
        ${SLAM_EXAMPLE_SOURCES})

TARGET_LINK_LIBRARIES(${SLAM_EXAMPLE_BINARY_NAME}
        ${SLAM_EXAMPLE_LIBRARIES})

INSTALL(TARGETS ${SLAM_EXAMPLE_BINARY_NAME}
        RUNTIME DESTINATION bin)
