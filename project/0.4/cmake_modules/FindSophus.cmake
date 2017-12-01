# - Try to find Sophus lib

if (SOPHUS_INCLUDE_DIR)

else (SOPHUS_INCLUDE_DIR)

  find_path(SOPHUS_INCLUDE_DIR NAMES sophus
      PATHS
     # ${CMAKE_INSTALL_PREFIX}/include
      /home/alex/slambook/3rdparty/Sophus/build
      #add path to local build, if build from src
    )

endif(SOPHUS_INCLUDE_DIR)

FIND_LIBRARY(Sophus_LIBRARIES NAMES Sophus
  PATHS
  /usr/lib
  /usr/local/lib
  /opt/local/lib
  /sw/lib
  )

