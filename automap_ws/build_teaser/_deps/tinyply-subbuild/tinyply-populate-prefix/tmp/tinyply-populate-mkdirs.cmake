# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/root/automap_ws/build_teaser/_deps/tinyply-src"
  "/root/automap_ws/build_teaser/_deps/tinyply-build"
  "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix"
  "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/tmp"
  "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/src/tinyply-populate-stamp"
  "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/src"
  "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/src/tinyply-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/src/tinyply-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/root/automap_ws/build_teaser/_deps/tinyply-subbuild/tinyply-populate-prefix/src/tinyply-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
