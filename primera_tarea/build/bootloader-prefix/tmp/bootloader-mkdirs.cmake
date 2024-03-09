# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/juni0/esp/v5.2/esp-idf/components/bootloader/subproject"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/tmp"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/src"
  "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/juni0/OneDrive/Escritorio/git_de_sistemas_tiempo_real/primera_tarea/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
