cmake_minimum_required(VERSION 3.20)
project(ahmp3 C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

option(AHMP3_ASAN "Build with AddressSanitizer" OFF)

if(MSVC)
  add_compile_options(/W4 /WX /utf-8 /fp:fast)
  if(AHMP3_ASAN)
    add_compile_options(/fsanitize=address /Zi)
    add_link_options(/INCREMENTAL:NO)
  endif()
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wall -Wextra -Werror -O2)
endif()

# The library is not named ahmp3: the command line tool is, and on Windows
# the two would both claim ahmp3.lib and ahmp3.pdb.
add_library(ahcodecs STATIC
  src/ahmp3_tables.c
  src/ahmp3_huff.c
  src/ahmp3_decode.c
  src/ahmp3_layer12.c
  src/ahmp3_bitstream.c
  src/ahmp3_encode.c
  src/ahwav.c
  src/ahflac.c
  src/ahaac_tables.c
  src/ahaac_decode.c
  src/ahm4a.c
  src/ahalac.c
  src/ahogg.c
  src/ahvorbis.c
  src/ahreader.c
  src/ahresample.c
  src/ahmix.c)
target_include_directories(ahcodecs PUBLIC include)
if(NOT MSVC)
  target_link_libraries(ahcodecs PUBLIC m)
endif()

add_executable(ahmp3_tool tools/ahmp3.c)
set_target_properties(ahmp3_tool PROPERTIES OUTPUT_NAME ahmp3)
target_link_libraries(ahmp3_tool ahcodecs)

add_executable(ahmp3_tests
  tests/tests.c
  tests/test_layers.c
  tests/test_wav.c
  tests/test_flac.c
  tests/test_aac.c
  tests/test_alac.c
  tests/test_vorbis.c
  tests/test_convert.c
  tests/test_reader.c)
target_link_libraries(ahmp3_tests ahcodecs)
