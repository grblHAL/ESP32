#
# Template for Web Builder, ESP32
#

[platformio]
src_dir = main
include_dir = main

[wifi_networking]
build_flags =

[env:%env_name%]
platform = espressif32 @ ~3.4.0
framework = espidf
board =%board%
board_build.partitions = partitions.csv
board_build.cmake_extra_args = %cmake_flags%
board_build.embed_files =
  main/embedded/favicon.ico
  main/embedded/ap_login.html
  main/embedded/index.html.gz
build_flags =
  -Wimplicit-fallthrough=1
  -Wno-missing-field-initializers
  -Wno-maybe-uninitialized
  -Wno-stringop-truncation
%build_flags%
lib_compat_mode = off
