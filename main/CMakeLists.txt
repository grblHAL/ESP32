# NOTE: most configuration options has been moved to my_machine.h.

# Enable these boards here and not in my_machine.h if you want to override board specific default core options.
# NOTE: if reflashing/updating these options will not be changed, use $RST=$ to propagate them.
OPTION(BOARD_BLACKBOX_X32 "Compile for BlackBox X32" OFF)

# WiFi vs. Ethernet support
 # Do not enable unless having a board that supports it!
OPTION(Ethernet "Compile with Ethernet support" OFF)

# Add my_plugin.c to compilation, add my_plugin.c to the main folder first
OPTION(AddMyPlugin "Add my_plugin.c" OFF)

# --- Do not change anything below unless you understand what you are doing ---

set(SRCS
 main.c
 driver.c
 nvs.c
 timers.c
 uart_serial.c
 ioports.c
 ioports_analog.c
 i2c.c
 ioexpand.c
 boards/BlackBoxX32.c
 networking/strutils.c
 grbl/grbllib.c
 grbl/canbus.c
 grbl/coolant_control.c
 grbl/crossbar.c
 grbl/nvs_buffer.c
 grbl/gcode.c
 grbl/machine_limits.c
 grbl/messages.c
 grbl/modbus.c
 grbl/motion_control.c
 grbl/nuts_bolts.c
 grbl/override.c
 grbl/planner.c
 grbl/protocol.c
 grbl/report.c
 grbl/settings.c
 grbl/sleep.c
 grbl/spindle_control.c
 grbl/state_machine.c
 grbl/stream.c
 grbl/stream_file.c
 grbl/stepper.c
 grbl/stepper2.c
 grbl/system.c
 grbl/tool_change.c
 grbl/alarms.c
 grbl/errors.c
 grbl/ngc_expr.c
 grbl/ngc_params.c
 grbl/ngc_flowctrl.c
 grbl/regex.c
 grbl/ioports.c
 grbl/vfs.c
 grbl/kinematics/corexy.c
 grbl/kinematics/wall_plotter.c
 grbl/kinematics/delta.c
 grbl/kinematics/polar.c
 littlefs_hal.c
 littlefs/lfs.c
 littlefs/lfs_util.c
 sdcard/fs_littlefs.c
)

if(AddMyPlugin)
set(MY_PLUGIN_SOURCE
 my_plugin.c
)
else()
set(MY_PLUGIN_SOURCE
 grbl/my_plugin.c
)
endif()

set(I2S_SOURCE
 i2s_out.c
)

set(I2S_S3_SOURCE
 i2s_out_s3.c
)

set(SDCARD_SOURCE
 sdcard/sdcard.c
 sdcard/ymodem.c
 sdcard/fs_fatfs.c
 sdcard/macros.c
)

set(KEYPAD_SOURCE
 keypad/keypad.c
 keypad/macros.c
 keypad/display/i2c_interface.c
)

set(WEBUI_SOURCE
 webui/args.c
 webui/login.c
 webui/commands_v2.c
 webui/commands_v3.c
 webui/server.c
 webui/fs_handlers.c
 webui/sdfs.c
 webui/flashfs.c
 esp_webui/fs_embedded.c
)

set(BLUETOOTH_SOURCE
 bluetooth.c
)

set(MODBUS_SOURCE
 spindle/modbus_rtu
)

set(SPINDLE_SOURCE
 spindle/select.c
 spindle/onoff.c
 spindle/offset.c
 spindle/pwm.c
 spindle/pwm_clone.c
 spindle/stepper.c
 spindle/vfd/gs20.c
 spindle/vfd/h100.c
 spindle/vfd/huanyang.c
 spindle/vfd/huanyang2.c
 spindle/vfd/modvfd.c
 spindle/vfd/spindle.c
 spindle/vfd/yl620.c
)

set(EEPROM_SOURCE
 eeprom/eeprom_24LC16B.c
 eeprom/eeprom_24AAxxx.c
)

set(NETWORKING_FTP_SOURCE
 networking/ftpd.c
 networking/sfifo.c
)

set(WIFI_SOURCE
 wifi.c
)

set(ETHERNET_SOURCE
 enet.c
)

set(NETWORKING_SOURCE
 wifi.c
 mqtt.c
 dns_server.c
 networking/httpd.c
 networking/http_upload.c
 networking/telnetd.c
 networking/websocketd.c
 networking/base64.c
 networking/sha1.c
 networking/urlencode.c
 networking/urldecode.c
 networking/utils.c
 networking/multipartparser.c
 networking/networking.c
 networking/fs_ram.c
 networking/fs_stream.c
 networking/webdav.c
 networking/ssdp.c
 networking/ftpd.c
 networking/sfifo.c
)

set(TRINAMIC_SPI_SOURCE
 spi.c
 trinamic_if.c
 motors/trinamic.c
 trinamic/common.c
 trinamic/tmc_interface.c
 trinamic/tmc2130.c
 trinamic/tmc2130hal.c
 trinamic/tmc2209.c
 trinamic/tmc2209hal.c
 trinamic/tmc5160.c
 trinamic/tmc5160hal.c
)

set(TRINAMIC_UART_SOURCE
 trinamic_if.c
 motors/trinamic.c
 trinamic/common.c
 trinamic/tmc_interface.c
 trinamic/tmc2209.c
 trinamic/tmc2209hal.c
)

set(LASER_SOURCE
 laser/coolant.c
 laser/lb_clusters.c
)

set(MISC_PLUGINS_SOURCE
 plugins/bltouch.c
 plugins/eventout.c
 plugins/feed_override_m220.c
 plugins/homing_pulloff.c
 plugins/pwm_servo_m280.c
 plugins/rgb_led_m150.c
 plugins/rgb_led_strips.c
)

set(EMBROIDERY_SOURCE
 embroidery/embroidery.c
 embroidery/brother.c
 embroidery/tajima.c
)

set(USB_SOURCE
 usb_serial.c
)

if("${target}" STREQUAL "esp32s3")
list (APPEND SRCS ${USB_SOURCE})
list (APPEND SRCS ${I2S_S3_SOURCE})
else()
list (APPEND SRCS ${I2S_SOURCE})
endif()

list (APPEND SRCS ${MY_PLUGIN_SOURCE})
list (APPEND SRCS ${NETWORKING_SOURCE})
list (APPEND SRCS ${KEYPAD_SOURCE})
list (APPEND SRCS ${TRINAMIC_SPI_SOURCE})
list (APPEND SRCS ${TRINAMIC_UART_SOURCE})
list (APPEND SRCS ${WEBUI_SOURCE})
list (APPEND SRCS ${SDCARD_SOURCE})
list (APPEND SRCS ${BLUETOOTH_SOURCE})
list (APPEND SRCS ${MODBUS_SOURCE})
list (APPEND SRCS ${SPINDLE_SOURCE})
list (APPEND SRCS ${EEPROM_SOURCE})
list (APPEND SRCS ${LASER_SOURCE})
#list (APPEND SRCS ${MISC_PLUGINS_SOURCE})
list (APPEND SRCS ${EMBROIDERY_SOURCE})

if(Ethernet OR ETHERNET_ENABLE EQUAL 1)
list (APPEND SRCS ${ETHERNET_SOURCE})
else()
list (APPEND SRCS ${WIFI_SOURCE})
endif()

idf_component_register(SRCS "${SRCS}" INCLUDE_DIRS ".")

target_compile_definitions("${COMPONENT_LIB}" PUBLIC GRBL_ESP32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LITTLEFS_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC STDIO_FS)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_CUSTOM_FILES=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_DYNAMIC_HEADERS=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_V09=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_11_KEEPALIVE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_POST=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_DYNAMIC_FILE_READ=1)

if(BOARD_BLACKBOX_X32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BLACKBOX_X32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_LIMIT_SIGNALS_INVERT_MASK=7)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_PROBE_SIGNAL_INVERT=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_ENABLE_SIGNALS_INVERT_MASK=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_STEPPER_IDLE_LOCK_TIME=255)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MIN_FEED_RATE_OVERRIDE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MAX_FEED_RATE_OVERRIDE=999)
endif()

target_add_binary_data("${COMPONENT_LIB}" "embedded/favicon.ico" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "embedded/ap_login.html" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "embedded/index.html.gz" BINARY)

unset(BOARD_BLACKBOX_X32 CACHE)
unset(Ethernet CACHE)
unset(AddMyPlugin CACHE)

#target_compile_options("${COMPONENT_LIB}" PRIVATE -Werror -Wall -Wextra -Wmissing-field-initializers)
target_compile_options("${COMPONENT_LIB}" PRIVATE -Wimplicit-fallthrough=1 -Wno-missing-field-initializers -Wno-maybe-uninitialized -Wno-stringop-truncation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
