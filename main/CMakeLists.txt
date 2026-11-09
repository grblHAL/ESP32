# NOTE: most configuration options has been moved to my_machine.h.

# Enable these boards here and not in my_machine.h if you want to override board specific default core options.
# NOTE: if reflashing/updating these options will not be changed, use $RST=$ to propagate them.
OPTION(BOARD_BLACKBOX_X32 "Compile for BlackBox X32" OFF)
OPTION(BOARD_CV30 "Compile for the Creality CV-30 laser box (ESP32-S2); on by default for esp32s2" ON)

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
 spi.c
 neopixel_rmt.c
 neopixel_spi.c
 boards/BlackBoxX32.c
 boards/ooznest_cnc.c
 grbl/grbllib.c
 grbl/canbus.c
 grbl/coolant_control.c
 grbl/crossbar.c
 grbl/crc.c
 grbl/nvs_buffer.c
 grbl/gcode.c
 grbl/machine_limits.c
 grbl/messages.c
 grbl/modbus.c
 grbl/modbus_rtu.c
 grbl/motion_control.c
 grbl/nuts_bolts.c
 grbl/override.c
 grbl/probe.c
 grbl/planner.c
 grbl/protocol.c
 grbl/report.c
 grbl/settings.c
 grbl/sleep.c
 grbl/spindle_control.c
 grbl/state_machine.c
 grbl/stream.c
 grbl/stream_file.c
 grbl/stream_json.c
 grbl/stepper.c
 grbl/stepper2.c
 grbl/strutils.c
 grbl/system.c
 grbl/tool_change.c
 grbl/alarms.c
 grbl/errors.c
 grbl/ngc_expr.c
 grbl/ngc_params.c
 grbl/ngc_flowctrl.c
 grbl/regex.c
 grbl/ioports.c
 grbl/encoders.c
 grbl/utf8.c
 grbl/vfs.c
 grbl/fs_device.c
 grbl/fs_embedded.c
 grbl/fs_ram.c
 grbl/kinematics/asymmetric_ganging.c
 grbl/kinematics/corexy.c
 grbl/kinematics/wall_plotter.c
 grbl/kinematics/delta.c
 grbl/kinematics/polar.c
 grbl/kinematics/asymmetric_ganging.c
 littlefs_hal.c
 littlefs/lfs.c
 littlefs/lfs_util.c
 sdcard/fs_littlefs.c
)

if(AddMyPlugin OR EXISTS ${CMAKE_CURRENT_LIST_DIR}/my_plugin.c)
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
 sdcard/fs_stream.c
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
 webui/webui_files.c
)

if("${target}" STREQUAL "esp32s3")
set(BLUETOOTH_SOURCE
 bluetooth_le.c
)
else()
set(BLUETOOTH_SOURCE
 bluetooth.c
)
endif()

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
 spindle/vfd/nowforever.c
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
 networking/etag.c
 networking/webdav.c
 networking/ssdp.c
 networking/ftpd.c
 networking/sfifo.c
)

set(TRINAMIC_SPI_SOURCE
 tmc_spi.c
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
 tmc_uart.c
 motors/trinamic.c
 trinamic/common.c
 trinamic/tmc_interface.c
 trinamic/tmc2209.c
 trinamic/tmc2209hal.c
)

set(LASER_SOURCE
 laser/co2.c
 laser/coolant.c
 laser/lb_clusters.c
 laser/ppi.c
)

set(MISC_PLUGINS_SOURCE
 plugins/bltouch.c
 plugins/eventout.c
 plugins/feed_override_m220.c
 plugins/fnc_expander.c
 plugins/hc595.c
 plugins/homing_pulloff.c
 plugins/mcp3221.c
 plugins/mcp4725.c
 plugins/mcp23017.c
 plugins/pca9654e.c
 plugins/pwm_servo_m280.c
 plugins/rgb_led_m150.c
 plugins/tooltable.c
)

set(EMBROIDERY_SOURCE
 embroidery/embroidery.c
 embroidery/brother.c
 embroidery/tajima.c
)

set(OPENPNP_SOURCE
 openpnp/openpnp.c
)

set(USB_SOURCE
 usb_serial.c
)

if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/../3rdParty.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../3rdParty.cmake)
endif()

if("${target}" STREQUAL "esp32s3")
list (APPEND SRCS ${USB_SOURCE})
list (APPEND SRCS ${I2S_S3_SOURCE})
elseif("${target}" STREQUAL "esp32s2")
# ESP32-S2: native USB like the S3, no I2S stepping driver, no Bluetooth.
list (APPEND SRCS ${USB_SOURCE})
set(BLUETOOTH_SOURCE "")
list (APPEND SRCS cv30.c)
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
list (APPEND SRCS ${SPINDLE_SOURCE})
list (APPEND SRCS ${EEPROM_SOURCE})
list (APPEND SRCS ${LASER_SOURCE})
list (APPEND SRCS ${MISC_PLUGINS_SOURCE})
list (APPEND SRCS ${EMBROIDERY_SOURCE})
list (APPEND SRCS ${OPENPNP_SOURCE})

list (APPEND SRCS ${ETHERNET_SOURCE})
list (APPEND SRCS ${WIFI_SOURCE})

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

if(BOARD_CV30 AND "${target}" STREQUAL "esp32s2")
# Creality CV-30 laser box. Motion defaults match the stock firmware's settings
# (so a stock Creality setup behaves as before); they apply on first boot or
# after $RST=$. Custom frames set $3, $23, $130/$131 etc. themselves.
# Changes from stock: laser enable is active-high (metered: it switches the
# laser's 24 V through Q1), homing is X+Y in one cycle (no Z) and homing sets
# the machine origin to 0,0.
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_CV30)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC CV30_BUILD="9") # shown in $I [BOARD:...]; bump for every CV-30-grblhal-N.bin
target_compile_definitions("${COMPONENT_LIB}" PUBLIC PROBE_ENABLE=0) # no probe input on this board
# Networking: Wi-Fi (station + setup access point), telnet for LightBurn,
# websocket, HTTP + WebDAV and FTP for file upload to the microSD card.
# The ESP3D WebUI is left out for now (size; uploads go to the SD card and
# $UPDATE hands them to the stock installer, see cv30.c).
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_SOFTAP=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBSOCKET_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC HTTP_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBDAV_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SDCARD_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MDNS_ENABLE=1) # cv30-laser.local
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NETWORK_STA_HOSTNAME="cv30-laser")
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NETWORK_AP_SSID="CV30-Laser-Setup")
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NETWORK_AP_HOSTNAME="cv30-laser")
if(CONFIG_ESP_CONSOLE_USB_CDC)
# Debug build (sdkconfig.debug): the IDF console, and with it panic output,
# goes out over native USB, so the grblHAL stream moves to UART0.
target_compile_definitions("${COMPONENT_LIB}" PUBLIC USB_SERIAL_CDC=0)
else()
target_compile_definitions("${COMPONENT_LIB}" PUBLIC USB_SERIAL_CDC=1) # the box talks over its native USB port
endif()
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_LASER_MODE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_INVERT_SPINDLE_PWM_PIN=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_INVERT_SPINDLE_ENABLE_PIN=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_SPINDLE_PWM_FREQ=1000)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_SPINDLE_RPM_MAX=1000.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_STEP_PULSE_MICROSECONDS=10.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_STEPPER_IDLE_LOCK_TIME=250)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_DIR_SIGNALS_INVERT_MASK=2)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_DIR_MASK=2)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_FORCE_SET_ORIGIN=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_CYCLE_0=3)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_CYCLE_1=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_SOFT_LIMIT_ENABLE=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_FEED_RATE=1500.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_SEEK_RATE=4500.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_DEBOUNCE_DELAY=20)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_HOMING_PULLOFF=5.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_X_STEPS_PER_MM=80.008f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_Y_STEPS_PER_MM=80.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_X_MAX_RATE=10000.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_Y_MAX_RATE=10000.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_X_ACCELERATION=500.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_Y_ACCELERATION=500.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_X_MAX_TRAVEL=220.0f)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_Y_MAX_TRAVEL=240.0f)
endif()

unset(BOARD_BLACKBOX_X32 CACHE)
unset(BOARD_CV30 CACHE)
unset(Ethernet CACHE)
unset(AddMyPlugin CACHE)

#target_compile_options("${COMPONENT_LIB}" PRIVATE -Werror -Wall -Wextra -Wmissing-field-initializers)
target_compile_options("${COMPONENT_LIB}" PRIVATE -Wimplicit-fallthrough=1 -Wno-missing-field-initializers -Wno-maybe-uninitialized -Wno-stringop-truncation -Wno-expansion-to-defined)
# IDF 5 toolchains define uint32_t as long unsigned int, so the shared grblHAL
# core's "%u" for uint32_t warns, and IDF makes that an error. Keep it a
# warning rather than patching the core submodules.
if(IDF_VERSION_MAJOR GREATER_EQUAL 5)
target_compile_options("${COMPONENT_LIB}" PRIVATE -Wno-error=format)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
