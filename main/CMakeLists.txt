# Select board to compile for:

# NOTE: Only one board may be enabled!
# If none is enabled pin mappings from generic_map.h will be used.

OPTION(BOARD_BDRING_V3P5 "Compile for v3.5 3-axis board" OFF)
OPTION(BOARD_BDRING_V4 "Compile for bdring v4 3-axis board" ON)
OPTION(BOARD_BDRING_I2S6A "Compile for bdring 6-axis I2S board - untested!" OFF)
OPTION(BOARD_CNC_BOOSTERPACK "Compile for CNC BoosterPack" OFF)
OPTION(BOARD_ESPDUINO32 "Compile for ESPDUINO-32 Wemos D1 R32" OFF)
OPTION(BOARD_SOURCERABBIT_4AXIS "Compile for SourceRabbit 4-axis board" OFF)
OPTION(BOARD_PROTONEER_3XX "Compile for Protoneer v3.xx boards" OFF)
OPTION(BOARD_FYSETC_E4 "Compile for Fysetc E4 v1.0 board" OFF)
OPTION(BOARD_XPRO_V5 "Compile for xPro v5 board - untested!" OFF)
OPTION(BOARD_MKS_DLC32_V2P0 "Compile for MKS DLC2!" OFF)
OPTION(BOARD_MY_MACHINE "Compile for my_machine_map.h" OFF)

OPTION(X_AXIS_GANGED "Compile with ganged X axis" OFF)
OPTION(X_AXIS_SQUARED "Compile with auto-squared X axis" OFF)
OPTION(Y_AXIS_GANGED "Compile with ganged Y axis" OFF)
OPTION(Y_AXIS_SQUARED "Compile with auto-squared Y axis" OFF)
OPTION(Z_AXIS_GANGED "Compile with ganged Z axis" OFF)
OPTION(Z_AXIS_SQUARED "Compile with auto-squared Z axis" OFF)

# The following plugins/options are supported:

OPTION(SafetyDoor "Safety door enable" ON)
OPTION(Networking "Wifi + protocols" OFF)
OPTION(Bluetooth "Bluetooth" OFF)
OPTION(Keypad "I2C Keypad" OFF)
OPTION(SDcard "SD Card Streaming" OFF)
# NOTE: SPI mode Trinamic drivers cannot currently be used with SD Card enabled!
OPTION(TMC2130 "Trinamic TMC2130 driver support" OFF)
OPTION(TMC5160 "Trinamic TMC5160 driver support" OFF)
OPTION(Trinamic "Trinamic driver support via I2C" OFF)
OPTION(WebUI "WebUI services" OFF)
OPTION(WebAuth "WebUI authentication" OFF)
OPTION(MPGMode "MPG mode" OFF)
OPTION(I2SStepping "Use I2S Stepping" OFF)

# Networking options (WiFi)
OPTION(SoftAP "Enable soft AP mode" OFF)
OPTION(FtpDaemon "Enable ftp daemon" OFF) # Requires SDcard On

# For Huanyan spindle support copy the spindle plugin code to the project.
OPTION(HUANYANG "Compile with Huanyang RS485 Spindle support" ON)

# ModBus options
OPTION(RS485_DIR_OUT "Compile with RS485 direction pin enabled" OFF)

# For EEPROM/FRAM support copy the EEPROM plugin code to the project.
OPTION(EEPROM "Compile with I2C EEPROM support" OFF)
OPTION(FRAM "Compile with I2C FRAM support" OFF)
OPTION(NOPROBE "Compile without probe support" OFF)

set(SDCARD_SOURCE sdcard/sdcard.c sdcard/ymodem.c)
set(KEYPAD_SOURCE keypad/keypad.c)
set(WEBUI_SOURCE webui/server.c webui/response.c webui/commands.c webui/flashfs.c )
set(BLUETOOTH_SOURCE bluetooth.c )
set(HUANYANG_SOURCE spindle/modbus spindle/select.c spindle/vfd/spindle.c spindle/vfd/huanyang.c)
set(EEPROM_SOURCE eeprom/eeprom_24LC16B.c eeprom/eeprom_24AAxxx.c)
set(NETWORKING_FTP_SOURCE networking/ftpd.c networking/sfifo.c networking/vfs.c)
set(NETWORKING_SOURCE
 wifi.c
 dns_server.c
 web/backend.c
 networking/http_upload.c
 networking/telnetd.c
 networking/websocketd.c
 networking/base64.c
 networking/sha1.c
 networking/urldecode.c
 networking/utils.c
 networking/multipartparser.c
 networking/networking.c
 )
 set(TRINAMIC_SOURCE
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

set(SRCS
 main.c
 driver.c
 nvs.c
 esp32-hal-uart.c
 ioports.c
 i2c.c
 ioexpand.c
 i2s_out.c
 networking/strutils.c
 grbl/grbllib.c
 grbl/coolant_control.c
 grbl/nvs_buffer.c
 grbl/gcode.c
 grbl/limits.c
 grbl/motion_control.c
 grbl/my_plugin.c
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
 grbl/stepper.c
 grbl/system.c
 grbl/tool_change.c
 grbl/alarms.c
 grbl/errors.c
 grbl/ngc_expr.c
 grbl/ngc_params.c
 grbl/regex.c
 grbl/ioports.c
 grbl/corexy.c
)

if(Networking)
list (APPEND SRCS ${NETWORKING_SOURCE})
if(SDcard AND FtpDaemon)
list (APPEND SRCS ${NETWORKING_FTP_SOURCE})
endif()
endif()

if(SDcard)
list (APPEND SRCS ${SDCARD_SOURCE})
endif()

if(Keypad)
list (APPEND SRCS ${KEYPAD_SOURCE})
endif()

if(Trinamic OR TMC2130 OR TMC5160)
list (APPEND SRCS ${TRINAMIC_SOURCE})
endif()

if(WebUI)
list (APPEND SRCS ${WEBUI_SOURCE})
endif()

if(Bluetooth)
list (APPEND SRCS ${BLUETOOTH_SOURCE})
endif()

if(HUANYANG)
list (APPEND SRCS ${HUANYANG_SOURCE})
endif()

if(EEPROM OR FRAM OR BOARD_CNC_BOOSTERPACK)
list (APPEND SRCS ${EEPROM_SOURCE})
endif()

idf_component_register(SRCS "${SRCS}" INCLUDE_DIRS ".")

target_compile_definitions("${COMPONENT_LIB}" PUBLIC GRBL_ESP32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC OVERRIDE_MY_MACHINE)

if(BOARD_BDRING_V3P5)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_V3P5)
elseif(BOARD_BDRING_V4)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_V4)
elseif(BOARD_BDRING_I2S6A)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BDRING_I2S6A)
elseif(BOARD_ESPDUINO32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_ESPDUINO32)
elseif(BOARD_CNC_BOOSTERPACK)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_CNC_BOOSTERPACK)
elseif(BOARD_SOURCERABBIT_4AXIS)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_SOURCERABBIT_4AXIS)
elseif(BOARD_PROTONEER_3XX)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_PROTONEER_3XX)
elseif(BOARD_FYSETC_E4)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_FYSETC_E4)
elseif(BOARD_XPRO_V5)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_XPRO_V5)
elseif(BOARD_MKS_DLC32_V2P0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_MKS_DLC32_V2P0)
elseif(BOARD_MY_MACHINE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_MY_MACHINE)
endif()

if(X_AXIS_GANGED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC X_GANGED)
endif()
if(X_AXIS_SQUARED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC X_AUTO_SQUARE)
endif()
if(Y_AXIS_GANGED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC Y_GANGED)
endif()
if(Y_AXIS_SQUARED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC Y_AUTO_SQUARE)
endif()
if(Z_AXIS_GANGED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC Z_GANGED)
endif()
if(Z_AXIS_SQUARED)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC Z_AUTO_SQUARE)
endif()

if(TMC2130)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE=2130)
endif()

if(TMC5160)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE=5160)
endif()

if(SafetyDoor)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SAFETY_DOOR_ENABLE=1)
endif()

if(Networking)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NETWORKING_ENABLE)
if(SDcard AND FtpDaemon)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC FTP_ENABLE)
endif()
endif()

if(SoftAP)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_SOFTAP)
endif()

if(Bluetooth)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BLUETOOTH_ENABLE)
endif()

if(SDcard)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SDCARD_ENABLE)
endif()

if(WebUI)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_ENABLE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC STDIO_FS)
target_add_binary_data("${COMPONENT_LIB}" "index.html.gz" BINARY)
if(WebAuth)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_AUTH_ENABLE)
endif()
endif()

if(Trinamic)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE)
endif()

if(Keypad)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC KEYPAD_ENABLE)
endif()

if(MPGMode)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MPG_MODE_ENABLE)
endif()

if(HUANYANG)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC VFD_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE_RPM_CONTROLLED)
endif()

if(RS485_DIR_OUT)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC RS485_DIR_ENABLE)
endif()

if(EEPROM OR BOARD_CNC_BOOSTERPACK)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_ENABLE)
endif()

if(FRAM)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_IS_FRAM)
endif()

if(NOPROBE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NOPROBE)
endif()

target_add_binary_data("${COMPONENT_LIB}" "favicon.ico" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "index.html" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "ap_login.html" BINARY)

unset(BOARD_BDRING_V3P5 CACHE)
unset(BOARD_BDRING_V4 CACHE)
unset(BOARD_BDRING_I2S6A CACHE)
unset(BOARD_CNC_BOOSTERPACK CACHE)
unset(BOARD_ESPDUINO32 CACHE)
unset(BOARD_SOURCERABBIT_4AXIS CACHE)
unset(BOARD_PROTONEER_3XX CACHE)
unset(BOARD_FYSETC_E4 CACHE)
unset(BOARD_XPRO_V5 CACHE)
unset(BOARD_MKS_DLC32_V2P0 CACHE)
unset(BOARD_MY_MACHINE CACHE)

unset(X_AXIS_GANGED CACHE)
unset(X_AXIS_SQUARED CACHE)
unset(Y_AXIS_GANGED CACHE)
unset(Y_AXIS_SQUARED CACHE)
unset(Z_AXIS_GANGED CACHE)
unset(Z_AXIS_SQUARED CACHE)

unset(SafetyDoor CACHE)
unset(Networking CACHE)
unset(FtpDaemon CACHE)
unset(SoftAP CACHE)
unset(Bluetooth CACHE)
unset(Keypad CACHE)
unset(SDcard CACHE)
unset(Trinamic CACHE)
unset(TMC2130 CACHE)
unset(TMC5160 CACHE)
unset(WebUI CACHE)
unset(WebAuth CACHE)
unset(MPGMode CACHE)
unset(HUANYANG CACHE)
unset(RS485_DIR_OUT CACHE)
unset(EEPROM CACHE)
unset(FRAM CACHE)
unset(NOPROBE CACHE)

#target_compile_options("${COMPONENT_LIB}" PRIVATE -Werror -Wall -Wextra -Wmissing-field-initializers)
target_compile_options("${COMPONENT_LIB}" PRIVATE -Wimplicit-fallthrough=1 -Wno-missing-field-initializers -Wno-maybe-uninitialized)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
