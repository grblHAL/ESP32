# Select board to compile for:

# NOTE: Only one board may be enabled!
# If none is enabled pin mappings from generic_map.h will be used.

OPTION(BOARD_BLACKBOX_X32 "Compile for BlackBox X32" OFF)
OPTION(BOARD_BDRING_V3P5 "Compile for v3.5 3-axis board" OFF)
OPTION(BOARD_BDRING_V4 "Compile for bdring v4 3-axis board" OFF)
OPTION(BOARD_BDRING_I2S6A "Compile for bdring 6-axis I2S board - untested!" OFF)
OPTION(BOARD_ESPDUINO32 "Compile for ESPDUINO-32 Wemos D1 R32" OFF)
OPTION(BOARD_SOURCERABBIT_4AXIS "Compile for SourceRabbit 4-axis board" OFF)
OPTION(BOARD_PROTONEER_3XX "Compile for Protoneer v3.xx boards" OFF)
OPTION(BOARD_FYSETC_E4 "Compile for Fysetc E4 v1.0 board" OFF)
OPTION(BOARD_XPRO_V5 "Compile for xPro v5 board - untested!" OFF)
OPTION(BOARD_MKS_DLC32_V2P0 "Compile for MKS DLC2" OFF)
OPTION(BOARD_MKS_TINYBEE_V1 "Compile for MKS Tinybee v1" OFF)
OPTION(BOARD_CNC3040 "Compile for CNC3040 controller" OFF)
OPTION(BOARD_ROOTCNC_V2 "Compile for RootCNC v2.x controller" OFF)
OPTION(BOARD_ROOTCNC_V3 "Compile for RootCNC v3 controller" OFF)
OPTION(BOARD_CNC_BOOSTERPACK "Compile for CNC BoosterPack" OFF)
OPTION(BOARD_MY_MACHINE "Compile for my_machine_map.h" OFF)

OPTION(X_AXIS_GANGED "Compile with ganged X axis" OFF)
OPTION(X_AXIS_SQUARED "Compile with auto-squared X axis" OFF)
OPTION(Y_AXIS_GANGED "Compile with ganged Y axis" OFF)
OPTION(Y_AXIS_SQUARED "Compile with auto-squared Y axis" OFF)
OPTION(Z_AXIS_GANGED "Compile with ganged Z axis" OFF)
OPTION(Z_AXIS_SQUARED "Compile with auto-squared Z axis" OFF)

# Driver options
OPTION(NOPROBE "Compile without probe support" OFF)
OPTION(SafetyDoor "Safety door enable" OFF)
OPTION(MPGMode "MPG mode" OFF)
OPTION(I2SStepping "Use I2S Stepping" OFF)

# The following plugin options are supported:

# Networking and WebUI plugin options
OPTION(WiFi "Wifi + protocols" OFF)
OPTION(Ethernet "Ethernet + protocols" OFF) # NOTE: no board maps has support for this!
OPTION(Bluetooth "Bluetooth" OFF)
OPTION(SoftAP "Enable soft AP mode" OFF) # WiFi only
OPTION(FtpDaemon "Enable ftp daemon" OFF) # Requires SDcard On
OPTION(WebDAV "Enable WebDAV protocol" OFF) # Requires SDcard On
OPTION(mDNS "Enable mDNS protocol" OFF)
OPTION(SSDP "Enable SSDP protocol" OFF)
OPTION(MQTTclient "Enable MQTT client API" OFF) # For use by plugin code.
OPTION(WebUI "WebUI services" OFF)
OPTION(WebAuth "WebUI authentication" OFF)

# SD Card plugin options
OPTION(SDcard "SD Card Streaming" OFF)

# Motor plugin options
# NOTE: SPI mode Trinamic drivers cannot currently be used with SD Card enabled!
OPTION(TMC2130 "Trinamic TMC2130 driver support" OFF)
OPTION(TMC2209 "Trinamic TMC2209 driver support" OFF)
OPTION(TMC2209_HWADDR "Trinamic TMC2209 drivers uses hardware addressing" OFF)
OPTION(TMC5160 "Trinamic TMC5160 driver support" OFF)
OPTION(Trinamic "Trinamic driver support via I2C" OFF)

# Spindle plugin options
# NOTE: up to four spindle options can be enabled!
OPTION(SPINDLE_HUANYANG1 "Compile with Huanyang v1 Modbus spindle support" OFF)
OPTION(SPINDLE_HUANYANG2 "Compile with Huanyang v2 Modbus spindle support" OFF)
OPTION(SPINDLE_GS20 "Compile with GS20 Modbus spindle support" OFF)
OPTION(SPINDLE_YL620A "Compile with Yalang YL620A Modbus spindle support" OFF)
OPTION(SPINDLE_MODVFD "Compile with generic MODVFD Modbus spindle support" OFF)
OPTION(SPINDLE_H100 "Compile with H100 Modbus spindle support" OFF)
OPTION(SPINDLE_STEPPER "Compile with stepper spindle" OFF)
OPTION(SPINDLE_PWM0 "Compile with standard PWM spindle" OFF)
OPTION(SPINDLE_PWM0_CLONE "Compile with standard PWM spindle + clone" OFF)
OPTION(SPINDLE_ALL "Compile with all spindles, including standard PWM spindle" OFF)
# RS485 communication channel option
OPTION(RS485_DIR_OUT "Compile with RS485 direction pin enabled" OFF)

# EEPROM/FRAM plugin options
OPTION(EEPROM "Compile with I2C EEPROM support" OFF)
OPTION(FRAM "Compile with I2C FRAM support" OFF)

# Laser plugin options
OPTION(LBCLUSTER "Compile with LaserBurn cluster support" OFF)
OPTION(LASERCOOLANT "Compile with Laser coolant plugin support" OFF)

# Keypad plugin options
OPTION(Keypad "I2C Keypad" OFF)

# Embroidery plugin options
OPTION(Embroidery "Embroidery" OFF)

# Add NGC expression support
OPTION(NGCExpr "Add NGC expression support" OFF)

# Add my_plugin.c to compilation, add my_plugin.c to the main folder first
OPTION(AddMyPlugin "Add my_plugin.c" OFF)

# --- Do not change anything below unless you understand what you are doing ---

set(SDCARD_SOURCE
 sdcard/sdcard.c
 sdcard/ymodem.c
 sdcard/fs_fatfs.c
 sdcard/macros.c
)
set(KEYPAD_SOURCE
 keypad/keypad.c
 keypad/macros.c
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
set(EMBROIDERY_SOURCE
 embroidery/embroidery.c
 embroidery/brother.c
 embroidery/tajima.c
)
set(USB_SOURCE
 usb_serial.c
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

set(SRCS
 main.c
 driver.c
 nvs.c
 uart_serial.c
 ioports.c
 i2c.c
 ioexpand.c
 i2s_out.c
 BlackBoxX32.c
 networking/strutils.c
 grbl/grbllib.c
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

list (APPEND SRCS ${MY_PLUGIN_SOURCE})

if(Wifi OR WIFI_ENABLE EQUAL 1)
list (APPEND SRCS ${WIFI_SOURCE})
endif()

if(Ethernet OR ETHERNET_ENABLE EQUAL 1)
list (APPEND SRCS ${ETHERNET_SOURCE})
endif()

if(WiFi OR WIFI_ENABLE EQUAL 1 OR Ethernet OR ETHERNET_ENABLE EQUAL 1)
list (APPEND SRCS ${NETWORKING_SOURCE})
if((SDcard OR WebUI OR SDCARD_ENABLE GREATER_EQUAL 1) AND (FtpDaemon OR FTP_ENABLE EQUAL 1))
list (APPEND SRCS ${NETWORKING_FTP_SOURCE})
endif()
endif()

if(Keypad OR KEYPAD_ENABLE GREATER_EQUAL 1 OR MACROS_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${KEYPAD_SOURCE})
endif()

if(Trinamic OR TMC2130 OR TMC5160 OR TRINAMIC_SPI_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${TRINAMIC_SPI_SOURCE})
endif()

if(TMC2209 OR TRINAMIC_UART_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${TRINAMIC_UART_SOURCE})
endif()

if(WebUI OR WEBUI_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${WEBUI_SOURCE})
list (APPEND SRCS ${SDCARD_SOURCE})
elseif(SDcard OR SDCARD_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${SDCARD_SOURCE})
endif()
if(Bluetooth OR BLUETOOTH_ENABLE EQUAL 1)
list (APPEND SRCS ${BLUETOOTH_SOURCE})
endif()

if(SPINDLE_ALL OR SPINDLE_HUANYANG1 OR SPINDLE_HUANYANG2 OR SPINDLE_GS20 OR SPINDLE_YL620A OR SPINDLE_MODVFD OR SPINDLE_H100)
OPTION(MODBUS_SPINDLE "" ON)
else()
OPTION(MODBUS_SPINDLE "" OFF)
endif()

if(MODBUS_SPINDLE OR VFD_ENABLE EQUAL -1 OR VFD_ENABLE GREATER_EQUAL 1 OR MODBUS_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${MODBUS_SOURCE})
endif()

#if(MODBUS_SPINDLE OR SPINDLE_STEPPER OR VFD_ENABLE EQUAL -1 OR VFD_ENABLE GREATER_EQUAL 1)
list (APPEND SRCS ${SPINDLE_SOURCE})
#endif()

if(EEPROM OR FRAM OR BOARD_CNC_BOOSTERPACK OR DEFINED(BOARD_CNC_BOOSTERPACK) OR EEPROM_ENABLE EQUAL 1)
list (APPEND SRCS ${EEPROM_SOURCE})
endif()
list (APPEND SRCS ${EEPROM_SOURCE})
if(LBCLUSTER OR LASERCOOLANT OR LB_CLUSTERS_ENABLE EQUAL 1)
list (APPEND SRCS ${LASER_SOURCE})
endif()

if(Embroidery OR EMBROIDERY_ENABLE EQUAL 1)
list (APPEND SRCS ${EMBROIDERY_SOURCE})
endif()

if("${target}" STREQUAL "esp32s3")
list (APPEND SRCS ${USB_SOURCE})
endif()

idf_component_register(SRCS "${SRCS}" INCLUDE_DIRS ".")

target_compile_definitions("${COMPONENT_LIB}" PUBLIC STEP_INJECT_ENABLE=1)

target_compile_definitions("${COMPONENT_LIB}" PUBLIC GRBL_ESP32)

idf_build_get_property(target IDF_TARGET)
if("${target}" STREQUAL "esp32s3")
target_compile_definitions("${COMPONENT_LIB}" PUBLIC GRBL_ESP32S3=1)
endif()

target_compile_definitions("${COMPONENT_LIB}" PUBLIC OVERRIDE_MY_MACHINE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LITTLEFS_ENABLE=1)

if(BOARD_BLACKBOX_X32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_BLACKBOX_X32)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_LIMIT_SIGNALS_INVERT_MASK=7)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_PROBE_SIGNAL_INVERT=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_ENABLE_SIGNALS_INVERT_MASK=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC DEFAULT_STEPPER_IDLE_LOCK_TIME=255)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MIN_FEED_RATE_OVERRIDE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MAX_FEED_RATE_OVERRIDE=999)
elseif(BOARD_BDRING_V3P5)
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
elseif(BOARD_MKS_TINYBEE_V1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_MKS_TINYBEE_V1)
elseif(BOARD_CNC3040)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_CNC3040)
elseif(BOARD_ROOTCNC_V2)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_ROOTCNC_V2)
elseif(BOARD_ROOTCNC_V3)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_ROOTCNC_V3)
elseif(BOARD_MY_MACHINE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BOARD_MY_MACHINE)
endif()

if(WebUI OR WEBUI_ENABLE GREATER_EQUAL 1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_INFLASH=1)
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

if(TMC2209)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE=2209)
if(TMC2209_HWADDR)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_UART_ENABLE=2)
else()
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_UART_ENABLE=1)
endif()
endif()

if(TMC5160)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE=5160)
endif()

if(SafetyDoor)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SAFETY_DOOR_ENABLE=1)
endif()

if(WiFi)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TELNET_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBSOCKET_ENABLE=1)
endif()

if(Ethernet)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC ETHERNET_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TELNET_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBSOCKET_ENABLE=1)
endif()

if(WiFi OR Ethernet)
if((SDcard OR WebUI) AND FtpDaemon)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC FTP_ENABLE=1)
endif()
if((SDcard OR WebUI) AND WebDAV)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBDAV_ENABLE=1)
endif()
if(mDNS)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MDNS_ENABLE=1)
endif()
if(SSDP)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SSDP_ENABLE=1)
endif()
if(MQTTclient)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MQTT_ENABLE=1)
endif()
endif()

if(SoftAP)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WIFI_SOFTAP=1)
endif()

if(Bluetooth)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC BLUETOOTH_ENABLE=1)
endif()

if(WebUI)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_ENABLE=3)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC STDIO_FS)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC HTTP_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_CUSTOM_FILES=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_DYNAMIC_HEADERS=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_V09=0)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_11_KEEPALIVE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_SUPPORT_POST=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LWIP_HTTPD_DYNAMIC_FILE_READ=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SDCARD_ENABLE=1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_AUTO_REPORT_INTERVAL=0)
if(WebAuth)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC WEBUI_AUTH_ENABLE=1)
endif()
elseif(SDcard)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SDCARD_ENABLE=1)
endif()

if(Trinamic)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC TRINAMIC_ENABLE)
endif()

if(Keypad)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC KEYPAD_ENABLE=1)
endif()

if(MPGMode)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC MPG_ENABLE=1)
endif()

if(NGCExpr)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC NGC_EXPRESSIONS_ENABLE=1)
endif()

if(Embroidery)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EMBROIDERY_ENABLE=1)
endif()

if(SPINDLE_ALL)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE0_ENABLE=-1)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC N_SPINDLE=8)
else()
	if(SPINDLE_HUANYANG1)
		list(APPEND SPINDLES 1)
	endif()
	if(SPINDLE_HUANYANG2)
		list(APPEND SPINDLES 2)
	endif()
	if(SPINDLE_GS20)
		list(APPEND SPINDLES 3)
	endif()
	if(SPINDLE_YL620A)
		list(APPEND SPINDLES 4)
	endif()
	if(SPINDLE_MODVFD)
		list(APPEND SPINDLES 5)
	endif()
	if(SPINDLE_H100)
		list(APPEND SPINDLES 6)
	endif()
	if(SPINDLE_PWM0)
		list(APPEND SPINDLES 11)
	endif()
    if(SPINDLE_PWM0_CLONE)
        list(APPEND SPINDLES 11)
        list(APPEND SPINDLES 17)
    endif()
	if(SPINDLE_STEPPER)
		list(APPEND SPINDLES 19)
	endif()
endif()

list(LENGTH SPINDLES SPINDLENUM) 

if(SPINDLENUM GREATER_EQUAL 1)
list(GET SPINDLES 0 SPINDLE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE0_ENABLE=${SPINDLE})
endif()
if(SPINDLENUM GREATER_EQUAL 2)
list(GET SPINDLES 1 SPINDLE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE1_ENABLE=${SPINDLE})
endif()
if(SPINDLENUM GREATER_EQUAL 3)
list(GET SPINDLES 2 SPINDLE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE2_ENABLE=${SPINDLE})
endif()
if(SPINDLENUM GREATER_EQUAL 4)
list(GET SPINDLES 3 SPINDLE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC SPINDLE3_ENABLE=${SPINDLE})
endif()

if(RS485_DIR_OUT)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC RS485_DIR_ENABLE=1)
endif()

if(EEPROM OR BOARD_CNC_BOOSTERPACK)
#target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_ENABLE=1)
endif()

if(FRAM)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC EEPROM_IS_FRAM=1)
endif()

if(NOPROBE)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC PROBE_ENABLE=0)
else()
target_compile_definitions("${COMPONENT_LIB}" PUBLIC PROBE_ENABLE=1)
endif()

if(LBCLUSTER)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LB_CLUSTERS_ENABLE=1)
endif()

if(LASERCOOLANT)
target_compile_definitions("${COMPONENT_LIB}" PUBLIC LASER_COOLANT_ENABLE=1)
endif()

target_add_binary_data("${COMPONENT_LIB}" "embedded/favicon.ico" BINARY)
target_add_binary_data("${COMPONENT_LIB}" "embedded/ap_login.html" BINARY)
if(WebUI OR WEBUI_ENABLE GREATER_EQUAL 1)
target_add_binary_data("${COMPONENT_LIB}" "embedded/index.html.gz" BINARY)
endif()

unset(BOARD_BLACKBOX_X32 CACHE)
unset(BOARD_ROOTCNC_V2 CACHE)
unset(BOARD_ROOTCNC_V3 CACHE)
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
unset(BOARD_MKS_TINYBEE_V1 CACHE)
unset(BOARD_CNC3040 CACHE)
unset(BOARD_MY_MACHINE CACHE)

unset(X_AXIS_GANGED CACHE)
unset(X_AXIS_SQUARED CACHE)
unset(Y_AXIS_GANGED CACHE)
unset(Y_AXIS_SQUARED CACHE)
unset(Z_AXIS_GANGED CACHE)
unset(Z_AXIS_SQUARED CACHE)

unset(SafetyDoor CACHE)
unset(WiFi CACHE)
unset(Ethernet CACHE)
unset(FtpDaemon CACHE)
unset(WebDAV CACHE)
unset(mDNS CACHE)
unset(SSDP CACHE)
unset(MQTTclient CACHE)
unset(SoftAP CACHE)
unset(Bluetooth CACHE)
unset(Keypad CACHE)
unset(SDcard CACHE)
unset(Trinamic CACHE)
unset(TMC2130 CACHE)
unset(TMC2209 CACHE)
unset(TMC2209_HWADDR CACHE)
unset(TMC5160 CACHE)
unset(WebUI CACHE)
unset(WebAuth CACHE)
unset(MPGMode CACHE)
unset(NGCExpr CACHE)
unset(Embroidery CACHE)
unset(AddMyPlugin CACHE)
unset(SPINDLE_ALL CACHE)
unset(SPINDLE_HUANYANG1 CACHE)
unset(SPINDLE_HUANYANG2 CACHE)
unset(SPINDLE_GS20 CACHE)
unset(SPINDLE_YL620A CACHE)
unset(SPINDLE_MODVFD CACHE)
unset(SPINDLE_H100 CACHE)
unset(SPINDLE_PWM0 CACHE)
unset(SPINDLE_PWM0_CLONE CACHE)
unset(SPINDLE_STEPPER CACHE)
unset(MODBUS_SPINDLE CACHE)
unset(RS485_DIR_OUT CACHE)
unset(EEPROM CACHE)
unset(FRAM CACHE)
unset(NOPROBE CACHE)
unset(LBCLUSTER CACHE)
unset(LASERCOOLANT CACHE)

#target_compile_options("${COMPONENT_LIB}" PRIVATE -Werror -Wall -Wextra -Wmissing-field-initializers)
target_compile_options("${COMPONENT_LIB}" PRIVATE -Wimplicit-fallthrough=1 -Wno-missing-field-initializers -Wno-maybe-uninitialized -Wno-stringop-truncation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
