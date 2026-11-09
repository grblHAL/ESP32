/*
  cv30.c - Creality CV-30 laser box: boot guard and return-to-stock

  The box's stock bootloader (ESP-IDF 4.4.2) has no app rollback, so a build
  of this firmware that crashes on start would loop forever. The stock
  firmware stays in ota_0 for good; grblHAL is installed into ota_1 by the
  stock firmware's own microSD updater (a file named CV-30-*.bin). So:

  - Boot guard: crash/watchdog resets are counted in RTC memory (kept across
    resets, cleared by a power cycle). After CV30_GUARD_LIMIT of them in a
    row without CV30_GUARD_HEALTHY_MS of clean running, boot goes back to the
    stock firmware.
  - Power-cycle escape: CV30_CYCLE_LIMIT power-ons in a row, each cut off
    within CV30_GUARD_HEALTHY_MS, also go back to the stock firmware. Needs
    no tools or working USB (the count is kept in NVS).
  - $STOCK: switch to the stock firmware now.
  - $UPDATE[=file]: hand a CV-30-*.bin on the microSD card (uploaded over
    Wi-Fi or copied there) to the stock installer: checks it, sets other
    CV-30-*.bin files aside, then restarts into the stock firmware, which
    installs it into this slot and boots it. To come back, give the stock
    updater a CV-30-*.bin with a file name it has not installed before.

  Part of grblHAL

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#ifdef BOARD_CV30

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <dirent.h>
#include <sys/stat.h>

#include "esp_attr.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_app_format.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_struct.h"
#include "soc/system_reg.h"
#include "driver/gpio.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#include "cv30.h"

#define CV30_STOCK_PREFIX       "CV-30-3DPrint"   // esp_app_desc_t.project_name of the stock firmware
#define CV30_GUARD_MAGIC        0x43563330        // "CV30"
#define CV30_GUARD_LIMIT        3
#define CV30_GUARD_HEALTHY_MS   10000
#define CV30_CYCLE_LIMIT        3
#define CV30_NVS_NAMESPACE      "cv30"
#define CV30_NVS_CYCLES         "pwr_cycles"
#define CV30_NVS_UPDATE         "last_update"
#define CV30_SD_ROOT            "/sdcard"
#define CV30_SLOT_SIZE          (1408 * 1024)     // ota_0/ota_1 in the stock partition table
// Where the stock firmware records the last file it installed (and skips it).
#define STOCK_NVS_NAMESPACE     "Grbl_ESP32"  // checked in the 2026-11-06 backup (nvs_tool.py)
#define STOCK_NVS_FILE          "CV-30 file name"

static RTC_NOINIT_ATTR uint32_t guard_magic;
static RTC_NOINIT_ATTR uint32_t guard_crashes;

static const esp_partition_t *stock_partition (void)
{
    const esp_partition_t *running = esp_ota_get_running_partition(), *stock = NULL;
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);

    for(; it != NULL && stock == NULL; it = esp_partition_next(it)) {
        const esp_partition_t *p = esp_partition_get(it);
        esp_app_desc_t desc;
        if(p->address != running->address && esp_ota_get_partition_description(p, &desc) == ESP_OK &&
            strncmp(desc.project_name, CV30_STOCK_PREFIX, strlen(CV30_STOCK_PREFIX)) == 0)
            stock = p;
    }
    esp_partition_iterator_release(it);

    return stock;
}

static bool boot_stock (void)
{
    const esp_partition_t *stock = stock_partition();

    return stock != NULL && esp_ota_set_boot_partition(stock) == ESP_OK;
}

// Power-on count kept in NVS (RTC memory does not survive a power cut).
// Returns the count including this boot; reset == true clears it.
static uint8_t power_cycles (bool reset)
{
    nvs_handle_t nvs;
    uint8_t count = 0;

    if(nvs_open(CV30_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        if(!reset) {
            nvs_get_u8(nvs, CV30_NVS_CYCLES, &count);
            count++;
        }
        nvs_set_u8(nvs, CV30_NVS_CYCLES, count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    return count;
}

static void guard_healthy (void *arg)
{
    guard_crashes = 0;
    power_cycles(true);
}

void cv30_boot_guard (void)
{
    static esp_timer_handle_t healthy_timer;

    switch(esp_reset_reason()) {

        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            if(guard_magic != CV30_GUARD_MAGIC)
                guard_crashes = 0;
            guard_crashes++;
            break;

        default: // power-on, $STOCK/esp_restart, brownout...: start afresh
            guard_crashes = 0;
            break;
    }
    guard_magic = CV30_GUARD_MAGIC;

    if(esp_reset_reason() == ESP_RST_POWERON) {
        esp_err_t ret = nvs_flash_init(); // app_main initialises it again later; harmless
        if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            nvs_flash_init();
        }
        if(power_cycles(false) >= CV30_CYCLE_LIMIT) {
            power_cycles(true);
            if(boot_stock())
                esp_restart();
        }
    }

    if(guard_crashes >= CV30_GUARD_LIMIT) {
        guard_crashes = 0;
        if(boot_stock())
            esp_restart();
    }

    const esp_timer_create_args_t args = { .callback = guard_healthy, .name = "cv30_guard" };
    if(esp_timer_create(&args, &healthy_timer) == ESP_OK)
        esp_timer_start_once(healthy_timer, CV30_GUARD_HEALTHY_MS * 1000ULL);
}

static status_code_t cmd_stock (sys_state_t state, char *args)
{
    if(!boot_stock()) {
        hal.stream.write("[MSG:Stock firmware not found]" ASCII_EOL);
        return Status_InvalidStatement;
    }

    hal.stream.write("[MSG:Restarting into the stock Creality firmware]" ASCII_EOL);
    hal.delay_ms(200, NULL);
    esp_restart();

    return Status_OK;
}

#if SDCARD_ENABLE

// What the stock updater installs: a .bin (any case) whose name contains "CV-30".
static bool update_candidate (const char *name)
{
    size_t n = strlen(name);

    return strstr(name, "CV-30") && n > 4 && strcasecmp(name + n - 4, ".bin") == 0;
}

// Is this an ESP32-S2 app image that fits the slot, and not the stock firmware?
static const char *update_check_image (const char *path)
{
    struct stat st;
    struct {
        esp_image_header_t image;
        esp_image_segment_header_t segment;
        esp_app_desc_t app;
    } __attribute__((packed)) head;

    if(stat(path, &st) != 0)
        return "file not found";
    if(st.st_size > CV30_SLOT_SIZE)
        return "image is larger than the 1408K slot";

    FILE *f = fopen(path, "rb");
    size_t n = f ? fread(&head, 1, sizeof(head), f) : 0;
    if(f)
        fclose(f);

    if(n != sizeof(head) || head.image.magic != ESP_IMAGE_HEADER_MAGIC)
        return "not an ESP32 firmware image";
    if(head.image.chip_id != ESP_CHIP_ID_ESP32S2)
        return "image is not built for the ESP32-S2";
    if(head.app.magic_word != ESP_APP_DESC_MAGIC_WORD)
        return "image has no app description";
    if(strncmp(head.app.project_name, CV30_STOCK_PREFIX, strlen(CV30_STOCK_PREFIX)) == 0)
        return "this is the stock firmware (already in the other slot)";

    return NULL;
}

// The stock updater silently skips a file name it has already installed.
static bool update_name_used (const char *name)
{
    nvs_handle_t nvs;
    char last[64] = "";
    size_t len = sizeof(last);
    bool used = false;

    if(nvs_open(STOCK_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if(nvs_get_str(nvs, STOCK_NVS_FILE, last, &len) == ESP_OK)
            used = strcmp(last[0] == '/' ? last + 1 : last, name) == 0;
        nvs_close(nvs);
    }
    len = sizeof(last);
    if(!used && nvs_open(CV30_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if(nvs_get_str(nvs, CV30_NVS_UPDATE, last, &len) == ESP_OK)
            used = strcmp(last, name) == 0;
        nvs_close(nvs);
    }

    return used;
}

static void write_nothing (const char *s)
{
    (void)s;
}

static status_code_t update_fail (const char *msg)
{
    hal.stream.write("[MSG:UPDATE: ");
    hal.stream.write(msg);
    hal.stream.write("]" ASCII_EOL);

    return Status_InvalidStatement;
}

static status_code_t cmd_update (sys_state_t state, char *args)
{
    char name[64] = "", path[sizeof(CV30_SD_ROOT) + 1 + 256], msg[160];
    uint_fast8_t found = 0;
    DIR *dir;
    struct dirent *e;

    if(!(state == STATE_IDLE || (state & STATE_ALARM)))
        return Status_IdleError;

    // Remount first: after the card has been taken out and put back, the old
    // mount no longer sees the card's files.
    char unmount[] = "$FU", mount[] = "$FM";
    system_execute_line(unmount, write_nothing);
    system_execute_line(mount, write_nothing);

    if(sdcard_getfs() == NULL)
        return update_fail("no microSD card");

    if(args && *args) {
        strncpy(name, *args == '/' ? args + 1 : args, sizeof(name) - 1);
        if(!update_candidate(name))
            return update_fail("the stock installer only takes .bin files with CV-30 in the name");
    } else if((dir = opendir(CV30_SD_ROOT))) {
        while((e = readdir(dir))) {
            if(update_candidate(e->d_name) && !update_name_used(e->d_name) && ++found == 1)
                strncpy(name, e->d_name, sizeof(name) - 1);
        }
        closedir(dir);
        if(found != 1)
            return update_fail(found ? "more than one new CV-30-*.bin on the card, name one: $UPDATE=<file>"
                                     : "no new CV-30-*.bin on the card");
    }

    if(update_name_used(name))
        return update_fail("this file name was installed before and would be skipped; rename it");

    snprintf(path, sizeof(path), CV30_SD_ROOT "/%s", name);
    const char *bad = update_check_image(path);
    if(bad)
        return update_fail(bad);

    // Only the chosen file may look installable to the stock updater.
    if((dir = opendir(CV30_SD_ROOT))) {
        while((e = readdir(dir))) {
            if(update_candidate(e->d_name) && strcmp(e->d_name, name)) {
                char from[sizeof(CV30_SD_ROOT) + 1 + 256], to[sizeof(from) + 4];
                snprintf(from, sizeof(from), CV30_SD_ROOT "/%s", e->d_name);
                snprintf(to, sizeof(to), "%s.old", from);
                rename(from, to);
            }
        }
        closedir(dir);
    }

    nvs_handle_t nvs;
    if(nvs_open(CV30_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, CV30_NVS_UPDATE, name);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    if(!boot_stock())
        return update_fail("stock firmware not found");

    snprintf(msg, sizeof(msg), "[MSG:UPDATE: handing %s to the stock installer, back in about 30 s]" ASCII_EOL, name);
    hal.stream.write(msg);
    hal.delay_ms(300, NULL);
    esp_restart();

    return Status_OK;
}

#endif // SDCARD_ENABLE

// --- Stepping diagnostics ---------------------------------------------------

static void diag_line (const char *fmt, ...)
{
    char buf[160];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    hal.stream.write(buf);
    hal.stream.write(ASCII_EOL);
}

#if CV30_DIAG

#define CV30_RMTMEM_BASE 0x3f416400  // RMTMEM (esp32s2.peripherals.ld), 64 words per channel

// $CVDIAG: how the step pins and RMT channels are actually set up.
static status_code_t cmd_cvdiag (sys_state_t state, char *args)
{
    static const struct { const char *name; uint8_t pin, ch; } steps[] = {
        { "X", X_STEP_PIN, X_AXIS }, { "Y", Y_STEP_PIN, Y_AXIS }
    };

    diag_line("[MSG:RMT clock %s, reset %s]",
              (REG_READ(DPORT_PERIP_CLK_EN0_REG) & DPORT_RMT_CLK_EN) ? "on" : "OFF",
              (REG_READ(DPORT_PERIP_RST_EN0_REG) & DPORT_RMT_RST) ? "HELD" : "released");
    diag_line("[MSG:RMT apb_conf %08lx]", (unsigned long)RMT.apb_conf.val);

    for(int i = 0; i < 2; i++) {
        uint32_t sel = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * steps[i].pin) & 0x1FF;
        volatile uint32_t *mem = (volatile uint32_t *)(CV30_RMTMEM_BASE + 0x100 * steps[i].ch);
        diag_line("[MSG:%s step GPIO%u out_sel %lu (RMT ch%u = %d) level %d]", steps[i].name, steps[i].pin,
                  (unsigned long)sel, steps[i].ch, RMT_SIG_OUT0_IDX + steps[i].ch, gpio_get_level(steps[i].pin));
        diag_line("[MSG:%s ch%u conf0 %08lx conf1 %08lx mem %08lx %08lx]", steps[i].name, steps[i].ch,
                  (unsigned long)RMT.conf_ch[steps[i].ch].conf0.val, (unsigned long)RMT.conf_ch[steps[i].ch].conf1.val,
                  (unsigned long)mem[0], (unsigned long)mem[1]);
    }

    return Status_OK;
}

#endif // CV30_DIAG

// $CVSTEP=<X|Y><count>[-]: step one axis by toggling its step pin directly,
// bypassing the RMT and the planner: a wiring test for motor + driver + pins
// (a motor that only buzzes usually has its coil pairs crossed). ~670 steps/s.
// The step pin is handed back to the RMT afterwards.
static status_code_t cmd_cvstep (sys_state_t state, char *args)
{
    if(!(state == STATE_IDLE || (state & STATE_ALARM)))
        return Status_IdleError;
    if(args == NULL || (*args != 'X' && *args != 'x' && *args != 'Y' && *args != 'y'))
        return Status_InvalidStatement;

    bool x = *args == 'X' || *args == 'x';
    int count = atoi(args + 1);
    bool reverse = strchr(args, '-') != NULL;
    uint8_t step = x ? X_STEP_PIN : Y_STEP_PIN, dir = x ? X_DIRECTION_PIN : Y_DIRECTION_PIN, ch = x ? X_AXIS : Y_AXIS;

    if(count < 1 || count > 4000)
        return Status_InvalidStatement;

    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    gpio_set_level(dir, reverse);
    esp_rom_gpio_connect_out_signal(step, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(step, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(step, 0);
    esp_rom_delay_us(1000);

    for(int i = 0; i < count; i++) {
        gpio_set_level(step, 1);
        esp_rom_delay_us(10);
        gpio_set_level(step, 0);
        esp_rom_delay_us(1490);
        if((i & 63) == 63)
            vTaskDelay(1); // keep the watchdog and network fed
    }

    esp_rom_gpio_connect_out_signal(step, RMT_SIG_OUT0_IDX + ch, false, false);
    diag_line("[MSG:CVSTEP %c %d steps %s on GPIO%u (dir GPIO%u=%d)]", x ? 'X' : 'Y', count, reverse ? "reverse" : "forward", step, dir, reverse);

    return Status_OK;
}

#ifndef CV30_BUILD
#define CV30_BUILD "dev"
#endif
#define CV30_STR2(x) #x

void my_plugin_init (void)
{
    // Shows in $I as [BOARD:...] so the running build can be told apart.
    hal.board = BOARD_NAME " (build " CV30_BUILD ")";

    static const sys_command_t command_list[] = {
        { "STOCK", cmd_stock, { .allow_blocking = On, .noargs = On }, { .str = "restart into the stock Creality firmware" } },
#if CV30_DIAG
        { "CVDIAG", cmd_cvdiag, { .allow_blocking = On, .noargs = On }, { .str = "CV-30: show step pin / RMT setup" } },
#endif
        { "CVSTEP", cmd_cvstep, { .allow_blocking = On }, { .str = "CV-30 motor wiring test: step X or Y directly, e.g. $CVSTEP=X400 or $CVSTEP=Y400-" } },
#if SDCARD_ENABLE
        { "UPDATE", cmd_update, { .allow_blocking = On }, { .str = "install a CV-30-*.bin from the microSD card via the stock installer" } }
#endif
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    system_register_commands(&commands);
}

#endif // BOARD_CV30
