/**
 * @file littlefs_api.c
 * @brief Maps the HAL of esp_partition <-> littlefs
 * @author Brian Pugh
 */

//#define ESP_LOCAL_LOG_LEVEL ESP_LOG_INFO

#include "driver.h"

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
//#include "esp_vfs.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "littlefs/lfs.h"
//#include "esp_littlefs.h"
//#include "littlefs_api.h"

/**
 * @brief littlefs definition structure
 */
typedef struct {
    lfs_t *fs;                                /*!< Handle to the underlying littlefs */
    SemaphoreHandle_t lock;                   /*!< FS lock */
    const esp_partition_t* partition;         /*!< The partition on which littlefs is located */
    struct lfs_config *cfg;                   /*!< littlefs Mount configuration */
} esp_littlefs_t;

#define CONFIG_LITTLEFS_PAGE_SIZE 256
#define CONFIG_LITTLEFS_READ_SIZE 128
#define CONFIG_LITTLEFS_WRITE_SIZE 128
#define CONFIG_LITTLEFS_LOOKAHEAD_SIZE 128
#define CONFIG_LITTLEFS_CACHE_SIZE 512     /* Old value was 128 */
#define CONFIG_LITTLEFS_BLOCK_CYCLES 512
#define CONFIG_LITTLEFS_BLOCK_SIZE 4096 /* ESP32 can only operate at 4kb */

static const char TAG[] = "esp_littlefs_api";

int littlefs_api_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    esp_littlefs_t * efs = c->context;
    size_t part_off = (block * c->block_size) + off;
    esp_err_t err = esp_partition_read(efs->partition, part_off, buffer, size);
    if (err) {
        ESP_LOGE(TAG, "failed to read addr %08x, size %08x, err %d", (unsigned int) part_off, (unsigned int) size, err);
        return LFS_ERR_IO;
    }
    return 0;
}

int littlefs_api_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    esp_littlefs_t * efs = c->context;
    size_t part_off = (block * c->block_size) + off;
    esp_err_t err = esp_partition_write(efs->partition, part_off, buffer, size);
    if (err) {
        ESP_LOGE(TAG, "failed to write addr %08x, size %08x, err %d", (unsigned int) part_off, (unsigned int) size, err);
        return LFS_ERR_IO;
    }
    return 0;
}

int littlefs_api_erase(const struct lfs_config *c, lfs_block_t block)
{
    esp_littlefs_t * efs = c->context;
    size_t part_off = block * c->block_size;
    esp_err_t err = esp_partition_erase_range(efs->partition, part_off, c->block_size);
    if (err) {
        ESP_LOGE(TAG, "failed to erase addr %08x, size %08x, err %d", (unsigned int) part_off, (unsigned int) c->block_size, err);
        return LFS_ERR_IO;
    }
    return 0;

}

int littlefs_api_sync(const struct lfs_config *c)
{
    /* Unnecessary for esp-idf */
    return 0;
}

struct lfs_config *esp32_littlefs_hal (void)
{
    static SemaphoreHandle_t _efs_lock = NULL;
    static esp_littlefs_t lfst = {0};
    static struct lfs_config t4_cfg = {
        // block device operations
        .read  = littlefs_api_read,
        .prog  = littlefs_api_prog,
        .erase = littlefs_api_erase,
        .sync  = littlefs_api_sync,
        // block device configuration
        .read_size = CONFIG_LITTLEFS_READ_SIZE,
        .prog_size = CONFIG_LITTLEFS_WRITE_SIZE,
        .block_size = CONFIG_LITTLEFS_BLOCK_SIZE,
        .cache_size = CONFIG_LITTLEFS_CACHE_SIZE,
        .lookahead_size = CONFIG_LITTLEFS_LOOKAHEAD_SIZE,
        .block_cycles = CONFIG_LITTLEFS_BLOCK_CYCLES
    };

    if((lfst.partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage")) == NULL) {
        ESP_LOGE(TAG, "partition \"%s\" could not be found", "spiffs");
//        err = ESP_ERR_NOT_FOUND;
        return NULL;
    }

    if(_efs_lock == NULL ){
        static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        if(_efs_lock == NULL ){
            _efs_lock = xSemaphoreCreateMutex();
            assert(_efs_lock);
        }
        portEXIT_CRITICAL(&mux);
    }

    xSemaphoreTake(_efs_lock, portMAX_DELAY);

    lfst.cfg = &t4_cfg;
    t4_cfg.context = &lfst;
    t4_cfg.block_count = lfst.partition->size / t4_cfg.block_size;

    if((lfst.lock = xSemaphoreCreateRecursiveMutex()) == NULL) {
        ESP_LOGE(TAG, "mutex lock could not be created");
//        err = ESP_ERR_NO_MEM;
//        goto exit;
    }

    xSemaphoreGive(_efs_lock);

    return &t4_cfg;
}
