#include "driver/gpio.h"
#include "esp_log.h"

esp_err_t __real_gpio_set_intr_type(gpio_num_t, gpio_int_type_t);
esp_err_t __wrap_gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t type)
{
  if (gpio_num < 0 || gpio_num > 39 || !GPIO_IS_VALID_GPIO(gpio_num))
  {
    ESP_LOGE("wrap", "invalid gpio_set_intr_type pin=%d", (int)gpio_num);
    return ESP_ERR_INVALID_ARG;
  }
  return __real_gpio_set_intr_type(gpio_num, type);
}
