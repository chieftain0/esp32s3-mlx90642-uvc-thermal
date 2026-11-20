/*
To use JPEG, comment out the CONFIG_FORMAT_UNCOMPR_CAM1=y line
in the skdconfig.defaults file and uncomment the respective MJEG line.
Delete sdkconfig file, full clean and rebuild the project.
*/

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb_device_uvc.h"

#include <MLX90642_depends.h>
#include <MLX90642.h>

#define SDA 5
#define SCL 6
#define USE_I2C_FM_PLUS
// #define USE_I2C_FM

#define SENSOR_WIDTH 32
#define SENSOR_HEIGHT 24
#define SENSOR_PIXEL_COUNT (SENSOR_WIDTH * SENSOR_HEIGHT)

static volatile uint8_t camera_enabled = 0;

static int16_t image_buf[SENSOR_PIXEL_COUNT] = {0};
static uint8_t gray_image_buf[SENSOR_PIXEL_COUNT] = {0};

#ifndef CONFIG_FORMAT_UNCOMPR_CAM1
#include "esp_jpeg_enc.h"
#define JPEG_BUF_SIZE (8 * 1024) // 8 KiB to be safe
static uint8_t uvc_frame_buf[JPEG_BUF_SIZE];
static int uvc_frame_buf_length = 0;
static uint8_t uvc_xfer_buf[JPEG_BUF_SIZE];
static jpeg_enc_handle_t jpeg = NULL;
#else
static uint8_t uvc_frame_buf[SENSOR_PIXEL_COUNT * 2];
static int uvc_frame_buf_length = SENSOR_PIXEL_COUNT * 2;
static uint8_t uvc_xfer_buf[SENSOR_PIXEL_COUNT * 2];
#endif

static uvc_fb_t uvc_fb;
static SemaphoreHandle_t frame_buffer_mutex;

static const char *TAG_MAIN = "main.c";

static int gray16_to_gray8(int16_t *data_in, uint8_t *data_out, int32_t data_in_len)
{
    if (data_in == NULL || data_out == NULL || data_in_len <= 0)
    {
        return -1;
    }

    int32_t max = INT16_MIN;
    int32_t min = INT16_MAX;

    for (int32_t i = 0; i < data_in_len; i++)
    {
        if (data_in[i] > max)
        {
            max = data_in[i];
        }
        if (data_in[i] < min)
        {
            min = data_in[i];
        }
    }

    int32_t range = max - min;
    if (range == 0)
    {
        range = 1;
    }

    for (uint32_t i = 0; i < data_in_len; i++)
    {
        data_out[i] = (uint8_t)((uint32_t)((data_in[i] - min) * 0xFF) / range);
    }

    return 0;
}

static int gray8_to_yuy2(uint8_t *data_in, uint8_t *data_out, int32_t data_in_len)
{
    if (data_in == NULL || data_out == NULL || data_in_len <= 0)
    {
        return -1;
    }

    for (uint32_t i = 0; i < data_in_len; i++)
    {
        data_out[2 * i] = data_in[i];
        data_out[2 * i + 1] = (uint8_t)128;
    }

    return 0;
}

static int thermal_init()
{
    int status;

#ifdef USE_I2C_FM_PLUS
    uint32_t i2c_speed = 1000000;
    uint8_t thermal_refresh_rate = MLX90642_REF_RATE_32HZ;
    uint8_t mlx90642_i2c_mode = MLX90642_I2C_MODE_FM_PLUS;
#elif defined(USE_I2C_FM)
    uint32_t i2c_speed = 400000;
    uint8_t thermal_refresh_rate = MLX90642_REF_RATE_16HZ;
    uint8_t mlx90642_i2c_mode = MLX90642_I2C_MODE_FM;
#else
    uint32_t i2c_speed = 400000;
    uint8_t thermal_refresh_rate = MLX90642_REF_RATE_16HZ;
    uint8_t mlx90642_i2c_mode = MLX90642_I2C_MODE_FM;
#endif

    status = MLX90642_I2Cinit(0, SA_90642_DEFAULT, SDA, SCL, i2c_speed, 0);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "I2C Init Failure");
        return status;
    }
    status = MLX90642_SetMeasMode(SA_90642_DEFAULT, MLX90642_CONT_MEAS_MODE);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Set MEAS Failure");
        return status;
    }
    status = MLX90642_SetOutputFormat(SA_90642_DEFAULT, MLX90642_NORMALIZED_DATA_OUTPUT);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Set Output Format Failure");
        return status;
    }
    status = MLX90642_SetRefreshRate(SA_90642_DEFAULT, thermal_refresh_rate);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Set Refresh Rate Failure");
        return status;
    }
    status = MLX90642_SetI2CMode(SA_90642_DEFAULT, mlx90642_i2c_mode);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Set I2C Mode Failure");
        return status;
    }
    status = MLX90642_GetDefect(SA_90642_DEFAULT);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Get Defect Failure");
        return status;
    }
    status = MLX90642_Init(SA_90642_DEFAULT);
    if (status != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Start Failure");
        return status;
    }
    return status;
}

static esp_err_t cam_start_cb(uvc_format_t format, int width, int height, int rate, void *ctx)
{
    camera_enabled = 1;
    ESP_LOGI(TAG_MAIN, "UVC start: %dx%d @ %d fps", width, height, rate);
    return ESP_OK;
}

static void cam_stop_cb(void *ctx)
{
    camera_enabled = 0;
    ESP_LOGI(TAG_MAIN, "UVC stop");
}

static uvc_fb_t *cam_fb_get_cb(void *ctx)
{
    xSemaphoreTake(frame_buffer_mutex, portMAX_DELAY); // hold until driver returns the fb

    struct timeval tv;
    gettimeofday(&tv, NULL);

    uvc_fb.buf = uvc_frame_buf;
    uvc_fb.len = uvc_frame_buf_length;
    uvc_fb.width = SENSOR_WIDTH;
    uvc_fb.height = SENSOR_HEIGHT;
    uvc_fb.format = UVC_FORMAT_JPEG;
    uvc_fb.timestamp = tv;
    return &uvc_fb;
}

static void cam_fb_return_cb(uvc_fb_t *fb, void *ctx)
{
    xSemaphoreGive(frame_buffer_mutex);
}

static void thermal_task(void *arg)
{
    while (true)
    {
        if (!camera_enabled)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int ready = MLX90642_IsReadWindowOpen(SA_90642_DEFAULT);

        if (ready == MLX90642_YES)
        {

            if (MLX90642_GetImage(SA_90642_DEFAULT, (uint16_t *)image_buf) != 0)
            {
                ESP_LOGW(TAG_MAIN, "Thermal Image Acq Failure");
                continue;
            }

            if (MLX90642_DefectCompensate(image_buf) != 0)
            {
                ESP_LOGW(TAG_MAIN, "Thermal Image Defect Compensation Failure");
                continue;
            }

            if (gray16_to_gray8(image_buf, gray_image_buf, SENSOR_PIXEL_COUNT) != 0)
            {
                ESP_LOGW(TAG_MAIN, "Grayscale conversion failed");
                continue;
            }

            xSemaphoreTake(frame_buffer_mutex, portMAX_DELAY);

#ifndef CONFIG_FORMAT_UNCOMPR_CAM1
            jpeg_error_t status = jpeg_enc_process(jpeg, gray_image_buf, SENSOR_PIXEL_COUNT, uvc_frame_buf, sizeof(uvc_frame_buf), &uvc_frame_buf_length);
            if (status != JPEG_ERR_OK)
            {
                ESP_LOGW(TAG_MAIN, "JPEG encoding failed: %d", status);
            }
#else
            int status = gray8_to_yuy2(gray_image_buf, uvc_frame_buf, sizeof(gray_image_buf));
            if (status != 0)
            {
                ESP_LOGW(TAG_MAIN, "YUY2 Encoding Failed");
            }
#endif

            xSemaphoreGive(frame_buffer_mutex);
        }
        else if (ready == MLX90642_NO)
        {
            MLX90642_Wait_ms(3);
        }
        else
        {
            ESP_LOGW(TAG_MAIN, "Read Window Failure");
            vTaskDelay(pdMS_TO_TICKS(3));
        }
    }
}

void app_main(void)
{
    esp_log_level_set("MLX90642_depends.c", ESP_LOG_WARN);
    esp_log_level_set("main.c", ESP_LOG_WARN);
    esp_rom_delay_us(500000); // allow the MLX90642 to boot after a power cycle

    if (thermal_init() != 0)
    {
        ESP_LOGE(TAG_MAIN, "Thermal Start Failure");
        esp_restart();
    }

#ifndef CONFIG_FORMAT_UNCOMPR_CAM1
    jpeg_enc_config_t enc_cfg = DEFAULT_JPEG_ENC_CONFIG();
    enc_cfg.width = SENSOR_WIDTH;
    enc_cfg.height = SENSOR_HEIGHT;
    enc_cfg.src_type = JPEG_PIXEL_FORMAT_GRAY;
    enc_cfg.subsampling = JPEG_SUBSAMPLE_GRAY;
    enc_cfg.quality = 100;
    if (jpeg_enc_open(&enc_cfg, &jpeg) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG_MAIN, "JPEG encoder open failed");
        return;
    }

    if (jpeg_enc_process(jpeg, gray_image_buf, SENSOR_PIXEL_COUNT, uvc_frame_buf, sizeof(uvc_frame_buf), &uvc_frame_buf_length) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG_MAIN, "Initial JPEG frame generation failed");
        return;
    }
#else
    int status = gray8_to_yuy2(gray_image_buf, uvc_frame_buf, sizeof(gray_image_buf));
    if (status != 0)
    {
        ESP_LOGW(TAG_MAIN, "Initial YUY2 frame generation failed");
    }
#endif

    frame_buffer_mutex = xSemaphoreCreateMutex();

    if (frame_buffer_mutex == NULL)
    {
        ESP_LOGE(TAG_MAIN, "Failed to create frame buffer mutex");
        return;
    }

    uvc_device_config_t uvc_cfg = {
        .uvc_buffer = uvc_xfer_buf,
        .uvc_buffer_size = sizeof(uvc_xfer_buf),
        .start_cb = cam_start_cb,
        .fb_get_cb = cam_fb_get_cb,
        .fb_return_cb = cam_fb_return_cb,
        .stop_cb = cam_stop_cb,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uvc_device_config(0, &uvc_cfg));
    ESP_ERROR_CHECK(uvc_device_init());

    BaseType_t thermal_task_result = xTaskCreate(thermal_task, "thermal_task", 4096, NULL, 5, NULL);

    if (thermal_task_result != pdPASS)
    {
        ESP_LOGE(TAG_MAIN, "Failed to create thermal task");
        esp_restart();
    }

    vTaskDelete(NULL);
}