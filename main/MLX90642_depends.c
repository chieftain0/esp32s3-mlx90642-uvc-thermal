#include <MLX90642_depends.h>
#include <MLX90642.h>

static i2c_master_bus_config_t mlx90642_master_bus_config;
static i2c_master_bus_handle_t mlx90642_master_bus_handle;
static i2c_device_config_t mlx90642_device_config;
static i2c_master_dev_handle_t mlx90642_device_handle;
static bool i2c_established = false;

static const char *TAG_MLX90642 = "MLX90642_depends.c";

static uint16_t defective_pixels_mask[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

int MLX90642_I2Cinit(uint8_t i2c_port, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t freq, uint8_t int_pllup)
{
    ESP_LOGI(TAG_MLX90642, "Initializing I2C: port=%d, address=0x%02X, SDA=%d, SCL=%d, frequency=%lu Hz",
             i2c_port, addr, sda, scl, (unsigned long)freq);

    mlx90642_master_bus_config = (i2c_master_bus_config_t){
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = int_pllup,
    };

    if (i2c_new_master_bus(&mlx90642_master_bus_config, &mlx90642_master_bus_handle) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "Failed to initialize I2C master bus");
        return -1;
    }

    ESP_LOGI(TAG_MLX90642, "I2C master bus initialized successfully");

    mlx90642_device_config = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = freq,
    };

    if (i2c_master_bus_add_device(mlx90642_master_bus_handle, &mlx90642_device_config, &mlx90642_device_handle) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "Failed to register I2C device at address 0x%02X", addr);
        return -1;
    }

    ESP_LOGI(TAG_MLX90642, "I2C device 0x%02X registered successfully", addr);

    i2c_established = true;

    ESP_LOGI(TAG_MLX90642, "MLX90642 I2C initialization completed");

    return 0;
}

/** MLX90642 block read I2C command
 * @note For more information refer to the MLX90642 datasheet
 *
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] startAddress Start address for the block read
 * @param[in] nMemAddressRead Number of words to read
 * @param[out] rData Pointer to where the read data will be stored
 *
 * @retval  <0 Error while configuring the MLX90642 device
 *
 */

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *rData)
{
    ESP_LOGI(TAG_MLX90642, "I2C read requested: slave=0x%02X, start address=0x%04X, words=%u",
             slaveAddr, startAddress, nMemAddressRead);

    if (!i2c_established || mlx90642_device_config.device_address != slaveAddr || nMemAddressRead == 0 || rData == NULL)
    {
        ESP_LOGE(TAG_MLX90642, "I2C read validation failed: established=%d, configured address=0x%02X, requested address=0x%02X, words=%u, data pointer=%p",
                 i2c_established, mlx90642_device_config.device_address, slaveAddr, nMemAddressRead, rData);
        return -1;
    }

    uint8_t addr[2] = {(uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF)};
    uint8_t buf[2 * nMemAddressRead];

    if (i2c_master_transmit_receive(mlx90642_device_handle, addr, sizeof(addr), buf, sizeof(buf), I2C_TIMEOUT) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "I2C read failed at address 0x%04X", startAddress);
        return -1;
    }

    for (uint32_t i = 0; i < nMemAddressRead; i++)
    {
        rData[i] = (uint16_t)buf[2 * i] << 8 | (uint16_t)buf[2 * i + 1];
        ESP_LOGD(TAG_MLX90642, "Read word[%lu] = 0x%04X", (unsigned long)i, rData[i]);
    }

    ESP_LOGI(TAG_MLX90642, "I2C read completed successfully: %u words", nMemAddressRead);

    return 0;
}

/** MLX90642 configuration I2C command
 * @note For more information refer to the MLX90642 datasheet
 *
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] writeAddress Configuration address to write to
 * @param[in] wData Data to write
 *
 * @retval  <0 Error while configuring the MLX90642 device
 *
 */
int MLX90642_Config(uint8_t slaveAddr, uint16_t writeAddress, uint16_t wData)
{
    ESP_LOGI(TAG_MLX90642, "Configuration write: slave=0x%02X, register=0x%04X, data=0x%04X",
             slaveAddr, writeAddress, wData);

    if (!i2c_established || mlx90642_device_config.device_address != slaveAddr)
    {
        ESP_LOGE(TAG_MLX90642, "Configuration write validation failed: I2C not established or address mismatch");
        return -1;
    }
    uint8_t buf[6] = {(uint8_t)(MLX90642_CONFIG_OPCODE >> 8), (uint8_t)(MLX90642_CONFIG_OPCODE & 0xFF),
                      (uint8_t)(writeAddress >> 8), (uint8_t)(writeAddress & 0xFF),
                      (uint8_t)(wData >> 8), (uint8_t)(wData & 0xFF)};

    if (i2c_master_transmit(mlx90642_device_handle, buf, sizeof(buf), I2C_TIMEOUT) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "Configuration write failed at register 0x%04X", writeAddress);
        return -1;
    }
    MLX90642_Wait_ms(15);

    ESP_LOGI(TAG_MLX90642, "Configuration write completed successfully");

    return 0;
}

/** MLX90642 I2C commands send
 * @note The addressed reset, start/sync measurement and sleep commands have the same I2C format. For more information refer to the MLX90642 datasheet
 *
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] i2c_cmd MLX90642 I2C command to send
 *
 * @retval  <0 Error while sending the I2C command to the MLX90642 device
 *
 */
int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd)
{
    ESP_LOGI(TAG_MLX90642, "Sending I2C command: slave=0x%02X, command=0x%04X", slaveAddr, i2c_cmd);

    if (!i2c_established || mlx90642_device_config.device_address != slaveAddr)
    {
        ESP_LOGE(TAG_MLX90642, "I2C command validation failed: I2C not established or address mismatch");
        return -1;
    }

    uint8_t buf[4] = {(uint8_t)(MLX90642_CMD_OPCODE >> 8), (uint8_t)(MLX90642_CMD_OPCODE & 0xFF),
                      (uint8_t)(i2c_cmd >> 8), (uint8_t)(i2c_cmd & 0xFF)};

    if (i2c_master_transmit(mlx90642_device_handle, buf, sizeof(buf), I2C_TIMEOUT) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "Failed to send I2C command 0x%04X", i2c_cmd);
        return -1;
    }

    ESP_LOGI(TAG_MLX90642, "I2C command 0x%04X sent successfully", i2c_cmd);

    return 0;
}

/** MLX90642 wake-up command
 * @note For more information refer to the MLX90642 datasheet
 *
 * @param[in] slaveAddr I2C slave address of the device
 *
 * @retval  <0 Error while sending the wake-up command to the MLX90642 device
 *
 */
int MLX90642_WakeUp(uint8_t slaveAddr)
{
    ESP_LOGI(TAG_MLX90642, "Sending wake-up command to slave 0x%02X", slaveAddr);

    if (!i2c_established || mlx90642_device_config.device_address != slaveAddr)
    {
        ESP_LOGE(TAG_MLX90642, "Wake-up validation failed: I2C not established or address mismatch");
        return -1;
    }

    uint8_t buf = 0x57;
    if (i2c_master_transmit(mlx90642_device_handle, &buf, 1, I2C_TIMEOUT) != ESP_OK)
    {
        ESP_LOGE(TAG_MLX90642, "Wake-up command failed for slave 0x%02X", slaveAddr);
        return -1;
    }

    ESP_LOGI(TAG_MLX90642, "Wake-up command sent successfully");

    return 0;
}

/** Delay function
 *
 * @param[in] wait_ms Time to wait in milliseconds
 *
 */
void MLX90642_Wait_ms(uint16_t time_ms)
{
    ESP_LOGD(TAG_MLX90642, "Waiting for %u ms", time_ms);

    uint32_t ticks = pdMS_TO_TICKS(time_ms);
    if (ticks == 0)
    {
        vTaskDelay(1);
    }
    else
    {
        vTaskDelay(ticks);
    }
}

/**
 * @brief Reads the defective pixel information from the MLX90642.
 *
 * @param[in] slaveAddr I2C address of the MLX90642.
 *
 * @return 0 on success, -1 on failure.
 */
int MLX90642_GetDefect(uint8_t slaveAddr)
{
    if (!i2c_established || mlx90642_device_config.device_address != slaveAddr)
    {
        ESP_LOGE(TAG_MLX90642, "I2C not initialized or slave address mismatch");
        return -1;
    }

    if (MLX90642_I2CRead(slaveAddr, MLX90642_DEFECTIVE_PIXEL_ADDR, MLX90642_READ_NUM_FOR_DEFECTIVE, defective_pixels_mask) != 0)
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Compensates defective pixels using the neighboring pixel values.
 *
 * @param[in] data Pointer to the image data.
 *
 * @return 0 on success, -1 if the data pointer is NULL.
 * 
 * 
 */
int MLX90642_DefectCompensate(int16_t *data)
{
    ESP_LOGI(TAG_MLX90642, "Starting defective pixel compensation");

    if (data == NULL)
    {
        ESP_LOGE(TAG_MLX90642, "Invalid image data pointer");
        return -1;
    }

    for (uint32_t i = 0; i < MLX90642_READ_NUM_FOR_DEFECTIVE; i++)
    {
        if (defective_pixels_mask[i] == 0xFFFF)
        {
            break;
        }
        else
        {
            uint16_t def_pix_idx = defective_pixels_mask[i] & 0x7FFF;
            if (def_pix_idx > 0 && def_pix_idx < MLX90642_TOTAL_NUMBER_OF_PIXELS - 1)
            {
                data[def_pix_idx] = (int16_t)(((int32_t)data[def_pix_idx - 1] + (int32_t)data[def_pix_idx + 1]) / 2);
            }
            if (def_pix_idx == 0)
            {
                data[def_pix_idx] = (int16_t)data[def_pix_idx + 1];
            }
            if (def_pix_idx == MLX90642_TOTAL_NUMBER_OF_PIXELS - 1)
            {
                data[def_pix_idx] = (int16_t)data[def_pix_idx - 1];
            }
        }
    }

    ESP_LOGI(TAG_MLX90642, "Defective pixel compensation completed");
    return 0;
}
