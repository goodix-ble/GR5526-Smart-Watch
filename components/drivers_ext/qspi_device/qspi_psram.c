#include "app_qspi.h"
#include "app_qspi_dma.h"
#include "app_log.h"
#include "qspi_psram.h"


#define DEFAULT_PSRAM_MODE_CONFIG                 {DMA0, DMA_Channel1, 1000, 0}
#define DEFAULT_PSRAM_QSPI_CONFIG                 {2, QSPI_CLOCK_MODE_3, 0}
#define DEFAULT_PSRAM_PARAM_CONFIG                {APP_QSPI_ID_1, g_qspi_pin_groups[QSPI1_PIN_GROUP_0], DEFAULT_PSRAM_MODE_CONFIG, DEFAULT_PSRAM_QSPI_CONFIG}


static  app_qspi_params_t g_qspi_psram_params;
static volatile uint8_t g_master_tdone = 0;
static volatile uint8_t g_master_rdone = 0;

static void app_qspi_callback(app_qspi_evt_t *p_evt)
{
    if (p_evt->type == APP_QSPI_EVT_TX_CPLT)
    {
        g_master_tdone = 1;
    }
    if (p_evt->type == APP_QSPI_EVT_RX_DATA)
    {
        g_master_rdone = 1;
    }
    if ((p_evt->type == APP_QSPI_EVT_ERROR) || (p_evt->type == APP_QSPI_EVT_ABORT))
    {
        g_master_tdone = 1;
        g_master_rdone = 1;
    }
}

uint32_t qspi_psram_init(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group, uint32_t clock_mode)
{
    uint32_t psram_id = 0;
    uint32_t retry = 20;
    uint16_t ret;
    app_qspi_params_t p_params = DEFAULT_PSRAM_PARAM_CONFIG;

    g_qspi_psram_params = p_params;

    g_qspi_psram_params.id                    = id;
    g_qspi_psram_params.pin_cfg               = g_qspi_pin_groups[pin_group];
    g_qspi_psram_params.init.clock_prescaler  = clock_prescaler;
    g_qspi_psram_params.init.clock_mode       = clock_mode;
    if(clock_prescaler == 2){
        g_qspi_psram_params.init.rx_sample_delay = 1;
    }  else {
        g_qspi_psram_params.init.rx_sample_delay = 0;
    }

    if(id == APP_QSPI_ID_2) {
        g_qspi_psram_params.dma_cfg.dma_instance = DMA1;
    }

    ret = app_qspi_init(&g_qspi_psram_params, app_qspi_callback);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial failed! Please check the input paraments: %d.", ret);
        return 1;
    }

    ret = app_qspi_dma_init(&g_qspi_psram_params);
    if (ret != 0)
    {
        APP_LOG_ERROR("QSPI initial dma failed! Please check the input paraments.");
        return 1;
    }

    while(retry--)  {
        qspi_psram_exit_quad_mode(g_qspi_psram_params.id);
        delay_us(5);
        qspi_psram_reset(g_qspi_psram_params.id);

        delay_us(5);

        psram_id = qspi_psram_read_id(g_qspi_psram_params.id);

        if( (psram_id != 0x0000) && (psram_id != 0xffff) ) {
            qspi_psram_enable_quad_mode(g_qspi_psram_params.id);
            break;
        }
        delay_us(10);
    }

    return psram_id;
}

void qspi_psram_deinit(app_qspi_id_t id)
{
    app_qspi_deinit(id);
}


void qspi_psram_reset(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_RESET_ENABLE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    g_master_tdone = 0;
    app_qspi_dma_command_async(id, &command);
    while(g_master_tdone == 0);

    command.instruction = PSRAM_CMD_RESET;
    g_master_tdone = 0;
    app_qspi_dma_command_async(id, &command);
    while(g_master_tdone == 0);

    return;
}

void qspi_psram_enable_quad_mode(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_ENTER_QUAD_MODE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    g_master_tdone = 0;
    app_qspi_dma_command_async(id, &command);
    while(g_master_tdone == 0);

    return;
}

void qspi_psram_exit_quad_mode(app_qspi_id_t id)
{
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_EXIT_QUAD_MODE,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_00_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPIFRF,
        .data_mode                  = QSPI_DATA_MODE_QUADSPI,
        .length                     = 0,
        .clock_stretch_en           = 1,
    };

    g_master_tdone = 0;
    app_qspi_dma_command_async(id, &command);
    while(g_master_tdone == 0);
    return;
}

uint32_t qspi_psram_read_id(app_qspi_id_t id)
{
    uint8_t data[2];
    app_qspi_command_t command = {
        .instruction                = PSRAM_CMD_READID,
        .address                    = 0,
        .instruction_size           = QSPI_INSTSIZE_08_BITS,
        .address_size               = QSPI_ADDRSIZE_24_BITS,
        .data_size                  = QSPI_DATASIZE_08_BITS,
        .dummy_cycles               = 0,
        .instruction_address_mode   = QSPI_INST_ADDR_ALL_IN_SPI,
        .data_mode                  = QSPI_DATA_MODE_SPI,
        .length                     = 2,
        .clock_stretch_en           = 1,
    };

    memset(&data[0], 0, 2);
    g_master_rdone = 0;
    app_qspi_dma_command_receive_async(id, &command, &data[0]);
    while(g_master_rdone == 0);

    return (((uint32_t)data[0] << 8) + ((uint32_t)data[1] << 0) );
}

uint32_t qspi_psram_init_as_xip(app_qspi_id_t id, uint32_t clock_prescaler, qspi_pins_group_e pin_group) {
    uint8_t psram_id = 0;

    psram_id = qspi_psram_init(id, clock_prescaler, pin_group, QSPI_CLOCK_MODE_3);
    printf("PSRAM ID: 0x%x \r\n", psram_id);

    app_qspi_mmap_device_t dev = {
        .dev_type = APP_QSPI_DEVICE_PSRAM,
        .rd.psram_rd = PSRAM_MMAP_CMD_QREAD_0BH,
        .psram_wr = PSRAM_MMAP_CMD_QWRITE_02H,
    };

    bool ret = app_qspi_config_memory_mappped(id, dev);

    return psram_id;
}
