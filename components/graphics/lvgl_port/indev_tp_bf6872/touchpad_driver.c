#include <stdbool.h>
#include "grx_hal.h"
#include "app_gpiote.h"
#include "app_i2c.h"
#include "touchpad_driver.h"

/********************************************************************************************
 *                       Defines
 ********************************************************************************************/
#define TOUCHPAD_RST_IO_TYPE                  APP_IO_TYPE_GPIOB
#define TOUCHPAD_RST_IO_MUX                   APP_IO_MUX_7
#define TOUCHPAD_RST_IO_PIN                   APP_IO_PIN_13

#define TOUCHPAD_INT_IO_TYPE                  APP_IO_TYPE_GPIOC
#define TOUCHPAD_INT_IO_MUX                   APP_IO_MUX_7
#define TOUCHPAD_INT_IO_PIN                   APP_IO_PIN_0

#define TOUCHPAD_SCL_IO_TYPE                  APP_IO_TYPE_GPIOB
#define TOUCHPAD_SCL_IO_MUX                   APP_IO_MUX_3
#define TOUCHPAD_SCL_IO_PIN                   APP_IO_PIN_14

#define TOUCHPAD_SDA_IO_TYPE                  APP_IO_TYPE_GPIOB
#define TOUCHPAD_SDA_IO_MUX                   APP_IO_MUX_3
#define TOUCHPAD_SDA_IO_PIN                   APP_IO_PIN_15

#define TOUCHPAD_I2C_ID                       APP_I2C_ID_1
#define TOUCHPAD_PERIPH_DEVICE                PERIPH_DEVICE_NUM_I2C1


#define TOUCHPAD_I2C_IO_CONFIG            {{ TOUCHPAD_SCL_IO_TYPE, TOUCHPAD_SCL_IO_MUX, TOUCHPAD_SCL_IO_PIN, APP_IO_PULLUP }, \
                                           { TOUCHPAD_SDA_IO_TYPE, TOUCHPAD_SDA_IO_MUX, TOUCHPAD_SDA_IO_PIN, APP_IO_PULLUP }}
#define TOUCHPAD_I2C_MODE_CONFIG          { DMA1, DMA1, DMA_Channel0, DMA_Channel1 }
#define TOUCHPAD_I2C_ATTR_CONFIG          { I2C_SPEED_400K, 0x00, I2C_ADDRESSINGMODE_7BIT, I2C_GENERALCALL_DISABLE }
#define TOUCHPAD_I2C_PARAM_CONFIG         { TOUCHPAD_I2C_ID, APP_I2C_ROLE_MASTER, TOUCHPAD_I2C_IO_CONFIG, TOUCHPAD_I2C_MODE_CONFIG, TOUCHPAD_I2C_ATTR_CONFIG }

#define TOUCHPAD_I2C_ADDR                 (0x2C)

static void     touchpad_dev_irq_handler(app_io_evt_t *p_evt);
static void     touchpad_dev_pin_init(void);
static void     touchpad_dev_reset(void);

static app_i2c_params_t     s_tp_params = TOUCHPAD_I2C_PARAM_CONFIG;

static const uint16_t s_touchpad_map_454p[503] =
{
    0,0,1,2,3,4,5,6,7,8,9,9,10,11,12,13,14,15,16,17,18,18,19,20,21,22,23,24,25,26,27
    ,27,28,29,30,31,32,33,34,35,36,36,37,38,39,40,41,42,43,44,45,46,46,47,48,49,50,51,
    52,53,54,55,55,56,57,58,59,60,61,62,63,64,64,65,66,67,68,69,70,71,72,73,73,74,
    75,76,77,78,79,80,81,82,83,83,84,85,86,87,88,89,90,91,92,92,93,94,95,96,97,98,99
    ,100,101,101,102,103,104,105,106,107,108,109,110,110,111,112,113,114,115,116,117
    ,118,119,120,120,121,122,123,124,125,126,127,128,129,129,130,131,132,133,134,135
    ,136,137,138,138,139,140,141,142,143,144,145,146,147,147,148,149,150,151,152,153
    ,154,155,156,157,157,158,159,160,161,162,163,164,165,166,166,167,168,169,170,171
    ,172,173,174,175,175,176,177,178,179,180,181,182,183,184,184,185,186,187,188,189
    ,190,191,192,193,194,194,195,196,197,198,199,200,201,202,203,203,204,205,206,207
    ,208,209,210,211,212,212,213,214,215,216,217,218,219,220,221,221,222,223,224,225
    ,226,227,228,229,230,231,231,232,233,234,235,236,237,238,239,240,240,241,242,243
    ,244,245,246,247,248,249,249,250,251,252,253,254,255,256,257,258,258,259,260,261
    ,262,263,264,265,266,267,268,268,269,270,271,272,273,274,275,276,277,277,278,279
    ,280,281,282,283,284,285,286,286,287,288,289,290,291,292,293,294,295,295,296,297
    ,298,299,300,301,302,303,304,305,305,306,307,308,309,310,311,312,313,314,314,315
    ,316,317,318,319,320,321,322,323,323,324,325,326,327,328,329,330,331,332,332,333
    ,334,335,336,337,338,339,340,341,342,342,343,344,345,346,347,348,349,350,351,351
    ,352,353,354,355,356,357,358,359,360,360,361,362,363,364,365,366,367,368,369,369
    ,370,371,372,373,374,375,376,377,378,379,379,380,381,382,383,384,385,386,387,388
    ,388,389,390,391,392,393,394,395,396,397,397,398,399,400,401,402,403,404,405,406
    ,406,407,408,409,410,411,412,413,414,415,416,416,417,418,419,420,421,422,423,424
    ,425,425,426,427,428,429,430,431,432,433,434,434,435,436,437,438,439,440,441,442
    ,443,443,444,445,446,447,448,449,450,451,452,453,
};

static const uint16_t s_touchpad_map_360p[503] =
{
    0, 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 7, 8, 9, 10, 10, 11, 12, 12, 13,
    14, 15, 15, 16, 17, 17, 18, 19, 20, 20, 21, 22, 22, 23, 24, 25, 25, 26, 27, 27,
    28, 29, 30, 30, 31, 32, 32, 33, 34, 35, 35, 36, 37, 37, 38, 39, 40, 40, 41, 42,
    42, 43, 44, 45, 45, 46, 47, 47, 48, 49, 50, 50, 51, 52, 52, 53, 54, 55, 55, 56,
    57, 57, 58, 59, 60, 60, 61, 62, 62, 63, 64, 65, 65, 66, 67, 67, 68, 69, 70, 70,
    71, 72, 72, 73, 74, 75, 75, 76, 77, 77, 78, 79, 80, 80, 81, 82, 82, 83, 84, 85,
    85, 86, 87, 87, 88, 89, 90, 90, 91, 92, 92, 93, 94, 95, 95, 96, 97, 97, 98, 99,
    100, 100, 101, 102, 102, 103, 104, 105, 105, 106, 107, 107, 108, 109, 110, 110, 111, 112, 112, 113,
    114, 115, 115, 116, 117, 117, 118, 119, 120, 120, 121, 122, 122, 123, 124, 125, 125, 126, 127, 127,
    128, 129, 130, 130, 131, 132, 132, 133, 134, 135, 135, 136, 137, 137, 138, 139, 140, 140, 141, 142,
    142, 143, 144, 145, 145, 146, 147, 147, 148, 149, 150, 150, 151, 152, 152, 153, 154, 155, 155, 156,
    157, 157, 158, 159, 160, 160, 161, 162, 162, 163, 164, 165, 165, 166, 167, 167, 168, 169, 170, 170,
    171, 172, 172, 173, 174, 175, 175, 176, 177, 177, 178, 179, 180, 180, 181, 182, 182, 183, 184, 185,
    185, 186, 187, 187, 188, 189, 190, 190, 191, 192, 192, 193, 194, 195, 195, 196, 197, 197, 198, 199,
    200, 200, 201, 202, 202, 203, 204, 205, 205, 206, 207, 207, 208, 209, 210, 210, 211, 212, 212, 213,
    214, 215, 215, 216, 217, 217, 218, 219, 220, 220, 221, 222, 222, 223, 224, 225, 225, 226, 227, 227,
    228, 229, 230, 230, 231, 232, 232, 233, 234, 235, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242,
    242, 243, 244, 245, 245, 246, 247, 247, 248, 249, 250, 250, 251, 252, 252, 253, 254, 255, 255, 256,
    257, 257, 258, 259, 260, 260, 261, 262, 262, 263, 264, 265, 265, 266, 267, 267, 268, 269, 270, 270,
    271, 272, 272, 273, 274, 275, 275, 276, 277, 277, 278, 279, 280, 280, 281, 282, 282, 283, 284, 285,
    285, 286, 287, 287, 288, 289, 290, 290, 291, 292, 292, 293, 294, 295, 295, 296, 297, 297, 298, 299,
    300, 300, 301, 302, 302, 303, 304, 305, 305, 306, 307, 307, 308, 309, 310, 310, 311, 312, 312, 313,
    314, 315, 315, 316, 317, 317, 318, 319, 320, 320, 321, 322, 322, 323, 324, 325, 325, 326, 327, 327,
    328, 329, 330, 330, 331, 332, 332, 333, 334, 335, 335, 336, 337, 337, 338, 339, 340, 340, 341, 342,
    342, 343, 344, 345, 345, 346, 347, 347, 348, 349, 350, 350, 351, 352, 352, 353, 354, 355, 355, 356,
    357, 357, 358,
};

static const uint16_t * s_touchpad_map = s_touchpad_map_454p;

static void(*p_touchpad_irq_notify)(void) = NULL;

/********************************************************************************************
 *                       Public Functions
 ********************************************************************************************/
void touchpad_dev_init(uint32_t scrn_type, void(* _irq_notify)(void))
{
    if(scrn_type == TOUCHPAD_SCRN_TYPE_360P) {
        s_touchpad_map = s_touchpad_map_360p;
    } else if(scrn_type == TOUCHPAD_SCRN_TYPE_454P) {
        s_touchpad_map = s_touchpad_map_454p;
    }
    
    p_touchpad_irq_notify = _irq_notify;
    app_i2c_init(&s_tp_params, NULL);
    touchpad_dev_pin_init();
    touchpad_dev_reset();
    delay_ms(50);
}

void touchpad_dev_deinit(void)
{
    app_io_deinit(TOUCHPAD_RST_IO_TYPE, TOUCHPAD_RST_IO_PIN);
    app_io_deinit(TOUCHPAD_INT_IO_TYPE, TOUCHPAD_INT_IO_PIN);
    app_i2c_deinit(TOUCHPAD_I2C_ID);
    return;
}

bool touchpad_dev_read_reg(uint8_t reg_addr, uint8_t *buffer, uint16_t len)
{
    uint16_t ret = 0;
    ret = app_i2c_mem_read_sync(TOUCHPAD_I2C_ID, TOUCHPAD_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, len, 1000);
    if (ret != APP_DRV_SUCCESS)
    {
        return false;
    }
    return true;
}

bool touchpad_dev_write_reg(uint8_t reg_addr, uint8_t value)
{
    uint16_t ret = 0;
    ret = app_i2c_mem_write_sync(TOUCHPAD_I2C_ID, TOUCHPAD_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
    if (ret != APP_DRV_SUCCESS)
    {
        return false;
    }
    return true;
}

bool touchpad_dev_read_pointer(int16_t  * x, int16_t * y)
{
    uint8_t read_data[5];

    touchpad_dev_read_reg(0x5c, read_data, 5);

    if ((read_data[0] == 128) || (read_data[0] == 0 && read_data[1] == 0 \
       && read_data[2] == 0 && read_data[3] == 0 && read_data[4] == 0))
    {
        /**
         * the finger leave the touchpad
         */
        return false;
    }
    else
    {
        /**
         * when interrupt generates, set the mcu not to sleep until finger leave the touchpad
         * or the mcu may lost the interrupts
         */
        *x = s_touchpad_map[((((((read_data[1] & 0x0f)) << 8) | read_data[2])) >> 2)];
        *y = s_touchpad_map[((((((read_data[3] & 0x0f)) << 8) | read_data[4])) >> 2)];
        return true;
    }
}

bool touchpad_dev_sleep(void)
{
    return touchpad_dev_write_reg(0xFD, 0x01);
}

bool touchpad_dev_wakeup(void)
{
    return touchpad_dev_write_reg(0xA5, 0x03);
}

/********************************************************************************************
 *                       Static Functions
 ********************************************************************************************/

static void touchpad_dev_irq_handler(app_io_evt_t *p_evt)
{
    if(p_evt->pin == TOUCHPAD_INT_IO_PIN)
    {
        if(p_touchpad_irq_notify) {
            p_touchpad_irq_notify();
        }
    }
}

static void touchpad_dev_pin_init(void)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;

    /**
     * reset pin
     */
    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_OUTPUT;
    io_init.pin  = TOUCHPAD_RST_IO_PIN;
    io_init.mux  = TOUCHPAD_RST_IO_MUX;
    app_io_init(TOUCHPAD_RST_IO_TYPE, &io_init);

     /**
     * interrupt pin
     */
    app_gpiote_param_t param[1] =
    {
         TOUCHPAD_INT_IO_TYPE,
         TOUCHPAD_INT_IO_PIN,
         APP_IO_MODE_IT_FALLING,
         APP_IO_PULLUP,
         touchpad_dev_irq_handler,
     };
    app_gpiote_init(param, 1);
    GPIO2->INTENSET = 0x1;
}


static void touchpad_dev_reset(void)
{
    app_io_write_pin(TOUCHPAD_RST_IO_TYPE, TOUCHPAD_RST_IO_PIN, APP_IO_PIN_RESET);
    delay_ms(100);
    app_io_write_pin(TOUCHPAD_RST_IO_TYPE, TOUCHPAD_RST_IO_PIN, APP_IO_PIN_SET);
    delay_ms(100);
}
