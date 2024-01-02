/**
  ****************************************************************************************
  * @file    app_io.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_drv.h"
#include "app_io.h"
#include "grx_hal.h"
#include "gr_soc.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define IO_MODE_NONE               0x0
#define IO_GROUP_MAX               (3)
#define IO_GROUP_MAX_PINS          (16)
#define IO_GROUP_MAX_PINS_GR551X   (32)
#define IO_GROUP_TYPE_MAX          (3)
#define GPIO_INT_PIN_MAX           (SOC_GPIO_PINS_MAX)
#define AON_INT_PIN_MAX            (SOC_AON_PINS_MAX)
#define PIN_MASK                   (0x1)
#define PIN_SHIF                   (1)
#define IRQ_NUM_NONE               (0)
#define GET_HANDLE(type)           ((gpio_regs_t *)io_info[type].handle)
#define GET_PIN_IRQ(type)          (io_info[type].irq)

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

typedef struct {
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    uint8_t           io_type;
#endif
    app_io_callback_t callback_func;
    void              *arg;
} io_evt_info_t;

typedef struct {
    gpio_regs_t *handle;
    IRQn_Type    irq;
} io_dev_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint16_t s_io_pull[IO_GROUP_TYPE_MAX][APP_IO_PULL_MAX] =
{
    { GPIO_NOPULL,     GPIO_PULLUP,     GPIO_PULLDOWN },
    { AON_GPIO_NOPULL, AON_GPIO_PULLUP, AON_GPIO_PULLDOWN },
    { MSIO_NOPULL,     MSIO_PULLUP,     MSIO_PULLDOWN },
};

static io_evt_info_t gpio_evt_info[GPIO_INT_PIN_MAX];
static io_evt_info_t aon_evt_info[AON_INT_PIN_MAX];

static const io_dev_info_t io_info[IO_GROUP_MAX] =
{
    {
        .handle = GPIO0,
        .irq  = EXT0_IRQn,
    },
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    {
        .handle = GPIO1,
        .irq  = EXT1_IRQn,
    },
#endif
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X)
    {
        .handle = GPIO2,
        .irq  = EXT2_IRQn,
    },
#endif
};

static const uint16_t s_io_mode[IO_GROUP_TYPE_MAX][APP_IO_MODE_MAX] =
{
    {
        IO_MODE_NONE,
        GPIO_MODE_INPUT,
        GPIO_MODE_OUTPUT,
        GPIO_MODE_MUX,
        GPIO_MODE_IT_RISING,
        GPIO_MODE_IT_FALLING,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        GPIO_MODE_IT_BOTH_EDGE,
#endif
        GPIO_MODE_IT_HIGH,
        GPIO_MODE_IT_LOW,
        IO_MODE_NONE
    },
    {
        IO_MODE_NONE,
        AON_GPIO_MODE_INPUT,
        AON_GPIO_MODE_OUTPUT,
        AON_GPIO_MODE_MUX,
        AON_GPIO_MODE_IT_RISING,
        AON_GPIO_MODE_IT_FALLING,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        AON_GPIO_MODE_IT_BOTH_EDGE,
#endif
        AON_GPIO_MODE_IT_HIGH,
        AON_GPIO_MODE_IT_LOW,
        IO_MODE_NONE
    },
    {
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR551X)
        MSIO_MODE_DIGITAL,
#endif
        MSIO_MODE_DIGITAL,
        MSIO_MODE_DIGITAL,
        MSIO_MODE_ANALOG
    },
};

#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
static const uint16_t s_io_strength[IO_GROUP_TYPE_MAX][APP_IO_STRENGTH_MAX] =
{
    {
        GPIO_STRENGTH_LOW,
        GPIO_STRENGTH_MEDIUM,
        GPIO_STRENGTH_HIGH,
        GPIO_STRENGTH_ULTRA,
    },
    {
        AON_GPIO_STRENGTH_LOW,
        AON_GPIO_STRENGTH_MEDIUM,
        AON_GPIO_STRENGTH_HIGH,
        AON_GPIO_STRENGTH_ULTRA,
    },
    {
        MSIO_STRENGTH_LOW,
        MSIO_STRENGTH_MEDIUM,
        MSIO_STRENGTH_HIGH,
        MSIO_STRENGTH_ULTRA,
    }
};

static const uint16_t s_io_speed[IO_GROUP_TYPE_MAX][APP_IO_SPPED_MAX] =
{
    {GPIO_SPEED_MEDIUM, GPIO_SPEED_HIGH},
    {AON_GPIO_SPEED_MEDIUM, AON_GPIO_SPEED_HIGH},
    {MSIO_SPEED_MEDIUM, MSIO_SPEED_HIGH},
};

static const uint16_t s_io_input_type[IO_GROUP_TYPE_MAX][APP_IO_INPUT_TYPE_MAX] =
{
    {GPIO_INPUT_TYPE_CMOS, GPIO_INPUT_TYPE_SCHMITT},
    {AON_GPIO_INPUT_TYPE_CMOS, AON_GPIO_INPUT_TYPE_SCHMITT},
    {MSIO_INPUT_TYPE_CMOS, MSIO_INPUT_TYPE_SCHMITT},
};
#endif

void EXT0_IRQHandler(void);
void EXT1_IRQHandler(void);
void EXT2_IRQHandler(void);
void AON_EXT_IRQHandler(void);

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
static int get_pin_index(uint32_t pin)
{
    int index = 0;
    while ((pin & PIN_MASK) != PIN_MASK)
    {
        index++;
        pin = pin >> PIN_SHIF;
    }
    return index;
}

uint16_t app_io_init(app_io_type_t type, app_io_init_t *p_init)
{
    gpio_init_t     io_config     = GPIO_DEFAULT_CONFIG;
    aon_gpio_init_t aon_io_config = AON_GPIO_DEFAULT_CONFIG;
    msio_init_t     msio_config   = MSIO_DEFAULT_CONFIG;
    gpio_regs_t    *p_handle      = NULL;

    if (NULL == p_init)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    soc_register_nvic(EXT0_IRQn, (uint32_t)EXT0_IRQHandler);
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
    soc_register_nvic(EXT1_IRQn, (uint32_t)EXT1_IRQHandler);
#endif
    soc_register_nvic(AON_EXT_IRQn, (uint32_t)AON_EXT_IRQHandler);
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5526X))
    soc_register_nvic(EXT2_IRQn, (uint32_t)EXT2_IRQHandler);
#endif

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            io_config.mode = s_io_mode[0][p_init->mode];
            io_config.pull = s_io_pull[0][p_init->pull];
            io_config.mux  = p_init->mux;
            io_config.pin  = p_init->pin;
            p_handle = GET_HANDLE(type);
            if ((!(p_init->pin & APP_IO_PINS_0_15)) || (p_handle == NULL))
                 return APP_DRV_ERR_INVALID_PARAM;
            hal_gpio_init(p_handle, &io_config);
            break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        case APP_IO_TYPE_NORMAL:
            io_config.mode = s_io_mode[0][p_init->mode];
            io_config.pull = s_io_pull[0][p_init->pull];
            io_config.mux  = p_init->mux;
            io_config.pin  = p_init->pin;

            if (APP_IO_PINS_0_15 & p_init->pin)
            {
                io_config.pin = (APP_IO_PINS_0_15 & p_init->pin);
                hal_gpio_init(GPIO0, &io_config);
            }
            if (APP_IO_PINS_16_31 & p_init->pin)
            {
                io_config.pin = (APP_IO_PINS_16_31 & p_init->pin) >> 16;
                hal_gpio_init(GPIO1, &io_config);
            }
            break;
#endif
        case APP_IO_TYPE_AON:
            aon_io_config.mode = s_io_mode[1][p_init->mode];
            aon_io_config.pull = s_io_pull[1][p_init->pull];
            aon_io_config.mux  = p_init->mux;
            aon_io_config.pin  = p_init->pin;
            if (!(p_init->pin & APP_AON_IO_PIN_ALL))
                return APP_DRV_ERR_INVALID_PARAM;
            hal_aon_gpio_init(&aon_io_config);
            break;

        case APP_IO_TYPE_MSIO:
            if (p_init->mode >= APP_IO_MODE_IT_RISING && p_init->mode <= APP_IO_MODE_IT_LOW)
            {
                return APP_DRV_ERR_INVALID_MODE;
            }
            if (APP_IO_MODE_OUTPUT == p_init->mode)
            {
                 msio_config.direction = MSIO_DIRECTION_OUTPUT;
            }
            else
            {
                msio_config.direction = MSIO_DIRECTION_INPUT;
            }
            msio_config.mode = s_io_mode[2][p_init->mode];
            msio_config.pull = s_io_pull[2][p_init->pull];
            msio_config.mux  = p_init->mux;
            msio_config.pin  = p_init->pin;
            if (!(p_init->pin & APP_MSIO_IO_PIN_ALL))
                return APP_DRV_ERR_INVALID_PARAM;
            hal_msio_init(MSIOA, &msio_config);
            break;

        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_io_deinit(app_io_type_t type, uint32_t pin)
{
    gpio_regs_t *p_handle = NULL;

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if ((!(pin & APP_IO_PINS_0_15)) || (p_handle == NULL))
                return APP_DRV_ERR_INVALID_PARAM;
            hal_gpio_deinit(p_handle, pin);
            break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        case APP_IO_TYPE_NORMAL:
            if (APP_IO_PINS_0_15 & pin)
            {
                hal_gpio_deinit(GPIO0, (APP_IO_PINS_0_15 & pin));
            }
            if (APP_IO_PINS_16_31 & pin)
            {
                hal_gpio_deinit(GPIO1, (APP_IO_PINS_16_31 & pin) >> 16);
            }
            break;
#endif
        case APP_IO_TYPE_AON:
            if (!(pin & APP_AON_IO_PIN_ALL))
                return APP_DRV_ERR_INVALID_PARAM;
            hal_aon_gpio_deinit(pin);
            break;

        case APP_IO_TYPE_MSIO:
            if (!(pin & APP_MSIO_IO_PIN_ALL))
                return APP_DRV_ERR_INVALID_PARAM;
            hal_msio_deinit(MSIOA, pin);
            break;

        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }

    return APP_DRV_SUCCESS;
}

app_io_pin_state_t app_io_read_pin(app_io_type_t type, uint32_t pin)
{
    app_io_pin_state_t pin_state = APP_IO_PIN_RESET;
    gpio_regs_t       *p_handle  = NULL;

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            pin_state = (app_io_pin_state_t)hal_gpio_read_pin(p_handle, pin);
            break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        case APP_IO_TYPE_NORMAL:
            if (APP_IO_PINS_0_15 & pin)
            {
                pin_state = (app_io_pin_state_t)hal_gpio_read_pin(GPIO0, pin);
            }
            if (APP_IO_PINS_16_31 & pin)
            {
                pin_state = (app_io_pin_state_t)hal_gpio_read_pin(GPIO1, pin >> 16);
            }
            break;
#endif
        case APP_IO_TYPE_AON:
            pin_state = (app_io_pin_state_t)hal_aon_gpio_read_pin(pin);
            break;

        case APP_IO_TYPE_MSIO:
            pin_state = (app_io_pin_state_t)hal_msio_read_pin(MSIOA, pin);
            break;

        default:
            break;
    }

    return pin_state;
}

uint16_t app_io_write_pin(app_io_type_t type, uint32_t pin, app_io_pin_state_t pin_state)
{
    gpio_regs_t *p_handle  = NULL;

    if (pin_state != APP_IO_PIN_RESET && pin_state != APP_IO_PIN_SET)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if (p_handle == NULL)
                return APP_DRV_ERR_INVALID_PARAM;
            hal_gpio_write_pin(p_handle, pin, (gpio_pin_state_t)pin_state);
            break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        case APP_IO_TYPE_NORMAL:
            if (APP_IO_PINS_0_15 & pin)
            {
                hal_gpio_write_pin(GPIO0, (uint16_t)(APP_IO_PINS_0_15 & pin), (gpio_pin_state_t)pin_state);
            }
            if (APP_IO_PINS_16_31 & pin)
            {
                hal_gpio_write_pin(GPIO1, (uint16_t)((APP_IO_PINS_16_31 & pin) >> 16), (gpio_pin_state_t)pin_state);
            }
            break;
#endif
        case APP_IO_TYPE_AON:
            hal_aon_gpio_write_pin(pin, (aon_gpio_pin_state_t)pin_state);
            break;

        case APP_IO_TYPE_MSIO:
            hal_msio_write_pin(MSIOA, pin, (msio_pin_state_t)pin_state);
            break;

        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_io_toggle_pin(app_io_type_t type, uint32_t pin)
{
    gpio_regs_t *p_handle  = NULL;

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if (p_handle == NULL)
                return APP_DRV_ERR_INVALID_PARAM;
            hal_gpio_toggle_pin(p_handle, pin);
            break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        case APP_IO_TYPE_NORMAL:
            if (APP_IO_PINS_0_15 & pin)
            {
                hal_gpio_toggle_pin(GPIO0, (uint16_t)(APP_IO_PINS_0_15 & pin));
            }
            if (APP_IO_PINS_16_31 & pin)
            {
                hal_gpio_toggle_pin(GPIO1, (uint16_t)((APP_IO_PINS_16_31 & pin) >> 16));
            }
            break;
#endif
        case APP_IO_TYPE_AON:
            hal_aon_gpio_toggle_pin(pin);
            break;

        case APP_IO_TYPE_MSIO:
            hal_msio_toggle_pin(MSIOA, pin);
            break;
        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }

    return APP_DRV_SUCCESS;
}


uint16_t app_io_set_speed(app_io_type_t type, uint32_t pin, app_io_speed_t speed)
{
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
    gpio_regs_t *p_handle  = NULL;
    uint32_t     io_speed;

    if (speed >= APP_IO_SPPED_MAX)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if (p_handle == NULL)
                return APP_DRV_ERR_INVALID_PARAM;

            io_speed = s_io_speed[0][speed];
            ll_gpio_set_pin_speed(p_handle, pin, io_speed);
            break;

        case APP_IO_TYPE_AON:
            io_speed = s_io_speed[1][speed];
            ll_aon_gpio_set_pin_speed(pin, io_speed);
            break;
        case APP_IO_TYPE_MSIO:
            io_speed = s_io_speed[2][speed];
            ll_msio_set_pin_speed(MSIOA, pin, io_speed);
            break;
        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }
    return APP_DRV_SUCCESS;
#else
    return APP_DRV_ERR_INVALID_MODE;
#endif
}


uint16_t app_io_set_strength(app_io_type_t type, uint32_t pin, app_io_strength_t strength)
{
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
    gpio_regs_t *p_handle  = NULL;
    uint32_t     io_strength;

    if (strength >= APP_IO_STRENGTH_MAX)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if (p_handle == NULL)
                return APP_DRV_ERR_INVALID_PARAM;

            io_strength = s_io_strength[0][strength];
            ll_gpio_set_pin_strength(p_handle, pin, io_strength);
            break;

        case APP_IO_TYPE_AON:
            io_strength = s_io_strength[1][strength];
            ll_aon_gpio_set_pin_strength(pin, io_strength);
            break;
        case APP_IO_TYPE_MSIO:
            io_strength = s_io_strength[2][strength];
            ll_msio_set_pin_strength(MSIOA, pin, io_strength);
            break;
        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }
    return APP_DRV_SUCCESS;
#else
    return APP_DRV_ERR_INVALID_MODE;
#endif
}

uint16_t app_io_set_intput_type(app_io_type_t type, uint32_t pin, app_io_input_type_t input_type)
{
#if ((APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5332X) || (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR5525X))
    gpio_regs_t *p_handle  = NULL;
    uint32_t     io_input_type;

    if (input_type >= APP_IO_INPUT_TYPE_MAX)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(type)
    {
        case APP_IO_TYPE_GPIOA:
        case APP_IO_TYPE_GPIOB:
        case APP_IO_TYPE_GPIOC:
            p_handle = GET_HANDLE(type);
            if (p_handle == NULL)
                return APP_DRV_ERR_INVALID_PARAM;

            io_input_type = s_io_input_type[0][input_type];
            ll_gpio_set_pin_input_type(p_handle, pin, io_input_type);
            break;

        case APP_IO_TYPE_AON:
            io_input_type = s_io_input_type[1][input_type];
            ll_aon_gpio_set_pin_input_type(pin, io_input_type);
            break;
        case APP_IO_TYPE_MSIO:
            io_input_type = s_io_input_type[2][input_type];
            ll_msio_set_pin_input_type(MSIOA, pin, io_input_type);
            break;
        default:
            return APP_DRV_ERR_INVALID_TYPE;
    }
    return APP_DRV_SUCCESS;
#else
    return APP_DRV_ERR_INVALID_MODE;
#endif
}

uint16_t app_io_event_register_cb(app_io_type_t type, app_io_init_t *p_init, app_io_callback_t io_evt_cb, void *arg)
{
    uint16_t ret;
    uint32_t pin = p_init->pin;
    uint8_t  pin_index;
    uint8_t  base_pins = 0;
    uint16_t group_pins;
    IRQn_Type irq;

    if (type == APP_IO_TYPE_MSIO)
    {
        return APP_DRV_ERR_INVALID_TYPE;
    }

    app_io_deinit(type, pin);
    ret = app_io_init(type, p_init);
    APP_DRV_ERR_CODE_CHECK(ret);

    if (type == APP_IO_TYPE_AON)
    {
        group_pins = AON_INT_PIN_MAX;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        switch (p_init->mode)
        {
            case APP_IO_MODE_IT_RISING:
                hal_pwr_config_ext_wakeup(pin, PWR_EXTWKUP_TYPE_RISING);
                break;

            case APP_IO_MODE_IT_FALLING:
                hal_pwr_config_ext_wakeup(pin, PWR_EXTWKUP_TYPE_FALLING);
                break;

            case APP_IO_MODE_IT_HIGH:
                hal_pwr_config_ext_wakeup(pin, PWR_EXTWKUP_TYPE_HIGH);
                break;

            case APP_IO_MODE_IT_LOW:
                hal_pwr_config_ext_wakeup(pin, PWR_EXTWKUP_TYPE_LOW);
                break;

            default: break;
        }
        extern void pwr_mgmt_wakeup_source_setup(uint32_t wakeup_source);
        pwr_mgmt_wakeup_source_setup(PWR_WKUP_COND_EXT);
#endif
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    else if (type == APP_IO_TYPE_NORMAL)
    {
        group_pins = IO_GROUP_MAX_PINS_GR551X;
        base_pins  = 0;
    }
#endif
    else
    {
        group_pins = IO_GROUP_MAX_PINS;
        base_pins  = IO_GROUP_MAX_PINS * type;
    }
    for (pin_index = 0; pin_index < group_pins; pin_index++)
    {
        if (pin & PIN_MASK)
        {
            switch(type)
            {
                case APP_IO_TYPE_GPIOA:
                case APP_IO_TYPE_GPIOB:
                case APP_IO_TYPE_GPIOC:
                    irq = GET_PIN_IRQ(type);
                    if (irq == IRQ_NUM_NONE)
                        return APP_DRV_ERR_INVALID_PARAM;
                    GLOBAL_EXCEPTION_DISABLE();
                    gpio_evt_info[pin_index + base_pins].callback_func = io_evt_cb;
                    gpio_evt_info[pin_index + base_pins].arg = arg;
                    GLOBAL_EXCEPTION_ENABLE();
                    hal_nvic_clear_pending_irq(irq);
                    hal_nvic_enable_irq(irq);
                    break;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
                case APP_IO_TYPE_NORMAL:
                    GLOBAL_EXCEPTION_DISABLE();
                    gpio_evt_info[pin_index + base_pins].callback_func = io_evt_cb;
                    gpio_evt_info[pin_index + base_pins].arg     = arg;
                    gpio_evt_info[pin_index + base_pins].io_type = APP_IO_TYPE_NORMAL;
                    GLOBAL_EXCEPTION_ENABLE();

                    hal_nvic_clear_pending_irq(EXT0_IRQn);
                    hal_nvic_enable_irq(EXT0_IRQn);
                    hal_nvic_clear_pending_irq(EXT1_IRQn);
                    hal_nvic_enable_irq(EXT1_IRQn);
                    break;
#endif
                case APP_IO_TYPE_AON:
                    GLOBAL_EXCEPTION_DISABLE();
                    aon_evt_info[pin_index + base_pins].callback_func = io_evt_cb;
                    aon_evt_info[pin_index + base_pins].arg = arg;
                    GLOBAL_EXCEPTION_ENABLE();
                    hal_nvic_clear_pending_irq(AON_EXT_IRQn);
                    hal_nvic_enable_irq(AON_EXT_IRQn);
                    break;
                default:
                    return APP_DRV_ERR_INVALID_TYPE;
            }
        }
        pin >>= PIN_SHIF;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_io_event_unregister(app_io_type_t type, uint32_t pin)
{
    uint16_t group_pins;
    uint8_t  pin_index;
    uint8_t  base_pins = 0;
    uint32_t pin_tmp = pin;
    io_evt_info_t *p_evt_cb = NULL;

    if (type == APP_IO_TYPE_MSIO)
    {
        return APP_DRV_ERR_INVALID_TYPE;
    }

    if (type == APP_IO_TYPE_AON)
    {
        group_pins = AON_INT_PIN_MAX;
        p_evt_cb = aon_evt_info;
    }
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
    else if (type == APP_IO_TYPE_NORMAL)
    {
        group_pins = IO_GROUP_MAX_PINS_GR551X;
        base_pins  = 0;
        p_evt_cb   = gpio_evt_info;
    }
#endif
    else
    {
        group_pins = IO_GROUP_MAX_PINS;
        base_pins  = IO_GROUP_MAX_PINS * type;
        p_evt_cb   = gpio_evt_info;
    }

    for (pin_index = 0; pin_index < group_pins; pin_index++)
    {
        if (pin_tmp & PIN_MASK)
        {
            GLOBAL_EXCEPTION_DISABLE();
            p_evt_cb[pin_index + base_pins].callback_func = NULL;
            p_evt_cb[pin_index + base_pins].arg = NULL;
#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
            gpio_evt_info[pin_index + base_pins].io_type = 0;
#endif
            GLOBAL_EXCEPTION_ENABLE();
        }
        pin_tmp >>= PIN_SHIF;
    }

    return app_io_deinit(type, pin);
}

void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    uint16_t pin_index = get_pin_index(gpio_pin);
    app_io_evt_t io_evt;
    int idx;

    for (idx = 0; idx < IO_GROUP_MAX; idx++)
    {
        if (GPIOx != io_info[idx].handle)
            continue;

        pin_index += idx * IO_GROUP_MAX_PINS;

#if (APP_DRIVER_CHIP_TYPE == APP_DRIVER_GR551X)
        if (gpio_evt_info[pin_index].io_type == APP_IO_TYPE_NORMAL)
        {
            io_evt.type = APP_IO_TYPE_NORMAL;
        }
        else
        {
            io_evt.type = (app_io_type_t)idx;
        }
        if (GPIO1 == GPIOx)
        {
            io_evt.pin  = (uint32_t)(gpio_pin << 16);
        }
        else
        {
            io_evt.pin  = gpio_pin;
        }
#else
        io_evt.type = (app_io_type_t)idx;
        io_evt.pin  = gpio_pin;
#endif
        io_evt.arg  = gpio_evt_info[pin_index].arg;
        if (gpio_evt_info[pin_index].callback_func != NULL)
            gpio_evt_info[pin_index].callback_func(&io_evt);
    }
}

void hal_aon_gpio_callback(uint16_t aon_gpio_pin)
{
    app_io_evt_t io_evt;
    uint16_t pin_index = 0;
    uint16_t aon_pin = aon_gpio_pin;
    int i;

    io_evt.type = APP_IO_TYPE_AON;
    for (i = 0; i < AON_INT_PIN_MAX; i++)
    {
        if (aon_pin & PIN_MASK)
        {
            io_evt.pin = 0x1U << pin_index;
            io_evt.arg = aon_evt_info[pin_index].arg;
            if (aon_evt_info[pin_index].callback_func != NULL)
                aon_evt_info[pin_index].callback_func(&io_evt);
        }
        pin_index++;
        aon_pin >>= PIN_SHIF;
    }
}

#define EXT_IQR_HANDLER(index) \
SECTION_RAM_CODE void EXT##index##_IRQHandler(void)\
{\
    hal_gpio_exti_irq_handler(io_info[index].handle);\
}

SECTION_RAM_CODE void AON_EXT_IRQHandler(void)
{
    hal_aon_gpio_irq_handler();
}

EXT_IQR_HANDLER(0)
#if (APP_DRIVER_CHIP_TYPE != APP_DRIVER_GR5332X)
EXT_IQR_HANDLER(1)
EXT_IQR_HANDLER(2)
#endif

