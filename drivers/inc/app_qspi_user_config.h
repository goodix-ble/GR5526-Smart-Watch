/****************************************************
 * Config for special User Scenarios.
 * Modify By Users
 *
 ********
 * This file is only included by app_qspi.h
 ****************************************************/

#ifndef __APP_QSPI_USER_CONFIG_H__
#define __APP_QSPI_USER_CONFIG_H__

/*********************************************************************
 *            PART 1:   Enable/Disable Enhanced Application Interface
 *********************************************************************/

/*
 * @brief:
 *      Supporting to Draw the whole Screen using Spliced way with two independent framebuffers in Synchronous mode
 * @note:
 *      1 - Enable  the corresponding API
 *      0 - Disable the corresponding API
 */
#define QSPI_SYNC_SCROLL_DRAW_SCREEN_SUPPORT            1u


/*
 * @brief:
 *      Supporting to Draw the whole Screen using Spliced way with two independent framebuffers in Asynchronous mode,
 *      The API has handled the continuous async call already,
 *      User can get the event result (APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT or APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL)
 *      from the Registered callback
 *
 * @note:
 *      1 - Enable  the corresponding API
 *      0 - Disable the corresponding API
 */

#define QSPI_ASYNC_SCROLL_DRAW_SCREEN_SUPPORT           1u


/*
 * @brief:
 *      Supporting to Draw the whole Screen using linked list way with serval independent framebuffers in Asynchronous mode,
 *      The API has handled the continuous async call already,
 *      User can get the event result (APP_QSPI_EVT_ASYNC_WR_SCRN_CPLT or APP_QSPI_EVT_ASYNC_WR_SCRN_FAIL)
 *      from the Registered callback
 *
 * @note:
 *      1 - Enable  the corresponding API
 *      0 - Disable the corresponding API
 */
#define QSPI_ASYNC_VERI_LINK_DRAW_SCREEN_SUPPORT        1u

/*
 * @brief:
 *      Supporting to Write the QSPI-PSRAM device with DMA LLP Mode
 *
 * @note:
 *      1 - Enable  the corresponding API
 *      0 - Disable the corresponding API
 */
#define QSPI_LLP_WRITE_PSRAM_SUPPORT                    1u

/*
 * @brief:
 *      Supporting to Blit the Rectangular area with LLP/SG Feature
 *
 * @note:
 *      1 - Enable  the corresponding API
 *      0 - Disable the corresponding API
 */
#define QSPI_BLIT_RECT_IMAGE_SUPPORT                    0u


/*********************************************************************
 *            PART 2:   Memory Adjust Setting
 *********************************************************************/
/*
 * @brief:
 *      DO NOT Change This Define
 *
 */
#define QSPI_DMA_LLP_FEATUTE_SUPPORT                    (QSPI_SYNC_SCROLL_DRAW_SCREEN_SUPPORT  ||       \
                                                         QSPI_ASYNC_SCROLL_DRAW_SCREEN_SUPPORT ||       \
                                                         QSPI_ASYNC_VERI_LINK_DRAW_SCREEN_SUPPORT ||    \
                                                         QSPI_LLP_WRITE_PSRAM_SUPPORT )

/*
 * @brief:
 *      Max blocks for LLP write, the bigger, gains higher speed and costs more ram space
 *      The proper range is [16, 64], and be times of 4
 *
 */
#define DMA_LLP_BLOCKS_FOR_WRITE                        48u

/*
 * @brief:
 *      Max blocks for LLP Blit, the bigger, gains higher speed and costs more ram space
 *      The proper range is [16, 64], and be times of 4
 *
 */
#define DMA_LLP_BLOCKS_FOR_BLIT                         32u


#endif /*__APP_QSPI_USER_CONFIG_H__*/

