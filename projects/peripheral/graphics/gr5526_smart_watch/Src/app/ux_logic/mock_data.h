#ifndef __MOCK_DATA_H__
#define __MOCK_DATA_H__

/**
 * @brief Init mock data which should be initialized before GUI (LVGL+WMS) is ready.
 */
void mock_data_init_pre_gui(void);

/**
 * @brief Init mock data which should be initialized after GUI (LVGL+WMS) is ready.
 */
void mock_data_init_post_gui(void);

#endif // __MOCK_DATA_H__
