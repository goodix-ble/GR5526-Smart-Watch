#ifndef __NOTIFICATION_CENTER_H__
#define __NOTIFICATION_CENTER_H__

#include "lvgl.h"

#define NOTIFICATION_ITEM_WIDTH    300
#define NOTIFICATION_ITEM_HEIGHT   120

typedef struct _st_notification_item
{
    uint32_t uid;
    void *icon;
    char *title;
    char *content;
    lv_img_dsc_t *thumbnail;
    struct _st_notification_item *next;
} notification_info_t;

void notification_center_add(uint32_t uid, void *icon, const char *title, uint16_t title_len, const char *content, uint16_t content_len);

notification_info_t *notification_center_get_header(void);

notification_info_t *notification_center_get_by_uid(uint16_t uid);

notification_info_t *notification_center_get_by_index(uint32_t index);

uint32_t notification_center_get_info_count(void);

void notification_center_delete_by_uid(uint32_t uid);

void notification_center_delete_by_info(notification_info_t *info);

void notification_center_clear(void);

#endif // __NOTIFICATION_CENTER_H__
