#include "notification_center.h"
#include "app_graphics_mem.h"
#include "hal_gfx_graphics.h"
#include "hal_gfx_cmdlist.h"
#include "osal.h"

#define MAX_NOTIFICATION_NUM 48

static notification_info_t *s_notifications = NULL;

static void create_content_cache(notification_info_t *info);
static void destroy_content_cache(notification_info_t *info);

static void create_content_cache_async(notification_info_t *info);
static void create_content_cache_async_wrapper(void *user_data);

static inline void free_notification_info(notification_info_t *info);

static inline void gpu_clear_buffer(hal_gfx_cmdlist_t *cl, void *bg, lv_point_t *size, uint32_t color);

void notification_center_add(uint32_t uid, void *icon, const char *title, uint16_t title_len, const char *content, uint16_t content_len)
{
    notification_info_t *new_info = osal_heap_malloc(sizeof(notification_info_t));

    new_info->uid   = uid;
    new_info->icon  = icon;
    new_info->title = osal_heap_malloc(title_len + 1);

    memcpy(new_info->title, title, title_len);
    new_info->title[title_len] = '\0';

    new_info->content = osal_heap_malloc(content_len + 1);

    memcpy(new_info->content, content, content_len);
    new_info->content[content_len] = '\0';

    new_info->thumbnail = NULL;
    new_info->next = NULL;

    osal_enter_critical();
    uint32_t i = 1;
    if (s_notifications)
    {
        notification_info_t *p = s_notifications;
        while (p->next)
        {
            i++;
            p = p->next;
        }
        p->next = new_info;
    }
    else
    {
        s_notifications = new_info;
    }

    if (i >= MAX_NOTIFICATION_NUM)
    {
        // Delete head (the oldest one)
        notification_center_delete_by_info(s_notifications);
    }

    create_content_cache_async(new_info);
    osal_exit_critical();
}

notification_info_t *notification_center_get_header(void)
{
    return s_notifications;
}

notification_info_t *notification_center_get_by_uid(uint16_t uid)
{
    notification_info_t *p = s_notifications;
    while (p)
    {
        if (p->uid == uid)
        {
            break;
        }
        p = p->next;
    }

    return p;
}

notification_info_t *notification_center_get_by_index(uint32_t index)
{
    notification_info_t *p = s_notifications;
    if (p)
    {
        while (p && index)
        {
            p = p->next;
            index--;
        }
    }
    return p;
}

uint32_t notification_center_get_info_count(void)
{
    uint32_t i = 0;
    notification_info_t *p = s_notifications;

    osal_enter_critical();
    while (p)
    {
        p = p->next;
        i++;
    }
    osal_exit_critical();

    return i;
}

void notification_center_delete_by_uid(uint32_t uid)
{
    notification_info_t *p = s_notifications;

    osal_enter_critical();
    if (s_notifications->uid == uid)
    {
        s_notifications = p->next;
        free_notification_info(p);
    }
    else
    {
        notification_info_t *q = p;
        while (p && p->uid != uid)
        {
            q = p;
            p = p->next;
        }

        if (p)
        {
            q->next = p->next;
            free_notification_info(p);
        }
    }
    osal_exit_critical();
}

void notification_center_delete_by_info(notification_info_t *info)
{
    notification_info_t *p = s_notifications;

    osal_enter_critical();

    if (s_notifications == info)
    {
        s_notifications = p->next;
        free_notification_info(p);
    }
    else
    {
        notification_info_t *q = p;
        while (p->next && p != info)
        {
            q = p;
            p = p->next;
        }

        if (p)
        {
            q->next = p->next;
            free_notification_info(p);
        }
    }

    osal_exit_critical();
}

void notification_center_clear(void)
{
    notification_info_t *p = s_notifications;
    notification_info_t *q;
    while (p)
    {
        q = p->next;
        free_notification_info(p);
        p = q;
    }
    s_notifications = NULL;
}

static void create_content_cache(notification_info_t *info)
{
    if (info->thumbnail)
    {
        return;
    }

    osal_enter_critical();
    info->thumbnail = osal_heap_malloc(sizeof(lv_img_dsc_t));
    info->thumbnail->data_size = NOTIFICATION_ITEM_WIDTH * NOTIFICATION_ITEM_HEIGHT * 2;
    info->thumbnail->data = app_graphics_mem_malloc(info->thumbnail->data_size);
    osal_exit_critical();

    info->thumbnail->header.always_zero = 0;
    info->thumbnail->header.cf = LV_IMG_CF_GDX_RGB565;
    info->thumbnail->header.w = NOTIFICATION_ITEM_WIDTH;
    info->thumbnail->header.h = NOTIFICATION_ITEM_HEIGHT;

    // Create item cache
    hal_gfx_cmdlist_t cmd = hal_gfx_cl_le_create();
    hal_gfx_cmdlist_t *cl = &cmd;
    hal_gfx_cl_bind_circular(cl);

    lv_draw_ctx_t *draw_ctx = lv_disp_get_default()->driver->draw_ctx;
    draw_ctx->buf = (void *)info->thumbnail->data;

    lv_area_t area = {
        .x1 = 0,
        .y1 = 0,
        .x2 = NOTIFICATION_ITEM_WIDTH - 1,
        .y2 = NOTIFICATION_ITEM_HEIGHT - 1,
    };
    lv_point_t size = {
        .x = NOTIFICATION_ITEM_WIDTH,
        .y = NOTIFICATION_ITEM_HEIGHT,
    };

    draw_ctx->clip_area = &area;
    draw_ctx->buf_area = &area;

    // Clear
    gpu_clear_buffer(cl, draw_ctx->buf, &size, 0);

    // Draw BG Rectangle
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_make(0x24, 0x24, 0x24);
    rect_dsc.radius = 24;
    lv_area_t draw_area = {
        .x1 = 0,
        .y1 = 24,
        .x2 = NOTIFICATION_ITEM_WIDTH - 1,
        .y2 = NOTIFICATION_ITEM_HEIGHT - 1,
    };
    lv_draw_rect(draw_ctx, &rect_dsc, &draw_area);

    // Draw Icon
    lv_draw_img_dsc_t img_dsc;
    lv_draw_img_dsc_init(&img_dsc);
    draw_area.x1 = 24;
    draw_area.y1 = 0;
    draw_area.x2 = draw_area.x1 + 48 - 1;
    draw_area.y2 = draw_area.y1 + 48 - 1;
    lv_draw_img(draw_ctx, &img_dsc, &draw_area, info->icon);

    // Draw Title
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.font = &lv_font_montserrat_20;
    draw_area.x1 = 80;
    draw_area.y1 = 30;
    draw_area.x2 = NOTIFICATION_ITEM_WIDTH - 1;
    draw_area.y2 = draw_area.y1 + 22 - 1;
    lv_draw_label(draw_ctx, &label_dsc, &draw_area, info->title, NULL);

    // Draw Content
    label_dsc.color = lv_color_make(0xCC, 0xCC, 0xCC);
    label_dsc.font = &lv_font_montserrat_20;
    draw_area.x1 = 10;
    draw_area.y1 = 52;
    draw_area.x2 = NOTIFICATION_ITEM_WIDTH - 11;
    draw_area.y2 = NOTIFICATION_ITEM_HEIGHT - 11;
    lv_draw_label(draw_ctx, &label_dsc, &draw_area, info->content, NULL);

    hal_gfx_cl_le_destroy(cl);
}

static void destroy_content_cache(notification_info_t *info)
{
    if (info->thumbnail)
    {
        if (info->thumbnail->data)
        {
            app_graphics_mem_free((void *)info->thumbnail->data);
        }
        osal_heap_free(info->thumbnail);
        info->thumbnail = NULL;
    }
    else
    {
        lv_async_call_cancel(create_content_cache_async_wrapper, (void *)info);
    }
}

static void create_content_cache_async(notification_info_t *info)
{
    lv_async_call(create_content_cache_async_wrapper, (void *)info);
}

static void create_content_cache_async_wrapper(void *user_data)
{
    create_content_cache((notification_info_t *)user_data);
}

static inline void free_notification_info(notification_info_t *info)
{
    if (info)
    {
        destroy_content_cache(info);

        osal_heap_free(info->title);
        info->title = NULL;
        osal_heap_free(info->content);
        info->content = NULL;
        osal_heap_free(info);
    }
}

static inline void gpu_clear_buffer(hal_gfx_cmdlist_t *cl, void *bg, lv_point_t *size, uint32_t color)
{
    extern uint32_t lv_port_get_fb_format(void);
    hal_gfx_bind_dst_tex((uintptr_t)bg, size->x, size->y, lv_port_get_fb_format(), -1);
    hal_gfx_set_clip(0, 0, size->x, size->y);
    hal_gfx_clear(color);
    hal_gfx_cl_submit(cl);
    hal_gfx_cl_wait(cl);
}
