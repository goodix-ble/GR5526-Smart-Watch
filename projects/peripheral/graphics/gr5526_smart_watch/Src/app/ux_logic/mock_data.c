#include "mock_data.h"
#include "lvgl.h"
#include "lv_img_dsc_list.h"
#include "notification_center.h"

void mock_data_init_pre_gui(void)
{

}

void mock_data_init_post_gui(void)
{
    // Mock data for notification center
    static const lv_img_dsc_t *ICON_LIST[] = {
        &wd_img_msg_default,
        &wd_img_msg_outlook,
        &wd_img_msg_phone,
        &wd_img_msg_qq,
        &wd_img_msg_sms,
        &wd_img_msg_twitter,
        &wd_img_msg_wechat,
    };

    static const char *TITLE_LIST[] = {
        "Test",
        "Outlook",
        "PhoneCall",
        "QQ",
        "SMS",
        "Twitter",
        "Wechat",
    };

    for (int i = 0; i < 20; i++)
    {
        notification_center_add(i, (void *)ICON_LIST[i % 7], TITLE_LIST[i % 7], strlen(TITLE_LIST[i % 7]), "Hello there, this is GR5526VGBIP LVGL8.3 Demo!", sizeof("Hello there, this is GR5526VGBIP LVGL8.3 Demo!") - 1);
    }
}


