#ifndef __PATCH_TAB_H_
#define __PATCH_TAB_H_

#include <stdint.h>

typedef uint16_t task_id_t;
typedef uint16_t msg_id_t;


typedef int (*msg_func_t)(msg_id_t const msgid, void const *param, task_id_t const dest_id, task_id_t const src_id);

typedef int (*llm_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

typedef int (*lli_hci_cmd_hdl_func_t)(void const *param, uint16_t opcode);

typedef int (*gapm_hci_evt_hdl_func_t)(uint16_t opcode, void const *param);

typedef struct
{
    msg_func_t ori_func_addr;
    msg_func_t new_func_addr;
} ke_msg_tab_item_t;

typedef struct
{
    llm_hci_cmd_hdl_func_t ori_func_addr;
    llm_hci_cmd_hdl_func_t new_func_addr;
} llm_hci_cmd_tab_item_t;

typedef struct
{
    lli_hci_cmd_hdl_func_t ori_func_addr;
    lli_hci_cmd_hdl_func_t new_func_addr;
} lli_hci_cmd_tab_item_t;

typedef struct
{
    gapm_hci_evt_hdl_func_t ori_func_addr;
    gapm_hci_evt_hdl_func_t new_func_addr;
} gapm_hci_evt_tab_item_t;

extern void ble_common_env_init(void);
extern void ble_con_env_init(void);
extern void ble_scan_env_init(void);
extern void ble_adv_env_init(void);
extern void ble_test_evn_init(void);
extern void ble_iso_env_init(void);
extern void ble_eatt_evn_init(void);
extern void ble_mul_link_env_init(void);
extern void ble_car_key_env_init(void);
extern void ble_bt_bredr_env_init(void);

// sdk task for common
extern int host_to_sdk_msg_handler_patch(msg_id_t const msgid, void *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int gap_activity_stopped_ind_handler_patch(msg_id_t const msgid, void const *p_param,
    task_id_t const dest_id, task_id_t const src_id);

// llm task for common
extern int llm_hci_command_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id,task_id_t const src_id);

// gapc task for common
extern int gapc_hci_handler_patch(msg_id_t const msgid, void const* event,
    task_id_t dest_id, task_id_t src_id);

// gapm task for common
extern int gapm_hci_handler_patch(msg_id_t const msgid, void const* event,
    task_id_t dest_id, task_id_t opcode);

extern int gapm_set_dev_config_cmd_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int gapm_activity_create_cmd_handler_patch(msg_id_t const msgid, void *p_param,
    task_id_t const dest_id, task_id_t const src_id);

//llm hci cmd handler for common
extern int hci_le_add_dev_to_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_rmv_dev_from_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_clear_rslv_list_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_set_addr_resol_en_cmd_handler_patch(void const *param, uint16_t opcode);

extern int hci_le_set_priv_mode_cmd_handler_patch(void const *param, uint16_t opcode);

#if CFG_MAX_CONNECTIONS
// sdk task for connection
extern int sec_rcv_encrypt_req_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int sec_rcv_bond_req_ind_handler_patch(msg_id_t const msgid, void *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int gap_connection_req_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// sdk task for connection
extern int gap_disconnect_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// llc task for connection
extern int llc_op_dl_upd_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int lld_llcp_rx_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int llc_op_ch_class_en_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int llc_op_ch_class_rep_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// gattc task for connection
extern int l2cc_lecb_sdu_recv_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// sdk task for gattc
extern int ble_sdk_gattc_extend_prf_cmp_evt_handler_patch(msg_id_t const msgid, void const *p_param,
    task_id_t const dest_id, task_id_t const src_id);

extern int ble_sdk_gattc_extend_prf_event_ind_handler_patch(msg_id_t const msgid, void const *p_param,
    task_id_t const dest_id, task_id_t const src_id);

extern int ble_sdk_gattc_extend_prf_sdp_srvc_ind_handler_patch(msg_id_t const msgid, void const *p_param,
    task_id_t const dest_id, task_id_t const src_id);
#endif

#if CFG_ISO_SUPPORT
// lli task for iso
extern int hci_command_lli_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id,task_id_t const src_id);

// lli hci cmd handler
extern int hci_le_big_create_sync_cmd_handler_patch(void const *p_param, uint16_t opcode);
#endif

#if CFG_MAX_SYNCS
// gapm hci cmd cmp evt for per sync
extern int hci_le_cmd_cmp_evt_per_sync_handler_patch(uint16_t opcode, void const *p_event);
#endif

#if CFG_CAR_KEY_SUPPORT
// llm task for car key
extern int llm_pub_key_gen_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// llm hci cmd handler for car key
extern int hci_le_rd_local_p256_public_key_cmd_handler_patch(void const *param, uint16_t opcode);
#endif

#if CFG_MUL_LINK_WITH_SAME_DEV
// gapm hci event for multiple link
extern int hci_le_adv_set_term_evt_handler_patch(uint16_t opcode, void const *p_event);

// gapc task for multiple link
extern int gapc_bond_cfm_handler_patch(msg_id_t const msgid, void *cfm,
    task_id_t const dest_id, task_id_t const src_id);

extern int lld_adv_end_ind_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);
#endif

#if CFG_EATT_SUPPORT
// sdk task for eatt
extern int l2cap_enh_lecb_rcv_reconfig_ind_handler_patch(msg_id_t const msgid, void *param,
    task_id_t const dest_id, task_id_t const src_id);

extern int l2cap_lecb_rcv_cmp_evt_handler_patch(msg_id_t const msgid, void const *param,
    task_id_t const dest_id, task_id_t const src_id);

// l2cc task for eatt
extern int l2cc_enh_lecb_connect_cfm_handler_patch(msg_id_t const msgid, void *param,
    task_id_t const dest_id, task_id_t const src_id);
#endif

#if CFG_DTM_TEST
// llm hci cmd handler for test
extern int hci_le_tx_test_v4_cmd_handler_patch(struct hci_le_tx_test_v4_cmd const *param, uint16_t opcode);
#endif

ke_msg_tab_item_t ke_msg_tab[] =
{
    // ble sdk task for common
    {(msg_func_t)0x00094a4d, (msg_func_t)host_to_sdk_msg_handler_patch},
    {(msg_func_t)0x0008dff5, (msg_func_t)gap_activity_stopped_ind_handler_patch},
    // llm task for common
    {(msg_func_t)0x000333a9, (msg_func_t)llm_hci_command_handler_patch},
    // gapc task for common
    {(msg_func_t)0x00012cc9, (msg_func_t)gapc_hci_handler_patch},
    // gapm task for common
    {(msg_func_t)0x00018555, (msg_func_t)gapm_hci_handler_patch},
    {(msg_func_t)0x0001a25d, (msg_func_t)gapm_set_dev_config_cmd_handler_patch},
    {(msg_func_t)0x00016c65, (msg_func_t)gapm_activity_create_cmd_handler_patch},

    #if CFG_MAX_CONNECTIONS
    // sdk task for connection
    {(msg_func_t)0x000990ed, (msg_func_t)sec_rcv_encrypt_req_ind_handler_patch},
    {(msg_func_t)0x00098edd, (msg_func_t)sec_rcv_bond_req_ind_handler_patch},
    {(msg_func_t)0x0008ecd9, (msg_func_t)gap_connection_req_ind_handler_patch},
    {(msg_func_t)0x0008f645, (msg_func_t)gap_disconnect_ind_handler_patch},
    // llc task for connection
    {(msg_func_t)0x0004d915, (msg_func_t)llc_op_dl_upd_ind_handler_patch},
    {(msg_func_t)0x00063255, (msg_func_t)lld_llcp_rx_ind_handler_patch},
    {(msg_func_t)0x0004d4b9, (msg_func_t)llc_op_ch_class_en_ind_handler_patch},
    {(msg_func_t)0x0004d575, (msg_func_t)llc_op_ch_class_rep_ind_handler_patch},
    // gattc task for connection
    {(msg_func_t)0x000432f1, (msg_func_t)l2cc_lecb_sdu_recv_ind_handler_patch},
    // sdk task for gattc
    {(msg_func_t)0x0008abad, (msg_func_t)ble_sdk_gattc_extend_prf_cmp_evt_handler_patch},
    {(msg_func_t)0x0008b019, (msg_func_t)ble_sdk_gattc_extend_prf_event_ind_handler_patch},
    {(msg_func_t)0x0008b0e9, (msg_func_t)ble_sdk_gattc_extend_prf_sdp_srvc_ind_handler_patch},
    #endif

    #if CFG_EATT_SUPPORT
    // sdk task for eatt
    {(msg_func_t)0x00095835, (msg_func_t)l2cap_enh_lecb_rcv_reconfig_ind_handler_patch},
    {(msg_func_t)0x00095b51, (msg_func_t)l2cap_lecb_rcv_cmp_evt_handler_patch},
    // l2cc task for eatt
    {(msg_func_t)0x000423f1, (msg_func_t)l2cc_enh_lecb_connect_cfm_handler_patch},
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(msg_func_t)0x00011e61, (msg_func_t)gapc_bond_cfm_handler_patch},
    {(msg_func_t)0x00050e09, (msg_func_t)lld_adv_end_ind_handler_patch},
    #endif

    #if CFG_ISO_SUPPORT
    // lli task for iso
    {(msg_func_t)0x00033375, (msg_func_t)hci_command_lli_handler_patch},
    #endif

    #if CFG_CAR_KEY_SUPPORT
    // llm task for car key
    {(msg_func_t)0x00071369, (msg_func_t)llm_pub_key_gen_ind_handler_patch},
    #endif
};

llm_hci_cmd_tab_item_t llm_hci_cmd_tab[] =
{
    // llm hci cmd for common
    {(llm_hci_cmd_hdl_func_t)0x00034425, (llm_hci_cmd_hdl_func_t)hci_le_add_dev_to_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00038669, (llm_hci_cmd_hdl_func_t)hci_le_rmv_dev_from_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00034a7d, (llm_hci_cmd_hdl_func_t)hci_le_clear_rslv_list_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x00038929, (llm_hci_cmd_hdl_func_t)hci_le_set_addr_resol_en_cmd_handler_patch},
    {(llm_hci_cmd_hdl_func_t)0x0003aea5, (llm_hci_cmd_hdl_func_t)hci_le_set_priv_mode_cmd_handler_patch},

    #if CFG_DTM_TEST
    // llm hci cmd for dtm test
    {(llm_hci_cmd_hdl_func_t)0x0003b9f5, (llm_hci_cmd_hdl_func_t)hci_le_tx_test_v4_cmd_handler_patch},
    #endif

    #if CFG_CAR_KEY_SUPPORT
    // llm hci cmd for car key
    {(llm_hci_cmd_hdl_func_t)0x00037841, (llm_hci_cmd_hdl_func_t)hci_le_rd_local_p256_public_key_cmd_handler_patch},
    #endif
};

#if CFG_ISO_SUPPORT
lli_hci_cmd_tab_item_t lli_hci_cmd_tab[] =
{
    {(lli_hci_cmd_hdl_func_t)0x00034765, (lli_hci_cmd_hdl_func_t)hci_le_big_create_sync_cmd_handler_patch},
};
#endif

gapm_hci_evt_tab_item_t gapm_hci_evt_tab[] =
{
    {NULL, NULL},

    #if CFG_MAX_SYNCS
    {(gapm_hci_evt_hdl_func_t)0x00034d8d, (gapm_hci_evt_hdl_func_t)hci_le_cmd_cmp_evt_per_sync_handler_patch},
    #endif

    #if CFG_MUL_LINK_WITH_SAME_DEV
    {(gapm_hci_evt_hdl_func_t)0x346ad, (gapm_hci_evt_hdl_func_t)hci_le_adv_set_term_evt_handler_patch},
    #endif
};

extern void reg_ke_msg_patch_tab(ke_msg_tab_item_t *ke_msg_tab, uint16_t ke_msg_cnt);
extern void reg_gapm_hci_evt_patch_tab(gapm_hci_evt_tab_item_t *gapm_hci_evt_tab, uint16_t gapm_hci_evt_cnt);
extern void reg_llm_hci_cmd_patch_tab(llm_hci_cmd_tab_item_t *llm_hci_cmd_tab, uint16_t llm_hci_cmd_cnt);

#if CFG_ISO_SUPPORT
extern void reg_lli_hci_cmd_patch_tab(lli_hci_cmd_tab_item_t *lli_hci_cmd_tab, uint16_t lli_hci_cmd_cnt);
#endif

#endif
