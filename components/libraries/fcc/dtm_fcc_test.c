#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>
#include "dtm_fcc_test_int.h"

void fcc_test_init(void)
{
    sch_prog_push = sch_prog_push_fcc;
}

void dtm_test_init(void)
{
    sch_prog_push = sch_prog_push_func;
}

void sch_prog_push_fcc(struct sch_prog_params* args)
{
    bb_watch_timer_start = bb_watch_timer_start_fcc;
    g_bb_watch_alarm.cb_alarm = bb_watch_timer_cbk_fcc;
    sch_prog_push_func(args);
}

void em_ble_txrxcntl_txpwr_setf(int elt_idx, uint8_t txpwr)
{
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, \
    (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)txpwr << 0));
}

uint32_t ble_rwblecntl_get(void)
{
    return REG_BLE_RD(BLE_RWBLECNTL_ADDR);
}

void ble_rwblecntl_set(uint32_t value)
{
    REG_BLE_WR(BLE_RWBLECNTL_ADDR, value);
}

void bb_watch_timer_cbk_fcc(struct sch_alarm_tag* elt)
{
    if(lld_test_env != NULL)
    {
        // Point to parameters
        struct lld_test_env_tag* test_par = lld_test_env;
        if ((test_par->type == TEST_TX) && (test_par->data_len < 255))
        {
            // Abort the event
            ble_rwblecntl_set(ble_rwblecntl_get() | BLE_RFTEST_ABORT_BIT);
        }
    }
}

uint16_t em_ble_rxccmpktcnt0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

void lld_test_frm_isr_fcc(uint32_t timestamp, bool abort)
{
    if(lld_test_env != NULL)
    {
        // Point to parameters
        struct lld_test_env_tag* test_par = lld_test_env;
        struct sch_arb_elt_tag* evt = &(lld_test_env->evt);

        // Remove event
        sch_arb_remove(evt, true);

        // Check test mode end
        if(test_par->state == TEST_EVT_END)
        {
            // Report test mode end to LLM
            struct lld_test_end_ind* ind = KE_MSG_ALLOC(LLD_TEST_END_IND, TASK_LLM, TASK_NONE, lld_test_end_ind);
            ind->status = CO_ERROR_NO_ERROR;
            ind->nb_pkt_recv = (test_par->type == TEST_RX) ? em_ble_rxccmpktcnt0_get(EM_BLE_CS_ACT_ID_TO_INDEX(TEST_LINK_ID)) : 0;
            ke_msg_send(ind);

            if (test_par->type == TEST_TX)
            {
                // Release TX buffer
                ble_util_buf_acl_tx_free(test_par->em_buf);
            }

            // Free event memory
            lld_test_cleanup();
        }
        else
        {
            // update event priority
            evt->current_prio = abort
                              ? (evt->current_prio + rwip_priority[RWIP_PRIO_ADV_IDX].increment)
                              : rwip_priority[RWIP_PRIO_ADV_IDX].value;

            do
            {
                // Reschedule ASAP
                SCH_ARB_ASAP_STG_SET(evt, SCH_ARB_FLAG_NO_ASAP, SCH_ARB_NO_PHASE, 0, 0);
                evt->time.hs = CLK_ADD_2(rwip_time_get().hs, FCC_IFS_HALF_SLOT);

                // Try to reschedule
                if (sch_arb_insert(evt) == 0)
                {
                    test_par->state = TEST_EVT_WAIT;
                    break;
                }
            }while(1);
        }
    }
}

void lld_test_frm_cbk_fcc(uint32_t timestamp, uint32_t dummy, uint8_t irq_type)
{
    FPB_PATCH_OFF();
    switch(irq_type)
    {
        case SCH_FRAME_IRQ_EOF:
        {
            lld_test_frm_isr_fcc(timestamp, false);
        } break;
        case SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO:
        case SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO:
        {
            lld_test_frm_isr_fcc(timestamp, true);
        } break;
        case SCH_FRAME_IRQ_RX:
        {
            lld_test_rx_isr(timestamp);
        } break;
    }
    FPB_PATCH_ON();
}

void bb_watch_timer_start_fcc(struct sch_prog_params* params)
{
    FPB_PATCH_OFF();
    g_bb_watch_alarm.time.hs = CLK_ADD_2(rwip_time_get().hs, 3);
    sch_alarm_clear(&g_bb_watch_alarm);
    sch_alarm_set(&g_bb_watch_alarm);
    if(TEST_LINK_ID == params->cs_idx)
    {
        uint8_t et_idx = sch_prog_env.et_idx_next_prog;
        sch_prog_env.tab[et_idx].frm_cbk = lld_test_frm_cbk_fcc;
    }
    FPB_PATCH_ON();
}
