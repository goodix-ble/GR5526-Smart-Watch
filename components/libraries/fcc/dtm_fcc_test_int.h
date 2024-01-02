#ifndef __DTM_FCC_TEST_INT_H__
#define __DTM_FCC_TEST_INT_H__

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdarg.h>

/// FCC IFS
#define FCC_IFS_HALF_SLOT     4

/// Clock
#define RWIP_MAX_CLOCK_TIME              ((1L<<28) - 1)
#define CLK_ADD_2(clock_a, clock_b)      ((uint32_t)(((clock_a) + (clock_b)) & RWIP_MAX_CLOCK_TIME))

/// Register Operations
#define REG_BLE_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)
#define REG_BLE_RD(addr)             (*(volatile uint32_t *)(addr))
#define EM_BLE_RD(addr)              (*(volatile uint16_t *)(addr))
#define EM_BLE_WR(addr, value)       (*(volatile uint16_t *)(addr)) = (value)

/// CS Index
#define TEST_LINK_ID                         (0)
#define EM_BLE_CS_ACT_ID_TO_INDEX(act_id)    (act_id)

/// Baseband Register Address
#define REG_EM_BLE_CS_SIZE           (148)
#define CO_ALIGN2_HI(val)            (((val)+1)&~1)
#define REG_CORE_BASE_ADDR           (0xB0000000)
#define REG_EM_BASE_ADDR             (0xB0008000)
#define BLE_RWBLECNTL_ADDR           (REG_CORE_BASE_ADDR + 0x00000000)
#define EM_COMMON_OFFSET             (0)

/// Exchange table area definition
#define EM_EXCH_TABLE_LEN            16
#define EM_ET_OFFSET                 (EM_COMMON_OFFSET)
#define EM_ET_END                    (EM_ET_OFFSET + EM_EXCH_TABLE_LEN * REG_EM_ET_SIZE)

/// Frequency table area definition
#define EM_FT_OFFSET                 (EM_ET_END)
#define EM_RF_FREQ_TABLE_LEN          40
#define EM_RF_VCO_TABLE_LEN           0
#define EM_FT_END                    (EM_FT_OFFSET + (EM_RF_VCO_TABLE_LEN + EM_RF_FREQ_TABLE_LEN) * sizeof(uint8_t))
#define EM_COMMON_END                (EM_FT_END)

/// Encryption area definition
#define EM_ENC_OFFSET                 CO_ALIGN2_HI(EM_COMMON_END)
#define EM_ENC_IN_OFFSET             (EM_ENC_OFFSET)
#define EM_ENC_IN_SIZE               (16)
#define EM_ENC_OUT_OFFSET            (EM_ENC_IN_OFFSET + EM_ENC_IN_SIZE)
#define EM_ENC_OUT_SIZE              (16)
#define EM_ENC_END                   (EM_ENC_OFFSET + EM_ENC_IN_SIZE + EM_ENC_OUT_SIZE)
#define EM_BLE_OFFSET                (EM_ENC_END)
#define EM_BLE_CS_OFFSET             (EM_BLE_OFFSET)
#define EM_BLE_RXCCMPKTCNT0_ADDR     (REG_EM_BASE_ADDR + 0x0000005A + EM_BLE_CS_OFFSET)

/// TXRXCNTL register definition
#define EM_BLE_TXRXCNTL_ADDR         (REG_EM_BASE_ADDR + 0x0000001A + EM_BLE_CS_OFFSET)

/// Baseband Register Bits
#define BLE_RFTEST_ABORT_BIT         ((uint32_t)0x04000000)

/// EM ET Size
#define REG_EM_ET_SIZE 16

/// Error Code
#define CO_ERROR_NO_ERROR             0x00

/// Convenient wrapper to ke_msg_alloc()
#define KE_MSG_ALLOC(id, dest, src, param_str) \
    (struct param_str*) ke_msg_alloc(id, dest, src, sizeof(struct param_str))

/// Build the first message ID of a task. (in fact a ke_msg_id_t)
#define TASK_FIRST_MSG(task) ((uint16_t)((task) << 8))

/// Tasks types definition, this value shall be in [0-254] range
#define TASK_ID_LLD          (2)
#define TASK_NONE            (0xFF)

/// Maximum length of switching pattern
#define BLE_MAX_SW_PAT_LEN     12

/// FPB Definitions 
typedef enum
{
  FPB_PATCH_OFF=0,
  FPB_PATCH_ON,
}fpb_t;

void fpb_set_state(fpb_t state);

fpb_t fpb_save_state(void);

void fpb_load_state(fpb_t state);

#define FPB_SAVE()       fpb_t __fpb_state_local_var=fpb_save_state()
#define FPB_LOAD()       fpb_load_state(__fpb_state_local_var)
#define FPB_PATCH_ON()   fpb_save_state();
#define FPB_PATCH_OFF()  fpb_load_state(FPB_PATCH_OFF)

/// Test mode event states
enum TEST_TYPE
{
    TEST_RX,
    TEST_TX,
};

/// Message API of the LLM task
/*@TRACE*/
enum lld_msg_id
{
    LLD_MSG_ID_FIRST = TASK_FIRST_MSG(TASK_ID_LLD),

    /*
     * ************** Msg LD->LLM****************
     */
    LLD_ADV_REP_IND,
    LLD_SCAN_REQ_IND,
    LLD_SYNC_START_REQ,
    LLD_PER_ADV_REP_IND,
    LLD_PER_ADV_RX_END_IND,
    LLD_SCAN_END_IND,
    LLD_ADV_END_IND,
    LLD_PER_ADV_END_IND,
    LLD_INIT_END_IND,
    LLD_TEST_END_IND,
};

enum KE_TASK_TYPE
{
    // Link Layer Tasks
    TASK_LLM,
};

/// Frame type
enum SCH_FRAME_IRQ
{
    /// Normal End of Event
    SCH_FRAME_IRQ_EOF        = 0x00,
    /// End of event due to an abort under the primary priority duration
    SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO  = 0x01,
    /// End of event due to an abort after the primary priority duration
    SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO  = 0x07,
    /// SKIP Event IRQ
    SCH_FRAME_IRQ_SKIP       = 0x04,
    /// RX ACL IRQ
    SCH_FRAME_IRQ_RX         = 0x02,
    /// TX ACL IRQ
    SCH_FRAME_IRQ_TX         = 0x03,
};

/// Priority index definition
enum rwip_prio_idx
{
    /// Default priority for scanning events
    RWIP_PRIO_SCAN_IDX,
    /// Default priority for auxillary scan/init (no_asap) rx events
    RWIP_PRIO_AUX_RX_IDX,
    /// Default priority for periodic adv rx events
    RWIP_PRIO_PER_ADV_RX_DFT_IDX,
    /// Default priority for initiating events
    RWIP_PRIO_INIT_IDX,
    /// LE connection events default priority
    RWIP_PRIO_CONNECT_DFT_IDX,
    /// LE connection events priority with activity
    RWIP_PRIO_CONNECT_ACT_IDX,
    /// Default priority for advertising events
    RWIP_PRIO_ADV_IDX,
    /// Default priority for advertising high duty cycle events
    RWIP_PRIO_ADV_HDC_IDX,
    /// Default priority for aux advertising events
    RWIP_PRIO_ADV_AUX_IDX,
    /// Default priority for periodic advertising events
    RWIP_PRIO_PER_ADV_IDX,
    /// Default priority for resolvable private addresses renewal event
    RWIP_PRIO_RPA_RENEW_IDX,

    RWIP_PRIO_IDX_MAX
};

/// ASAP type definition
enum sch_arb_elt_asap_type
{
    /// 00: No ASAP
    SCH_ARB_FLAG_NO_ASAP                = 0,
    /// 01: ASAP no limit
    SCH_ARB_FLAG_ASAP_NO_LIMIT,
    /// 10: ASAP with limit
    SCH_ARB_FLAG_ASAP_LIMIT,
    SCH_ARB_FLAG_MAX
};

/// ASAP slot parity definition
enum sch_arb_elt_asap_phase
{
    SCH_ARB_PHASE_0,
    SCH_ARB_PHASE_1,
    SCH_ARB_PHASE_2,
    SCH_ARB_PHASE_3,
    SCH_ARB_NO_PHASE,
};

/// Test mode event states
enum TEST_EVT_STATE
{
    TEST_EVT_WAIT,
    TEST_EVT_ACTIVE,
    TEST_EVT_END,
};

/// Set ASAP settings
#define SCH_ARB_ASAP_STG_SET(evt, type, phase, resched_att, prio_inc) \
(evt->asap_settings = \
( (((type) << 14) & 0xC000) | (((phase) << 11) & 0x3800) | \
  (((resched_att) << 4) & 0x03F0) | (((prio_inc) << 0) & 0x000F) ));

/// List
struct co_list_hdr
{
    /// Pointer to next co_list_hdr
    struct co_list_hdr *next;
};

/// simplify type name of list element header
typedef struct co_list_hdr co_list_hdr_t;

/// Message Identifier. The number of messages is limited to 0xFFFF.
typedef uint16_t ke_msg_id_t;

/// Task Identifier. Composed by the task type and the task index.
typedef uint16_t ke_task_id_t;

/// Callback for interrupt related to the frame
typedef void (*frm_cbk_t)(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

/// Time information
typedef struct
{
    /// Integer part of the time (in half-slot)
    uint32_t hs;
    /// Fractional part of the time (in half-us) (range: 0-624)
    uint16_t hus;
    /// Bluetooth timestamp value (in us) 32 bits counter
    uint32_t bts;
} rwip_time_t;

/// Alarm information
struct sch_alarm_tag
{
    /// List element for chaining in the Even Arbiter lists
    struct co_list_hdr hdr;

    /// Timestamp of alarm expiry (in BT half-slots)
    rwip_time_t        time;

    /// Call back function invoked upon alarm expiry
    void (*cb_alarm)(struct sch_alarm_tag*);
};

/// Test mode end indication structure
struct lld_test_end_ind
{
    /// Status (BLE error code)
    uint8_t status;

    /// Number of packets received
    uint16_t nb_pkt_recv;
};

/// Additional parameters for BLE frame
struct sch_prog_params_ble
{
    /// Advertising channel type: 0x0: AE Start on Primary channel | 0x1: AE Start on Secondary channel
    uint8_t ae_nps;
    /// Reserved audio event (0: reTx, 1:primary)
    uint8_t rsvd;
    /// Isochronous event (0: normal event | 1: iso event)
    uint8_t iso;
    /// Start Instant Correction (SIC) bit value
    uint8_t sic;
};

/// Parameters for a programmed frame
struct sch_prog_params
{
    /// Callback for handling interrupts related to the frame
    frm_cbk_t frm_cbk;
    /// Timestamp (in half-slots, based on local clock) and event offset (in half-us) of the programmed frame
    rwip_time_t time;
    /// Bandwidth duration of the event using priority 1 (in half us)
    uint32_t bandwidth;
    /// Dummy value reported when an event happen during the frame or the frame is completed
    uint32_t dummy;
    /// Priority during duration of bandwidth
    uint8_t prio_1;
    /// Priority after bandwidth elapsed
    uint8_t prio_2;
    /// Priority after trigger conditions
    uint8_t prio_3;
    /// Priority when specific action occurs during the event
    uint8_t pti_prio;
    /// Control structure index
    uint8_t cs_idx;
    /// Mode (0: BLE, 1:BT)
    uint8_t mode;

    union
    {
        /// Additional parameters for BLE frame
        struct sch_prog_params_ble ble;
    } add;
};

/// Schedule Program Push Hook
typedef void (*sch_prog_push_func_t)(struct sch_prog_params*);

/// Scheduling Arbiter Element
/*@TRACE*/
struct sch_arb_elt_tag
{
    /// List element for chaining in the Even Arbiter lists
    struct co_list_hdr hdr;

    /// Programming time expressed in half-slots and half-us
    rwip_time_t time;

    /// Scheduling time limit in base time (half-slots) (only for ASAP LIMIT requests)
    uint32_t asap_limit;

    /// Minimum duration of the event or frame (in half-us)
    uint32_t duration_min;

    /**
     * ASAP settings field
     * bit |15  14|13  12  11|     10      |    9..4     |   3..0   |
     * def | TYPE |   Phase  | To protect  | Resched att | Prio inc |
     *
     * Type:
     *  - 00: No ASAP
     *  - 01: ASAP no limit
     *  - 10: ASAP with limit
     *  - 11: ASAP with limit, no parity check
     *
     * Phase: (only for ASAP requests)
     *  - 0: phase 0
     *  - 1: phase 1
     *  - 2: phase 2
     *  - 3: phase 3
     *  - 4: don't care
     *
     * Number of rescheduling attempts:
     *  - The remaining number of rescheduling attempts.
     *  - Rescheduling happens when the event is overlapped by a higher priority event
     *  - Only used for ASAP requests
     *
     * Priority increment:
     *  - The current priority value is incremented each time the event is overlapped by a new insertion and postponed
     *  - Only used for ASAP requests
     */
    /*@trc_desc
     *bit |15..14|  13..11  |  10  |    9..4     |   3..0   |
     *def | Type |   Phase  | Rsvd | Resched_att | Prio_inc |
     *
     *Phase: only if Type = 01..11
     *Resched_att: only if Type = 01..11
     *Prio_inc: only if Type = 01..11
     *
     *@trc_ref Type:
     *  - 00: No ASAP
     *  - 01: ASAP no limit
     *  - 10: ASAP with limit
     *  - 11: ASAP with limit and no parity check
     *
     *@trc_ref Phase:
     *  - 000: phase 0
     *  - 001: phase 1
     *  - 010: phase 2
     *  - 011: phase 3
     *  - 100: don-t care
     */
    uint16_t asap_settings;

    /// Current priority
    uint8_t current_prio;
    /// Latency to notify to stop the activity before next activity is notified to start (in half-slots, 0 if no stop required)
    uint8_t stop_latency;
    // GDX Controller, Activity Role
    uint8_t activity_role;
	// GDX Controller, CS Index
    uint8_t cs_idx;

    /************************************************************************************
     * ISR CALLBACKS
     ************************************************************************************/

    /// Start notification call back function
    void (*cb_start)(struct sch_arb_elt_tag*);
    /// Stop notification call back function
    void (*cb_stop)(struct sch_arb_elt_tag*);
    /// Cancel notification call back function
    void (*cb_cancel)(struct sch_arb_elt_tag*);
};

/// LLD test mode environment structure
struct lld_test_env_tag
{
    /// Pointer to inquiry event
    struct sch_arb_elt_tag evt;

    /// Buffer used for sending the test data
    uint16_t em_buf;

    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel, N = (F - 2402) / 2
    uint8_t channel;

    /// Length of test data
    uint8_t data_len;

    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t payload;

    /// current state of the test mode
    uint8_t state;

    /**
     * CTE length
     * 0x00 No Constant Tone Extension
     * 0x02 - 0x14 Length of the Constant Tone Extension in 8 us units
     * All other values Reserved for future use
     */
    uint8_t cte_len;

    /**
     * CTE type
     * 0x00 AoA Constant Tone Extension
     * 0x01 AoD Constant Tone Extension with 1 us slots
     * 0x02 AoD Constant Tone Extension with 2 us slots
     * All other values Reserved for future use
     */
    uint8_t cte_type;

    /**
     * Slot durations
     * 0x01 Switching and sampling slots are 1 us each
     * 0x02 Switching and sampling slots are 2 us each
     * All other values Reserved for future use
     */
    uint8_t slot_dur;

    /**
     * Length of switching pattern
     * 0x02 - 0x4B The number of Antenna IDs in the pattern
     * All other values Reserved for future use
     */
    uint8_t switching_pattern_len;
};

/// Structure of a frame element
struct sch_prog_frm_elt
{
    /// Timestamp of the programmed frame (in BLE half slots, based on local clock)
    uint32_t timestamp;
    /// Callback for handling interrupts related to the frame
    frm_cbk_t frm_cbk;
    /// Dummy value (to be reported to the driver)
    uint32_t dummy;
    /// Indicate if the frame is valid (programmed and not skipped or finished)
    bool valid;
};

/// SCH_PROG environment structure
struct sch_prog_env_tag
{
    /// Frame elements pool
    struct sch_prog_frm_elt tab[REG_EM_ET_SIZE];

    /// Exchange table index of the oldest entry currently used by the HW
    uint8_t et_idx_current;

    /// Next exchange table index to program
    uint8_t et_idx_next_prog;

    /// Number of programmed frames
    uint8_t nb_prog;
};

/// API functions of the RF driver that are used by the BLE or BT software
struct rwip_rf_api
{
    /// Function called upon HCI reset command reception
    void (*reset)(void);
    /// Function called to enable/disable force AGC mechanism (true: en / false : dis)
    void (*force_agc_enable)(bool);
    /// Function called when TX power has to be decreased for a specific link id
    bool (*txpwr_dec)(uint8_t);
    /// Function called when TX power has to be increased for a specific link id
    bool (*txpwr_inc)(uint8_t);
    /// Function called when TX power has to be set to max for a specific link id
    void (*txpwr_max_set)(uint8_t);
    /// Function called to convert a TX power CS power field into the corresponding value in dBm
    int8_t (*txpwr_dbm_get)(uint8_t, uint8_t);
    /// Function called to convert a power in dBm into a control structure tx power field
    uint8_t (*txpwr_cs_get)(int8_t, uint8_t);
    /// Function called to convert the RSSI read from the control structure into a real RSSI
    int8_t (*rssi_convert)(uint8_t);
    /// Function used to read a RF register
    uint32_t (*reg_rd)(uint32_t);
    /// Function used to write a RF register
    void (*reg_wr)(uint32_t, uint32_t);
    /// Function called to put the RF in deep sleep mode
    void (*sleep)(void);
    /// Function called to set tx Power
    uint8_t (*tx_pwr_set)(uint8_t, int8_t, bool);
    /// Index of minimum TX power
    uint8_t txpwr_min;
    /// Index of maximum TX power
    uint8_t txpwr_max;
    /// GDX Controller : Index of default TX power
    uint8_t txpwr_default;
    /// RSSI high threshold ('real' signed value in dBm)
    int8_t rssi_high_thr;
    /// RSSI low threshold ('real' signed value in dBm)
    int8_t rssi_low_thr;
    /// interferer threshold ('real' signed value in dBm)
    int8_t rssi_interf_thr;
    /// RF wakeup delay (in slots)
    uint8_t wakeup_delay;
    /// For ranging link configure RF and modem to enter RANGING mode (one way)
    void (*gdx_one_way_ranging_mode_en)(uint8_t, bool);
    /// For configure SX parameters to new frequency (one way)
    void (*gdx_one_way_ranging_cfg_sx_param)(uint8_t, uint8_t);
    /// For ranging link configure RF and modem to enter RANGING mode (two way)
    void (*gdx_two_way_ranging_mode_en)(uint8_t, bool);
    /// For configure SX parameters to new frequency (two way)
    void (*gdx_two_way_ranging_cfg_sx_param)(uint8_t, uint8_t);
};

/// Internal API for priority
struct rwip_prio
{
    ///value
    uint8_t value;
    ///Increment
    uint8_t increment;
};

struct lld_test_params
{
    /// Type (0: RX | 1: TX)
    uint8_t type;

    /// RF channel, N = (F - 2402) / 2
    uint8_t channel;

    /// Length of test data
    uint8_t data_len;

    /**
     * Packet payload
     * 0x00 PRBS9 sequence "11111111100000111101" (in transmission order) as described in [Vol 6] Part F, Section 4.1.5
     * 0x01 Repeated "11110000" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x02 Repeated "10101010" (in transmission order) sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x03 PRBS15 sequence as described in [Vol 6] Part F, Section 4.1.5
     * 0x04 Repeated "11111111" (in transmission order) sequence
     * 0x05 Repeated "00000000" (in transmission order) sequence
     * 0x06 Repeated "00001111" (in transmission order) sequence
     * 0x07 Repeated "01010101" (in transmission order) sequence
     * 0x08-0xFF Reserved for future use
     */
    uint8_t payload;

    /**
     * Tx/Rx PHY
     * For Tx PHY:
     * 0x00 Reserved for future use
     * 0x01 LE 1M PHY
     * 0x02 LE 2M PHY
     * 0x03 LE Coded PHY with S=8 data coding
     * 0x04 LE Coded PHY with S=2 data coding
     * 0x05-0xFF Reserved for future use
     * For Rx PHY:
     * 0x00 Reserved for future use
     * 0x01 LE 1M PHY
     * 0x02 LE 2M PHY
     * 0x03 LE Coded PHY
     * 0x04-0xFF Reserved for future use
     */
    uint8_t phy;

    /**
     * CTE length
     * 0x00 No Constant Tone Extension
     * 0x02 - 0x14 Length of the Constant Tone Extension in 8 us units
     * All other values Reserved for future use
     */
    uint8_t cte_len;

    /**
     * CTE type
     * 0x00 AoA Constant Tone Extension
     * 0x01 AoD Constant Tone Extension with 1 us slots
     * 0x02 AoD Constant Tone Extension with 2 us slots
     * All other values Reserved for future use
     */
    uint8_t cte_type;

    /**
     * Slot durations
     * 0x01 Switching and sampling slots are 1 us each
     * 0x02 Switching and sampling slots are 2 us each
     * All other values Reserved for future use
     */
    uint8_t slot_dur;

    /**
     * Length of switching pattern
     * 0x02 - 0x4B The number of Antenna IDs in the pattern
     * All other values Reserved for future use
     */
    uint8_t switching_pattern_len;

    /// Antenna IDs
    uint8_t antenna_id[BLE_MAX_SW_PAT_LEN];

    /// Transmit power level in dBm (0x7E: minimum | 0x7F: maximum | range: -127 to +20)
    int8_t  tx_pwr_lvl;
};

/// External Objects
extern void (*bb_watch_timer_start)(struct sch_prog_params* params);
extern struct sch_alarm_tag  g_bb_watch_alarm;
extern struct lld_test_env_tag* lld_test_env;
extern void bb_watch_timer_start_patch(struct sch_prog_params* params);
extern void bb_watch_timer_cbk_patch(struct sch_alarm_tag* elt);
extern void bb_watch_timer_start_fcc(struct sch_prog_params* params);
extern void bb_watch_timer_cbk_fcc(struct sch_alarm_tag* elt);
extern rwip_time_t rwip_time_get(void);
extern uint8_t sch_alarm_clear(struct sch_alarm_tag* elt);
extern void sch_alarm_set(struct sch_alarm_tag* elt);
extern frm_cbk_t lld_test_frm_cbk;
extern struct sch_prog_env_tag sch_prog_env;
extern struct rwip_rf_api rwip_rf;
extern const struct rwip_prio rwip_priority[RWIP_PRIO_IDX_MAX];
extern uint8_t sch_arb_remove(struct sch_arb_elt_tag *elt, bool not_waiting);
extern uint8_t (*sch_arb_insert)(struct sch_arb_elt_tag *);
extern void lld_test_cleanup(void);
extern void ble_util_buf_acl_tx_free(uint16_t buf);
extern void *ke_msg_alloc(ke_msg_id_t const id, ke_task_id_t const dest_id, 
    ke_task_id_t const src_id, uint16_t const param_len);
extern void ke_msg_send(void const *param_ptr);
extern void lld_test_rx_isr(uint32_t timestamp);
extern sch_prog_push_func_t sch_prog_push;
extern void sch_prog_push_func(struct sch_prog_params* params);
extern void sch_prog_push_fcc(struct sch_prog_params* params);
extern uint8_t lld_test_stop(void);
extern uint8_t lld_test_start_func(struct lld_test_params* params);
#endif
