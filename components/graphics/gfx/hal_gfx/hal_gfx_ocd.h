
#ifndef HAL_GFX_DEBUGMON_H__
#define HAL_GFX_DEBUGMON_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


uint32_t hal_gfx_ocd_read(void);
uint32_t hal_gfx_ocd_read_hi(void);
uint32_t hal_gfx_ocd_read_lo(void);
void hal_gfx_ocd_write(uint32_t addr, uint32_t value);

void hal_gfx_ocd_clear(void);
void hal_gfx_ocd_start(void);
void hal_gfx_ocd_stop(void);
uint32_t hal_gfx_ocd_read_counter(uint32_t counter);

#define BIT(b)  ((uint32_t)1U<<(b))

// Counters
//----------------------------------------------------------------------------------------------------------------------

#define HAL_GFX_OCD_TOP        (0U)
#define HAL_GFX_OCD_C0         (1U)
#define HAL_GFX_OCD_C0_IMEM    (2U)
#define HAL_GFX_OCD_C0_SCHED   (3U)
#define HAL_GFX_OCD_C1         (4U)
#define HAL_GFX_OCD_C1_IMEM    (5U)
#define HAL_GFX_OCD_C1_SCHED   (6U)
#define HAL_GFX_OCD_C2         (7U)
#define HAL_GFX_OCD_C2_IMEM    (8U)
#define HAL_GFX_OCD_C2_SCHED   (9U)
#define HAL_GFX_OCD_C3         (10U)
#define HAL_GFX_OCD_C3_IMEM    (11U)
#define HAL_GFX_OCD_C3_SCHED   (12U)
#define HAL_GFX_OCD_MS         (13U)

typedef enum {
    // Top
    //-----------------------------
                               // linting
    HAL_GFX_OCD_C_TOTAL         = /*HAL_GFX_OCD_TOP|*/BIT(6),
    HAL_GFX_OCD_C_BUSY          = /*HAL_GFX_OCD_TOP|*/BIT(7),
    HAL_GFX_OCD_C_BUSY_FBS      = /*HAL_GFX_OCD_TOP|*/BIT(8),
    HAL_GFX_OCD_C_BUSY_SPLIT    = /*HAL_GFX_OCD_TOP|*/BIT(9),
    HAL_GFX_OCD_C_BUSY_RAST     = /*HAL_GFX_OCD_TOP|*/BIT(10), // Activity Monitor Rasterizer
    HAL_GFX_OCD_C_BUSY_CONF     = /*HAL_GFX_OCD_TOP|*/BIT(11), // Activity Monitor ConfRegist
    HAL_GFX_OCD_C_BUSY_CMD      = /*HAL_GFX_OCD_TOP|*/BIT(12),
    HAL_GFX_OCD_E_COMMANDS_RAST = /*HAL_GFX_OCD_TOP|*/BIT(13), // Rasterizer commands
    HAL_GFX_OCD_E_PRW_CONF      = /*HAL_GFX_OCD_TOP|*/BIT(14), // ConfRegs PR Write
    HAL_GFX_OCD_E_PRR_CONF      = /*HAL_GFX_OCD_TOP|*/BIT(15),
    HAL_GFX_OCD_E_CLW_CONF      = /*HAL_GFX_OCD_TOP|*/BIT(16), // ConfRegs CL Write
    HAL_GFX_OCD_E_RFW_CONF      = /*HAL_GFX_OCD_TOP|*/BIT(17), // ConfRegs RF Write
    HAL_GFX_OCD_E_RFR_CONF      = /*HAL_GFX_OCD_TOP|*/BIT(18), // ConfRegs RF Read
    HAL_GFX_OCD_E_RINGTRIG_CMD  = /*HAL_GFX_OCD_TOP|*/BIT(19), // CmdList Processor Ring Trig
    HAL_GFX_OCD_E_WTRIG_CMD     = /*HAL_GFX_OCD_TOP|*/BIT(20), // CmdList Processor W Trig

    // Core 0
    //-----------------------------

    HAL_GFX_OCD_C0_C_BUSY          = HAL_GFX_OCD_C0 | BIT(6),  // Activity Monitor Core
    HAL_GFX_OCD_C0_C_BUSY_PIPE     = HAL_GFX_OCD_C0 | BIT(7),
    HAL_GFX_OCD_C0_C_BUSY_IMEM     = HAL_GFX_OCD_C0 | BIT(8),
    HAL_GFX_OCD_C0_C_BUSY_RF       = HAL_GFX_OCD_C0 | BIT(9),  // Activity Monitor Regs
    HAL_GFX_OCD_C0_C_BUSY_TEX      = HAL_GFX_OCD_C0 | BIT(10), // Activity Monitor TexMap
    HAL_GFX_OCD_C0_E_PIXELS        = HAL_GFX_OCD_C0 | BIT(11),
    HAL_GFX_OCD_C0_E_INSTRUCTIONS  = HAL_GFX_OCD_C0 | BIT(12), // Execution Pipeline Instructions
    HAL_GFX_OCD_C0_E_PREF_TEX      = HAL_GFX_OCD_C0 | BIT(13), // Texture Map Prefetch
    HAL_GFX_OCD_C0_E_READ_TEX      = HAL_GFX_OCD_C0 | BIT(14), // Texture Map Read
    HAL_GFX_OCD_C0_C_STALL_FB      = HAL_GFX_OCD_C0 | BIT(15),
    HAL_GFX_OCD_C0_C_STALL_IMEM    = HAL_GFX_OCD_C0 | BIT(16),
    HAL_GFX_OCD_C0_C_STALL_TEX     = HAL_GFX_OCD_C0 | BIT(17),
    HAL_GFX_OCD_C0_C_STALL_RF_RAST = HAL_GFX_OCD_C0 | BIT(18),
    HAL_GFX_OCD_C0_C_STALL_RF_TEX  = HAL_GFX_OCD_C0 | BIT(19),
    HAL_GFX_OCD_C0_E_READ_RF       = HAL_GFX_OCD_C0 | BIT(20), // Register File RegFile Read
    HAL_GFX_OCD_C0_E_WRITE_RF      = HAL_GFX_OCD_C0 | BIT(21), // Register File RegFile Write
    HAL_GFX_OCD_C0_E_READ_COORD_RF = HAL_GFX_OCD_C0 | BIT(22), // Register File Coord read
    HAL_GFX_OCD_C0_E_WRITE_COORD_RF= HAL_GFX_OCD_C0 | BIT(23), // Register File Coord write
    HAL_GFX_OCD_C0_C_READY         = HAL_GFX_OCD_C0 | BIT(24), // Core Ready


    HAL_GFX_OCD_C0_E_INSTR_R_IMEM       = HAL_GFX_OCD_C0_IMEM | BIT(6), // Instruction Memory Inst read
    HAL_GFX_OCD_C0_E_INSTR_W_IMEM       = HAL_GFX_OCD_C0_IMEM | BIT(7), // Instruction Memory Inst write


    HAL_GFX_OCD_C0_E_THREAD_WAIT_SCHED  = HAL_GFX_OCD_C0_SCHED | BIT(6),  // Scheduler Wait
    HAL_GFX_OCD_C0_E_THREAD_DONE_SCHED  = HAL_GFX_OCD_C0_SCHED | BIT(7),  // Scheduler Yield
    HAL_GFX_OCD_C0_E_INSTR_DONE_SCHED   = HAL_GFX_OCD_C0_SCHED | BIT(8),  // Scheduler Valid
    HAL_GFX_OCD_C0_E_TEX_DONE_SCHED     = HAL_GFX_OCD_C0_SCHED | BIT(9),  // Scheduler Done
    HAL_GFX_OCD_C0_E_FORK_SCHED         = HAL_GFX_OCD_C0_SCHED | BIT(10), // Scheduler Fork
    HAL_GFX_OCD_C0_C_WREADY_SCHED       = HAL_GFX_OCD_C0_SCHED | BIT(11), // Write Ready
    HAL_GFX_OCD_C0_C_WIDLE_SCHED        = HAL_GFX_OCD_C0_SCHED | BIT(12), // Write Idle
    HAL_GFX_OCD_C0_C_RREADY_SCHED       = HAL_GFX_OCD_C0_SCHED | BIT(13), // Read Ready
    HAL_GFX_OCD_C0_C_RIDLE_SCHED        = HAL_GFX_OCD_C0_SCHED | BIT(14), // Read Idle

    // Core 1
    //-----------------------------

    HAL_GFX_OCD_C1_C_BUSY          = HAL_GFX_OCD_C1 | BIT(6),
    HAL_GFX_OCD_C1_C_BUSY_PIPE     = HAL_GFX_OCD_C1 | BIT(7),
    HAL_GFX_OCD_C1_C_BUSY_IMEM     = HAL_GFX_OCD_C1 | BIT(8),
    HAL_GFX_OCD_C1_C_BUSY_RF       = HAL_GFX_OCD_C1 | BIT(9),
    HAL_GFX_OCD_C1_C_BUSY_TEX      = HAL_GFX_OCD_C1 | BIT(10),
    HAL_GFX_OCD_C1_E_PIXELS        = HAL_GFX_OCD_C1 | BIT(11),
    HAL_GFX_OCD_C1_E_INSTRUCTIONS  = HAL_GFX_OCD_C1 | BIT(12),
    HAL_GFX_OCD_C1_E_PREF_TEX      = HAL_GFX_OCD_C1 | BIT(13),
    HAL_GFX_OCD_C1_E_READ_TEX      = HAL_GFX_OCD_C1 | BIT(14),
    HAL_GFX_OCD_C1_C_STALL_FB      = HAL_GFX_OCD_C1 | BIT(15),
    HAL_GFX_OCD_C1_C_STALL_IMEM    = HAL_GFX_OCD_C1 | BIT(16),
    HAL_GFX_OCD_C1_C_STALL_TEX     = HAL_GFX_OCD_C1 | BIT(17),
    HAL_GFX_OCD_C1_C_STALL_RF_RAST = HAL_GFX_OCD_C1 | BIT(18),
    HAL_GFX_OCD_C1_C_STALL_RF_TEX  = HAL_GFX_OCD_C1 | BIT(19),
    HAL_GFX_OCD_C1_E_READ_RF       = HAL_GFX_OCD_C1 | BIT(20),
    HAL_GFX_OCD_C1_E_WRITE_RF      = HAL_GFX_OCD_C1 | BIT(21),
    HAL_GFX_OCD_C1_E_READ_COORD_RF = HAL_GFX_OCD_C1 | BIT(22),
    HAL_GFX_OCD_C1_E_WRITE_COORD_RF= HAL_GFX_OCD_C1 | BIT(23),
    HAL_GFX_OCD_C1_C_READY         = HAL_GFX_OCD_C1 | BIT(24),


    HAL_GFX_OCD_C1_E_INSTR_R_IMEM       = HAL_GFX_OCD_C1_IMEM | BIT(6),
    HAL_GFX_OCD_C1_E_INSTR_W_IMEM       = HAL_GFX_OCD_C1_IMEM | BIT(7),


    HAL_GFX_OCD_C1_E_THREAD_WAIT_SCHED  = HAL_GFX_OCD_C1_SCHED | BIT(6),
    HAL_GFX_OCD_C1_E_THREAD_DONE_SCHED  = HAL_GFX_OCD_C1_SCHED | BIT(7),
    HAL_GFX_OCD_C1_E_INSTR_DONE_SCHED   = HAL_GFX_OCD_C1_SCHED | BIT(8),
    HAL_GFX_OCD_C1_E_TEX_DONE_SCHED     = HAL_GFX_OCD_C1_SCHED | BIT(9),
    HAL_GFX_OCD_C1_E_FORK_SCHED         = HAL_GFX_OCD_C1_SCHED | BIT(10),
    HAL_GFX_OCD_C1_C_WREADY_SCHED       = HAL_GFX_OCD_C1_SCHED | BIT(11),
    HAL_GFX_OCD_C1_C_WIDLE_SCHED        = HAL_GFX_OCD_C1_SCHED | BIT(12),
    HAL_GFX_OCD_C1_C_RREADY_SCHED       = HAL_GFX_OCD_C1_SCHED | BIT(13),
    HAL_GFX_OCD_C1_C_RIDLE_SCHED        = HAL_GFX_OCD_C1_SCHED | BIT(14),

    // Core 2
    //-----------------------------

    HAL_GFX_OCD_C2_C_BUSY          = HAL_GFX_OCD_C2 | BIT(6),
    HAL_GFX_OCD_C2_C_BUSY_PIPE     = HAL_GFX_OCD_C2 | BIT(7),
    HAL_GFX_OCD_C2_C_BUSY_IMEM     = HAL_GFX_OCD_C2 | BIT(8),
    HAL_GFX_OCD_C2_C_BUSY_RF       = HAL_GFX_OCD_C2 | BIT(9),
    HAL_GFX_OCD_C2_C_BUSY_TEX      = HAL_GFX_OCD_C2 | BIT(10),
    HAL_GFX_OCD_C2_E_PIXELS        = HAL_GFX_OCD_C2 | BIT(11),
    HAL_GFX_OCD_C2_E_INSTRUCTIONS  = HAL_GFX_OCD_C2 | BIT(12),
    HAL_GFX_OCD_C2_E_PREF_TEX      = HAL_GFX_OCD_C2 | BIT(13),
    HAL_GFX_OCD_C2_E_READ_TEX      = HAL_GFX_OCD_C2 | BIT(14),
    HAL_GFX_OCD_C2_C_STALL_FB      = HAL_GFX_OCD_C2 | BIT(15),
    HAL_GFX_OCD_C2_C_STALL_IMEM    = HAL_GFX_OCD_C2 | BIT(16),
    HAL_GFX_OCD_C2_C_STALL_TEX     = HAL_GFX_OCD_C2 | BIT(17),
    HAL_GFX_OCD_C2_C_STALL_RF_RAST = HAL_GFX_OCD_C2 | BIT(18),
    HAL_GFX_OCD_C2_C_STALL_RF_TEX  = HAL_GFX_OCD_C2 | BIT(19),
    HAL_GFX_OCD_C2_E_READ_RF       = HAL_GFX_OCD_C2 | BIT(20),
    HAL_GFX_OCD_C2_E_WRITE_RF      = HAL_GFX_OCD_C2 | BIT(21),
    HAL_GFX_OCD_C2_E_READ_COORD_RF = HAL_GFX_OCD_C2 | BIT(22),
    HAL_GFX_OCD_C2_E_WRITE_COORD_RF= HAL_GFX_OCD_C2 | BIT(23),
    HAL_GFX_OCD_C2_C_READY         = HAL_GFX_OCD_C2 | BIT(24),


    HAL_GFX_OCD_C2_E_INSTR_R_IMEM       = HAL_GFX_OCD_C2_IMEM | BIT(6),
    HAL_GFX_OCD_C2_E_INSTR_W_IMEM       = HAL_GFX_OCD_C2_IMEM | BIT(7),


    HAL_GFX_OCD_C2_E_THREAD_WAIT_SCHED  = HAL_GFX_OCD_C2_SCHED | BIT(6),
    HAL_GFX_OCD_C2_E_THREAD_DONE_SCHED  = HAL_GFX_OCD_C2_SCHED | BIT(7),
    HAL_GFX_OCD_C2_E_INSTR_DONE_SCHED   = HAL_GFX_OCD_C2_SCHED | BIT(8),
    HAL_GFX_OCD_C2_E_TEX_DONE_SCHED     = HAL_GFX_OCD_C2_SCHED | BIT(9),
    HAL_GFX_OCD_C2_E_FORK_SCHED         = HAL_GFX_OCD_C2_SCHED | BIT(10),
    HAL_GFX_OCD_C2_C_WREADY_SCHED       = HAL_GFX_OCD_C2_SCHED | BIT(11),
    HAL_GFX_OCD_C2_C_WIDLE_SCHED        = HAL_GFX_OCD_C2_SCHED | BIT(12),
    HAL_GFX_OCD_C2_C_RREADY_SCHED       = HAL_GFX_OCD_C2_SCHED | BIT(13),
    HAL_GFX_OCD_C2_C_RIDLE_SCHED        = HAL_GFX_OCD_C2_SCHED | BIT(14),

    // Core 3
    //-----------------------------

    HAL_GFX_OCD_C3_C_BUSY          = HAL_GFX_OCD_C3 | BIT(6),
    HAL_GFX_OCD_C3_C_BUSY_PIPE     = HAL_GFX_OCD_C3 | BIT(7),
    HAL_GFX_OCD_C3_C_BUSY_IMEM     = HAL_GFX_OCD_C3 | BIT(8),
    HAL_GFX_OCD_C3_C_BUSY_RF       = HAL_GFX_OCD_C3 | BIT(9),
    HAL_GFX_OCD_C3_C_BUSY_TEX      = HAL_GFX_OCD_C3 | BIT(10),
    HAL_GFX_OCD_C3_E_PIXELS        = HAL_GFX_OCD_C3 | BIT(11),
    HAL_GFX_OCD_C3_E_INSTRUCTIONS  = HAL_GFX_OCD_C3 | BIT(12),
    HAL_GFX_OCD_C3_E_PREF_TEX      = HAL_GFX_OCD_C3 | BIT(13),
    HAL_GFX_OCD_C3_E_READ_TEX      = HAL_GFX_OCD_C3 | BIT(14),
    HAL_GFX_OCD_C3_C_STALL_FB      = HAL_GFX_OCD_C3 | BIT(15),
    HAL_GFX_OCD_C3_C_STALL_IMEM    = HAL_GFX_OCD_C3 | BIT(16),
    HAL_GFX_OCD_C3_C_STALL_TEX     = HAL_GFX_OCD_C3 | BIT(17),
    HAL_GFX_OCD_C3_C_STALL_RF_RAST = HAL_GFX_OCD_C3 | BIT(18),
    HAL_GFX_OCD_C3_C_STALL_RF_TEX  = HAL_GFX_OCD_C3 | BIT(19),
    HAL_GFX_OCD_C3_E_READ_RF       = HAL_GFX_OCD_C3 | BIT(20),
    HAL_GFX_OCD_C3_E_WRITE_RF      = HAL_GFX_OCD_C3 | BIT(21),
    HAL_GFX_OCD_C3_E_READ_COORD_RF = HAL_GFX_OCD_C3 | BIT(22),
    HAL_GFX_OCD_C3_E_WRITE_COORD_RF= HAL_GFX_OCD_C3 | BIT(23),
    HAL_GFX_OCD_C3_C_READY         = HAL_GFX_OCD_C3 | BIT(24),


    HAL_GFX_OCD_C3_E_INSTR_R_IMEM       = HAL_GFX_OCD_C3_IMEM | BIT(6),
    HAL_GFX_OCD_C3_E_INSTR_W_IMEM       = HAL_GFX_OCD_C3_IMEM | BIT(7),


    HAL_GFX_OCD_C3_E_THREAD_WAIT_SCHED  = HAL_GFX_OCD_C3_SCHED | BIT(6),
    HAL_GFX_OCD_C3_E_THREAD_DONE_SCHED  = HAL_GFX_OCD_C3_SCHED | BIT(7),
    HAL_GFX_OCD_C3_E_INSTR_DONE_SCHED   = HAL_GFX_OCD_C3_SCHED | BIT(8),
    HAL_GFX_OCD_C3_E_TEX_DONE_SCHED     = HAL_GFX_OCD_C3_SCHED | BIT(9),
    HAL_GFX_OCD_C3_E_FORK_SCHED         = HAL_GFX_OCD_C3_SCHED | BIT(10),
    HAL_GFX_OCD_C3_C_WREADY_SCHED       = HAL_GFX_OCD_C3_SCHED | BIT(11),
    HAL_GFX_OCD_C3_C_WIDLE_SCHED        = HAL_GFX_OCD_C3_SCHED | BIT(12),
    HAL_GFX_OCD_C3_C_RREADY_SCHED       = HAL_GFX_OCD_C3_SCHED | BIT(13),
    HAL_GFX_OCD_C3_C_RIDLE_SCHED        = HAL_GFX_OCD_C3_SCHED | BIT(14),

    // Memory System
    //-----------------------------

    HAL_GFX_OCD_MS_C_BUSY          = HAL_GFX_OCD_MS | BIT(6),  // Memory System Busy
    HAL_GFX_OCD_MS_C_BUSY_CMD      = HAL_GFX_OCD_MS | BIT(7),  // Memory System Busy cmd
    HAL_GFX_OCD_MS_C_BUSY_FW0      = HAL_GFX_OCD_MS | BIT(8),  // Memory System Busy fw
    HAL_GFX_OCD_MS_C_BUSY_TR01     = HAL_GFX_OCD_MS | BIT(9),  // Memory System Busy cr1
    HAL_GFX_OCD_MS_C_BUSY_TR00     = HAL_GFX_OCD_MS | BIT(10), // Memory System Busy cr0
    HAL_GFX_OCD_MS_C_BUSY_FW1      = HAL_GFX_OCD_MS | BIT(11),
    HAL_GFX_OCD_MS_C_BUSY_TR11     = HAL_GFX_OCD_MS | BIT(12),
    HAL_GFX_OCD_MS_C_BUSY_TR10     = HAL_GFX_OCD_MS | BIT(13),
    HAL_GFX_OCD_MS_C_BUSY_FW2      = HAL_GFX_OCD_MS | BIT(14),
    HAL_GFX_OCD_MS_C_BUSY_TR21     = HAL_GFX_OCD_MS | BIT(15),
    HAL_GFX_OCD_MS_C_BUSY_TR20     = HAL_GFX_OCD_MS | BIT(16),
    HAL_GFX_OCD_MS_C_BUSY_FW3      = HAL_GFX_OCD_MS | BIT(17),
    HAL_GFX_OCD_MS_C_BUSY_TR31     = HAL_GFX_OCD_MS | BIT(18),
    HAL_GFX_OCD_MS_C_BUSY_TR30     = HAL_GFX_OCD_MS | BIT(19),
    HAL_GFX_OCD_MS_E_AR_M0         = HAL_GFX_OCD_MS | BIT(20),
    HAL_GFX_OCD_MS_E_R_M0          = HAL_GFX_OCD_MS | BIT(21),
    HAL_GFX_OCD_MS_E_AR_M1         = HAL_GFX_OCD_MS | BIT(22),
    HAL_GFX_OCD_MS_E_R_M1          = HAL_GFX_OCD_MS | BIT(23),
    HAL_GFX_OCD_MS_E_AW_M0         = HAL_GFX_OCD_MS | BIT(24),
    HAL_GFX_OCD_MS_E_W_M0          = HAL_GFX_OCD_MS | BIT(25),
    HAL_GFX_OCD_MS_E_AW_M1         = HAL_GFX_OCD_MS | BIT(26),
    HAL_GFX_OCD_MS_E_W_M1          = HAL_GFX_OCD_MS | BIT(27),
    HAL_GFX_OCD_MS_E_AR_CL         = HAL_GFX_OCD_MS | BIT(28),
    HAL_GFX_OCD_MS_E_R_CL          = HAL_GFX_OCD_MS | BIT(29)

} hal_gfxocd_counters;

// Counters
//-----------------------------------------------------------------------------------------------------------------------

#define HAL_GFX_OCD_START     (0x01U)
#define HAL_GFX_OCD_STOP      (0x02U)
#define HAL_GFX_OCD_CLEAR     (0x04U)
#define HAL_GFX_OCD_MAP       (0x08U)
#define HAL_GFX_OCD_MAXVAL    (0x10U)
#define HAL_GFX_OCD_EXPTIME   (0x00U)
#define HAL_GFX_OCD_CLEARIRQS (0x03U)

#ifdef __cplusplus
}
#endif

#endif

