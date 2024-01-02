
#ifndef HAL_GFX_BLENDER_INTERN_H__
#define HAL_GFX_BLENDER_INTERN_H__

#include "hal_gfx_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PRELOAD_ADDR 31U

//------------------------------------------------------------------------------------------------
// Prefetch Policy
//------------------------------------------------------------------------------------------------
#define PREFETCH_TEXEL (0x00008000U)  //(1U<<15)
#define PRE_T0         (0x00000000U)  //(0U<<13)
#define PRE_T1         (0x00002000U)  //(1U<<13)
#define PRE_T2         (0x00004000U)  //(2U<<13)
#define PRE_T3         (0x00006000U)  //(3U<<13)
#define PRE_TXTY       (0x00000000U)  //(0U<<12)
#define PRE_XY         (0x00001000U)  //(1U<<12)
#define PRE_IMG1       (0x00000000U)  //(0U<<10)
#define PRE_IMG0       (0x00000400U)  //(1U<<10)
#define PRE_IMG2       (0x00000800U)  //(2U<<10)
#define PRE_IMG3       (0x00000C00U)  //(3U<<10)


static const uint32_t PRE_TEX[4] = {PRE_IMG0, PRE_IMG1, PRE_IMG2, PRE_IMG3};

//------------------------------------------------------------------------------------------------
// Predication
//------------------------------------------------------------------------------------------------
#define PRED_ALWAYS       (0U) // Alwayas execute
#define PRED_SET          (1U) // When flag set
#define PRED_NOTSET       (2U) // When flag not set
#define PRED_SEQ          (3U) // Can preexec next instruction
#define PRED_YIELD_ALWAYS (4U) // Yield always
#define PRED_YIELD_SET    (5U) // Yield always and exec if flag set
#define PRED_YIELD_NOSEQ  (6U) // Yield always and exec if no flag no set
#define PRED_BARRIER      (7U)  // Set Barrier

//------------------------------------------------------------------------------------------------

// Address Generator
//----------------------------------------------
#define P_NOP      (0U)

#define P_READTEXC (1U)
#define P_READTEX  (3U)
#define PA_LOAD    (5U)

#define P_PIXOUT   (2U)
#define PA_STORE   (5U)
#define PAMUL      (6U)

    //COND READ TEX
    //--------------------
#define P_A_FF     (1U << 4) //do not read if A=ff
#define P_A_00     (2U << 4) //do not read if A=00
#define P_A_0F     (3U << 4) //do not read if A=00 or A=FF
#define P_A_NBG    (4U << 4) //Do not read if not bg
#define P_A_BG     (5U << 4) //Do not read if     bg

    // Post Addr Increment
    //--------------------
#define P_Xp1      (1U << 4)
#define P_Yp1      (2U << 4)
#define P_Xm1      (3U << 4)
#define P_Ym1      (4U << 4)
#define P_Xm3Yp1   (5U << 4)
#define P_Xd2Yd2   (6U << 4)
#define P_Xd2      (7U << 4)

    // Post Pixout
    //--------------------
#define P_COND     (1U << 4)
    //PO_A_00    = 2 << 4  / /do not pixout if T0A=0
//----------------------------------------------
#define P_OUT     (0U)
#define P_T0      (0U)
#define P_T1      (1U)
#define P_T2      (2U)
#define P_T3      (3U)
//----------------------------------------------
#define P_XY      (0U)
#define P_TXTY    (1U)
//----------------------------------------------
#define P_IMG0    (0U)
#define P_IMG1    (1U)
#define P_IMG2    (2U)
#define P_IMG3    (3U)
//----------------------------------------------
#define I_NOP     (0U)

//RGB
//----------------------------------------------
#define R_NOP     (0U)
#define R_MADRGB  (1U)
#define R_MOVRGB  (2U)
#define R_ADDRGB  (3U)
#define R_MSBRGB  (5U)
#define MASK_RGB  (0U<<4)
#define MASK_RG   (1U<<4)
#define MASK_RB   (2U<<4)
#define MASK_R    (3U<<4)
#define MASK_GB   (4U<<4)
#define MASK_G    (5U<<4)
#define MASK_B    (6U<<4)
#define MASK_NOP  (7U<<4)
//----------------------------------------------
#define TX_NOP     ( 0U)
#define TX_ADD     (28U)
#define TX_AND     ( 9U)
#define TX_SWIZL   ( 8U)
#define TX_YUVCMD  (16U)
//----------------------------------------------
#define R0_OUT     (0U)
#define R0_T0RGB   (0U)
#define R0_T1RGB   (1U)
#define R0_T2RGB   (2U)
#define R0_T3RGB   (3U)
//----------------------------------------------
#define R1_T0RGB   ( 0U)
#define R1_T1RGB   ( 1U)
#define R1_T2RGB   ( 2U)
#define R1_T3RGB   ( 3U)
#define R1_T0AAA   ( 4U)
#define R1_T1AAA   ( 5U)
#define R1_T2AAA   ( 6U)
#define R1_T3AAA   ( 7U)
#define R1_iT0RGB  ( 8U)
#define R1_iT1RGB  ( 9U)
#define R1_iT2RGB  (10U)
#define R1_iT3RGB  (11U)
#define R1_iT0AAA  (12U)
#define R1_iT1AAA  (13U)
#define R1_iT2AAA  (14U)
#define R1_iT3AAA  (15U)
#define R1_C0RGB   (16U)
#define R1_C1RGB   (17U)
#define R1_C2RGB   (18U)
#define R1_C3RGB   (19U)
#define R1_C0AAA   (20U)
#define R1_C1AAA   (21U)
#define R1_C2AAA   (22U)
#define R1_C3AAA   (23U)
#define R1_T3RRR   (24U)
#define R1_T3GGG   (25U)
#define R1_T3BBB   (26U)
#define R1_T3GBR   (27U)
#define R1_T3BRG   (28U)
#define R1_111     (29U)
#define R1_000     (30U)
//----------------------------------------------
#define R2_T0RGB  (0U)
#define R2_T1RGB  (1U)
#define R2_T2RGB  (2U)
#define R2_T3RGB  (3U)
#define R2_C0RGB  (4U)
#define R2_C1RGB  (5U)
#define R2_C2RGB  (6U)
#define R2_C3RGB  (7U)
//----------------------------------------------
// ALPHA
//----------------------------------------------
#define A_NOP    (0U)
#define A_MOV    (1U)
#define A_MAD    (2U)
#define A_MUL    (3U)
//----------------------------------------------
#define A0_OUT   (0U)
#define A0_T0A   (0U)
#define A0_T1A   (1U)
#define A0_T2A   (2U)
#define A0_T3A   (3U)
//----------------------------------------------
#define A1_T0A   ( 0U)
#define A1_T1A   ( 1U)
#define A1_T2A   ( 2U)
#define A1_T3A   ( 3U)
#define A1_iT0A  ( 4U)
#define A1_iT1A  ( 5U)
#define A1_iT2A  ( 6U)
#define A1_iT3A  ( 7U)
#define A1_C0A   (11U)
#define A1_C1A   (12U)
#define A1_C2A   (13U)
#define A1_1     (14U)
#define A1_T0B   ( 8U)
#define A1_T0G   ( 9U)
#define A1_T0R   (10U)
#define A1_0     (15U)
//----------------------------------------------
#define A2_T0A   (0U)
#define A2_T1A   (1U)
#define A2_T2A   (2U)
#define A2_T3A   (3U)
#define A2_C0A   (4U)
#define A2_C1A   (5U)
#define A2_C2A   (6U)
#define A2_C3A   (7U)
//----------------------------------------------
// COMPARE
//----------------------------------------------
#define C_NOP      (0U)
#define C_CMP_EQ   (1U)
#define C_CMP_NEQ  (2U)
#define C_CMP_LESS (3U)
#define C_ALPHA    (1U<<5)
//----------------------------------------------
#define C_T0RGB    (0U)
#define C_T1RGB    (1U)
#define C_T2RGB    (2U)
#define C_T3RGB    (3U)
#define C_C0RGB    (4U)
#define C_C1RGB    (5U)
#define C_C2RGB    (6U)
#define C_C3RGB    (7U)
//----------------------------------------------

void hal_gfx_blender_init(void);

#ifdef __cplusplus
}
#endif

#endif
