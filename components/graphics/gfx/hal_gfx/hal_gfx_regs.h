
#ifndef HAL_GFX_REGS_H__
#define HAL_GFX_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_GFX_HOLDCMD      (0xff000000U)

#ifndef HAL_GFXP_HOLDCMD
#define HAL_GFXP_HOLDCMD     (0xff000000U)
#endif

#define HAL_GFXP_HOLDGPFLAG  ((uint32_t)1U << 27)

    // Texture Mapping and FB
    //-----------------------------
#define HAL_GFX_TEX0_BASE        (0x000U)
#define HAL_GFX_TEX0_BASE_H      (0x804U)
#define HAL_GFX_TEX0_FSTRIDE     (0x004U)
#define HAL_GFX_TEX0_RESXY       (0x008U)
    //-----------------------------
#define HAL_GFX_TEX1_BASE        (0x010U)
#define HAL_GFX_TEX1_BASE_H      (0x80cU)
#define HAL_GFX_TEX1_FSTRIDE     (0x014U)
#define HAL_GFX_TEX1_RESXY       (0x018U)
#define HAL_GFX_TEX_COLOR        (0x01cU)
    //-----------------------------
#define HAL_GFX_TEX2_BASE        (0x020U)
#define HAL_GFX_TEX2_BASE_H      (0x814U)
#define HAL_GFX_TEX2_FSTRIDE     (0x024U)
#define HAL_GFX_TEX2_RESXY       (0x028U)
    //-----------------------------
#define HAL_GFX_TEX3_BASE        (0x030U)
#define HAL_GFX_TEX3_BASE_H      (0x81cU)
#define HAL_GFX_TEX3_FSTRIDE     (0x034U)
#define HAL_GFX_TEX3_RESXY       (0x038U)

    // Rasterizer
    //-----------------------------
#define HAL_GFX_DRAW_CMD         (0x100U | HAL_GFXP_HOLDCMD)
#define HAL_GFX_DRAW_CMD_NOHOLD  (0x100U)
#define HAL_GFX_DRAW_STARTXY     (0x104U)
#define HAL_GFX_DRAW_ENDXY       (0x108U)
#define HAL_GFX_CLIPMIN          (0x110U)
#define HAL_GFX_CLIPMAX          (0x114U)
#define HAL_GFX_MATMULT          (0x118U)
#define HAL_GFX_CODEPTR          (0x11CU)
    //-----------------------------
#define HAL_GFX_DRAW_PT0_X       (0x120U)
#define HAL_GFX_DRAW_PT0_Y       (0x124U)
#define HAL_GFX_DRAW_COLOR       (0x12cU)
     //-----------------------------
#define HAL_GFX_DRAW_PT1_X       (0x130U)
#define HAL_GFX_DRAW_PT1_Y       (0x134U)
     //-----------------------------
#define HAL_GFX_DRAW_PT2_X       (0x140U)
#define HAL_GFX_DRAW_PT2_Y       (0x144U)
     //-----------------------------
#define HAL_GFX_DRAW_PT3_X       (0x150U)
#define HAL_GFX_DRAW_PT3_Y       (0x154U)
    //-----------------------------
//NemaP specific -->
    //-----------------------------
#define HAL_GFX_BYPASS_ADDR      (0x138U)
#define HAL_GFX_BYPASS_DATA      (0x13cU)
    //-----------------------------
#define HAL_GFX_MM00             (0x160U)
#define HAL_GFX_MM01             (0x164U)
#define HAL_GFX_MM02             (0x168U)
#define HAL_GFX_MM10             (0x16cU)
#define HAL_GFX_MM11             (0x170U)
#define HAL_GFX_MM12             (0x174U)
#define HAL_GFX_MM20             (0x178U)
#define HAL_GFX_MM21             (0x17cU)
#define HAL_GFX_MM22             (0x180U)
    //-----------------------------
#define HAL_GFX_DEPTH_START_L    (0x184U)
#define HAL_GFX_DEPTH_START_H    (0x188U)
#define HAL_GFX_DEPTH_DX_L       (0x18cU)
#define HAL_GFX_DEPTH_DX_H       (0x190U)
#define HAL_GFX_DEPTH_DY_L       (0x194U)
#define HAL_GFX_DEPTH_DY_H       (0x198U)
    //-----------------------------
#define HAL_GFX_RED_DX           (0x1a0U)
#define HAL_GFX_RED_DY           (0x1a4U)
#define HAL_GFX_GRE_DX           (0x1a8U)
#define HAL_GFX_GRE_DY           (0x1acU)
#define HAL_GFX_BLU_DX           (0x1b0U)
#define HAL_GFX_BLU_DY           (0x1b4U)
#define HAL_GFX_ALF_DX           (0x1b8U)
#define HAL_GFX_ALF_DY           (0x1bcU)
#define HAL_GFX_RED_INIT         (0x1c0U)
#define HAL_GFX_GRE_INIT         (0x1c4U)
#define HAL_GFX_BLU_INIT         (0x1c8U)
#define HAL_GFX_ALF_INIT         (0x1ccU)
//<--
//NemaT specific -->
    //-----------------------------
#define HAL_GFX_VERTEX0_X       (0x160U)
#define HAL_GFX_VERTEX0_Y       (0x164U)
#define HAL_GFX_VERTEX0_Z       (0x168U)
#define HAL_GFX_VERTEX0_W       (0x16cU)
    //-----------------------------
#define HAL_GFX_VERTEX1_X       (0x170U)
#define HAL_GFX_VERTEX1_Y       (0x174U)
#define HAL_GFX_VERTEX1_Z       (0x178U)
#define HAL_GFX_VERTEX1_W       (0x17cU)
    //-----------------------------
#define HAL_GFX_VERTEX2_X       (0x180U)
#define HAL_GFX_VERTEX2_Y       (0x184U)
#define HAL_GFX_VERTEX2_Z       (0x188U)
#define HAL_GFX_VERTEX2_W       (0x18cU)
    //-----------------------------
#define HAL_GFX_VERTEX3_X       (0x190U)
#define HAL_GFX_VERTEX3_Y       (0x194U)
#define HAL_GFX_VERTEX3_Z       (0x198U)
#define HAL_GFX_VERTEX3_W       (0x19cU)

    //-----------------------------
#define HAL_GFX_ZFUNC            (0x1e0U)
#define HAL_GFX_DEPTH_MIN        (0x1e4U)
#define HAL_GFX_DEPTH_MAX        (0x1e8U)
    //-----------------------------

    // Varyings
    //-----------------------------
#define HAL_GFX_VERTEX0_VAR0_F0  (0x320U)
#define HAL_GFX_VERTEX0_VAR0_F1  (0x324U)
#define HAL_GFX_VERTEX0_VAR0_F2  (0x328U)
#define HAL_GFX_VERTEX0_VAR0_F3  (0x32cU)
#define HAL_GFX_VERTEX0_VAR1_F0  (0x330U)
#define HAL_GFX_VERTEX0_VAR1_F1  (0x334U)
#define HAL_GFX_VERTEX0_VAR1_F2  (0x338U)
#define HAL_GFX_VERTEX0_VAR1_F3  (0x33cU)
#define HAL_GFX_VERTEX0_VAR2_F0  (0x340U)
#define HAL_GFX_VERTEX0_VAR2_F1  (0x344U)
#define HAL_GFX_VERTEX0_VAR2_F2  (0x348U)
#define HAL_GFX_VERTEX0_VAR2_F3  (0x34cU)
#define HAL_GFX_VERTEX0_VAR3_F0  (0x350U)
#define HAL_GFX_VERTEX0_VAR3_F1  (0x354U)
#define HAL_GFX_VERTEX0_VAR3_F2  (0x358U)
#define HAL_GFX_VERTEX0_VAR3_F3  (0x35cU)
#define HAL_GFX_VERTEX0_VAR4_F0  (0x360U)
#define HAL_GFX_VERTEX0_VAR4_F1  (0x364U)
#define HAL_GFX_VERTEX0_VAR4_F2  (0x368U)
#define HAL_GFX_VERTEX0_VAR4_F3  (0x36cU)
#define HAL_GFX_VERTEX0_VAR5_F0  (0x370U)
#define HAL_GFX_VERTEX0_VAR5_F1  (0x374U)
#define HAL_GFX_VERTEX0_VAR5_F2  (0x378U)
#define HAL_GFX_VERTEX0_VAR5_F3  (0x37cU)
#define HAL_GFX_VERTEX0_VAR6_F0  (0x380U)
#define HAL_GFX_VERTEX0_VAR6_F1  (0x384U)
#define HAL_GFX_VERTEX0_VAR6_F2  (0x388U)
#define HAL_GFX_VERTEX0_VAR6_F3  (0x38cU)
#define HAL_GFX_VERTEX0_VAR7_F0  (0x390U)
#define HAL_GFX_VERTEX0_VAR7_F1  (0x394U)
#define HAL_GFX_VERTEX0_VAR7_F2  (0x398U)
#define HAL_GFX_VERTEX0_VAR7_F3  (0x39cU)
    //-----------------------------
#define HAL_GFX_VERTEX1_VAR0_F0  (0x3a0U)
#define HAL_GFX_VERTEX1_VAR0_F1  (0x3a4U)
#define HAL_GFX_VERTEX1_VAR0_F2  (0x3a8U)
#define HAL_GFX_VERTEX1_VAR0_F3  (0x3acU)
#define HAL_GFX_VERTEX1_VAR1_F0  (0x3b0U)
#define HAL_GFX_VERTEX1_VAR1_F1  (0x3b4U)
#define HAL_GFX_VERTEX1_VAR1_F2  (0x3b8U)
#define HAL_GFX_VERTEX1_VAR1_F3  (0x3bcU)
#define HAL_GFX_VERTEX1_VAR2_F0  (0x3c0U)
#define HAL_GFX_VERTEX1_VAR2_F1  (0x3c4U)
#define HAL_GFX_VERTEX1_VAR2_F2  (0x3c8U)
#define HAL_GFX_VERTEX1_VAR2_F3  (0x3ccU)
#define HAL_GFX_VERTEX1_VAR3_F0  (0x3d0U)
#define HAL_GFX_VERTEX1_VAR3_F1  (0x3d4U)
#define HAL_GFX_VERTEX1_VAR3_F2  (0x3d8U)
#define HAL_GFX_VERTEX1_VAR3_F3  (0x3dcU)
#define HAL_GFX_VERTEX1_VAR4_F0  (0x3e0U)
#define HAL_GFX_VERTEX1_VAR4_F1  (0x3e4U)
#define HAL_GFX_VERTEX1_VAR4_F2  (0x3e8U)
#define HAL_GFX_VERTEX1_VAR4_F3  (0x3ecU)
#define HAL_GFX_VERTEX1_VAR5_F0  (0x3f0U)
#define HAL_GFX_VERTEX1_VAR5_F1  (0x3f4U)
#define HAL_GFX_VERTEX1_VAR5_F2  (0x3f8U)
#define HAL_GFX_VERTEX1_VAR5_F3  (0x3fcU)
#define HAL_GFX_VERTEX1_VAR6_F0  (0x400U)
#define HAL_GFX_VERTEX1_VAR6_F1  (0x404U)
#define HAL_GFX_VERTEX1_VAR6_F2  (0x408U)
#define HAL_GFX_VERTEX1_VAR6_F3  (0x40cU)
#define HAL_GFX_VERTEX1_VAR7_F0  (0x410U)
#define HAL_GFX_VERTEX1_VAR7_F1  (0x414U)
#define HAL_GFX_VERTEX1_VAR7_F2  (0x418U)
#define HAL_GFX_VERTEX1_VAR7_F3  (0x41cU)
    //-----------------------------
#define HAL_GFX_VERTEX2_VAR0_F0  (0x420U)
#define HAL_GFX_VERTEX2_VAR0_F1  (0x424U)
#define HAL_GFX_VERTEX2_VAR0_F2  (0x428U)
#define HAL_GFX_VERTEX2_VAR0_F3  (0x42cU)
#define HAL_GFX_VERTEX2_VAR1_F0  (0x430U)
#define HAL_GFX_VERTEX2_VAR1_F1  (0x434U)
#define HAL_GFX_VERTEX2_VAR1_F2  (0x438U)
#define HAL_GFX_VERTEX2_VAR1_F3  (0x43cU)
#define HAL_GFX_VERTEX2_VAR2_F0  (0x440U)
#define HAL_GFX_VERTEX2_VAR2_F1  (0x444U)
#define HAL_GFX_VERTEX2_VAR2_F2  (0x448U)
#define HAL_GFX_VERTEX2_VAR2_F3  (0x44cU)
#define HAL_GFX_VERTEX2_VAR3_F0  (0x450U)
#define HAL_GFX_VERTEX2_VAR3_F1  (0x454U)
#define HAL_GFX_VERTEX2_VAR3_F2  (0x458U)
#define HAL_GFX_VERTEX2_VAR3_F3  (0x45cU)
#define HAL_GFX_VERTEX2_VAR4_F0  (0x460U)
#define HAL_GFX_VERTEX2_VAR4_F1  (0x464U)
#define HAL_GFX_VERTEX2_VAR4_F2  (0x468U)
#define HAL_GFX_VERTEX2_VAR4_F3  (0x46cU)
#define HAL_GFX_VERTEX2_VAR5_F0  (0x470U)
#define HAL_GFX_VERTEX2_VAR5_F1  (0x474U)
#define HAL_GFX_VERTEX2_VAR5_F2  (0x478U)
#define HAL_GFX_VERTEX2_VAR5_F3  (0x47cU)
#define HAL_GFX_VERTEX2_VAR6_F0  (0x480U)
#define HAL_GFX_VERTEX2_VAR6_F1  (0x484U)
#define HAL_GFX_VERTEX2_VAR6_F2  (0x488U)
#define HAL_GFX_VERTEX2_VAR6_F3  (0x48cU)
#define HAL_GFX_VERTEX2_VAR7_F0  (0x490U)
#define HAL_GFX_VERTEX2_VAR7_F1  (0x494U)
#define HAL_GFX_VERTEX2_VAR7_F2  (0x498U)
#define HAL_GFX_VERTEX2_VAR7_F3  (0x49cU)
    //-----------------------------
#define HAL_GFX_VARYING0_INFO    (0x4a0U)
#define HAL_GFX_VARYING1_INFO    (0x4a4U)
#define HAL_GFX_VARYING2_INFO    (0x4a8U)
#define HAL_GFX_VARYING3_INFO    (0x4acU)
#define HAL_GFX_VARYING4_INFO    (0x4b0U)
#define HAL_GFX_VARYING5_INFO    (0x4b4U)
#define HAL_GFX_VARYING6_INFO    (0x4b8U)
#define HAL_GFX_VARYING7_INFO    (0x4bcU)

    // Vertex
    //-----------------------------
#define HAL_GFX_VERTEX_VA_PTR    (0x300U)
#define HAL_GFX_VERTEX_VS_PTR    (0x304U)
#define HAL_GFX_VERTEX_REGADDR   (0x308U)
#define HAL_GFX_VERTEX_REGDATA   (0x30cU)
#define HAL_GFX_VERTEX_CONADDR   (0x310U)
#define HAL_GFX_VERTEX_CONDATA   (0x314U)
#define HAL_GFX_VERTEX_TRIGGER   (0x318U | HAL_GFX_HOLDCMD)
    //-----------------------------
#define HAL_GFX_VERTEX_MVP00     (0x3c0U)
#define HAL_GFX_VERTEX_MVP01     (0x3c4U)
#define HAL_GFX_VERTEX_MVP02     (0x3c8U)
#define HAL_GFX_VERTEX_MVP03     (0x3ccU)
#define HAL_GFX_VERTEX_MVP10     (0x3d0U)
#define HAL_GFX_VERTEX_MVP11     (0x3d4U)
#define HAL_GFX_VERTEX_MVP12     (0x3d8U)
#define HAL_GFX_VERTEX_MVP13     (0x3dcU)
#define HAL_GFX_VERTEX_MVP20     (0x3e0U)
#define HAL_GFX_VERTEX_MVP21     (0x3e4U)
#define HAL_GFX_VERTEX_MVP22     (0x3e8U)
#define HAL_GFX_VERTEX_MVP23     (0x3ecU)
#define HAL_GFX_VERTEX_MVP30     (0x3f0U)
#define HAL_GFX_VERTEX_MVP31     (0x3f4U)
#define HAL_GFX_VERTEX_MVP32     (0x3f8U)
#define HAL_GFX_VERTEX_MVP33     (0x3fcU)
    //-----------------------------

    // ViewPort
    //-----------------------------
#define HAL_GFX_VIEWPORT_X     (0x4c0U)
#define HAL_GFX_VIEWPORT_Y     (0x4c4U)
#define HAL_GFX_VIEWPORT_W     (0x4c8U)
#define HAL_GFX_VIEWPORT_H     (0x4ccU)
    //-----------------------------
//<--

    // Processor
    //-----------------------------
#define HAL_GFX_ROPBLENDER_BLEND_MODE  (0x1d0U)
#define HAL_GFX_ROPBLENDER_DST_CKEY    (0x1d4U)
#define HAL_GFX_ROPBLENDER_CONST_COLOR (0x1d8U)
    //-----------------------------

    // Processor
    //-----------------------------
#define HAL_GFX_IMEM_ADDR        (0x0c4U)
#define HAL_GFX_IMEM_DATAH       (0x0c8U)
#define HAL_GFX_IMEM_DATAL       (0x0ccU)
#define HAL_GFX_FORKPTR          (0x0d0U)
#define HAL_GFX_FORKDATA         (0x0d8U | HAL_GFX_HOLDCMD)
#define HAL_GFX_FORKDATAH        (0x0dcU)
    //-----------------------------
#define HAL_GFX_C0_REG           (0x200U)
#define HAL_GFX_C1_REG           (0x204U)
#define HAL_GFX_C2_REG           (0x208U)
#define HAL_GFX_C3_REG           (0x20cU)
#define HAL_GFX_CMAX_REG         (0x20cU)
#define HAL_GFX_FRAG_CONADDR     (0x210U)
#define HAL_GFX_FRAG_CONDATA     (0x214U)

    // Status & Control
    //-----------------------------
#define HAL_GFX_CLID             (0x148U)
#define HAL_GFX_LOADCTRL         (0x1f0U)
#define HAL_GFX_CONFIG           (0x1f0U)
#define HAL_GFX_CONFIGH          (0x1f4U)
#define HAL_GFX_IDREG            (0x1ecU)
#define HAL_GFX_CMDSTATUS        (0x0e8U)
#define HAL_GFX_CMDRINGSTOP      (0x0ecU)
#define HAL_GFX_CMDRINGSTOP_H    (0x844U)
#define HAL_GFX_CMDADDR          (0x0f0U)
#define HAL_GFX_CMDADDR_H        (0x84cU)
#define HAL_GFX_CMDSIZE          (0x0f4U)
#define HAL_GFX_INTERRUPT        (0x0f8U)
#define HAL_GFX_IRQ_ID           (0xff0U)

// bits [11:8]: GP_FLAGS_IRQ_MASK
// bits [ 7:4]: GP_FLAGS_MASK
// bits [ 3:0]: GP_FLAGS
#define HAL_GFX_GP_FLAGS         (0xff4U)

#define HAL_GFX_SYS_INTERRUPT    (0xff8U)
#define HAL_GFX_STATUS           (0x0fcU)

#define HAL_GFX_BREAKPOINT       (0x080U)
#define HAL_GFX_BREAKPOINT_MASK  (0x08CU)
    //-----------------------------

    // Debug & Monitor
    //-----------------------------
#define HAL_GFX_DBG_STATUS       (0x2f0U)
#define HAL_GFX_DBG_ADDR         (0x2f4U)
#define HAL_GFX_DBG_DATA         (0x2f8U)
#define HAL_GFX_DBG_CTRL         (0x2fcU)
    //-----------------------------

    // Clockgating control
    //----------------------------
#define HAL_GFX_CGCMD            (0x090U)
#define HAL_GFX_CGCTRL           (0x094U)

    // Dirty Region
    //----------------------------
#define HAL_GFX_DIRTYMIN         (0x098U)
#define HAL_GFX_DIRTYMAX         (0x09cU)

#ifdef __cplusplus
}
#endif

#endif
