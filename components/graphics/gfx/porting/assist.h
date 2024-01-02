#ifndef __ASSIST_H__
#define __ASSIST_H__

//-------------------------------------------
#define PASS_VALUE 0x1750
#define FAIL_VALUE 0xDEAD
//-------------------------------------------
#define W32BIT 2
#define W16BIT 1
#define W8BIT  0

#define TB_CAMERA_START 1

typedef volatile unsigned int tsi_vint;
//-------------------------------------------------------------
typedef enum {
        T2D_DFLT_TEST   =  0x0,
        T2D_REF_TEST    =  0x1,
        T2D_IMPL_TEST   =  0x2,
} T2D_TESTTYPE;
//-------------------------------------------------------------

int ts_rand(void);
void ts_srand(unsigned int seed);
int ts_strlen(const char * text);
void ts_strcpy(char * s1, const char * s2);
char * ts_strcat(char *dest, const char *src);
int ts_putchar(int character);
void reverse(char s[]);
void itoa(int n, char s[]);
void itoa_0(int n, char s[]);
void itoa_2(int n, char s[]);
void aitoa(int n, char s[]);
void decimal_to_binary(int n,char s[]);
int get_return_value(void);
void print_t(char * text);
void aprint(const char * text);
void nprint(int number);
void printh(int number);
int get_assist_time(void);
void nprintd(int number);
void printd(int number);
void snapshot(void);
void snap_region(int baseaddr, int size, char * text);
void snapshot_name(char * filename, int index);
void snapshot_imp_path(char * filepath);
void snapshot_ref_path(char * filepath);
void snapshot_format(int format, void *base_virt, int image_size);
void fast_image_load(char * filename, int resx, int resy, int offset, int mode_size);
void fast_data_load(const char * filename, int size, int offset, int b32);
void fast_clr_fb(int baseaddr, int resx, int resy, int /*offset*/ color);
void tb_VGA2BMPindex(int index);
void tb_START(int index);
int cmp_image(char * filename, int ref_index, int imp_index);
// void set_fb(int baseaddr);
void set_fb(int baseaddr, int resx, int resy, int stride, int color_mode );
void theend();
void showtime();
void read_openfile(char * text);
int read_file(void);
void read_closefile(void);
void write_openfile(char * text);
void write_openfile_const(const char * text);
void write_file(int byte);
void write_closefile(void);
void delay(int ticks);
int tb_testno(void);
int tb_testmode(void);
void PMU_command(int cmd);
void PMU_sleep(int time);
int PMU_Status(void);
int cmp_insns(char * filename, void *base_addr, int num);
int tb_get_random_seed(void);
//-----------------------------------------------------------
//#ifndef __lm32__
#define tsi_print(...) printf(__VA_ARGS__)
//#else
//void tsi_print(const char * text,...);
//#endif
int check_crc(unsigned int val, unsigned int testno);
int test_check (int crc_ok, char* filename, unsigned int index, unsigned int tb_testno);
int test_report(int result,const char *text,...);
void tsi_fprint(const char * text,...);
void itoh(int n,char s[]);
void pow_reset_perf();
void pow_print_perf();
unsigned int tb_read_gpi(void);
unsigned int tb_read_gpo(void);
void tb_set_gpo(unsigned int gpo);
void tb_set_ctrl_sigmon(unsigned int sigmon_Cntr_Ctrl);
unsigned int tb_get_ctrl_sigmon();
unsigned int tb_get_sigmon();
void tb_set_clks(unsigned int clocks);
void tb_disable_clks(unsigned int clocks);
void tb_enable_clks(unsigned int clocks);
unsigned int tb_get_clks(unsigned int clocks);
void tb_set_clk_frequency(unsigned int frequency);
unsigned int tb_get_clk_frequency();
void tb_set_ahb_delay(unsigned int delay);
unsigned int tb_get_ahb_delay();
void tb_enable_mask(unsigned int address_mask, unsigned int mask_en);
void tb_disable_mask();
//--------------------------------------------------



//-----------------------------------------------------------
#endif // __ASSIST_H__
