// -----------------------------------------------------------------------------
// Copyright (c) 2008 Think Silicon Ltd
// Think Silicon Ltd Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon Ltd
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less then a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon Ltd.
//
//  the software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement.  in no event shall the
//  Think Silicon Ltd be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//                    Think Silicon Ltd
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------
// FILE NAME  : Sw/Src/assist.c
// KEYWORDS   :
// PROJECT    : OpenVG
// PURPOSE    : Assist functions for debugging (open/close files, dump Graphics Memory, etc)
//              Works with Assist.v
// DEPARTMENT : online IP Products
// AUTHOR     : IS
// GENERATION : 9/30/2008  12:53
// RECEIVER   : TR
// -----------------------------------------------------------------------------
// RELEASE HISTORY for module architecture
// VERSION    DATE                      AUTHOR            DESCRIPTION
//  1.0       9/30/2008  12:53            IS            Initial Release
//  1.1      11/11/2010                   IS            Major recoding to improve portability
// -----------------------------------------------------------------------------

#ifndef _TSI_ASSIST_C_
#define _TSI_ASSIST_C_

#include <stdio.h>
#include "assist.h"
#include <stdarg.h>
//-------------------------------------------
#define BASE_Assist           0x40000000
//-------------------------------------------
//#define DEBUG_TEXT
//-------------------------------------------
T2D_TESTTYPE ref_imp = T2D_IMPL_TEST;
//-------------------------------------------------------------
typedef volatile struct __TEXTPRINT
{
    tsi_vint  character;          // 00
    tsi_vint  rname;              // 04
    tsi_vint  rdata;              // 08
    tsi_vint  wname;              // 0C
    tsi_vint  wdata;              // 10
    tsi_vint  extra1;             // 14
    tsi_vint  extra2;             // 18
    tsi_vint  extra3;             // 1C
    tsi_vint  rimage_file;        // 20
    tsi_vint  rimage_size;        // 24
    tsi_vint  rimage_offset;      // 28
    tsi_vint  kernel_file;        // 2c
    tsi_vint  kernel_size;        // 30
    tsi_vint  extra4;             // 34
    tsi_vint  extra5;             // 38
    tsi_vint  extra6;             // 3c
    tsi_vint  extra7;             // 40
    tsi_vint  tb_start;           // 44
    tsi_vint  tb_start1;          // 48 // not used
    tsi_vint  tb_start2;          // 4c // not used
    tsi_vint  PMU_CMD;            // 50
    tsi_vint  PMU_STATUS;         // 54
    tsi_vint  PMU_sleep;          // 58
    tsi_vint  fb_baseaddr;        // 5c
    tsi_vint  fb_sizeYX;          // 60
    tsi_vint  fb_color;           // 64
    tsi_vint  fb_stride;          // 68
    tsi_vint  test_report;        // 6c
    tsi_vint  test_rpt_32;        // 70
    tsi_vint  gp_input;           // 74
    tsi_vint  gp_output;          // 78
    tsi_vint  sigmon_Cntr_Ctrl;   // 7c
    tsi_vint  sigmon_Cntr;        // 80
    tsi_vint  clk_gate_ctrl;      // 84
    tsi_vint  address_mask;       // 88
    tsi_vint  address_mask_en;    // 8c
    tsi_vint  clk_ctrl;           // 90
    tsi_vint  ahb_read_delay;     // 94
    tsi_vint  random_seed;        // 98

} TEXTPRINT;

#define PTR_TEXTPRINT       ((TEXTPRINT *) BASE_Assist)
//-------------------------------------------------------------

#ifndef FPGA
typedef unsigned long uintptr_t;
void *memcpy(void *dst, const void *src, size_t len)
 {
         size_t i;

         /*
          * memcpy does not support overlapping buffers, so always do it
          * forwards. (Don't change this without adjusting memmove.)
          *
          * For speedy copying, optimize the common case where both pointers
          * and the length are word-aligned, and copy word-at-a-time instead
          * of byte-at-a-time. Otherwise, copy by bytes.
          *
          * The alignment logic below should be portable. We rely on
          * the compiler to be reasonably intelligent about optimizing
          * the divides and modulos out. Fortunately, it is.
          */

         if ((uintptr_t)dst % sizeof(long) == 0 &&
             (uintptr_t)src % sizeof(long) == 0 &&
             len % sizeof(long) == 0) {

                 long *d = dst;
                 const long *s = src;

                 for (i=0; i<len/sizeof(long); i++) {
                         d[i] = s[i];
                 }
         }
         else {
                 char *d = dst;
                 const char *s = src;

                 for (i=0; i<len; i++) {
                         d[i] = s[i];
                 }
         }

         return dst;
 }
#endif

// Replacement for common std C function in case they are not available
// Random Number generation
//-------------------------------------------------------------
static unsigned long int next = 1;
//-------------------------------------------------------------
int ts_rand(void) // RAND_MAX assumed to be 32767
{
    next = next * 1103515245 + 12345;
    return (unsigned int)(next&0x7fffffff);
//    return (unsigned int)(next/65536) % 32768;
}
//-------------------------------------------------------------
void ts_srand(unsigned int seed)
{
    next = seed;
}
//-------------------------------------------------------------
int ts_strlen(const char * text)
{
    int i=0;
    while (text[i++]!=0);
    return i;
}
//-------------------------------------------------------------
void ts_strcpy(char * s1, const char * s2)
{
    if (!s1 || !s2)
        return;

    while (*s2) {
        *s1++ = *s2++;
    }

    *s1 = 0;
}
//-------------------------------------------------------------
char * ts_strcat(char *dest, const char *src)
{
    size_t i,j;
    for (i = 0; dest[i] != '\0'; i++)
        ;
    for (j = 0; src[j] != '\0'; j++)
        dest[i+j] = src[j];
    dest[i+j] = '\0';
    return dest;
}
//-------------------------------------------------------------
int ts_putchar(int character)
{
    return(PTR_TEXTPRINT->character= character);
}

// ASCII String Manipulation
/* reverse:  reverse string s in place */
//-------------------------------------------------------------
void reverse(char s[])
{
    int i, j;

    for (i = 0, j = ts_strlen(s)-1; i<j; i++, j--) {
        char c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}


void
itoh(int n,char s[])
{
    const char hex_lookup[] = "0123456789abcdef";
    int len = 0 ;
    int tmp_n = n;
    for (; tmp_n; tmp_n >>= 4) {
        ++len;
    }
    s[len] = '\0';
    len--;
    for (; len >= 0; n >>= 4, --len) {
        //tsi_print("%d, %c\n",(n&0xf),hex_lookup[n & 0xf]);
        s[len] = hex_lookup[n & 0xf];
    }
}

void itoa(int n, char s[])
{
         if(n<10)  { s[0]='0'+n; s[1]='\0';}
    else if(n<100) { s[0]='0'+(n/10);  s[1]='0'+(n%10);       s[2]='\0';}
    else           { s[0]='0'+(n/100); s[1]='0'+((n%100)/10); s[2]='0'+(n%10); s[3]='\0'; }

}
void itoa_0(int n, char s[])
{
    s[0]=s[1]=s[2]=s[3]='0';
         if(n<10)   { s[3]='0'+n; }
    else if(n<100)  { s[2]='0'+(n/10);   s[3]='0'+(n%10);       }
    else if(n<1000) { s[1]='0'+(n/100);  s[2]='0'+((n%100)/10); s[3]='0'+(n%10);  }
	else            { s[0]='0'+(n/1000); s[1]='0'+(n/100);      s[2]='0'+((n%100)/10); s[3]='0'+(n%10); }

}
//-------------------------------------------------------------
/* itoa_2:  convert n to characters in s */
void itoa_2(int n, char s[])
{
    int i=10;
    int isNeg=0;
    int temp;
    int base=10;
    int d=1;
    int k=1;
    if(n<0){ isNeg=1; temp=-n; }
    else {temp=n;}
    while (i<temp) { i=i*base; k=k+1; }
    if (isNeg) {s[k+1]='\0'; s[k]='-';}
    else {s[k]='\0';}
    s[0]='0'+ temp*base/i;
    for (i=k-1;i>0; i=i-1){
        s[i]='0'+ ((temp%(d*base))/d);
        d = d*base;
    }
}
//-------------------------------------------------------------
/* itoa:  convert n to characters in s */
void aitoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* record sign */
        n = -n;          /* make n positive */
    i = 0;
    do {       /* generate digits in reverse order */
        s[i++] = n % 10 + '0';   /* get next digit */
    } while ((n /= 10) > 0);     /* delete it */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';

////    reverse(s);
}
//-------------------------------------------------------------
void decimal_to_binary(int n,char s[]){
    int temp_n, i;
    int divider[8]={128,64,32,16,8,4,2,1};
    temp_n=n;
    for (i=0; i<8; i++) {
        int calc = temp_n/divider[i];
        if (calc) {s[7-i]='1' - 48; temp_n=temp_n-divider[i];}
        else {s[7-i]='0' - 48;}
    }
    s[8]='\0';
}

// Gets a comparison return value
//-----------------------------------------------------------------------------------------------------------------------
int get_return_value(void)
//-----------------------------------------------------------------------------------------------------------------------
{
    return (PTR_TEXTPRINT->extra4==PASS_VALUE);
}
//-----------------------------------------------------------------------------------------------------------------------

// Prints formatted text (supported formats: %c, %d, %x, %s || )
//-----------------------------------------------------------------------------------------------------------------------
//void
//tsi_print(const char *text,...)
//{
//#ifdef DEBUG_TEXT
//    const char *str;
//    unsigned int i,j;
//    va_list args;
//    va_start(args,text);
//    i=0;
//    while (text[i]!='\0'){
//        if( text[i]=='%'){
//            i++;
//            switch(text[i]){
//                case 'c': PTR_TEXTPRINT->character=va_arg(args,unsigned int); break;
//                case 'u':
//                case 'd': PTR_TEXTPRINT->extra3=va_arg(args,unsigned int); break;
//                case 'x': PTR_TEXTPRINT->extra1=va_arg(args,unsigned int); break;
//                case 's': str=va_arg(args,const char *); j=0;
//                            while(str[j]!='\0'/*NULL*/){
//                                PTR_TEXTPRINT->character=str[j];
//                                j++;
//                            }
//                            break;
//                case 'l': if ( text[++i]=='u') {
//                              PTR_TEXTPRINT->extra3=va_arg(args,unsigned int);
//                          } else {
//                  PTR_TEXTPRINT->character=text[i];
//                          }
//                          break;
//                default:  PTR_TEXTPRINT->character=text[i]; break;
//            }
//        }else if ( text[i]=='\\'){
//            i++;
//            switch(text[++i]){
//                case 'n': PTR_TEXTPRINT->character=-0x0A; break;
//                case 'r': PTR_TEXTPRINT->character=-0x0D; break;
//                case 't': PTR_TEXTPRINT->character=-0x09; break;
//                default:  PTR_TEXTPRINT->character=text[i]; break;
//            }
//        }else{
//            PTR_TEXTPRINT->character=text[i];
//        }
//        i++;
//    }
//    va_end(args);
//#endif
//}


int test_check (int crc_ok, char* filename, unsigned int index, unsigned int tb_testno){

    int img_ok = cmp_image(filename, index, tb_testno);

    test_report((img_ok && crc_ok), "Test %d",tb_testno );

    return (crc_ok && img_ok);

}
// Reports test failure
//-----------------------------------------------------------------------------------------------------------------------
int
test_report(int result,const char *text,...)
{
#ifdef DEBUG_TEXT

    PTR_TEXTPRINT->test_report=0xffff;
    const char *str;
    unsigned int i,j;
    va_list args;
    va_start(args,text);
    i=0;
    while (text[i]!='\0'){
        if( text[i]=='%'){
            i++;
            switch(text[i]){
                case 'c': PTR_TEXTPRINT->test_report=va_arg(args,unsigned int)|((char)'c'<<24); break;
                case 'u':
                case 'd': PTR_TEXTPRINT->test_report=(va_arg(args,unsigned int)|((char)'d'<<24)); break;
                case 'x': PTR_TEXTPRINT->test_rpt_32=va_arg(args,unsigned int); break;
                case 's': str=va_arg(args,const char *); j=0;
                            while(str[j]!='\0'/*NULL*/){
                                PTR_TEXTPRINT->test_report=str[j]|('c'<<24);
                                j++;
                            }
                            break;
                case 'l': if ( text[++i]=='u') {
                              PTR_TEXTPRINT->test_report=va_arg(args,unsigned int)|((char)'d'<<23);
                          } else {
                              PTR_TEXTPRINT->test_report=text[i]|((char)'c'<<24);
                          }
                          break;
                default:  PTR_TEXTPRINT->test_report=text[i]|((char)'c'<<24); break;
            }
        }else if ( text[i]=='\\'){
            i++;
            switch(text[++i]){
                case 'n': PTR_TEXTPRINT->test_report=-0x0A|((char)'a'<<24); break;
                case 'r': PTR_TEXTPRINT->test_report=-0x0D|((char)'a'<<24); break;
                case 't': PTR_TEXTPRINT->test_report=-0x09|((char)'a'<<24); break;
                default:  PTR_TEXTPRINT->test_report=text[i]|((char)'a'<<24); break;
            }
        }else{
            PTR_TEXTPRINT->test_report=text[i]|((char)'c'<<24);
        }
        i++;
    }
    va_end(args);
    switch(result)
    {
        case 0: PTR_TEXTPRINT->character=0x0000AE27; return -1; break;
        case 1: PTR_TEXTPRINT->character=0x0000AE28; return  0; break;
        case 2: PTR_TEXTPRINT->character=0x0000AE29; return  0; break;
       default: return -1; break;
    }
#endif
}

// Prints text on the console
//-----------------------------------------------------------------------------------------------------------------------
void print_t(char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    int i,j;
    i=ts_strlen(text);
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->character= text[j];
        }
    //  New Line
    PTR_TEXTPRINT->character= 0x0A;
#endif
}

// Same as print_t without new line character
//-----------------------------------------------------------------------------------------------------------------------
void aprint(const char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    int i,j;
    i=ts_strlen(text);
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->character= text[j];
        }
#endif
}

// To be removed
//-----------------------------------------------------------------------------------------------------------------------
void nprint(int number)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    PTR_TEXTPRINT->extra1= number;
#endif
}

// Print a Hex number on screen
//-----------------------------------------------------------------------------------------------------------------------
void printh(int number)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    PTR_TEXTPRINT->extra1= number;
#endif
}

// Print a decimal number on screen
//-----------------------------------------------------------------------------------------------------------------------
int get_assist_time(void)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    return (PTR_TEXTPRINT->extra1);
#endif
}

// Print a decimal number on screen
//-----------------------------------------------------------------------------------------------------------------------
void nprintd(int number)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    PTR_TEXTPRINT->extra3= number;
#endif
}

// Print a decimal number on screen
//-----------------------------------------------------------------------------------------------------------------------
void printd(int number)
//-----------------------------------------------------------------------------------------------------------------------
{
#ifdef DEBUG_TEXT
    PTR_TEXTPRINT->extra3= number;
#endif
}


// Get a snapshot of Graphics Memory - filename is generated automatically (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void snapshot(void)
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->character=0x0000AE22;
}
//-----------------------------------------------------------------------------------------------------------------------


// Get a snapshot of Graphics Memory - filename defined as argument (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void snapshot_name(char * filename, int index)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;

    char snps_txt[400]; char snps_indx[4];
    extern T2D_TESTTYPE ref_imp;

    ts_strcpy (snps_txt,"");
    ts_strcat (snps_txt,filename);
    ts_strcat (snps_txt, ".");
    itoa      (index, snps_indx);
    ts_strcat (snps_txt, snps_indx);

    if      (ref_imp==T2D_REF_TEST)  ts_strcat(snps_txt, ".ref");
    else if (ref_imp==T2D_IMPL_TEST) ts_strcat(snps_txt, ".imp");

    i=ts_strlen(snps_txt);
    PTR_TEXTPRINT->extra2= 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->extra2= snps_txt[j];
        }
    PTR_TEXTPRINT->extra2= 0x0A01;
}

void snapshot_format(int format, void *base_virt, int image_size) {

}


// Define Graphics Memory snapshot path - file path defined as argument (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void snapshot_imp_path(char * filepath)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;
    i=ts_strlen(filepath);
    PTR_TEXTPRINT->extra4= 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->extra4= filepath[j];
        }
}


// Define Graphics Memory snapshot path - file path defined as argument (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void snapshot_ref_path(char * filepath)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;
    i=ts_strlen(filepath);
    PTR_TEXTPRINT->extra5= 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->extra5= filepath[j];
        }
}

// Force image into Graphics Memory (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void fast_image_load(char * filename, int resx, int resy, int offset, int mode_size)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;

    offset = (offset /*& 0x0FFFFFFF*/);
    i=ts_strlen(filename);
    PTR_TEXTPRINT->rimage_size   = (resx << 16) + resy;
    PTR_TEXTPRINT->rimage_offset = (offset>>2);
    PTR_TEXTPRINT->rimage_file= 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->rimage_file= filename[j];
        }

    PTR_TEXTPRINT->extra5=mode_size;

    PTR_TEXTPRINT->rimage_file= 0x515E;

    PTR_TEXTPRINT->rimage_file= 0x0A01;
}

// Force Data load into Graphics Memory (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void fast_data_load(const char * filename, int size, int offset, int b32)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;

    offset = (offset /*& 0x00FFFFFF*/);
    i      = ts_strlen(filename);
    PTR_TEXTPRINT->rimage_size   = (b32<<30) | size;
    PTR_TEXTPRINT->rimage_offset = offset;
    PTR_TEXTPRINT->rimage_file   = 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->rimage_file= filename[j];
        }
    PTR_TEXTPRINT->rimage_file= 0x0A03;
}


// Clear (set color) Graphics Memory (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void fast_clr_fb(int baseaddr, int resx, int resy, int color )
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->fb_baseaddr = baseaddr;
    PTR_TEXTPRINT->fb_sizeYX = (resy << 16) | resx;
    PTR_TEXTPRINT->fb_color = color;
    PTR_TEXTPRINT->character = 0xae26;
}


// Get a snapshot of specific region (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void snap_region(int baseaddr, int size, char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
    write_openfile(text);
    PTR_TEXTPRINT->rimage_offset = baseaddr;
    PTR_TEXTPRINT->rimage_size = size;
    PTR_TEXTPRINT->character = 0xae2a;
    write_closefile();
}

// Set frame-buffer address (Assist.v)
//-----------------------------------------------------------------------------------------------------------------------
void set_fb(int baseaddr, int resx, int resy, int stride, int color_mode )
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->fb_baseaddr = baseaddr;
    PTR_TEXTPRINT->fb_sizeYX   = (resy << 16) | resx;
    PTR_TEXTPRINT->fb_color    = color_mode;
    PTR_TEXTPRINT->fb_stride   = stride;
}

// Set VGA2BMP Index number
//-----------------------------------------------------------------------------------------------------------------------
void tb_VGA2BMPindex(int index)
{
    PTR_TEXTPRINT->extra6 = index;
}
//-----------------------------------------------------------------------------------------------------------------------
void tb_START(int index)
{
    PTR_TEXTPRINT->tb_start = index;
}
// Compare Images with Reference Images
//-----------------------------------------------------------------------------------------------------------------------
int cmp_image(char * filename, int ref_index, int imp_index)
//-----------------------------------------------------------------------------------------------------------------------
{

    if ( (ref_imp!=T2D_REF_TEST ) &
         (ref_imp!=T2D_DFLT_TEST) )  {

        int i,j;
        char snps_txt[300]; char snps_indx[4];

        // implementation image filename
        // ------------------------------------
        ts_strcpy (snps_txt,"");
        ts_strcat (snps_txt,filename);
        ts_strcat (snps_txt, ".");
        itoa      (imp_index, snps_indx);
        ts_strcat (snps_txt, snps_indx);


        i=ts_strlen(snps_txt);
        PTR_TEXTPRINT->rimage_file= 0xfffe;
        for(j=0;j<i;j++){
            PTR_TEXTPRINT->rimage_file= snps_txt[j];
        }
        PTR_TEXTPRINT->rimage_file= 0x0A10;

        // reference image filename
        // ------------------------------------
        ts_strcpy (snps_txt,"");
        ts_strcat (snps_txt,filename);
        ts_strcat (snps_txt, ".");
        itoa      (ref_index, snps_indx);
        ts_strcat (snps_txt, snps_indx);


        i=ts_strlen(snps_txt);
        PTR_TEXTPRINT->rimage_file= 0xffff;
        for(j=0;j<i;j++){
            PTR_TEXTPRINT->rimage_file= snps_txt[j];
        }
        PTR_TEXTPRINT->rimage_file= 0x0A02;
    }
    return (get_return_value());
}


// Compare written values with  Reference txt
//-----------------------------------------------------------------------------------------------------------------------
int cmp_insns(char *filename, void *base_addr, int num)
//-----------------------------------------------------------------------------------------------------------------------
{
    tsi_print("---------------------------------\n");
    tsi_print("Compare %s\n", filename);
    //----------------------------------------------------------------
    char fpga_result_file[400];
    //----------------------------------------------------------------
    ts_strcat(fpga_result_file, filename);
    ts_strcat(fpga_result_file, ".imp.txt");
    //----------------------------------------------------------------
    int *result_index_addr =(int *) base_addr;
    int result_index = num;
    tsi_print("address: 0x%x\n",base_addr);
    tsi_print("Writing 0x%x value(s)\n", num);
    tsi_print("file: %s\n",fpga_result_file);
    tsi_print("[0x%x] = 0x%x\n",(unsigned int)result_index_addr,(unsigned int)*result_index_addr);
    //----------------------------------------------------------------
    if(result_index == 0) {
        tsi_fprint("error there are no results to read \n");
    } else {
        for(int k = 0; k < result_index; k++) {
            tsi_print("[0x%x] = 0x%x\n",(unsigned int)result_index_addr,(unsigned int)*result_index_addr);
            PTR_TEXTPRINT->kernel_size = (unsigned int)*result_index_addr;
            result_index_addr++;
        }
    }
    //----------------------------------------------------------------
    int i,j;
    char snps_txt[300];
    ts_strcpy (snps_txt,"");
    ts_strcat (snps_txt,filename);
    i=ts_strlen(snps_txt);
    //----------------------------------------------------------------
    PTR_TEXTPRINT->rimage_file= 0xffff;
    for(j=0;j<i;j++) PTR_TEXTPRINT->rimage_file= snps_txt[j];
    //----------------------------------------------------------------
    PTR_TEXTPRINT->rimage_file= 0x0A04;
    return (get_return_value());
    return 0;
}

// End the Simulation
//-----------------------------------------------------------------------------------------------------------------------
void theend()
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->character=0x0000AE21;
}

// Show the current time and difference from last invocation
//-----------------------------------------------------------------------------------------------------------------------
void showtime()
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->character=0x0000AE23;
}

// Resets PowerEstimator Perfomance counters
//-----------------------------------------------------------------------------------------------------------------------
void pow_reset_perf()
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->character=0x0000AE24;
}

// Prints & Resets PowerEstimator Perfomance counters
//-----------------------------------------------------------------------------------------------------------------------
void pow_print_perf()
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->character=0x0000AE25;
}

// Opens a Filename for reading
//-----------------------------------------------------------------------------------------------------------------------
void read_openfile(char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;
    i=ts_strlen(text);
                     PTR_TEXTPRINT->rname= 0xffff;  //Reset Verilog String
    for(j=0;j<i;j++) PTR_TEXTPRINT->rname= text[j]; //Load each character
                     PTR_TEXTPRINT->rname= 0x0A01;  //Mark end
}

// Reads data from Opened filename
//-----------------------------------------------------------------------------------------------------------------------
int read_file(void)
//-----------------------------------------------------------------------------------------------------------------------
{
    return PTR_TEXTPRINT->rdata;
}

// Close Filename for reading
//-----------------------------------------------------------------------------------------------------------------------
void read_closefile(void)
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->rname= 0x0A02;
}

// Open a file for Writing
//-----------------------------------------------------------------------------------------------------------------------
void write_openfile(char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
    int i,j;
    i=ts_strlen(text);
    PTR_TEXTPRINT->wname= 0xffff;
    for(j=0;j<i;j++){
        PTR_TEXTPRINT->wname= text[j];
        }
    PTR_TEXTPRINT->wname= 0x0A01;
}

void write_openfile_const(const char * text)
//-----------------------------------------------------------------------------------------------------------------------
{
    write_openfile((char *)text);
}

// Write a byte on the previously opened file
//-----------------------------------------------------------------------------------------------------------------------
void write_file(int byte)
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->wdata= byte;
}

// Closes Opened File
//-----------------------------------------------------------------------------------------------------------------------
void write_closefile(void)
//-----------------------------------------------------------------------------------------------------------------------
{
    PTR_TEXTPRINT->wname= 0x0A02;
}


// Prints formatted text (supported formats: %c, %d, %x, %s || )
//-----------------------------------------------------------------------------------------------------------------------
void
tsi_fprint(const char *text,...)
{
#ifdef DEBUG_TEXT
    const char *str;
    char str_t[9];
    unsigned int i,j;
    va_list args;
    va_start(args,text);
    i=0;
    while (text[i]!='\0'){
        if( text[i]=='%'){
            i++;
            switch(text[i]){
                case 'c': PTR_TEXTPRINT->wdata=va_arg(args,unsigned int); break;
                case 'u':
                case 'd': itoa_2(va_arg(args,unsigned int),str_t); j=0;
                          while(str_t[j]!='\0'){PTR_TEXTPRINT->wdata=str_t[j];j++;} break;
                case 'x': itoh(va_arg(args,unsigned int),str_t); j=0;
                          while(str_t[j]!='\0'){ PTR_TEXTPRINT->wdata=str_t[j];j++;} break;
                case 's': str=va_arg(args,const char *); j=0;
                            while(str[j]!='\0'/*NULL*/){
                                PTR_TEXTPRINT->wdata=str[j];
                                j++;
                            }
                            break;
                case 'l': if ( text[++i]=='u') {
                              PTR_TEXTPRINT->extra3=va_arg(args,unsigned int);
                          } else {
                              PTR_TEXTPRINT->wdata=text[i];
                          }
                          break;
                default:  PTR_TEXTPRINT->wdata=text[i]; break;
            }
        }else if ( text[i]=='\\'){
            i++;
            switch(text[++i]){
                case 'n': PTR_TEXTPRINT->wdata=-0x0A; break;
                case 'r': PTR_TEXTPRINT->wdata=-0x0D; break;
                case 't': PTR_TEXTPRINT->wdata=-0x09; break;
                default:  PTR_TEXTPRINT->wdata=text[i]; break;
            }
        }else{
            PTR_TEXTPRINT->wdata=text[i];
        }
        i++;
    }
    va_end(args);
#endif
}

// Delay Ticks
//--------------------------------------------------
void delay(int ticks) {
//--------------------------------------------------
    volatile int i;
    for(i=0;i<ticks;i++);
    }

// Get Test Number
//--------------------------------------------------
int tb_testno(void) {
    return (PTR_TEXTPRINT->extra2);
    }
//--------------------------------------------------

// Get Test Mode
//--------------------------------------------------
int tb_testmode(void) {
    return (PTR_TEXTPRINT->extra7);
    }
//--------------------------------------------------

// After how much time PMU sends hal_gfx to sleep
//--------------------------------------------------
void PMU_sleep(int time) {
    PTR_TEXTPRINT->PMU_sleep = time;
}
//--------------------------------------------------

// Send command to PMU
//--------------------------------------------------
void PMU_command(int cmd) {
    PTR_TEXTPRINT->PMU_CMD = cmd;
}

// Get PMU Status
//--------------------------------------------------
int PMU_Status(void) {
    return (PTR_TEXTPRINT->PMU_STATUS);
    }
//--------------------------------------------------

// Tb-Assist Read General Purpose Input value
//--------------------------------------------------
unsigned int tb_read_gpi(void) {

    return (PTR_TEXTPRINT->gp_input);
}
//--------------------------------------------------

// Tb-Assist Read General Purpose Output value
//--------------------------------------------------
unsigned int tb_read_gpo(void) {

    return (PTR_TEXTPRINT->gp_output);
}
//--------------------------------------------------

// Tb-Assist Read General Purpose Output value
//--------------------------------------------------
void tb_set_gpo(unsigned int gpo) {

    PTR_TEXTPRINT->gp_output = gpo;
}
//--------------------------------------------------

// Tb-Assist Write Control General Purpose
// Input toggle counter
//--------------------------------------------------
void tb_set_ctrl_sigmon(unsigned int sigmon_Cntr_Ctrl) {

    PTR_TEXTPRINT->sigmon_Cntr_Ctrl = sigmon_Cntr_Ctrl;
}
//--------------------------------------------------

// Tb-Assist Read Control General Purpose
// Input Toggle Counter
//--------------------------------------------------
unsigned int tb_get_ctrl_sigmon() {

    return (PTR_TEXTPRINT->sigmon_Cntr_Ctrl);
}
//--------------------------------------------------


// Tb-Assist Read General Purpose Input Toggle Counter
//--------------------------------------------------
unsigned int tb_get_sigmon() {

    return (PTR_TEXTPRINT->sigmon_Cntr);
}
//--------------------------------------------------

void tb_set_clks(unsigned int clocks){
    PTR_TEXTPRINT->clk_gate_ctrl = clocks;
    delay(100);
}

void tb_disable_clks(unsigned int clocks){
    PTR_TEXTPRINT->clk_gate_ctrl &= ~clocks;
    delay(100);
}

void tb_enable_clks(unsigned int clocks){
    PTR_TEXTPRINT->clk_gate_ctrl |= clocks;
    delay(100);
}

unsigned int tb_get_clks(unsigned int clocks){
    return PTR_TEXTPRINT->clk_gate_ctrl;
}

// Clock generators: set half period
// Supported are integer divisions of ref_clk
// the implementation of the PLL has 50/50 duty cycle
void tb_set_clk_frequency(unsigned int frequency){
    PTR_TEXTPRINT->clk_ctrl = 1000.0/((float)frequency);
}

unsigned int tb_get_clk_frequency(){
    return 1000/(PTR_TEXTPRINT->clk_ctrl);
}


// Enables AHB delay for reads on iMemCtrl1 AHB memory
void tb_set_ahb_delay(unsigned int delay){
    PTR_TEXTPRINT->ahb_read_delay = delay;
}

unsigned int tb_get_ahb_delay(){
    return (PTR_TEXTPRINT->ahb_read_delay);
;
}

// Get random number from simulator
int tb_get_random_seed(void){
    return (PTR_TEXTPRINT->random_seed);
}

void tb_enable_mask(unsigned int address_mask, unsigned int mask_en){
    PTR_TEXTPRINT->address_mask |= address_mask;
    PTR_TEXTPRINT->address_mask_en |= mask_en;
    delay(100);
}

void tb_disable_mask(){
    PTR_TEXTPRINT->address_mask = 0x00000000;
    PTR_TEXTPRINT->address_mask_en =0;
    delay(100);
}


#endif // _TSI_ASSIST_C_
