# -*- coding:utf-8 -*-
######################################################################################################################
#  Usage :
#       used to transfer GR55xx's Keilv5 App Project to makefile file which is compiled by GNU Compiler
#  Command :
#        pyhon keil2makefile.py  *.uvprojx  [-d0]  [-h/--help]
#  Run Envrioment Requerd:
#       1. python3
#       2. linux/shell or linux/shell-like (such as mingw64, cygwin, msys2 ...)
#
#  Copyright(C) 2019, Shenzhen Goodix Technology Co., Ltd
######################################################################################################################


import os
import re
import shutil
import sys, platform
import xml.etree.ElementTree as XmlParser

global g_makefile_path
g_makefile_path = "../make_gcc/"

# Optional : "0" - gr5515, "1" - gr5513
##global g_target_chip_type
##g_target_chip_type = "0"

# The Default Makefile Template is located:
#
class MAKEFILE_TEMPLATE():

    g_makefile_template_target_name_key = \
    '''
MAKE_TARGET_NAME := '''

    g_makefile_template_chip_type_key = \
    '''
CHIP_TYPE := '''

    g_makefile_template_prj_src_c_key = \
    '''
PRJ_C_SRC_FILES:= '''

    g_makefile_template_prj_src_asm_key =\
    '''
PRJ_ASM_SRC_FILES := '''

    g_makefile_template_prj_c_include_key = \
    '''
PRJ_C_INCLUDE_PATH := '''

    g_makefile_template_prj_c_microdefs_key = \
    '''
PRJ_C_MICRO_DEFINES := '''

    g_makefile_template_prj_asm_include_key = \
    '''
PRJ_ASM_INCLUDE_PATH := '''

    g_makefile_template_prj_asm_microdefs_key = \
    '''
PRJ_ASM_MICRO_DEFINES := '''

    g_makefile_template_is_win_os = \
    '''
IS_WIN_OS := '''

    g_makefile_template_head_str = \
'''#########################################################################################################
# GNU Compiler makefile for Goodix BLE Application Project
# Copyright(C) 2019, Shenzhen Goodix Technology Co., Ltd
#
# Default Location of GCC Compile Ref Files
#
#   {APP_ROOT_DIR} is root directory in GR55xx SDK
#
#   1. sdk lib for gcc          : {APP_ROOT_DIR}/platform/soc/linker/gcc/libble_sdk.a
#   2. symbol file for gcc      : {APP_ROOT_DIR}/platform/soc/linker/gcc/rom_symbol_gcc.txt
#   3. link file for gcc        : {APP_ROOT_DIR}/platform/soc/linker/gcc/gcc_linker.lds & gcc_linker_graphics.lds
#   4. startup asemmbly file    : (APP_ROOT_DIR)/platform/arch/arm/cortex-m/gcc/startup_gr55xx.s
#   5. gcc Makefile Template    : (APP_ROOT_DIR)/tools/windows/gcc/Makefile.template
#########################################################################################################


#########################################################################################################
###                                Different Configuration For Different Project
#########################################################################################################

## Set Vars by Input Value
# target name
    '''

    g_makefile_template_common_str = \
    '''

# Set echo cmd
ECHO = @echo

#########################################################################################################
###                                   Common Configuration Area, Change carefully
#########################################################################################################

## Set Compiler (CC/ASM use same compile cmd for now)
CROSS_COMPILE 	= arm-none-eabi-
CC 				= $(CROSS_COMPILE)gcc
ASM				= $(CROSS_COMPILE)gcc
CPP 			= $(CROSS_COMPILE)cpp
LINK			= $(CROSS_COMPILE)gcc
OBJCOPY 		= $(CROSS_COMPILE)objcopy

## Set Common Flags for C/ASM
COMMON_COMPILE_FLAGS += -std=gnu99 --inline
COMMON_COMPILE_FLAGS += -ggdb3
COMMON_COMPILE_FLAGS += -ffunction-sections -fdata-sections
COMMON_COMPILE_FLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16  -mapcs-frame -mthumb-interwork -mthumb -mcpu=cortex-m4
COMMON_COMPILE_FLAGS += -gdwarf-2 -MD

## Set CFLAGS
# Set include path
CFLAGS += $(foreach inc,$(PRJ_C_INCLUDE_PATH),-I $(inc))
# Set macro-defines Flags
CFLAGS += $(foreach md,$(PRJ_C_MICRO_DEFINES),-D$(md))
CFLAGS += $(COMMON_COMPILE_FLAGS)
CFLAGS += -O2

## Set ASMFLAGS
ASMFLAGS += $(foreach inc,$(PRJ_ASM_INCLUDE_PATH),-I $(inc))
# Set macro-defines Flags
ASMFLAGS += $(foreach md,$(PRJ_ASM_MICRO_DEFINES),-D$(md))
ASMFLAGS += $(COMMON_COMPILE_FLAGS)

## Set default compile ref files
LINK_SCRIPT 		= ./gcc_linker.lds
# LINK_SCRIPT 		= ../../../../../platform/soc/linker/gcc/gcc_linker_graphics.lds

PATCH_FILE  		= ../../../../../platform/soc/linker/gcc/rom_symbol_gcc.txt

GCC_STARTUP_ASM_FILE 	= ../../../../../platform/arch/arm/cortex-m/gcc/startup_gr55xx.s

ifeq ($(IS_WIN_OS),true)
	BLE_TOOL_BIN		= ..\\..\\..\\..\\..\\build\\binaries\\ble_tools\\Keil\\ble_tools.exe
else
	BLE_TOOL_BIN		= ..\\..\\..\\..\\..\\build\\binaries\\ble_tools\\GCC\\ble_tools.gcc
endif


## Set LDFLAGS
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -specs=nano.specs
LDFLAGS += -L../../../../../platform/soc/linker/gcc/ -lble_sdk
# LDFLAGS += -L../../../../../platform/soc/linker/gcc/ -lgraphics_lvgl
# LDFLAGS += -L../../../../../platform/soc/linker/gcc/ -lgraphics_sdk

## Set compile output directory
BUILD 		?= out
BUILD_OBJ	?= $(BUILD)/obj
BUILD_LST	?= $(BUILD)/lst


## Set source files and objects
SRC_C	:= $(PRJ_C_SRC_FILES)
SRC_ASM := $(GCC_STARTUP_ASM_FILE) $(PRJ_ASM_SRC_FILES)
OBJ_C 	:= $(SRC_C:.c=.o)
OBJ_ASM := $(SRC_ASM:.s=.o)

OBJ 	:= $(OBJ_C) $(OBJ_ASM)
OBJ_ADJUST 	= $(patsubst %.o,$(BUILD_OBJ)/%.o,$(notdir $(OBJ)))


## verbosity switch
V ?= 0
ifeq ($(V),0)
	V_CC = @echo " $(CC) " $<;
	V_ASM = @echo " $(ASM) " $<;
	V_CPP = @echo " $(CPP) " $<;
	V_LINK = @echo " $(LINK) " $<;
	V_OBJCOPY = @echo " $(OBJCOPY) " $<;
else
	V_OPT = '-v'
endif


SRC_PATH += $(dir $(SRC_C))
SRC_PATH += $(dir $(SRC_ASM))
MAKE_PATH = $(foreach n,$(SRC_PATH),:$(n))
vpath %.c ./$(MAKE_PATH)
vpath %.s ./$(MAKE_PATH)
## default make goal
all: mk_path $(BUILD)/$(MAKE_TARGET_NAME).bin $(BUILD)/$(MAKE_TARGET_NAME).hex

##  compile C & asm files
$(BUILD_OBJ)/%.o : %.c
	$(V_CC) $(CC) $(CFLAGS) -c $< -o $@

## how to compile assembly files
$(BUILD_OBJ)/%.o : %.s
	$(V_ASM) $(ASM) $(ASMFLAGS) -c $< -o $@

## make depends
$(BUILD)/$(MAKE_TARGET_NAME).hex: $(BUILD_LST)/$(MAKE_TARGET_NAME).elf
	$(ECHO) "compile hex file ..."
	$(V_OBJCOPY) $(OBJCOPY) -O ihex $< $@

$(BUILD)/$(MAKE_TARGET_NAME).bin: $(BUILD_LST)/$(MAKE_TARGET_NAME).elf
	$(ECHO) "compile binary file ..."
	$(V_OBJCOPY) $(OBJCOPY) -O binary $< $@

$(BUILD_LST)/$(MAKE_TARGET_NAME).elf: $(OBJ_ADJUST)
	$(ECHO) "compile .elf file ..."
	$(V_LINK) $(LINK) $(CFLAGS) -T $(LINK_SCRIPT) $(PATCH_FILE) $(OBJ_ADJUST) $(LDFLAGS) -Wl,-Map=$(BUILD_LST)/$(MAKE_TARGET_NAME).map -o $@


mk_path :
	mkdir -p  $(BUILD)
	mkdir -p  $(BUILD_OBJ)
	mkdir -p  $(BUILD_LST)

flash: $(BUILD)/$(MAKE_TARGET_APP).bin
	$(ECHO) "Writing $< to the GR55xx-SK board"
	programer -t fw -p burn -i $(BUILD)/$(MAKE_TARGET_APP).bin

clean:
	rm -rf $(BUILD)

'''


    # g_makefile_template_target_name_key
    # g_makefile_template_target_app_key
    # g_makefile_template_chip_type_key
    # g_makefile_template_prj_src_c_key
    # g_makefile_template_prj_src_asm_key
    # g_makefile_template_prj_c_include_key
    # g_makefile_template_prj_c_microdefs_key
    # g_makefile_template_prj_asm_include_key
    # g_makefile_template_prj_asm_microdefs_key
    # g_makefile_template_head_str
    # g_makefile_template_common_str

    @staticmethod
    def __is_win_os():
        sys = platform.system()
        print(">>> OS type: " + sys)
        if sys.lower() == "windows" :
            return True
        else :
            return False

    @staticmethod
    def make(ProjectDict):
        if not os.path.isdir(g_makefile_path) :
            os.mkdir(g_makefile_path)
        mkfile = open(g_makefile_path + "Makefile",  "w+")
        if not mkfile :
            raise Exception(">>> Create makefile failed...")

        mkfile.write(MAKEFILE_TEMPLATE.g_makefile_template_head_str)
        # MAKEFILE_TEMPLATE.__fill_single_value(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_chip_type_key, g_target_chip_type)
        MAKEFILE_TEMPLATE.__fill_single_value(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_target_name_key, ProjectDict["APP_NAME"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_src_c_key, ProjectDict["C_SRC_FILES"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_src_asm_key, ProjectDict["ASM_SRC_FILES"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_c_include_key, ProjectDict["C_INCLUDES"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_c_microdefs_key, ProjectDict["C_DEFINES"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_asm_include_key, ProjectDict["ASM_INCLUDES"])
        MAKEFILE_TEMPLATE.__fill_list(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_prj_asm_microdefs_key, ProjectDict["ASM_DEFINES"])

        Is_win = MAKEFILE_TEMPLATE.__is_win_os()
        if Is_win :
            MAKEFILE_TEMPLATE.__fill_single_value(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_is_win_os,  "true")
        else :
            MAKEFILE_TEMPLATE.__fill_single_value(mkfile, MAKEFILE_TEMPLATE.g_makefile_template_is_win_os,  "false")
        mkfile.write(MAKEFILE_TEMPLATE.g_makefile_template_common_str)
        mkfile.close()
        return True

    @staticmethod
    def __fill_single_value(MKFile, Key, Value):
        MKFile.write(Key + " " + Value + "\n")
        return

    @staticmethod
    def __fill_list(MKFile, Key, ListValue):
        MKFile.write(Key + "  \\" + "\n")
        for val in ListValue :
            MKFile.write(val + "  \\" + "\n")
        MKFile.write("\n")
        return

class KEIL5_PARSER():
    g_project_file = None
    g_targets_dict = []

    def __init__(self, file=''):
        if file is None:
            raise Exception("file object cannot be None")
        if not os.path.exists(file):
            raise Exception("invalid file : " + file)
        self.g_project_file = file
        pass

    def parse(self):
        xml_p = XmlParser.parse(self.g_project_file)
        root = xml_p.getroot()
        for targets in root.findall("Targets"):
            for target in targets.findall("Target"):
                tgt_ret = self.__parse_target(target)
                if tgt_ret :
                    self.g_targets_dict.append(tgt_ret)

        # print(self.g_targets_dict)
        return self.g_targets_dict

    def __parse_target(self, target):
        '''must return dict type'''
        target_dict = {}
        target_option_dict = {}
        target_groups_dict = {}

        target_name = target.find("TargetName").text

        # check Toolset
        if target.find("ToolsetName").text != "ARM-ADS" :
            raise Exception("+++ Just support transfer ARM-ADS keil Project")
            #print(">>> Just support transfer ARM-ADS keil Project")

        # parse <TargetOption>...</TargetOption>
        targetOption = target.find("TargetOption")
        target_option_dict = self.__parse_target_option(targetOption)

        if target_option_dict :
            target_dict['TARGET_NAME'] = target_name
            target_dict = {**target_dict, **target_option_dict}

        # parse <Groups>...</Groups>
        targetGroups = target.find("Groups")
        target_groups_dict = self.__parse_target_groups(targetGroups)

        if target_groups_dict :
            target_dict['TARGET_NAME'] = target_name
            target_dict['SRC_FILES'] = target_groups_dict
            target_dict['C_SRC_FILES'] = []
            target_dict['ASM_SRC_FILES'] = []
            for group_key in target_groups_dict.keys() :
                for i in range(len(target_groups_dict[group_key]) ):
                    file = target_groups_dict[group_key][i]
                    lower_file = file.lower()
                    if  lower_file.endswith('.s') or lower_file.endswith('.asm')  :
                        if "startup" not in lower_file  :
                            target_dict['ASM_SRC_FILES'].append(file)
                    else:
                        target_dict['C_SRC_FILES'].append(file)

        return target_dict

    def __parse_target_option(self, target_option):
        '''type(target_option): xml Element'''
        '''must return dict type'''
        options = {}

        if target_option :
            # parse common options
            cmonOption = target_option.find("TargetCommonOption")
            if cmonOption :
                options['DEVICE'] = cmonOption.find("Device").text
                options['APP_NAME'] = cmonOption.find("OutputName").text

            # parse Arm-ADS options
            targetArmAds = target_option.find("TargetArmAds")
            if targetArmAds :
                # for c compile
                cads = targetArmAds.find("Cads")
                if cads :
                    variousControls = cads.find("VariousControls")
                    if variousControls :
                        defines  = variousControls.find("Define").text
                        options["C_DEFINES"] = self.__parse_micro_defines(defines)
                        includes = variousControls.find("IncludePath").text
                        options["C_INCLUDES"] = self.__parse_include_path(includes)  # includes_dict
                # for asm compile
                asmads = targetArmAds.find("Aads")
                if asmads :
                    variousControls = asmads.find("VariousControls")
                    if variousControls :
                        defines  = variousControls.find("Define").text
                        options["ASM_DEFINES"] = self.__parse_micro_defines(defines)  #defs_dict
                        includes = variousControls.find("IncludePath").text
                        options["ASM_INCLUDES"] = self.__parse_include_path(includes)  # includes_dict
                # for Link

        return options

    def __parse_micro_defines(self, defines):
        '''
        defines fromat :   " DEF_A  , DEFB,   DEFC, ...."
        not support   -via  xxx.option   for now!
        '''
        defs_dict = []
        if defines :
            # split by ','
            defs = defines.split(',')
            for ds in defs:
                # trim space in head and tail
                d = ds.strip()
                if d :
                    d = d.replace(" ", "")
                if d :
                    defs_dict.append(d)
        return defs_dict

    def __parse_include_path(self, includes):
        '''
        defines fromat :   " path_a; path_b; ...."
        '''
        includes_dict = []
        if includes :
            # split by ';'
            incs = includes.split(';')
            for inc in incs :
                # trim space in head and tail
                inc_update = inc.replace('\\', '/').strip()
                if  inc_update :
                    includes_dict.append(inc_update)
        return includes_dict

    def __parse_target_groups(self, target_groups):
        '''
        type(target_groups): xml Element
        return dict type
        format: {group_name1 : [file, file, ...], group_name2 : [file, file, ...], ...}
        '''
        # <Groups>
        #   <Group>
        #       <GroupName>...</GroupName>
        #       <GroupOption>...</GroupOption>
        #       <Files>...</Files>
        #   </Group>
        # </Groups>

        #
        groups = {}

        for group in target_groups.findall("Group") :
            grp_name = group.find("GroupName").text


            # check Group is included or not ?
            grp_opt = group.find("GroupOption")
            if grp_opt :
                cmonProp = grp_opt.find("CommonProperty")
                if cmonProp :
                    includeInBuild = cmonProp.find("IncludeInBuild")
                    if (includeInBuild != None) and (int(includeInBuild.text) == 0) :
                        print(">>> Cause of disable build, ignore the whole group : " + grp_name)
                        continue
            # List files
            files = group.find("Files")
            files_dict = []

            if files :
                for file in files :
                    file_name = file.find("FileName").text.lower()
                    file_type = file.find("FileType").text
                    file_path = file.find("FilePath").text

                    # check file is included or not
                    file_opt  = file.find("FileOption")
                    if file_opt :
                        cmonProp = file_opt.find("CommonProperty")
                        if cmonProp :
                            includeInBuild = cmonProp.find("IncludeInBuild")
                            if (includeInBuild != None) and (int(includeInBuild.text) == 0) :
                                print(">>> Cause of disable build, ignore file " + file_name + " in group " + grp_name)
                                continue

                    # not use .s file, just c/cpp file
                    if ( file_name.endswith('.c') or file_name.endswith('.cpp')  or ( ( file_name.endswith('.asm')  or file_name.endswith('.s') ) and  file_name != "startup_gr55xx.s") ) :
                        files_dict.append(file_path.replace("\\", "/") )

                if files_dict:
                    groups[grp_name] = files_dict
        return groups

# give special handle to some projects
def handle_special_project(ProjectName, ParseResult) :
    ParseResultUpdate = ParseResult
    # if ProjectName.startswith("ble_app_template_freertos" ) or ProjectName.startswith("ble_app_throughput" ):
        # gcc / armcc use different header files and port files
        # print(ParseResult)
    for i in range(len(ParseResult))  :
        # 1. update header
        C_INCS = ParseResult[i]["C_INCLUDES"]
        C_INCS_UPDATE = []
        for C_INC in C_INCS :
            C_INCS_UPDATE.append(C_INC.replace("RVDS", "GCC").replace("rvds", "GCC"))
        ParseResultUpdate[i]["C_INCLUDES"] = C_INCS_UPDATE

        ASM_INCS = ParseResult[i]["ASM_INCLUDES"]
        ASM_INCS_UPDATE = []
        for ASM_INC in ASM_INCS :
            ASM_INCS_UPDATE.append(ASM_INC.replace("RVDS", "GCC").replace("rvds", "GCC"))
        ParseResultUpdate[i]["ASM_INCLUDES"] = ASM_INCS_UPDATE

        # 2. update c source
        C_SRCS = ParseResult[i]["C_SRC_FILES"]
        C_SRCS_UPDATE = []
        for C_SRC in C_SRCS :
            C_SRCS_UPDATE.append(C_SRC.replace("RVDS", "GCC").replace("rvds", "GCC"))
        ParseResultUpdate[i]["C_SRC_FILES"] = C_SRCS_UPDATE

        ASM_SRCS = ParseResult[i]["ASM_SRC_FILES"]
        ASM_SRCS_UPDATE = []
        for ASM_SRC in ASM_SRCS :
            ASM_SRCS_UPDATE.append(ASM_SRC.replace("RVDS", "GCC").replace("rvds", "GCC"))
        ParseResultUpdate[i]["ASM_SRC_FILES"] = ASM_SRCS_UPDATE

        # 3. TODO : NOT Update SRC_FILES for now

    return ParseResultUpdate

origin_ld_path = ""

#############################################################################################################
#############################################################################################################
#############################################################################################################
g_link_script_file  = ""
g_config_file       = '../Src/config/custom_config.h'
g_chip              = "CHIP_VER"
g_load_addr         = "APP_CODE_LOAD_ADDR"
g_run_addr          = "APP_CODE_RUN_ADDR"
g_gcc_linker        = "gcc_linker.lds"
g_key_words         = "FLASH (rx) : ORIGIN = "
g_lds_path          = "platform/soc/linker/gcc/"
g_lds_file          = g_lds_path + g_gcc_linker

class LinkerScript():
    def __init__(self) -> None:
        self.ld_path_ = ''
        self.chip_ver_ = ''
        self.load_addr_ = ''
        self.run_addr_ = ''

    def get_lds_path(self, path):
        count = path.count('../')
        for i in range(count):
            self.ld_path_ += '../'
        self.ld_path_ += g_lds_file
        return self.ld_path_

    def get_config_args(self):
        if not os.path.exists(g_config_file):
            print("Project configuration file does't exists ...")
            return False
        with open(g_config_file, "r", encoding='utf-8') as f_r:
            lines = f_r.readlines()
        for line in lines:
            if g_chip in line:
                version = line.split(g_chip)
                self.chip_ver_ = version[1].strip() #special index
            if g_load_addr in line:
                addr = line.split(g_load_addr)
                self.load_addr_ = addr[1].strip() #special index
                self.run_addr_  = addr[1].strip() #special index
        return True

    @staticmethod
    def copy_file(src, dst):
        shutil.copy(src, dst)

    def make(self, file_path):
        if not os.path.isdir(g_makefile_path) :
            os.mkdir(g_makefile_path)
        if not os.path.exists(file_path):
            print("lds file does't exist ...")
            return False
        self.copy_file(file_path, g_makefile_path)
        path = g_makefile_path + g_gcc_linker
        with open(path, "r", encoding='utf-8') as f_r:
            lines = f_r.readlines()
        f_r.close()
        tag = False
        with open(path, "w+", encoding='utf-8') as f_w:
            for line in lines:
                if g_key_words in line:
                    tag = True
                    line = re.sub('0x[0-9a-f]+[, ]*,', self.load_addr_ + ',', line)
                    print(line)
                f_w.write(line)
        f_w.close()
        if tag:
            return True
        print("Can't find the flash address ...")
        return False
#############################################################################################
#############################################################################################
#############################################################################################
# sys params:
#       Param 0 :  self of keil2makefile.py
#       Param 1 :  *.uvprojx
#       Param 2 :  optional, if equals -d0, select the first target to generate Makefile in default; else, ignore this param
if __name__ == "__main__":
    parm_keil_prj = None
    is_generate_first_target = True
    which_target = 0
    is_print_help = False

    for i in range(len(sys.argv)) :
        if sys.argv[i].endswith(".uvprojx"):
            parm_keil_prj = sys.argv[i]
            continue
        if sys.argv[i].lower() == "-d0":
            is_generate_first_target = True
            continue
        if sys.argv[i].lower() == "-h" or sys.argv[i].lower() == "--help":
            is_print_help = True
            break

    if (parm_keil_prj == None ) or (is_print_help == True):
        print(">>> Usuage: python keil2makefile.py  *.uvprojx  [-d0] [-h/--help]")
        print(">>>>>> Note 1 :  .py and *.uvprojx must in the same directory")
        print(">>>>>> Note 2 :  -d0 is optional, if specified, select the first target to generate Makfile in default")
        print(">>>>>> Note 3 :  -h or --help - print usuage help")
        sys.exit(0)

    file_path = parm_keil_prj
    prj_file = os.path.basename(file_path)
    dir_path = os.path.dirname(file_path)

    if ( not os.path.exists(os.getcwd() + "/" + prj_file) )  and os.path.exists(file_path) :
        raise Exception (">>> The .py script and keil project files must at the same directory !")

    print(">>> Transfer project : %s" % prj_file)
    parseResult = KEIL5_PARSER(prj_file).parse()

    parseResult = handle_special_project(prj_file, parseResult)
    target_num = len(parseResult)

    if (target_num <= 1) :
        which_target = 0

    ivalue = 0
    if ( is_generate_first_target == False ) and (target_num > 1 ):
        print(">>> Find more than one compile target, Please select which one to generate makefile ?")
        for i in range(target_num) :
            print("++++++ %d : %s / %s" % (i, parseResult[i]["TARGET_NAME"], parseResult[i]["APP_NAME"]))
        print("")
        iput = input(">>> Enter the selected order : ")
        try :
            ivalue = int(iput)
            if ivalue < 0 or ivalue >= target_num :
                raise Exception("")
        except Exception as e:
            raise Exception(">>> invalid input : " +  iput)
        finally :
            which_target = ivalue

    print(">>> The goal project name : %s / %s" % (parseResult[which_target]["TARGET_NAME"], parseResult[which_target]["APP_NAME"]))
    Ret = MAKEFILE_TEMPLATE.make(parseResult[which_target])

    if Ret :
        print(">>> Generate Makefile Successfully, located at " + g_makefile_path + "Makefile")
    else:
        print(">>> Generate Makefile failed ...")
    ##############################################################################################################################
    ##############################################################################################################################
    lds_path = ''
    for path in parseResult[which_target]["C_INCLUDES"]:
        if 'components' in path:
            lds_path = path
            break
    print(">>> Generate lds file starting ...")
    ld_test = LinkerScript()
    if not ld_test.get_config_args():
        sys.exit(0)
    if ld_test.make(ld_test.get_lds_path(lds_path)):
        print(">>> Generate lds file finish ...")
        sys.exit(0)

