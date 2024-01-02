# -*- coding:utf-8 -*-
######################################################################################################################
#  Usage :
#       used to transfer GR55xx's Keilv5 App Project to iar project which is compiled by iar Compiler
#  Command :
#        pyhon keil2iarprj.py  *.uvprojx
#  Run Envrioment Requerd:
#       1. python3
#       2. linux/shell or linux/shell-like (such as mingw64, cygwin, msys2 ...)
#
#  Copyright(C) 2022, Shenzhen Goodix Technology Co., Ltd
######################################################################################################################


import os
import re
import shutil
import sys, platform
import xml.etree.ElementTree as ET

SYMBOL_TXT   = '$PROJ_DIR$\..\..\..\..\..\platform\soc\linker\iar\rom_symbol_iar.txt'
STARTUP_FILE = '$PROJ_DIR$\..\..\..\..\..\platform\platform\arch\arm\cortex-m\iar\startup_gr55xx.s'
SDK_LIB      = '$PROJ_DIR$\..\..\..\..\..\platform\soc\linker\iar\ble_sdk.a'

TEMPALTE_PRJ = "..\\..\\..\\..\\..\\platform\\soc\\linker\\iar\\prj\\"
TEMPLATE_EWD = 'template.ewd'
TEMPLATE_EWP = 'template.ewp'
TEMPLATE_EWT = 'template.ewt'
TEMPLATE_EWW = 'template.eww'
TEMPLATE_DRV = 'template.driver.xcl'
PROJECT_PATH = "../IAR/"
SETTING_PATH = "../IAR/settings/"
class IARPrjGen():
    def __init__(self, prj) -> None:
        self.inc_str_ = ''
        self.mac_str_ = ''
        self.src_list_ = []
        self.prj_ = prj
        self.ewd = ''
        self.ewp = ''
        self.ewt = ''
        self.eww = ''
        self.drv = ''
        self.freertos = False

    def copy_file(self, src, dst):
        ewd = src + TEMPLATE_EWD
        ewp = src + TEMPLATE_EWP
        ewt = src + TEMPLATE_EWT
        eww = src + TEMPLATE_EWW
        drv = src + 'settings/' + TEMPLATE_DRV
        if not os.path.isdir(PROJECT_PATH) :
            os.mkdir(PROJECT_PATH)
        if not os.path.isdir(SETTING_PATH) :
            os.mkdir(SETTING_PATH)
        # print(ewd)
        # if not os.path.exists(ewd) or not os.path.exists(ewp) \
        #   or os.path.exists(ewt) or os.path.exists(eww):
        #     print("Files don't exist ...")
        #     return False
        dst_ewd = dst + self.prj_ + '.ewd'
        dst_ewp = dst + self.prj_ + '.ewp'
        dst_ewt = dst + self.prj_ + '.ewt'
        dst_eww = dst + self.prj_ + '.eww'
        dst_drv = dst + 'settings/' + self.prj_ + '.' + self.prj_ + '.driver.xcl'
        shutil.copy(ewd, dst_ewd)
        shutil.copy(ewp, dst_ewp)
        shutil.copy(ewt, dst_ewt)
        shutil.copy(eww, dst_eww)
        shutil.copy(drv, dst_drv)
        self.ewd = PROJECT_PATH + self.prj_ + '.ewd'
        self.ewp = PROJECT_PATH + self.prj_ + '.ewp'
        self.ewt = PROJECT_PATH + self.prj_ + '.ewt'
        self.eww = PROJECT_PATH + self.prj_ + '.eww'
        self.drv = PROJECT_PATH + self.prj_ + '.driver.xcl'
        return True

    def parse_file(self, path):
        src_str = ''
        port_file = ''
        tree = ET.parse(path)
        root = tree.getroot()
        for Targets_node in root.findall("Targets"):
            for Target_node in Targets_node.findall("Target"):
                for TargetOption_node in Target_node.findall("TargetOption"):
                    for TargetArmAds_node in TargetOption_node.findall("TargetArmAds"):
                        for Cads_node in TargetArmAds_node.findall("Cads"):
                            for VariousControls_node in Cads_node.findall("VariousControls"):
                                for Define_node in VariousControls_node.findall("Define"):
                                    if Define_node.text != None:
                                        self.mac_str_ += Define_node.text
                                for IncludePath_node in VariousControls_node.findall("IncludePath"):
                                    if IncludePath_node.text != None:
                                        self.inc_str_ += IncludePath_node.text.replace("RVDS", "IAR").replace("rvds", "iar")
                for Groups_node in Target_node.findall("Groups"):
                    for Group_node in Groups_node.findall("Group"):
                        for GroupName_node in Group_node.findall("GroupName"):
                            if (GroupName_node.text == "gr_stack_lib"):
                                continue
                            src_str = ''
                            src_str += "<group>\n"
                            src_str += "<name>"
                            src_str += GroupName_node.text
                            src_str += "</name>\n"
                            self.src_list_.append(src_str)
                            for Files_node in Group_node.findall("Files"):
                                for File_node in Files_node.findall("File"):
                                    for FilePath_node in File_node.findall("FilePath"):
                                        src_str = ''
                                        src_str += "<file>\n"
                                        src_str += "<name>"
                                        src_str += "$PROJ_DIR$\\"
                                        if (FilePath_node.text.__contains__("RVDS") or FilePath_node.text.__contains__("rvds")) and FilePath_node.text.__contains__("port.c"):
                                            self.freertos = True
                                            port_file = FilePath_node.text
                                        src_str += FilePath_node.text.replace("RVDS", "IAR").replace("rvds", "iar")
                                        src_str += "</name>\n"
                                        src_str += "</file>\n"
                                        self.src_list_.append(src_str)
                            if self.freertos:
                                src_str = ''
                                src_str += "<file>\n"
                                src_str += "<name>"
                                src_str += "$PROJ_DIR$\\"
                                src_str += port_file.replace("RVDS", "IAR").replace("rvds", "iar").replace("port.c", "portasm.s")
                                src_str += "</name>\n"
                                src_str += "</file>\n"
                                self.src_list_.append(src_str)
                            self.src_list_.append("</group>\n")

    def macro_list_get(self):
        macros_list = re.split(' ', self.mac_str_.replace(',', ' '))
        return macros_list

    def inc_list_get(self):
        inc_list = re.split(';', self.inc_str_)
        return inc_list

    def src_list_get(self):
        return self.src_list_

    def set_ewd_file(self):
        # print(self.ewd)
        ewd_tree = ET.parse(self.ewd)
        ewd_root = ewd_tree.getroot()
        for configuration_node in ewd_root.findall("configuration"):
            for name_node in configuration_node.findall("name"):
                name_node.text = "GRxx_Soc"
        ewd_tree.write(self.ewd)

    def set_ewp_file(self):
        ewp_tree = ET.parse(self.ewp)
        ewp_root = ewp_tree.getroot()
        for configuration_node in ewp_root.findall("configuration"):
            for name_node in configuration_node.findall("name"):
                name_node.text = self.prj_
            for settings_node in configuration_node.findall("settings"):
                for data_node in settings_node.findall("data"):
                    for option_node in data_node.findall("option"):
                        for name_node in option_node.findall("name"):
                            if (name_node.text == "CCDefines"):
                                option_node.remove(option_node[1])
                                for list in self.macro_list_get():
                                    node = ET.Element("state")
                                    node.text = list
                                    option_node.append(node)
                            if (name_node.text == "CCIncludePath2"):
                                option_node.remove(option_node[1])
                                for list in self.inc_list_get():
                                    node = ET.Element("state")
                                    node.text = "$PROJ_DIR$\\" + list
                                    option_node.append(node)
                            if (name_node.text == "OOCOutputFile"):
                                node = ET.Element("state")
                                node.text = self.prj_ + ".bin"
                                option_node.append(node)
                            if (name_node.text == "IlinkOutputFile"):
                                node = ET.Element("state")
                                node.text = self.prj_ + ".out"
                                option_node.append(node)
                            if (name_node.text == "FPU2"):
                                option_node.remove(option_node[2])
                                node = ET.Element("state")
                                node.text = "4"
                                option_node.append(node)
                            if (name_node.text == "NrRegs"):
                                option_node.remove(option_node[2])
                                node = ET.Element("state")
                                node.text = "1"
                                option_node.append(node)
        ewp_tree.write(self.ewp)
        ewp_handle = open(self.ewp, "r+")
        file_ctnt = ewp_handle.read()
        ewp_handle.seek(0, 0)
        ewp_handle.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
        ewp_handle.write(file_ctnt)
        ewp_handle.seek(0)

        src_list = []
        for str in self.src_list_:
            str = str.replace("keil\\Startup.S", "iar\\startup_gr55xx.s")
            src_list.append(str)

        all_lines = ewp_handle.readlines()
        ewp_handle.seek(0)
        for line in all_lines:
            if "</project>" in line:
                for str in src_list:
                    ewp_handle.write(str)
            ewp_handle.write(line)
        ewp_handle.close()

    def set_eww_file(self):
        eww_handle = open(self.eww, "r+")
        all_lines = eww_handle.readlines()
        eww_handle.seek(0)
        for line in all_lines:
            if "iar_empty" in line:
                line = line.replace("iar_empty", self.prj_ + '.ewp')
            eww_handle.write(line)
        eww_handle.close()

    def generate_prj(self):
        self.set_ewd_file()
        self.set_eww_file()
        self.set_ewp_file()
        print(">>> Generate IAR project successfully, locate at ", self.eww)
#################################################################################
#################################################################################
G_CONFIG_FILE       = '../Src/config/custom_config.h'
G_CHIP              = "CHIP_VER"
G_LOAD_ADDR         = "APP_CODE_LOAD_ADDR"
G_RUN_ADDR          = "APP_CODE_RUN_ADDR"
G_IAR_ICF           = "gr55xx_template.icf"
G_KEY1_WORDS        = "define symbol __ICFEDIT_intvec_start__ = "
G_KEY2_WORDS        = "define symbol __ICFEDIT_region_IROM1_start__ = "
G_ICF_PATH          = "..\\..\\..\\..\\..\\platform\\soc\\linker\\iar\\"
G_LDS_FILE          = G_ICF_PATH + G_IAR_ICF
class IARICFConfig():
    def __init__(self) -> None:
        self.ld_path_ = ''
        self.chip_ver_ = ''
        self.load_addr_ = ''
        self.run_addr_ = ''

    def get_config_args(self):
        if not os.path.exists(G_CONFIG_FILE):
            print("Project configuration file does't exists ...")
            return False
        with open(G_CONFIG_FILE, "r", encoding='utf-8') as f_r:
            lines = f_r.readlines()
        for line in lines:
            if G_CHIP in line:
                version = line.split(G_CHIP)
                self.chip_ver_ = version[1].strip() #special index
            if G_LOAD_ADDR in line:
                addr = line.split(G_LOAD_ADDR)
                self.load_addr_ = addr[1].strip() #special index
                self.run_addr_  = addr[1].strip() #special index
        return True

    @staticmethod
    def copy_file(src, dst):
        shutil.copy(src, dst)

    def make(self, file_path):
        if os.path.isdir(PROJECT_PATH):
           shutil.rmtree(PROJECT_PATH)
        os.mkdir(PROJECT_PATH)
        if not os.path.exists(file_path):
            print("IAR icf file does't exist ...")
            return False
        if self.chip_ver_ == '':
            self.chip_ver_ = "gr55xx"
        path = PROJECT_PATH + self.chip_ver_.replace("0x", "gr") + ".icf"
        self.copy_file(file_path, path)
        with open(path, "r", encoding='utf-8') as f_r:
            lines = f_r.readlines()
        f_r.close()
        tag = False
        with open(path, "w+", encoding='utf-8') as f_w:
            for line in lines:
                if G_KEY1_WORDS in line or G_KEY2_WORDS in line:
                    tag = True
                    line = re.sub('0x[0-9a-f]+[; ]', self.load_addr_ + ";", line)
                    # print(">>> " + self.load_addr_ , line)
                f_w.write(line)
        f_w.close()
        if tag:
            return True
        print("Can't find the flash address ...")
        return False

#######################################################################################
#######################################################################################
#######################################################################################
#######################################################################################
def main():
    prj = sys.argv[1]
    name = prj.split('.')
    print(">>> Transfer project: ", prj)
    system = platform.system()
    print(">>> OS type: " + system)
    icf = IARICFConfig()
    if not icf.get_config_args():
        return
    if icf.make(G_LDS_FILE):
        gen = IARPrjGen(name[0])
        if not gen.copy_file(TEMPALTE_PRJ, "../IAR/"):
            return
        gen.parse_file(prj)
        gen.generate_prj()

if __name__ == "__main__":
   main()