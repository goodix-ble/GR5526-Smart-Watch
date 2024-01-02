# GR5526-Smart-Watch

[EN](https://github.com/nixlong/GR5526-Smart-Watch/blob/main/README.md)   |  [中文](https://github.com/nixlong/GR5526-Smart-Watch/blob/main/README_zh.md)


## 1. Introduction

### 1.1 The Design

This is a  ref-design for Smart Watch Based On GR5526 BLE SoC, Supported 2.5D Embedded GPU and Lvgl_8.31 GUI Framework. Customers can get higher fps and enhanced performance for smart watch through the Chip's ability.


### 1.2 GR5526

The Goodix GR5526 family is a single-mode, low-power Bluetooth 5.3 System-on-Chip (SoC). 

It can be configured as a Broadcaster, an Observer, a Central, a Peripheral, and supports the combination of all the above roles, making it an ideal choice for Internet of Things (IoT), LE Audio, and smart wearable devices.
Based on ARM® Cortex®-M4F CPU core, the GR5526 series integrates Bluetooth 5.3 Protocol Stack, a 2.4 GHz RF transceiver, on-chip 1 MBytes programmable Flash memory, 512KBytes RAM, and multiple peripherals. GR5526 delivers a feature-rich display and graphics solution by providing the option of graphics acceleration (GPU + Display Controller) and system-in-package (SiP) 8 MBytes pseudostatic RAM (PSRAM) to accommodate display while still leaving plenty of resource for wearable schemes.


- Click [Me](https://www.goodix.com/en/product/connectivity/ble/gr5526) to view the GR5526 Introduction


## 2. Smart Watch

### 2.1 Ref Project 

The Ref Project is located at [projects/peripheral/graphics/gr5526_smart_watch](projects/peripheral/graphics/gr5526_smart_watch), It's compiled with Keil µVision5 in default. it also can be compiled by gcc and IAR.


### 2.2 Set UP the Development Environment

 To Set up the development environment, the following software is required:

- Keil µVision5: Integrated Development Environment (IDE) used by the Ref-Design project
- Device Family Pack (DFP) that supports ARM Cortex-M4 FPU Chip
- J-Link: programming and debugging tool 
- Serial port assistant tool: log printout
- GR5526 family chip programming algorithm file for Keil: you can find the .flm file in [build/keil](build/keil)


### 2.3 Hardware Related

- This Ref-Design is based on the GR5526 Starter Kit Board. you can find introduction to SK board here [GR5526_SK](https://www.goodix.com/en/kit/gr5526_starter_kit)
- Click  [GR5526_Diagram](https://www.goodix.com/en/docview/GR5526-SK-BASIC-RevC_Rev.1.0?objectId=159&objectType=document&version=313) to view the circuit diagram for GR5526 SK Board 


### 2.4 Demo effect for Ref-Design 

- You can watch the Demo effect :

  - Click the web link : [Smart_Watch_Video](https://www.bilibili.com/video/BV1Re411X7P6/?share_source=copy_web&vd_source=253f7e2d634ff4f728c7e7bfa218f990)
  -  Scan the following QRCode to view the video. 

  ![](./resource/GR5526_Smart_Watch_Video.png) 





