# GR5526-Smart-Watch

[EN](https://github.com/nixlong/GR5526-Smart-Watch/blob/main/README.md)   |  [中文](https://github.com/nixlong/GR5526-Smart-Watch/blob/main/README_zh.md)


## 1. 介绍

### 1.1 参考设计

本Smart Watch (智能手表) 参考设计基于 GR5526 BLE SoC进行实现, GR5526 包含一个 2.5D GPU, 设计支持 Lvgl_8.31  GUI框架. 用户的穿戴产品可以通过 GR5526的能力获得更好的帧率和更强的能力表达.




### 1.2 GR5526

GR5526系列是汇顶科技推出的Bluetooth 5.3单模低功耗蓝牙系统级芯片,可配置为广播者、观察者、外围设备或中央设备,并支持上述各种角色的组合应用。GR5526广泛应用于物联网(IoT)、和智能穿戴设备领域。

基于ARM® Cortex®-M4F CPU内核,GR5526系列集成了蓝牙5.3协议栈、2.4 GHz射频收发器、片上集成 1MB可编程Flash存储、512KB RAM及多种外设,提供图形化处理单元(GPU )+ 显示控制器(DC)解决方案,支持8 MB SiP PSRAM,可为用户提供更丰富的数据空间与强大的图形化表现能力,为可穿戴设备方案提供丰富的片上资源。


- 点击 [链接](https://www.goodix.com/zh/product/connectivity/ble/gr5526) 查看 GR5526 介绍



## 2. Smart Watch

### 2.1 工程说明 

参考工程位于 [projects/peripheral/graphics/gr5526_smart_watch](projects/peripheral/graphics/gr5526_smart_watch), 默认由 Keil µVision5 构建. 同时也支持 gcc和IAR 编译. 


### 2.2 构建开发环境

 构建参考工程的开发环境, 需要依赖以下软件: 

- Keil µVision5: 参考工程默认的编译构建IDE
- Device Family Pack (DFP) : 支持 ARM Cortex-M4 FPU 的 DFP包
- J-Link: 程序下载调试工具
- 串口助手: 用于程序的串口日志打印
- GR5526 keil 下载算法文件:  .flm 算法文件默认放置在目录 [build/keil](build/keil)



### 2.3 硬件相关

- 本Smart Watch参考设计基于 GR5526 SK开发板进行,  用户可以从这里找到关于开发板的介绍  [GR5526_SK](https://www.goodix.com/en/kit/gr5526_starter_kit)

- 点击 [GR5526_Diagram](https://www.goodix.com/en/docview/GR5526-SK-BASIC-RevC_Rev.1.0?objectId=159&objectType=document&version=313) 查看 GR5526 SK 板的电路图, 用户可以参考SK电路图, 进行自己的二次开发修改. 进行产品开发或 DIY 

  

### 2.4 参考设计的演示效果 

- 参考设计演示视频(部分界面)
  
  https://github.com/nixlong/GR5526-Smart-Watch/blob/main/resource/smart_watch_video.mp4
  
<iframe width="854" height="480" src="https://github.com/nixlong/GR5526-Smart-Watch/blob/main/resource/smart_watch_video.mp4" frameborder="0" allowfullscreen></iframe>


- 可以通过下面方式观看参考设计的演示效果(更多界面) :

  - 点击跳转到B站观看 : [Smart_Watch_Video](https://www.bilibili.com/video/BV1Re411X7P6/?share_source=copy_web&vd_source=253f7e2d634ff4f728c7e7bfa218f990)
  -  使用手机扫描演示视频的二维码观看. 

  ![](./resource/GR5526_Smart_Watch_Video.png) 





