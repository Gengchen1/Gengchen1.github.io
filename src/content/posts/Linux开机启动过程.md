---
title: Linux开启启动过程
date: 2025-03-31
summary: 学习Linux系统从上电到用户界面发生了什么？
category: Linux
tags:
  - Linux
comments: true
---
今天这篇文章主要来分析，从摁下电源键，到用户界面，都发生了什么？

1. BIOS/UEFI启动:
   - 按下电源键后，主板向CPU发送复位信号，CPU重置所有寄存器并进入16位实模式（real mode）,其指令指针（cs:IP）被硬件强制指向`0xFFF0`地址，即BIOS程序的入口。
   - Real mode下CPU可直接访问1MB的内存空间，为BIOS执行提供基础环境。
   - BISO/UEFI执行任务如下：
     1. **POST自检**(Power-On Self Test) 检测关键硬件（CPU、内存、显卡、磁盘）是否正常工作，若出错则在屏幕上显示错误信息。
     2. **硬件初始化**：初始化中断向量表和中断服务程序，为后续操作提供基础服务（如磁盘读写中断）；
     3. **加载引导程序**。 
	 > BIOS和UEFI的区别是BIOS依赖于MBR(主引导记录)，最大只支持2TB磁盘，而UEFI支持更大的磁盘且具有快速安全启动功能。
2. 引导程序（bootloader）U-Boot、GRUB、RedBoot
   - 从磁盘中查找系统内核所在位置；
   - 将系统内核加载到内存中；
   - 启动内核代码；
3. 内核（kernel）
   - 检查硬件并加载设备驱动和其他内核模块
4. `init`初始进程启动（systemd）
>`systemd`是Linux上所有其他进程的父进程。
- 执行多项任务，确保系统启动；
- 检查需要加载驱动程序的剩余硬件；
- 挂载文件系统和磁盘，使其可访问；
- 启动所需的后台服务如：声音、电源、网络；
- 进入图形界面后，处理用户登录，并加载带有面板和菜单的桌面环境；
- `systemd`通过`basic.taeget`来决定启动多用户文本界面还是图形界面
>`systemd`负责所有初始化，并在linux启动时在后台运行。

   ![Linux启动过程](./attachments/Pasted%20image%2020250312170218.png)
   [【双语视界】按下电源键后，Linux是如何启动的？一步步揭秘!](https://www.bilibili.com/video/BV1fPRuYYECH/?share_source=copy_web&vd_source=aadea25fda118912d01970bc99de2d9b)

**总结：**
首先，BIOS或UEFI启动，初始化硬件（如键盘、屏幕、硬盘），UEFI比传统BIOS更快且支持更大磁盘（通过GPT分区表）。接着，POST检查硬件是否正常。随后，BIOS/UEFI加载引导程序（如Grub2），Grub2定位并加载Linux内核到内存。内核接管后解压自身，加载驱动和模块，启动init进程（现代系统用systemd）。Systemd负责挂载文件系统、启动服务（如网络、声音），最终加载图形界面。

BIOS/UEFI 启动初始化硬件，加载bootloader(GRUB) -> GRUB定位加载内核到内存-> kernel加载驱动，启动init进程 -> init进程（比如Systemd）挂载文件系统，启动服务，加载shell / 桌面.

