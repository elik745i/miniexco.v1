# ESP32S3

#### 介绍
ESP32 S3 音视频开发板测试例子 QQ群 417461652

#### 软件架构
软件架构说明

#### 使用说明

1.  本次例程需要一张SD卡，并将xtd.wav文件拷贝到SD里面，方便测试NS4168功放
2.  menuconfig 配置

1️⃣ flash ：
    Flash SPI mode：DIO
    Flash SPI speed: 80MHz
    Flash size:16M
2️⃣ PSRAM :
    Mode (QUAD/OCT) of SPI RAM chip in use ：Quad Mode PSRAM
    Set RAM clock speed:80MHz
3️⃣ Partition Table：
    Partition Table：Custom partitition table CSV

3.  IDF 运行版本5.1

### 例程运行日记

I (27) boot: ESP-IDF v5.1.2-185-g3662303f31-dirty 2nd stage bootloader
I (27) boot: compile time Mar 25 2024 09:16:43
I (28) boot: Multicore bootloader
I (32) boot: chip revision: v0.2
I (36) boot.esp32s3: Boot SPI Speed : 80MHz
I (40) boot.esp32s3: SPI Mode       : DIO
I (45) boot.esp32s3: SPI Flash Size : 16MB
I (50) boot: Enabling RNG early entropy source...
I (55) boot: Partition Table:
I (59) boot: ## Label            Usage          Type ST Offset   Length
I (66) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (74) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (81) boot:  2 factory          factory app      00 00 00010000 00c00000
I (89) boot: End of partition table
I (93) esp_image: segment 0: paddr=00010020 vaddr=3c0a0020 size=27388h (160648) map
I (130) esp_image: segment 1: paddr=000373b0 vaddr=3fc99400 size=0574ch ( 22348) load
I (135) esp_image: segment 2: paddr=0003cb04 vaddr=40374000 size=03514h ( 13588) load
I (139) esp_image: segment 3: paddr=00040020 vaddr=42000020 size=964a8h (615592) map
I (255) esp_image: segment 4: paddr=000d64d0 vaddr=40377514 size=11e1ch ( 73244) load
I (280) boot: Loaded app from partition at offset 0x10000
I (281) boot: Disabling RNG early entropy source...
I (292) cpu_start: Multicore app
I (292) esp_psram: Found 8MB PSRAM device
I (292) esp_psram: Speed: 40MHz
I (293) cpu_start: Pro cpu up.
I (297) cpu_start: Starting app cpu, entry point is 0x40375670
0x40375670: call_start_cpu1 at F:/ESP-IDF/510/esp-idf_v5.1/esp-idf/components/esp_system/port/cpu_start.c:157

I (0) cpu_start: App cpu up.
I (1978) esp_psram: SPI SRAM memory test OK
I (1987) cpu_start: Pro cpu start user code
I (1987) cpu_start: cpu freq: 160000000 Hz
I (1987) cpu_start: Application information:
I (1991) cpu_start: Project name:     ESP32-S3-SPK
I (1996) cpu_start: App version:      1
I (2001) cpu_start: Compile time:     Mar 25 2024 09:16:18
I (2007) cpu_start: ELF file SHA256:  272005302c946877...
I (2013) cpu_start: ESP-IDF:          v5.1.2-185-g3662303f31-dirty
I (2020) cpu_start: Min chip rev:     v0.0
I (2024) cpu_start: Max chip rev:     v0.99 
I (2029) cpu_start: Chip rev:         v0.2
I (2034) heap_init: Initializing. RAM available for dynamic allocation:
I (2042) heap_init: At 3FCA4460 len 000452B0 (276 KiB): DRAM
I (2048) heap_init: At 3FCE9710 len 00005724 (21 KiB): STACK/DRAM
I (2055) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (2061) heap_init: At 600FE010 len 00001FD8 (7 KiB): RTCRAM
I (2067) esp_psram: Adding pool of 8192K of PSRAM memory to heap allocator
I (2076) spi_flash: detected chip: gd
I (2079) spi_flash: flash io: dio
I (2083) sleep: Configure to isolate all GPIO pins in sleep state
I (2090) sleep: Enable automatic switching of GPIO sleep configuration
I (2097) app_start: Starting scheduler on CPU0
I (2102) app_start: Starting scheduler on CPU1
I (2102) main_task: Started on CPU0
I (2112) esp_psram: Reserving pool of 32K of internal memory for DMA/internal allocations
I (2122) main_task: Calling app_main()
I (2152) pp: pp rom version: e7ae62f
I (2152) net80211: net80211 rom version: e7ae62f
I (2162) wifi:wifi driver task: 3fcb675c, prio:23, stack:6656, core=0
I (2172) wifi:wifi firmware version: 91b9630
I (2172) wifi:wifi certification version: v7.0
I (2172) wifi:config NVS flash: enabled
I (2172) wifi:config nano formating: disabled
I (2172) wifi:Init data frame dynamic rx buffer num: 32
I (2182) wifi:Init static rx mgmt buffer num: 5
I (2182) wifi:Init management short buffer num: 32
I (2192) wifi:Init static tx buffer num: 16
I (2192) wifi:Init tx cache buffer num: 32
I (2202) wifi:Init static tx FG buffer num: 2
I (2202) wifi:Init static rx buffer size: 1600
I (2202) wifi:Init static rx buffer num: 10
I (2212) wifi:Init dynamic rx buffer num: 32
I (2212) wifi_init: rx ba win: 6
I (2222) wifi_init: tcpip mbox: 32
I (2222) wifi_init: udp mbox: 6
I (2222) wifi_init: tcp mbox: 6
I (2232) wifi_init: tcp tx win: 5760
I (2232) wifi_init: tcp rx win: 5760
I (2242) wifi_init: tcp mss: 1440
I (2242) wifi_init: WiFi IRAM OP enabled
I (2242) wifi_init: WiFi RX IRAM OP enabled
I (2252) phy_init: phy_version 620,ec7ec30,Sep  5 2023,13:49:13
I (2292) wifi:mode : null
I (2292) wifi:mode : softAP (ec:da:3b:5b:5d:f9)
I (2302) wifi:Total power save buffer number: 8
I (2302) wifi:Init max length of beacon: 752/752
I (2302) wifi:Init max length of beacon: 752/752
I (2302) esp_netif_lwip: DHCP server started on interface WIFI_AP_DEF with IP: 192.168.4.1
I (2842) wifi:Total power save buffer number: 8
I (2842) WIFI message: wifi_init_softap finished. SSID:ESP32-S3 PASSWORD:12345678 CHANNEL:1
I (2852) esp_netif_lwip: DHCP server started on interface WIFI_AP_DEF with IP: 192.168.4.1
I (2862) s3 ll_cam: DMA Channel=4
I (2862) cam_hal: cam init ok
I (2862) sccb: pin_sda 37 pin_scl 36
I (2872) sccb: sccb_i2c_port=1
I (2882) camera: Detected camera at address=0x30
I (2882) camera: Detected OV2640 camera
I (2882) camera: Camera PID=0x26 VER=0x42 MIDL=0x7f MIDH=0xa2
I (2972) cam_hal: buffer_size: 16384, half_buffer_size: 1024, node_buffer_size: 1024, node_cnt: 16, total_cnt: 60
I (2972) cam_hal: Allocating 61440 Byte frame buffer in PSRAM
I (2982) cam_hal: Allocating 61440 Byte frame buffer in PSRAM
I (2982) cam_hal: cam config ok
I (2992) ov2640: Set PLL: clk_2x: 0, clk_div: 0, pclk_auto: 0, pclk_div: 8
I (3072) main: Camera Init OK.
I (3072) main_task: Returned from app_main()