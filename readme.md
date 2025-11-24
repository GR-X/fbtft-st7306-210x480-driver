# ST7306 LCD 显示屏驱动 for Luckfox Pico Zero (RV1106G3)
## 代码简介
 本代码为鱼鹰光电2.9寸全反屏8色 液晶显示屏（分辨率 210x480）在 Luckfox Pico Zero (RV1106)​ 开发板上的 Linux 显示驱动。通过移植提供的示例到 FBTFT 框架实现，将 SPI 接口的屏幕注册为系统的 Framebuffer 设备（如 /dev/fb0），从而支持标准的 Framebuffer 操作。

## 核心组件
| 组件 | 型号/规格 | 备注 |
| :--- | :--- | :--- |
| 开发板 | Luckfox Pico Zero | 基于 Rockchip RV1106g3 |
| LCD 屏幕 | 鱼鹰光电2.9寸全反屏8色  ST7306 | 分辨率 210x480，SPI 接口 |

## 引脚连接对照表
| ST7306 引脚 | 功能 | Luckfox Pico Zero 引脚 | 说明 |
| :--- | :--- | :--- | :--- |
| `VCC` | 电源 (3.3V) | 3.3V | - |
| `GND` | 地 | GND | - |
| `SCL`/`CLK` | SPI 时钟 | SPI0_CLK | - |
| `SDA`/`MOSI` | SPI 数据输出 | SPI0_MOSI | - |
| `CS` | 片选 | SPI0_CS0 | - |
| `DC` (或 `A0`) | 数据/命令 | GPIO0_A3 | 在设备树中定义为 `dc-gpios` |
| `RST` | 复位 | GPIO0_A4 | 在设备树中定义为 `reset-gpios` |
| `TE` | 撕裂效应同步  | GPIO1_C7 | 在设备树中定义为 `te-gpios` |

## 为 ST7306 添加内核支持
### 一 修改 Kconfig 文件
找到并编辑 FBTFT 驱动目录下的 Kconfig文件，添加以下内容：
``` 
config FB_TFT_ST7306
	tristate "FB driver for the ST7306 LCD Controller"
	depends on FB_TFT
	help
	  Generic Framebuffer support for ST7306
```
### 二 修改 Makefile 文件
在同一目录下的 Makefile文件中添加编译规则:
```
obj-$(CONFIG_FB_TFT_ST7306)      += fb_st7306.o  
```
### 三 修改rv1106g-luckfox-pico-zero.dts
修改设备树,添加设备节点
```
&spi0 {
	status = "okay";
    pinctrl-names = "default";
    pinctrl-0 = <&spi0m0_pins>; // 确保此引脚组在pinctrl节点中已正确定义

    /* ！！！关键：必须禁用或删除默认的spidev节点，否则地址冲突 ！！！ */
    spidev@0 {
       status = "disabled";
    };

    fbtft@0 {
        status = "okay";
		compatible = "sitronix,st7306";  // 驱动名称，需内核支持
		reg = <0>;
		spi-max-frequency = <48000000>;
		spi-cpol;
		spi-cpha;
		//rotate = <90>;
		fps = <60>;
		rgb;
		buswidth = <8>;
		dc-gpios = <&gpio0 RK_PA3 GPIO_ACTIVE_HIGH>;		// DC 引脚
		reset-gpios = <&gpio0 RK_PA4 GPIO_ACTIVE_LOW>;		// RESET 引脚
		interrupt-parent = <&gpio1>;
		//interrupts = <RK_PC7 IRQ_TYPE_EDGE_RISING>; 
		te-gpios = <&gpio1 RK_PC7 GPIO_ACTIVE_HIGH>;
		// backlight = <&backlight>;  // 背光已取消，不使用
		width = <210>;
        height = <480>;
		debug = <0x7>;

    };
};

```
### 四 内核配置
1. 进入内核配置页面:
```
./build.sh kernelconfig  
```
2. 导航到驱动位置:
```
Device Drivers  --->
    Staging drivers  --->
        Support for small TFT LCD display modules  --->
            <*> FB driver for the ST7306 LCD Controller
```
3. 选择编译方式（*编译进内核，M编译为模块）

### 五 编译与测试
1. 编译打包固件根据luckfox官方说明编译烧录即可,这里不过多说明
2. 如果加载模块成功(如果不编入内核需要使用`insmod`安装`fb_st7306.ko`模块)则出现`/dev/fb0`

请根据你的屏幕对应的实际帧缓冲区设备节点（如 `/dev/fb0`)进行测试。以下是一些常用的测试命令：
| 测试命令 |预期效果| 说明 |
| :--- | :--- | :--- |
| `cat /dev/zero > /dev/fb0` | 	清屏为黑色​ | 向屏幕写入全零数据，通常使屏幕变黑，是最基本的测试|
| `cat /dev/urandom > /dev/fb0` | 随机噪点（花屏） |用随机数据填充屏幕，产生雪花噪点，用于快速检查每个像素点是否都能被单独寻址并正确显示颜色。|
| `dd if=/dev/zero of=/dev/fb0 bs=1024 count=100`| 	部分清屏​   | 使用 dd命令可以精确控制写入的数据量（bs块大小 * count块数），观察屏幕局部区域的变化。|
### 六 效果展示

<img src="./images/1.jpg" width="200">

### 七 其他说明
本人第一次尝试接触linux驱动开发..很多概念都还不是很清楚,估计很多错误...仅供参考--