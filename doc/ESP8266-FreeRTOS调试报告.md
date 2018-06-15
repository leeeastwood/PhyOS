## 基本情况 ##
- **运行环境**：Ubuntu 16.04 64位
- **cp2102Driver**[https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers]("驱动下载地址")
- **rtos工程**[https://github.com/SuperHouse/esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos "github/esp-open-rtos")
- **开发板**ESP8266-13E带cp2102串口
- **代码量**看图片![](https://github.com/leeeastwood/PhyOS/blob/master/picture/391550267022807809.jpg)

## 准备工作 ##
1. 安装[https://github.com/pfalcon/esp-open-sdk/](https://github.com/pfalcon/esp-open-sdk/ "esp-open-sdk")
	1. 安装依赖
	```bash
	$ sudo apt-get install make unrar-free autoconf automake libtool gcc g++ gperf \
    flex bison texinfo gawk ncurses-dev libexpat-dev python-dev python python-serial \
    sed git unzip bash help2man wget bzip2
	$ sudo apt-get install libtool-bin
	```
	**坑：**ncurses-dev和libexpat-dev安装提示错误，并推荐了两个名称类似的包，安装即可。
	2. 下载
	$ git clone --recursive https://github.com/pfalcon/esp-open-sdk.git
	3. 安装
	进入esp-open-sdk用make命令安装即可
	**坑：**make的时候它还会在其他站点下载依赖，由于那个站点不稳定，无法下载或下载出错，重复好几次才能正常安装。解决方法：
> A way to fix this issue is this pull request: jcmvbkbc/crosstool-NG#53
This also fixes #307
It boils down to update the URL template in crosstool-NG/scripts/build/companion_libs/210-expat.sh from
http://downloads.sourceforge.net/project/expat/expat/${CT_EXPAT_VERSION}
to
https://github.com/libexpat/libexpat/releases/download/R_${CT_EXPAT_VERSION//[.]/_}
2. esp-open-rtos让用这个`make toolchain esptool libhal STANDALONE=n`命令安装sdk，我是用普通make安装的又用那个走了一遍。
3. 配置环境变量`export PATH=/home/ubuntu/esp-open-sdk/xtensa-lx106-elf/bin:$PATH`
**坑：**需要进入su用户再配置一遍，因为如果通过串口下载程序需要用root用户
4. 安装esptool.py[https://github.com/espressif/esptool](https://github.com/espressif/esptool)
	`$ pip install esptool`
**坑：**安装sdk的时候已经安装过一遍， 同上需要在root用户下再安装一遍
5. 下载
    `git clone --recursive https://github.com/Superhouse/esp-open-rtos.git`
6. 准备头文件
> To build any examples that use WiFi, create include/private_ssid_config.h defining the two macro defines:
`#define WIFI_SSID "mywifissid"`
`#define WIFI_PASS "my secret password"`

	**坑：**路由器看不到当前成员，需要用串口的log查看分配到的ip地址。
7. 编译sample
    `make flash -j4 -C examples/http_server ESPPORT=/dev/ttyUSB0`
-j4是并行的意思好像。flash是编译并烧录的意思。在root用户下不需要按住flash按钮
**坑：**会出现找不到ttyUSB0的情况（查看dev文件下明明存在），重新拔插即可
8. http-server 
**坑：**http-server.c文件的最后建立任务时`xTaskCreate(&httpd_task, "HTTP Daemon", 128, NULL, 2, NULL);`分配的128b空间太小，改为384可以运行。最坑的是这个错误在查看通过串口的log才发现。
该示例运行起来加载的比较慢。