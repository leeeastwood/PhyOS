-LPC54018是AWS支持的另一个module，根据AWS官方教程，跟着教程能跑出与STM32L475E相同的程序。

-根据教程的指引下载了nxp的IDE和SDK，其中nxp也是基于eclipse的，SDK是在MCUXpresso官方网站上下载的。

-LPC54018是通过Jtag下载程序的，板载USB只是USB而已。由于需要Jtag的20针转10针模块，程序未能烧入到板子里。

-下载AWS的demo源码后可知其依旧是MQTT的程序，程序内有wifi的驱动，及一些其他板级支持包，依旧没有httpserver。

-mongoose的两个源文件内需要引入lwip，进而实现webserver。 nxpSDK中有lwip的支持，只是在这个demo中没有引入，通过IDE将其引入后
（引入方式：项目上邮件，在下方找到“Manage SDK Components”，点选后，在最右侧的middleware-Network中有lwip的包），
重新编译demo却发现，wifi_qca的头文件明明存在却不能被引用并编译，没有引入lwip之前是能够正常编译的。由此可以想到lwip与wifi_qca的文件有冲突。
经过排查发现两个文件夹下有netbuf.h两个相同名称的文件。下一步准备看看如何将这两个文件分离。

-在IDE中找到了SDK自带的demo，其中有一个用lwip实现的httpserver，但是不能走wifi，而是需要base板上的RJ45接口。

-总之，AWS实现的wifi驱动与lwip有冲突，使得mongoose移植失败。
