/**
  @page AWS AWS Cloud application

  @verbatim
  ******************************************************************************
  * @file    readme.txt 
  * @author  MCD Application Team
  * @brief   Description of AWS Cloud application.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Application Description 


This application implements an Amazon AWS Cloud IoT client for the STM32F769I-Discovery board
using the on-board Ethernet interface driven by the LwIP TCP/IP stack,
and mbedTLS for secure network communication.

The application connects to Amazon AWS IoT Cloud with the credentials provided
by the user. When the User button is pushed, it sends a LED toggle command to the IoT Cloud
endpoint, which returns back the message to the board and trigs the LED toggle. 

Upon a double-push of the User button, the application exits.


@par Directory contents

---> in .

Inc
  ethernetif.h                       Ethernet interface header file.
  flash.h                            Management of the internal flash memory.
  FreeRTOSConfig.h                   FreeRTOS configuration.
  lwipopts.h                         LWIP configuration.
  main.h                             Header containing config parameters for the application and globals.
  stm32f7xx_hal_conf.h               HAL configuration file.
  stm32f7xx_it.h                     STM32 interrupt handlers header file.

Src
  ethernetif.c                       Ethernet network interface drivers for lwIP.
  flash_f7.c                         Flash programing interface.
  main.c                             Main application file.
  stm32f7xx_hal_msp.c                Specific initializations.
  stm32f7xx_hal_timebase_tim.c       Overriding of the native weak HAL time base functions
  stm32f7xx_it.c                     STM32 interrupt handlers.
  system_stm32f7xx.c                 System initialization.

---> in Projects/Common/AWS
Inc:
  aws_iot_config.h
  aws_mbedtls_config.h               Application-specific mbedTLS configuration. 
  network_platform.h
  timer_platform.h

Src:
  aws_network_st_wrapper.c               AWS SDK network porting layer. 
  aws_subscribe_publish_sensor_values.c  Sample application.
  aws_timer.c                            AWS SDK time porting layer. 

Verisign_Comodo_Plume.crt            List of root CA certificates to be pasted on the board console at first launch.
                                     AWS use Verisign; the RTC init over HTTPS uses Comodo.
                                     Plume must be replaced by the appropriate certificate if the user runs RFU
                                     over HTTPS.  
---> in Projects/Common/Shared
Inc
  cloud.h
  firewall.h
  firewall_wrapper.h
  heap.h
  http_util.h
  iot_flash_config.h
  mbedtls_net.h
  msg.h
  net.h
  net_internal.h
  rfu.h
  sensors_data.h
  timedate.h
  timingSystem.h
  version.h

Src
  cloud.c                             cloud application init and deinit functions
  entropy_hardware_poll.c             RNG entropy source for mbedTLS.
  firewall.c                          IoT Discovery Kit firewall init functions.
  firewall_wrapper.c                  IoT Discovery Kit wrapper to firewall protected functions.
  heap.c                              Heap check functions.
  http_util.c                         Helpers to build and execute HTTP requests.
  iot_flash_config.c                  Dialog and storage management utils for the user-configured settings.
  lwip_net.c                          LwIP on ethernet init/deinit functions for NET (STM32F769I-DISCO).
  mbedtls_net.c                       Network adpater for mbedTLS on NET.
  mbedtls_patch.c                     Stub to firewall protected functions.
  net.c                               Network socket API.
  net_tcp_lwip.c                      NET TCP / LwIP on ethernet implementation.
  net_tls_mbedtls.c                   NET TLS / mbedTLS implementation.
  rfu.c                               Firmware versioning and change management.
  sensors_data.c                      Manage sensors of STM32L475 IoT board.
  stackswitch.s
  STM32CubeRTCInterface.c             Libc time porting to the RTC.
  timedate.c                          Initialization of the RTC from the network.
  timingSystem.c                      Libc time porting to the RTC.
   
@par Hardware and Software environment
  - STM32F769I-Discovery board.

  - Ethernet router.
      * With a transparent Internet connectivity: No proxy, no firewall blocking the outgoing trafic.
      * Running a DHCP server delivering the IP and DNS configuration to the board.

  - Amazon AWS account. Security keys and endpoint for AWS IoT 
    service must be created.

    see http://docs.aws.amazon.com/iot/latest/developerguide/iot-gs.html

@par How to use it ? 

In order to make the program work, you must do the following:

 - WARNING: Before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

 - Open the IAR IDE and compile the project (see the release note for detailed 
   information about the version). Alternatively you can use the Keil uVision 
   toolchain (see the release note for detailed information about the version). 
   Alternatively you can use the System Workbench for STM32 (see the release note 
   for detailed information about the version). 

 - Program the firmware on the STM32 board: If you generated a raw binary file, 
   you can copy (or drag and drop) it from Projects\STM32F769I-Discovery\Applications\Cloud\AWS\EWARM\STM32F769I-Discovery\Exe
   to the USB mass storage location created when you plug the STM32 
   board to your PC. If the host is a Linux PC, the STM32 device can be found in 
   the /media folder with the name e.g. "DIS_L4IOT". For example, if the created mass 
   storage location is "/media/DIS_L4IOT", then the command to program the board 
   with a binary file named "my_firmware.bin" is simply: cp my_firmware.bin 
   /media/DIS_L4IOT. 

   Alternatively, you can program the STM32 board directly through one of the 
   supported development toolchains, or thanks to the STM32 ST-Link Utility.
  
 - Configure the required settings (to be done only once): 
   - When the board is connected to a PC with USB (ST-LINK USB port), 
     open a serial terminal emulator, find the board's COM port and configure it with:
      - 8N1, 115200 bauds, no HW flow control
      - set the line endings to LF or CR-LF (Transmit) and LF (receive).

   - At first boot, enter the required parameters:

     - Enter the TLS network security keys needed for Amazon AWS connection (root CA key, 
       device certificate, private key).

       In order to use HTTPS for initializing the RTC, the root CA certificate of the target 
       server must be provided together (in the same copy-paste) with the Amazon root CA certificate.
       You can use the provided file: Verisign_Comodo_Plume.crt.
      
     - Enter the Amazon AWS IoT end-point (Amazon server address) and device ID.

   - After the parameters are configured, it is possible to change them by restarting the board 
     and pushing User button (blue button) when prompted during the initialization.

@par Connectivity test

  - By default the AWS project tries to automatically connect to Amazon AWS IoT cloud 
    (using the parameters set above). 

  - Push the User button (blue button) to publish the LED desired value. A message will be sent back
    by AWS IoT cloud and make the LED toggle.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
