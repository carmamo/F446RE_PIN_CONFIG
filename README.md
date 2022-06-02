# F446RE_PIN_CONFIG
## 02/06/

[PDF with Images](F446RE_PIN_CONFIG.pdf)

# 1. Description

1.1. Project

```
Project Name F446RE_PIN_CONFIG
Board Name custom
Generated with: STM32CubeMX 6.5.
Date 06/02/
```
## 1.2. MCU

```
MCU Series STM32F
MCU Line STM32F
MCU name STM32F446RETx
MCU Package LQFP
MCU Pin number 64
```
1.3. Core(s) information

```
Core(s) Arm Cortex-M
```

```
Configuration Report
```
# 2. Pinout Configuration


```
Configuration Report
```
# 3. Pins Configuration

```
Pin Number
LQFP
```
```
Pin Name
(function after
reset)
```
```
Pin Type Alternate
Function(s)
```
```
Label
```
```
1 VBAT Power
5 PH0-OSC_IN I/O RCC_OSC_IN
6 PH1-OSC_OUT I/O RCC_OSC_OUT
7 NRST Reset
12 VSSA Power
13 VDDA Power
18 VSS Power
19 VDD Power
21 PA5 I/O SPI1_SCK
22 PA6 I/O SPI1_MISO
23 PA7 I/O SPI1_MOSI
30 VCAP_1 Power
31 VSS Power
32 VDD Power
33 PB12 I/O I2S2_WS
34 PB13 I/O I2S2_CK
36 PB15 I/O I2S2_SD
37 PC6 I/O I2S2_MCK
39 PC8 I/O SDIO_D
40 PC9 I/O SDIO_D
41 PA8 * I/O GPIO_Input SDIO_BSD_Input
44 PA11 I/O USB_OTG_FS_DM
45 PA12 I/O USB_OTG_FS_DP
46 PA13 I/O SYS_JTMS-SWDIO
47 VSS Power
48 VDD Power
49 PA14 I/O SYS_JTCK-SWCLK
51 PC10 I/O SDIO_D
52 PC11 I/O SDIO_D
53 PC12 I/O SDIO_CK
54 PD2 I/O SDIO_CMD
55 PB3 I/O SYS_JTDO-SWO
58 PB6 I/O I2C1_SCL
59 PB7 I/O I2C1_SDA
60 BOOT0 Boot
63 VSS Power
```

```
Configuration Report
```
```
Pin Number
LQFP
```
```
Pin Name
(function after
reset)
```
```
Pin Type Alternate
Function(s)
```
```
Label
```
```
64 VDD Power
```
* The pin is affected with an I/O function


```
Configuration Report
```
# 4. Clock Tree Configuration


```
Configuration Report
```
# 5. Software Project

5.1. Project Settings

```
Name Value
Project Name F446RE_PIN_CONFIG
Project Folder /home/carlos/STM32CubeIDE/workspace_1.9.0/F446RE_PIN_CONFIG
Toolchain / IDE STM32CubeIDE
Firmware Package Name and Version STM32Cube FW_F4 V1.27.
Application Structure Advanced
Generate Under Root Yes
Do not generate the main() No
Minimum Heap Size 0x
Minimum Stack Size 0x
```
5.2. Code Generation Settings

```
Name Value
STM32Cube MCU packages and embedded software Copy only the necessary library files
Generate peripheral initialization as a pair of '.c/.h' files No
Backup previously generated files when re-generating No
Keep User Code when re-generating Yes
Delete previously generated files when not re-generated Yes
Set all free pins as analog (to optimize the power
consumption)
```
```
No
Enable Full Assert No
```
5.3. Advanced Settings - Generated Function Calls

```
Rank Function Name Peripheral Instance Name
1 SystemClock_Config RCC
2 MX_GPIO_Init GPIO
3 MX_SDIO_SD_Init SDIO
4 MX_FATFS_Init FATFS
5 MX_I2C1_Init I2C
6 MX_I2S2_Init I2S
7 MX_SPI1_Init SPI
8 MX_USB_DEVICE_Init USB_DEVICE
```

```
Configuration Report
```
# 6. Power Consumption Calculator report

6.1. Microcontroller Selection

```
Series STM32F
Line STM32F
MCU STM32F446RETx
Datasheet DS10693_Rev
```
6.2. Parameter Selection

```
Temperature 25
Vdd 3.
```
6.3. Battery Selection

```
Battery Li-SOCL2(A3400)
Capacity 3400.0 mAh
Self Discharge 0.08 %/month
Nominal Voltage 3.6 V
Max Cont Current 100.0 mA
Max Pulse Current 200.0 mA
Cells in series 1
Cells in parallel 1
```

```
Configuration Report
```
6.4. Sequence

```
Step Step1 Step
Mode RUN STOP_UDM (Under Drive)
Vdd 3.3 3.
Voltage Source Battery Battery
Range Scale1-High No Scale
Fetch Type RAM/FLASH/REGON/ART/P
REFETCH
```
```
n/a
```
```
CPU Frequency 180 MHz 0 Hz
Clock Configuration HSE PLL Regulator_LP Flash-PwrDwn
Clock Source Frequency 4 MHz 0 Hz
Peripherals
Additional Cons. 0 mA 0 mA
Average Current 46 mA 55 μA
Duration 0.1 ms 0.9 ms
DMIPS 225.0 0.
Ta Max 98.02 104.
Category In DS Table In DS Table
```
6.5. Results

```
Sequence Time 1 ms Average Current 4.65 mA
Battery Life 1 month Average DMIPS 225.0 DMIPS
```
6.6. Chart


Configuration Report


```
Configuration Report
```
# 7. Peripherals and Middlewares Configuration

## 7.1. I2C

## I2C: I2C

7.1.1. Parameter Settings:

```
Master Features:
I2C Speed Mode Standard Mode
I2C Clock Speed (Hz) 100000
Slave Features:
Clock No Stretch Mode Disabled
Primary Address Length selection 7-bit
Dual Address Acknowledged Disabled
Primary slave address 0
General Call address detection Disabled
```
## 7.2. I2S

Mode: Half-Duplex Master
mode: Master Clock Output
7.2.1. Parameter Settings:

```
Generic Parameters:
Transmission Mode Mode Master Receive *
Communication Standard I2S Philips
Data and Frame Format 16 Bits Data on 16 Bits Frame
Selected Audio Frequency 48 KHz *
Real Audio Frequency 48.828 KHz *
Error between Selected and Real 1.72 % *
Clock Parameters:
Clock Source I2S PLL Clock
Clock Polarity Low
```
## 7.3. RCC

High Speed Clock (HSE): Crystal/Ceramic Resonator
7.3.1. Parameter Settings:


```
Configuration Report
```
```
System Parameters:
VDD voltage (V) 3.
Instruction Cache Enabled
Prefetch Buffer Enabled
Data Cache Enabled
Flash Latency(WS) 5 WS (6 CPU cycle)
RCC Parameters:
HSI Calibration Value 16
TIM Prescaler Selection Disabled
HSE Startup Timout Value (ms) 100
LSE Startup Timout Value (ms) 5000
Power Parameters:
Power Regulator Voltage Scale Power Regulator Voltage Scale 1
Power Over Drive Disabled
```
## 7.4. SDIO

Mode: SD 4 bits Wide bus
7.4.1. Parameter Settings:

```
SDIO parameters:
Clock transition on which the bit capture is made Rising transition
SDIO Clock divider bypass Disable
SDIO Clock output enable when the bus is idle Disable the power save for the clock
SDIO hardware flow control The hardware control flow is disabled
SDIOCLK clock divide factor 0
```
## 7.5. SPI

Mode: Full-Duplex Master
7.5.1. Parameter Settings:

```
Basic Parameters:
Frame Format Motorola
Data Size 8 Bits
First Bit MSB First
Clock Parameters:
Prescaler (for Baud Rate) 2
```

```
Configuration Report
```
```
Baud Rate 42.0 MBits/s *
Clock Polarity (CPOL) Low
Clock Phase (CPHA) 1 Edge
Advanced Parameters:
CRC Calculation Disabled
NSS Signal Type Software
```
## 7.6. SYS

Debug: Trace Asynchronous Sw
Timebase Source: TIM

7.7. USB_OTG_FS
Mode: Device_Only
7.7.1. Parameter Settings:

```
Speed Device Full Speed 12MBit/s
Low power Disabled
Link Power Management Disabled
VBUS sensing Disabled
Signal start of frame Disabled
```
## 7.8. FATFS

mode: SD Card
7.8.1. Set Defines:

```
Version:
FATFS version R0.12c
Function Parameters:
FS_READONLY (Read-only mode) Disabled
FS_MINIMIZE (Minimization level) Disabled
USE_STRFUNC (String functions) Enabled with LF -> CRLF conversion
USE_FIND (Find functions) Disabled
USE_MKFS (Make filesystem function) Enabled
USE_FASTSEEK (Fast seek function) Enabled
USE_EXPAND (Use f_expand function) Disabled
USE_CHMOD (Change attributes function) Disabled
USE_LABEL (Volume label functions) Disabled
```

```
Configuration Report
```
```
USE_FORWARD (Forward function) Disabled
Locale and Namespace Parameters:
CODE_PAGE (Code page on target) Latin 1
USE_LFN (Use Long Filename) Disabled
MAX_LFN (Max Long Filename) 255
LFN_UNICODE (Enable Unicode) ANSI/OEM
STRF_ENCODE (Character encoding) UTF-
FS_RPATH (Relative Path) Disabled
Physical Drive Parameters:
VOLUMES (Logical drives) 1
MAX_SS (Maximum Sector Size) 4096 *
MIN_SS (Minimum Sector Size) 512
MULTI_PARTITION (Volume partitions feature) Disabled
USE_TRIM (Erase feature) Disabled
FS_NOFSINFO (Force full FAT scan) 0
System Parameters:
FS_TINY (Tiny mode) Disabled
FS_EXFAT (Support of exFAT file system) Disabled
FS_NORTC (Timestamp feature) Dynamic timestamp
FS_REENTRANT (Re-Entrancy) Enabled
FS_TIMEOUT (Timeout ticks) 1000
USE_MUTEX Disabled
SYNC_t (O/S sync object) osSemaphoreId_t
FS_LOCK (Number of files opened simultaneously) 2
```
7.8.2. Advanced Settings:

```
SDIO/SDMMC:
SDIO instance SDIO
Use dma template Enabled
BSP code for SD Generic
```
7.8.3. Platform Settings:

```
Detect_SDIO PA
```
## 7.9. FREERTOS

Interface: CMSIS_V


```
Configuration Report
```
7.9.1. Config parameters:

```
API:
FreeRTOS API CMSIS v
Versions:
FreeRTOS version 10.3.
CMSIS-RTOS version 2.
MPU/FPU:
ENABLE_MPU Disabled
ENABLE_FPU Disabled
Kernel settings:
USE_PREEMPTION Enabled
CPU_CLOCK_HZ SystemCoreClock
TICK_RATE_HZ 1000
MAX_PRIORITIES 56
MINIMAL_STACK_SIZE 128
MAX_TASK_NAME_LEN 16
USE_16_BIT_TICKS Disabled
IDLE_SHOULD_YIELD Enabled
USE_MUTEXES Enabled
USE_RECURSIVE_MUTEXES Enabled
USE_COUNTING_SEMAPHORES Enabled
QUEUE_REGISTRY_SIZE 8
USE_APPLICATION_TASK_TAG Disabled
ENABLE_BACKWARD_COMPATIBILITY Enabled
USE_PORT_OPTIMISED_TASK_SELECTION Disabled
USE_TICKLESS_IDLE Disabled
USE_TASK_NOTIFICATIONS Enabled
RECORD_STACK_HIGH_ADDRESS Disabled
Memory management settings:
Memory Allocation Dynamic / Static
TOTAL_HEAP_SIZE 15360
Memory Management scheme heap_
Hook function related definitions:
USE_IDLE_HOOK Disabled
USE_TICK_HOOK Disabled
USE_MALLOC_FAILED_HOOK Disabled
USE_DAEMON_TASK_STARTUP_HOOK Disabled
CHECK_FOR_STACK_OVERFLOW Disabled
Run time and task stats gathering related definitions:
GENERATE_RUN_TIME_STATS Disabled
```

```
Configuration Report
```
```
USE_TRACE_FACILITY Enabled
USE_STATS_FORMATTING_FUNCTIONS Disabled
Co-routine related definitions:
USE_CO_ROUTINES Disabled
MAX_CO_ROUTINE_PRIORITIES 2
Software timer definitions:
USE_TIMERS Enabled
TIMER_TASK_PRIORITY 2
TIMER_QUEUE_LENGTH 10
TIMER_TASK_STACK_DEPTH 256
Interrupt nesting behaviour configuration:
LIBRARY_LOWEST_INTERRUPT_PRIORITY 15
LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
Added with 10.2.1 support:
MESSAGE_BUFFER_LENGTH_TYPE size_t
USE_POSIX_ERRNO Disabled
CMSIS-RTOS V2 flags:
USE_OS2_THREAD_SUSPEND_RESUME Enabled
USE_OS2_THREAD_ENUMERATE Enabled
USE_OS2_EVENTFLAGS_FROM_ISR Enabled
USE_OS2_THREAD_FLAGS Enabled
USE_OS2_TIMER Enabled
USE_OS2_MUTEX Enabled
```
7.9.2. Include parameters:

```
Include definitions:
vTaskPrioritySet Enabled
uxTaskPriorityGet Enabled
vTaskDelete Enabled
vTaskCleanUpResources Disabled
vTaskSuspend Enabled
vTaskDelayUntil Enabled
vTaskDelay Enabled
xTaskGetSchedulerState Enabled
xTaskResumeFromISR Enabled
xQueueGetMutexHolder Enabled
xSemaphoreGetMutexHolder Disabled
pcTaskGetTaskName Disabled
uxTaskGetStackHighWaterMark Enabled
xTaskGetCurrentTaskHandle Enabled
```

```
Configuration Report
```
```
eTaskGetState Enabled
xEventGroupSetBitFromISR Disabled
xTimerPendFunctionCall Enabled
xTaskAbortDelay Disabled
xTaskGetHandle Disabled
uxTaskGetStackHighWaterMark2 Disabled
```
7.9.3. Advanced settings:

```
Newlib settings (see parameter description first):
USE_NEWLIB_REENTRANT Enabled *
Project settings (see parameter description first):
Use FW pack heap file Enabled
```
## 7.10. USB_DEVICE

Class For FS IP: Communication Device Class (Virtual Port Com)
7.10.1. Parameter Settings:

```
Basic Parameters:
USBD_MAX_NUM_INTERFACES (Maximum number of supported interfaces) 1
USBD_MAX_NUM_CONFIGURATION (Maximum number of supported configuration) 1
USBD_MAX_STR_DESC_SIZ (Maximum size for the string descriptors) 512
USBD_SELF_POWERED (Enabled self power) Enabled
USBD_DEBUG_LEVEL (USBD Debug Level) 0: No debug message
USBD_LPM_ENABLED (Link Power Management) 1: Link Power Management supported
Class Parameters:
USB CDC Rx Buffer Size 2048
USB CDC Tx Buffer Size 2048
```
7.10.2. Device Descriptor:

```
Device Descriptor:
VID (Vendor IDentifier) 1155
LANGID_STRING (Language Identifier) English(United States)
MANUFACTURER_STRING (Manufacturer Identifier) STMicroelectronics
Device Descriptor FS:
PID (Product IDentifier) 22336
PRODUCT_STRING (Product Identifier) STM32 Virtual ComPort
```

```
Configuration Report
```
```
CONFIGURATION_STRING (Configuration Identifier) CDC Config
INTERFACE_STRING (Interface Identifier) CDC Interface
```
* User modified value


```
Configuration Report
```
# 8. System Configuration

```
8.1. GPIO configuration
```
```
IP Pin Signal GPIO mode GPIO pull/up pull
down
```
```
Max
Speed
```
```
User Label
```
```
I2C1 PB6 I2C1_SCL Alternate Function Open
Drain
```
```
No pull-up and no pull-down Very High
*
PB7 I2C1_SDA Alternate Function Open
Drain
```
```
No pull-up and no pull-down Very High
*
I2S2 PB12 I2S2_WS Alternate Function Push Pull No pull-up and no pull-down Low
PB13 I2S2_CK Alternate Function Push Pull No pull-up and no pull-down Low
PB15 I2S2_SD Alternate Function Push Pull No pull-up and no pull-down Low
PC6 I2S2_MCK Alternate Function Push Pull No pull-up and no pull-down Low
RCC PH0-
OSC_IN
```
```
RCC_OSC_IN n/a n/a n/a
PH1-
OSC_OUT
```
```
RCC_OSC_OUT n/a n/a n/a
SDIO PC8 SDIO_D0 Alternate Function Push Pull No pull-up and no pull-down Very High
PC9 SDIO_D1 Alternate Function Push Pull No pull-up and no pull-down Very High
PC10 SDIO_D2 Alternate Function Push Pull No pull-up and no pull-down Very High
PC11 SDIO_D3 Alternate Function Push Pull No pull-up and no pull-down Very High
PC12 SDIO_CK Alternate Function Push Pull No pull-up and no pull-down Very High
PD2 SDIO_CMD Alternate Function Push Pull No pull-up and no pull-down Very High
SPI1 PA5 SPI1_SCK Alternate Function Push Pull No pull-up and no pull-down Very High
*
PA6 SPI1_MISO Alternate Function Push Pull No pull-up and no pull-down Very High
*
PA7 SPI1_MOSI Alternate Function Push Pull No pull-up and no pull-down Very High
*
SYS PA13 SYS_JTMS-
SWDIO
```
```
n/a n/a n/a
PA14 SYS_JTCK-
SWCLK
```
```
n/a n/a n/a
PB3 SYS_JTDO-
SWO
```
```
n/a n/a n/a
```
USB_OTG_
FS

```
PA11 USB_OTG_FS_
DM
```
```
Alternate Function Push Pull No pull-up and no pull-down Very High
*
PA12 USB_OTG_FS_
DP
```
```
Alternate Function Push Pull No pull-up and no pull-down Very High
*
GPIO PA8 GPIO_Input Input mode No pull-up and no pull-down n/a SDIO_BSD_Input
```

```
Configuration Report
```
8.2. DMA configuration
nothing configured in DMA service


```
Configuration Report
```
8.3. NVIC configuration
8.3.1. NVIC

```
Interrupt Table Enable Preenmption Priority SubPriority
Non maskable interrupt true 0 0
Hard fault interrupt true 0 0
Memory management fault true 0 0
Pre-fetch fault, memory access fault true 0 0
Undefined instruction or illegal state true 0 0
System service call via SWI instruction true 0 0
Debug monitor true 0 0
Pendable request for system service true 15 0
System tick timer true 15 0
TIM6 global interrupt and DAC1, DAC2
underrun error interrupts
```
```
true 15 0
USB On The Go FS global interrupt true 5 0
PVD interrupt through EXTI line 16 unused
Flash global interrupt unused
RCC global interrupt unused
I2C1 event interrupt unused
I2C1 error interrupt unused
SPI1 global interrupt unused
SPI2 global interrupt unused
SDIO global interrupt unused
FPU global interrupt unused
```
8.3.2. NVIC Code generation

```
Enabled interrupt Table Select for init
sequence ordering
```
```
Generate IRQ
handler
```
```
Call HAL handler
```
```
Non maskable interrupt false true false
Hard fault interrupt false true false
Memory management fault false true false
Pre-fetch fault, memory access fault false true false
Undefined instruction or illegal state false true false
System service call via SWI instruction false false false
Debug monitor false true false
Pendable request for system service false false false
System tick timer false false true
TIM6 global interrupt and DAC1, DAC2
underrun error interrupts
```
```
false true true
USB On The Go FS global interrupt false true true
```

```
Configuration Report
```
* User modified value


```
Configuration Report
```
# 9. System Views

9.1. Category view
9.1.1. Current


```
Configuration Report
```
# 10. Docs & Resources

```
Type Link
Presentations https://www.st.com/resource/en/product_presentation/stm32-
stm8_embedded_software_solutions.pdf
Presentations https://www.st.com/resource/en/product_presentation/stm32_eval-
tools_portfolio.pdf
Presentations https://www.st.com/resource/en/product_presentation/stm32_stm8_functi
onal-safety-packages.pdf
Presentations https://www.st.com/resource/en/product_presentation/stm32-
stm8_software_development_tools.pdf
Training Material https://www.st.com/resource/en/sales_guide/sg_sc2154.pdf
Flyers https://www.st.com/resource/en/flyer/flnucleolrwan.pdf
Flyers https://www.st.com/resource/en/flyer/flstm32nucleo.pdf
Flyers https://www.st.com/resource/en/flyer/flstmcsuite.pdf
Flyers https://www.st.com/resource/en/flyer/flstm32trust.pdf
Product
Certifications
```
```
https://www.st.com/resource/en/certification_document/stm32_authenticat
ion_can.pdf
Application Notes https://www.st.com/resource/en/application_note/an1181-electrostatic-
discharge-sensitivity-measurement-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an1709-emc-design-
guide-for-stm8-stm32-and-legacy-mcus-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an2606-stm32-
microcontroller-system-memory-boot-mode-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an2639-soldering-
recommendations-and-package-information-for-leadfree-ecopack-mcus-
and-mpus-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an2834-how-to-get-the-
best-adc-accuracy-in-stm32-microcontrollers-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an2867-oscillator-
design-guide-for-stm8afals-stm32-mcus-and-mpus-stmicroelectronics.pdf
Application Notes https://www.st.com/resource/en/application_note/an2945-stm8s-and-
```

```
Configuration Report
```
```
stm32-mcus-a-consistent-832bit-product-line-for-painless-migration-
stmicroelectronics.pdf
```
Application Notes https://www.st.com/resource/en/application_note/an3070-managing-the-
driver-enable-signal-for-rs485-and-iolink-communications-with-the-
stm32s-usart-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3126-audio-and-
waveform-generation-using-the-dac-in-stm32-products-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3154-can-protocol-
used-in-the-stm32-bootloader-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3155-usart-protocol-
used-in-the-stm32-bootloader-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3156-usb-dfu-
protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3364-migration-and-
compatibility-guidelines-for-stm32-microcontroller-applications-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3997-audio-playback-
and-recording-using-the-stm32f4discovery-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an3998-pdm-audio-
software-decoding-on-stm32-microcontrollers-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4013-stm32-
crossseries-timer-overview-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4031-using-the-
stm32f2-stm32f4-and-stm32f7-series-dma-controller-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4221-i2c-protocol-
used-in-the-stm32-bootloader-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4229-how-to-
implement-a-vocoder-solution-using-stm32-microcontrollers-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4230-stm32-
microcontroller-random-number-generation-validation-using-the-nist-


```
Configuration Report
```
```
statistical-test-suite-stmicroelectronics.pdf
```
Application Notes https://www.st.com/resource/en/application_note/an4277-using-stm32-
device-pwm-shutdown-features-for-motor-control-and-digital-power-
conversion-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4286-spi-protocol-
used-in-the-stm32-bootloader-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4488-getting-started-
with-stm32f4xxxx-mcu-hardware-development-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4566-extending-the-
dac-performance-of-stm32-microcontrollers-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4640-peripherals-
interconnections-on-stm32f4057xx-stm32f4157xx-stm32f42xxx-
stm32f43xxx-stm32f446xx-and-stm32f469479xx-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4655-virtually-
increasing-the-number-of-serial-communication-peripherals-in-stm32-
applications-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4658-migration-of-
applications-from-stm32f429439-lines-to-stm32f446-line-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4739-stm32cube-
firmware-examples-for-stm32f4-series-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4750-handling-of-soft-
errors-in-stm32-applications-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4759-using-the-
hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-
stm32-microcontrollers-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4776-generalpurpose-
timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4803-highspeed-si-
simulations-using-ibis-and-boardlevel-simulations-using-hyperlynx-si-on-
stm32-mcus-and-mpus-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4838-managing-
memory-protection-unit-in-stm32-mcus-stmicroelectronics.pdf


```
Configuration Report
```
Application Notes https://www.st.com/resource/en/application_note/an4850-stm32-mcus-
spreadspectrum-clock-generation-principles-properties-and-
implementation-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4879-usb-hardware-
and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4908-stm32-usart-
automatic-baud-rate-detection-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4989-stm32-
microcontroller-debug-toolbox-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4995-using-an-
electromyogram-technique-to-detect-muscle-activity-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5020-digital-camera-
interface-dcmi-on-stm32-mcus-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5027-interfacing-pdm-
digital-microphones-using-stm32-mcus-and-mpus-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5036-thermal-
management-guidelines-for-stm32-applications-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5073-receiving-spdif-
audio-stream-with-the-stm32f4f7h7-series-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5156-introduction-to-
stm32-microcontrollers-security-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5225-usb-typec-
power-delivery-using-stm32-mcus-and-mpus-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an5543-enhanced-
methods-to-handle-spi-communication-on-stm32-devices-
stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4899-stm32-
microcontroller-gpio-configuration-for-hardware-settings-and-lowpower-
consumption-stmicroelectronics.pdf

Application Notes https://www.st.com/resource/en/application_note/an4760-quadspi-
interface-on-stm32-microcontrollers-and-microprocessors--
stmicroelectronics.pdf


```
Configuration Report
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an1202_freertos_guide-
freertos-guide-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an1602_semihosting_in
_truestudio-how-to-do-semihosting-in-truestudio-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an1801_stm32cubeprog
rammer_in_truestudio-installing-stm32cubeprogrammer-in-truestudio-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/atollic_editing_keyboard
_shortcuts-atollic-editing-keyboard-shortcuts-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/iar_to_atollic_truestudio
_migration_guide-truestudio-for-arm-migration-guide-iar-embedded-
workbench-to-truestudio-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/stm32cubemx_installatio
n_in_truestudio-stm32cubemx-installation-in-truestudio-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an2656-stm32f10xxx-
lcd-glass-driver-firmware-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an2790-tft-lcd-
interfacing-with-the-highdensity-stm32f10xxx-fsmc-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3078-stm32-
inapplication-programming-over-the-ic-bus-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3116-stm32s-adc-
modes-and-their-applications-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3174-implementing-
receivers-for-infrared-remote-control-protocols-using-stm32f10xxx-
microcontrollers-stmicroelectronics.pdf
```
Application Notes https://www.st.com/resource/en/application_note/an3241-qvga-tftlcd-


```
Configuration Report
```
for related Tools
& Software

```
direct-drive-using-the-stm32f10xx-fsmc-peripheral-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3307-guidelines-for-
obtaining-iec-60335-class-b-certification-for-any-stm32-application-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3364-migration-and-
compatibility-guidelines-for-stm32-microcontroller-applications-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3965-
stm32f40xstm32f41x-inapplication-programming-using-the-usart-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3969-eeprom-
emulation-in-stm32f40xstm32f41x-microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3990-upgrading-
stm32f4discovery-board-firmware-using-a-usb-key-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3997-audio-playback-
and-recording-using-the-stm32f4discovery-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an3998-pdm-audio-
software-decoding-on-stm32-microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4044-floating-point-
unit-demonstration-on-stm32-microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4187-using-the-crc-
peripheral-in-the-stm32-family-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4323-getting-started-
with-stemwin-library-stmicroelectronics.pdf
```
Application Notes
for related Tools

```
https://www.st.com/resource/en/application_note/an4365-using-stm32f4-
mcu-power-modes-with-best-dynamic-efficiency-stmicroelectronics.pdf
```

```
Configuration Report
```
& Software

Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4435-guidelines-for-
obtaining-ulcsaiec-607301603351-class-b-certification-in-any-stm32-
application-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4457-implementing-
an-emulated-uart-on-stm32f4-microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4499-stm32--
nrf51822-bluetooth-low-energy-system-solution-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4502-stm32-
smbuspmbus-embedded-software-expansion-for-stm32cube-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4657-stm32-
inapplication-programming-iap-using-the-usart-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4666-parallel-
synchronous-transmission-using-gpio-and-dma-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4678-full-duplex-spi-
emulation-for-stm32f4-microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4701-proprietary-
code-readout-protection-on-microcontrollers-of-the-stm32f4-series-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4739-stm32cube-
firmware-examples-for-stm32f4-series-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4758-proprietary-
code-readout-protection-on-stm32l4-stm32l4-stm32g4-and-stm32wb-
series-mcus-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4759-using-the-
hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-
stm32-microcontrollers-stmicroelectronics.pdf
```

```
Configuration Report
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4841-digital-signal-
processing-for-stm32-microcontrollers-using-cmsis-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4852-programming-
an-external-flash-memory-using-the-uart-bootloader-builtin-stm32-
microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an4968-proprietary-
code-read-out-protection-pcrop-on-stm32f72xxx-and-stm32f73xxx-
microcontrollers-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5054-secure-
programming-using-stm32cubeprogrammer-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5056-integration-
guide-for-the-xcubesbsfu-stm32cube-expansion-package-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5143-how-to-migrate-
motor-control-application-software-from-sdk-v43-to-sdk-v5x-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5166-guidelines-for-
control-and-customization-of-power-boards-with-stm32-mc-sdk-v50-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5360-getting-started-
with-projects-based-on-the-stm32mp1-series-in-stm32cubeide-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5361-getting-started-
with-projects-based-on-dualcore-stm32h7-microcontrollers-in-
stm32cubeide-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5394-getting-started-
with-projects-based-on-the-stm32l5-series-in-stm32cubeide-
stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5418-how-to-build-a-
simple-usbpd-sink-application-with-stm32cubemx-stmicroelectronics.pdf
```
Application Notes https://www.st.com/resource/en/application_note/an5426-migrating-


```
Configuration Report
```
for related Tools
& Software

```
graphics-middleware-projects-from-stm32cubemx-540-to-stm32cubemx-
550-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5464-position-control-
of-a-threephase-permanent-magnet-motor-using-xcubemcsdk-or-
xcubemcsdkful-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5564-getting-started-
with-projects-based-on-dualcore-stm32wl-microcontrollers-in-
stm32cubeide-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5698-adapting-the-
xcubestl-functional-safety-package-for-stm32-iec-61508-compliant-to-
other-safety-standards-stmicroelectronics.pdf
```
Application Notes
for related Tools
& Software

```
https://www.st.com/resource/en/application_note/an5731-stm32cubemx-
and-stm32cubeide-threadsafe-solution-stmicroelectronics.pdf
```
Errata Sheets https://www.st.com/resource/en/errata_sheet/es0298-stm32f446xcxe-
device-limitations-stmicroelectronics.pdf

Datasheet https://www.st.com/resource/en/datasheet/dm00141306.pdf

Programming
Manuals

```
https://www.st.com/resource/en/programming_manual/pm0214-stm32-
cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf
```
Reference
Manuals

```
https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-
advanced-armbased-32bit-mcus-stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn0516-overview-of-the-
stm32f0xf100xxf103xx-and-stm32f2xxf30xf4xx-mcus-pmsm-singledual-
foc-sdk-v40-stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1163-description-of-
wlcsp-for-microcontrollers-and-recommendations-for-its-use-
stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1204-tape-and-reel-
shipping-media-for-stm32-microcontrollers-in-bga-packages-
stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1205-tape-and-reel-
shipping-media-for-stm8-and-stm32-microcontrollers-in-fpn-packages-
stmicroelectronics.pdf
```

```
Configuration Report
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1206-tape-and-reel-
shipping-media-for-stm8-and-stm32-microcontrollers-in-qfp-packages-
stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1207-tape-and-reel-
shipping-media-for-stm8-and-stm32-microcontrollers-in-so-packages-
stmicroelectronics.pdf
```
Technical Notes
& Articles

```
https://www.st.com/resource/en/technical_note/tn1208-tape-and-reel-
shipping-media-for-stm8-and-stm32-microcontrollers-in-tssop-and-ssop-
packages-stmicroelectronics.pdf
```

