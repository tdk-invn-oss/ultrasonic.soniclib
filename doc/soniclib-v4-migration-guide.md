# Migration guide : Soniclib V3 to Soniclib V4

This guide describes what changed between Soniclib V4 and V3.
First it describes the code changes, then what you might have to change in your Soniclib integration

## What's new

The Soniclib V4 has several API breaking changes.
This version introduces code re-organization in its internal code

The changes :
* Re-organize firmware initializations
* Remove static definition of init firmware : now the init firmware is set at runtime
* Remove static compilation of range finding feature
* New internal log module : ch_log
* Remove useless API from ch_bsp

## Project changes

The things you might have to change in your project build

### New files

* If using `ch_log` module, add compilation/declaration of :
   * `invn\soniclib\ch_log.h`
   * `invn\soniclib\ch_log.c`

* If using a firmware with range finding features, add compilation/declaration of :
   * `invn/soniclib/ch_rangefinder.c`
   * `invn/soniclib/ch_rangefinder.h`
   * `invn/soniclib/ch_rangefinder_types.h`

### Defines

* Remove definition of symbols :
   * `INCLUDE_ALGO_EXTERNAL`
   * `CHIRP_NO_INIT_FW`
   * `CHIRP_NO_TX_OPTIMIZATION`
* If using `ch_log` module, define symbol `CH_LOG_MODULE_LEVEL` to a value between 0 and 6 (4 = `CH_LOG_LEVEL_ERROR`, more info in `ch_log.h`)

## API Changes

The new version introduces several API changes, some new API and some API with type change.

### New API

You might have to call these new API.

* **[mandatory]** `uint8_t ch_group_init(ch_group_t *grp_ptr, uint8_t num_devices, uint8_t num_buses, uint16_t rtc_cal_pulse_ms)`

This new function replaces definitions done in Board Support Package, you shall call it first in your application

* **[optional][ICUx0201]** `uint8_t ch_set_init_firmware(ch_dev_t *dev_ptr, ch_fw_init_func_t fw_init_func)`

This function, used only with ICU devices, is necessary to define an init firmware if your main firmware doesn't includes sensor initialization or you need to run tx optimization (ringdown reduce) in your application

* **[optional][ICUx0201]** `uint8_t ch_set_algo_config(ch_dev_t *dev_ptr, const void *algo_cfg_ptr)`

This function will write the sensor algorithm configuration to sensor

### Prototype change

#### GPR Firmware Configuration (CHx01)

These changes only apply for applications using CHx01 sensor and General Purpose Rangefinding firmware

* **[CHx01]** `uint8_t ch_get_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr)`
* **[CHx01]** `uint8_t ch_set_config(ch_dev_t *dev_ptr, ch_config_t *config_ptr)`

These 2 API were previously used to set or get the sensor measure configuration and the sensor algo configuration.
Now these API only change the **sensor measure configuration**. The algo is set/get with other dedicated commands.

You will have to move the setting of following variables to `ch_rangefinder_set_algo_config()` :
* `config_ptr->static_range`
* `config_ptr->thresh_ptr`
* `config_ptr->time_plan`

#### GPT Firmware Configuration (ICUx0201)

This change only apply for applications using ICUx0201 sensor and General Purpose Transceiver firmware

* **[ICUx0201]** ` uint8_t ch_meas_init(ch_dev_t *dev_ptr, uint8_t meas_num, const ch_meas_config_t *meas_config_ptr,
                     const void *thresh_ptr)`

This API were previously used to set the sensor measure configuration and the GPT algo configuration.
Now thus API only change the **sensor measure configuration**. The algo is set/get with other dedicated commands.

You will have to move the setting of following variables to `icu_gpt_algo_configure()` :
* `meas_config.num_ranges`
* `meas_config.ringdown_cancel_samples`
* `meas_config.static_filter_samples`
* `meas_config.iq_output_format`
* `meas_config.filter_update_interval`
* `thresh_ptr`

#### Prototype change

The following API have only type changes :

* `uint32_t ch_get_freerun_interval_ticks(ch_dev_t *dev_ptr)` --> `uint16_t ch_get_freerun_interval_ticks(ch_dev_t *dev_ptr)`
* `uint32_t ch_usec_to_ticks(ch_dev_t *dev_ptr, uint32_t num_usec)` --> `uint16_t ch_usec_to_ticks(ch_dev_t *dev_ptr, uint32_t num_usec)`
* `uint32_t ch_ticks_to_usec(ch_dev_t *dev_ptr, uint32_t num_ticks)` --> `
uint32_t ch_ticks_to_usec(ch_dev_t *dev_ptr, uint16_t num_ticks)`

## New includes

The code of library targeting the range finding feature or the GPT firmware have been moved to dedicated files.
The new files are described below :

### ch_rangefinder_types.h

This file contains definitions which are specific to range finding feature but which needs to be known by other types of firmware

### ch_rangefinder.h

This file defines the API which are specific to range finding firmware, they were previously defined in `soniclib.h`.
Their implementation didn't change, only the place they are defined.
These API targets CHx01 GPR firmware and are backward compatible with ICUx0201 GPT firmware.

* `uint32_t ch_get_tof_us(ch_dev_t *dev_ptr)`
* `uint16_t ch_get_static_range(ch_dev_t *dev_ptr)`
* `uint16_t ch_set_static_range(ch_dev_t *dev_ptr, uint16_t num_samples)`
* `uint8_t ch_set_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index, uint16_t amplitude)`
* `uint16_t ch_get_threshold(ch_dev_t *dev_ptr, uint8_t threshold_index)`
* `uint8_t ch_set_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr)`
* `uint8_t ch_get_thresholds(ch_dev_t *dev_ptr, ch_thresholds_t *thresh_ptr)`
* `uint8_t ch_get_num_thresholds(ch_dev_t *dev_ptr)`
* `uint8_t ch_get_num_targets(ch_dev_t *dev_ptr)`
* `uint8_t ch_set_rx_holdoff(ch_dev_t *dev_ptr, uint16_t num_samples)`
* `uint16_t ch_get_rx_holdoff(ch_dev_t *dev_ptr)`
* `uint32_t ch_get_target_range(ch_dev_t *dev_ptr, uint8_t target_num, ch_range_t range_type)`
* `uint16_t ch_get_target_amplitude(ch_dev_t *dev_ptr, uint8_t target_num)`
* `uint32_t ch_get_target_tof_us(ch_dev_t *dev_ptr, uint8_t target_num)`
* `uint8_t ch_is_target_in_ringdown(ch_dev_t *dev_ptr)`

### icu_gpt.h

This file defines the API which are specific to ICUx0201 GPT firmware.

#### Existing API in Soniclib V3

These API were previously defined in `soniclib.h`.
Their implementation didn't change, only the place they are defined and their prefix.

* `uint8_t ch_meas_set_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t num_ranges)`
* `uint8_t ch_meas_get_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num)`
* `uint8_t ch_meas_set_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr)`
* `uint8_t ch_meas_get_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr)`
* `uint8_t ch_meas_set_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)`
* `uint16_t ch_meas_get_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num)`
* `uint8_t ch_meas_set_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)`
* `uint16_t ch_meas_get_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num)`
* `uint8_t ch_meas_set_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)`
* `uint16_t ch_meas_get_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num)`
* `uint8_t ch_meas_set_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t update_interval)`
* `uint8_t ch_meas_get_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num)`

* New names :

```c
uint8_t icu_gpt_set_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t num_ranges)
uint8_t icu_gpt_get_num_ranges(ch_dev_t *dev_ptr, uint8_t meas_num)
uint8_t icu_gpt_set_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr)
uint8_t icu_gpt_get_thresholds(ch_dev_t *dev_ptr, uint8_t meas_num, ch_thresholds_t *lib_thresh_buf_ptr)
uint8_t icu_gpt_set_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)
uint16_t icu_gpt_get_rx_holdoff(ch_dev_t *dev_ptr, uint8_t meas_num)
uint8_t icu_gpt_set_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)
uint16_t icu_gpt_get_ringdown_cancel(ch_dev_t *dev_ptr, uint8_t meas_num)
uint8_t icu_gpt_set_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num, uint16_t num_samples)
uint16_t icu_gpt_get_static_filter(ch_dev_t *dev_ptr, uint8_t meas_num)
uint8_t icu_gpt_set_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num, uint8_t update_interval)
uint8_t icu_gpt_get_filter_update(ch_dev_t *dev_ptr, uint8_t meas_num)
```

#### New API

* Initialization of GPT algorithm

```c
uint8_t icu_gpt_algo_init(ch_dev_t *dev_ptr, InvnAlgoRangeFinderConfig *algo_cfg);
```

Sends the algo init signal to sensor and save the pointer to the algorithm configuration stored in application.

* Configuration of GPT algorithm

```c
uint8_t icu_gpt_algo_configure(ch_dev_t *dev_ptr, uint8_t meas_num, const icu_gpt_algo_config_t *algo_config_ptr,
                               const ch_thresholds_t *lib_thresh_ptr);
```

Copy the configuration of algorithm to the configuration variable. The configuration is not sent to sensor.

* Reset the configuration of GPT algorithm

```c
void icu_gpt_algo_reset(ch_dev_t *dev_ptr, uint8_t meas_num);
```

Reset the configuration variable of algorithm

* Get or Set the GPT algorithm IQ output format

```c
uint8_t icu_gpt_algo_set_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num, ch_output_type_t output_format);
ch_output_type_t icu_gpt_algo_get_iq_output(ch_dev_t *dev_ptr, uint8_t meas_num);
```

Set or get the IQ output format to sensor.
The Set API sends the algorithm configuration to sensor.

* Update the GPT algorithm configuration depending on new CIC ODR

```c
void icu_gpt_algo_update_odr(ch_dev_t *dev_ptr, uint8_t meas_num, ch_odr_t new_odr);
```

This API will update the number of samples of GPT configuration attributes depending on `new_odr`.
This API will **NOT** sends the algorithm configuration to sensor. You shall call `ch_set_algo_config()` for that.
This API will **NOT** update the CIC ODR of sensor. You shall call `ch_meas_set_odr()` for that.

* Read the number of detected targets

```c
uint8_t icu_gpt_algo_get_num_targets(ch_dev_t *dev_ptr);
```

This API will read directly from sensor the number of targets detected by last run of GPT algorithm.

* Read the amplitude of a detected target

```c
uint16_t icu_gpt_algo_get_target_amplitude(ch_dev_t *dev_ptr, uint8_t target_num);
```

This API will read directly from sensor the amplitude of target `target_num` in last run of GPT algorithm.

* Read the range of a detected target in microseconds

```c
uint32_t icu_gpt_algo_get_target_tof_us(ch_dev_t *dev_ptr, uint8_t target_num);
```

This API will read the targets list and compute its range in microseconds.

* Read the range of a detected target in millimiters

```c
uint32_t icu_gpt_algo_get_target_range(ch_dev_t *dev_ptr, uint8_t target_num, ch_range_t range_type);
```

This API will read the targets list and compute its range in millimeters.

* Set the data output format type and rate

```c
uint8_t icu_gpt_algo_set_data_output(ch_dev_t *dev_ptr, const ch_output_t *output_ptr);
```
This API sets both the data output format type and rate for sample data within a measurement.
This API his created for backward compatibility with `ch_set_data_output()`

* Return if a target is detected in the ringdown

```c
uint8_t icu_gpt_algo_is_target_in_ringdown(ch_dev_t *dev_ptr);
```

This API reads the output of last run of GPT algorithm and returns if a target has been detected in the ringdown.

* Print GPT algorithm detection thresholds

```c
uint8_t icu_gpt_display_algo_thresholds(ch_dev_t *dev_ptr);
```

This API is an utils function, only usefull at debug time. It will print to the debug console the detection thresholds (using `ch_log_printf()`).

## API Removed from soniclib.h

### API moved to ch101_sonicsync soniclib plugin

* `uint8_t ch_set_time_plan(ch_dev_t *dev_ptr, ch_time_plan_t time_plan)`
* `ch_time_plan_t ch_get_time_plan(ch_dev_t *dev_ptr)`

These API were moved ch101_sonicsync firmware plugin because it relies on a feature specific to this only one firmware.

### API moved to ch101_gppc soniclib plugin

* `int8_t ch_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff)`
* `uint8_t ch_get_static_coeff(ch_dev_t *dev_ptr)`
* `uint8_t ch_get_rx_pulse_length(ch_dev_t *dev_ptr)`
* `uint8_t ch_set_modulated_tx_data(ch_dev_t *dev_ptr, uint8_t tx_data)`
* `uint8_t ch_get_demodulated_rx_data(ch_dev_t *dev_ptr, uint8_t rx_pulse_length, uint8_t *data_ptr)`

These API were moved ch101_gppc firmware plugin because it relies on a feature specific to this only one firmware.
These API were renamed :

```c
int8_t ch101_gppc_set_static_coeff(ch_dev_t *dev_ptr, uint8_t static_coeff);
uint8_t ch101_gppc_get_static_coeff(ch_dev_t *dev_ptr);
uint8_t ch101_gppc_get_rx_pulse_length(ch_dev_t *dev_ptr);
uint8_t ch_set_modulated_tx_data(ch_dev_t *dev_ptr, uint8_t tx_data);
uint8_t ch_get_demodulated_rx_data(ch_dev_t *dev_ptr, uint8_t rx_pulse_length, uint8_t *data_ptr);
```

## New Log module

A new log module have been introduced in Soniclib to better handle the debug messages.
This log module commonizes the formatting of debug messages and adds a level filtering of messages.
This log module can be used by the application too.

### Log levels

The module defines the following levels of messages :

```c
#define CH_LOG_LEVEL_TRACE   0
#define CH_LOG_LEVEL_DEBUG   1
#define CH_LOG_LEVEL_INFO    2
#define CH_LOG_LEVEL_WARNING 3
#define CH_LOG_LEVEL_ERROR   4
#define CH_LOG_LEVEL_APP     5
#define CH_LOG_LEVEL_DISABLE 6
```

* The *trace* level is the most verbose, the *error* level is the less verbose of lib.
* The *app* level is a level dedicated to application. The *app* messages have no specific formatting.
* The *disable* level is used to completely remove log messages from build

### Message formatting

The formatting of module is as follow :

```console
I CH_DRV  write_firmware                Programming sensor...
I CH_DRV  write_firmware                Wrote 5802 bytes in 1074662912 ms.
```

* First group is the log level defined with 1 character :
   * 'T' : Trace
   * 'D' : Debug
   * 'I' : Info
   * 'W' : Warning
   * 'E' : Error
* Second group is the software module generating the message (7 first characters)
* Third group is the function name generating the message (30 first characters)
* Last group is the message, left aligned

### Defining the log level

Define `CH_LOG_MODULE_LEVEL` in your build system with a value between 0 and 6

### Available macros

For every log level, there is the following macros defined

* `CH_LOG_<LEVEL>(...)` : This macro prints a message ending with a line feed
* `CH_LOG_<LEVEL>_START(...)` : This macro starts a line with the log prefix and a message
* `CH_LOG_<LEVEL>_MSG(...)` : This macro continues a line started by `CH_LOG_<LEVEL>_START(...)`
* `CH_LOG_<LEVEL>_END()` : This macro ends a line started by `CH_LOG_<LEVEL>_START(...)` or `CH_LOG_<LEVEL>_MSG(...)`

### BSP implementation

The print of log messages to the console is left to the charge of the following BSP function :

```c
void chbsp_print_str(const char *str);
```

The implementation of this function is left to user / board integrator


## Soniclib integration changes

1. Update project
   a. Add build of `invn/soniclib/ch_log.c`
   b. If necessary add build of `invn/soniclib/ch_rangefinder.c` (for CHx01 firmware)
   c. Remove definition of symbol `INCLUDE_ALGO_EXTERNAL` (if defined)
   d. Remove definition of symbol `CHIRP_NO_TX_OPTIMIZATION` (if defined)

2. Update BSP

Remove initialization of following symbols from `chbsp_board_init()`

```c
   grp_ptr->num_ports        = CHBSP_MAX_DEVICES;
   grp_ptr->num_buses        = CHBSP_NUM_BUSES;
   grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;
```

3. Update application

3.1. Rename ch_bsp API which are no more parts of Soniclib :

* `chbsp_board_init()`
* `chbsp_periodic_timer_init()`
* `chbsp_periodic_timer_irq_enable()`
* `chbsp_periodic_timer_start()`
* `chbsp_proc_sleep()`
* Add a prototype for the previous functions

3.2. Add call to `ch_group_init()`

* This function has to be called before any other call to a Soniclib function.
* This function will initialize the group variables which were previously initialized in BSP :
   * num_ports
   * num_buses
   * rtc_cal_pulse_ms
* Code example :

```c
ch_group_init(&chirp_group, CHIRP_MAX_NUM_SENSORS, CHIRP_NUM_BUSES, CHIRP_RTC_CAL_PULSE_MS);
```

3.3 Add call to `ch_set_init_firmware()` if needing a specific init firmware

The init firmware is needed in 2 cases :
* if your main firmware (with the algorithm part) doesn't contain the hardware initialization part of sensor (init, calibration...)
* if you need to run tx optimization (to reduce the ringdown)

In the first case you need to call `ch_set_init_firmware()` before calling `ch_group_start()`.
In the second case you need to call `ch_set_init_firmware()` before calling `ch_meas_optimize()`

* Code example :

```c
group_rc |= ch_set_init_firmware(dev_ptr, icu_init_no_txopt_init);
```

`icu_init_no_txopt_init` is a firmware containing only sensor hardware initialization (but not the tx optimization feature).

You will need to include the path the the init firmware function to your main. Here `#include <invn/soniclib/sensor_fw/icu_init-no-txopt/icu_init-no-txopt.h>`

3.4 ICU GPT based applications : change sensor algorithm configuration

The configuration of algorithm on sensor side now shall explicitely be set, it results in the necessity to call few more API.

* Add explicit includes to the firmware header

```c
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>
#include <invn/soniclib/ch_rangefinder_types.h>
```

* Initialize the algorithm

Call `icu_gpt_algo_init()` to "register" the algo instance structure and store algo configuration in it in future soniclib/icu_gpt API calls :

```c
/* Register variable to store the algorithm config  */
chirp_error = icu_gpt_algo_init(dev_ptr, &gpt_algo_instance[dev_num]);
```

* Configure the algorithm

The configuration of algorithm attributes have moved `ch_meas_init()` to `icu_gpt_algo_configure()`

```c
/* Change algorithm config to sensor */
chirp_error |= icu_gpt_algo_configure(dev_ptr, CH_DEFAULT_MEAS_NUM, &algo_config, &chirp_detect_thresholds);
```

The configuration will then be sent to sensor calling :

```c
/* Write algo config to sensor */
chirp_error = ch_set_algo_config(dev_ptr, &gpt_algo_instance[dev_num]);

```

This function will write the algorithm configuration in sensor memory, to validate the configuration on sensor, call `ch_init_algo()` :

```c
/* Init algo with new config */
chirp_error = ch_init_algo(dev_ptr);
```

3.5 CHx01 GPR based applications

The configuration of algorithm have been separated from the configuration of the sensor measure. As a result, the algorithm configuration now shall explicitely be set.

* Add explicit includes to the firmware header

```c
#include <invn/soniclib/ch_rangefinder.h>
```

* Set algorithm configuration

```c
algo_config.static_range = CHIRP_STATIC_REJECT_SAMPLES;
algo_config.thresh_ptr = &chirp_detect_thresholds;
chirp_error = ch_rangefinder_set_algo_config(dev_ptr, &algo_config);
```

This function will send the number of samples of static range and the multi-thresholds array if the firmware is compatible.