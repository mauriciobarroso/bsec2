/**
  ******************************************************************************
  * @file           : bsec2.h
  * @author         : Mauricio Barroso Benavides
  * @date           : Apr 8, 2023
  * @brief          : Header of ESP-IDF BSEC2 component
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSEC2_H_
#define BSEC2_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "esp_timer.h"

/* Dependent library headers */
#include "bme68x_lib.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bsec_interface_multi.h"

/* Exported macro ------------------------------------------------------------*/
#ifndef ARRAY_LEN
#define ARRAY_LEN(array)	(sizeof(array)/sizeof(array[0]))
#endif

#define BSEC_CHECK_INPUT(x, shift)				(x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR								UINT16_C(140)
#define BSEC_INSTANCE_SIZE								3272
#define BSEC_E_INSUFFICIENT_INSTANCE_SIZE	(bsec_library_return_t)-105

/* Exported typedef ----------------------------------------------------------*/
typedef bsec_output_t bsec_data_t;
typedef bsec_virtual_sensor_t bsec_sensor_t;

typedef struct {
	bsec_data_t output[BSEC_NUMBER_OUTPUTS];
	uint8_t n_outputs;
} bsec_outputs_t;

typedef struct bsec2_t bsec2_t;

typedef void (* bsec_callback_t)(const bme68x_data_t data, const bsec_outputs_t outputs, bsec2_t bsec2);

struct bsec2_t {
	bme68x_lib_t sensor;
	bsec_version_t version;
	bsec_library_return_t status;
	bsec_bme_settings_t bme_conf;
	bsec_callback_t new_data_callback;
	bsec_outputs_t outputs;
	uint8_t op_mode;
	float ext_temp_offset;
	uint32_t ovf_counter;
	uint32_t last_millis;
	uint8_t * bsec_instance;
};

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize an instance of BSEC2
 *
 * @param me   : Pointer to a structure instance of bsec2_t
 * @param arg  : Read callback
 * @param intf : Pointer to the interface descriptor
 *
 * @return True if everything initialized correctly
 */
bool bsec2_init(bsec2_t * const me, void * arg, bme68x_intf_t intf);

/**
 * @brief Function that sets the desired sensors and the sample rates
 *
 * @param me   : Pointer to a structure instance of bsec2_t
 *
 * @param sensor_list	: The list of output sensors
 * @param n_sensors		: Number of outputs requested
 * @param sample_Rate	: The sample rate of requested sensors
 *
 * @return	true for success, false otherwise
 */
bool bsec2_update_subscription(bsec2_t * const me, bsec_sensor_t sensor_list[], uint8_t n_sensors,
		float sample_rate);

/**
 * @brief Callback from the user to read data from the BME68x using parallel/forced mode, process and store outputs
 *
 * @param me : Pointer to a structure instance of bsec2_t
 *
 * @return	true for success, false otherwise
 */
bool bsec2_run(bsec2_t * const me);

/**
 * @brief Function to attach a callback function
 *
 * @param me : Pointer to a structure instance of bsec2_t
 */
void bsec2_attach_callback(bsec2_t * const me, bsec_callback_t callback);

/**
 * @brief Function to get the BSEC outputs
 *
 * @param me : Pointer to a structure instance of bsec2_t
 *
 * @return	pointer to BSEC outputs if available else nullptr
 */
const bsec_outputs_t * bsec2_get_outputs(bsec2_t * const me);

/**
 * @brief Function to get the BSEC output by sensor id
 *
 * @param me : Pointer to a structure instance of bsec2_t
 *
 * @return	pointer to BSEC output, nullptr otherwise
 */
bsec_data_t bsec2_get_data(bsec2_t * const me, bsec_sensor_t id);

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 *
 * @param me    : Pointer to a structure instance of bsec2_t
 * @param state : Pointer to a memory location, to hold the state
 *
 * @return	true for success, false otherwise
 */
bool bsec2_get_state(bsec2_t * const me, uint8_t * state);

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 *
 * @param me    : Pointer to a structure instance of bsec2_t
 * @param state : Pointer to a memory location that contains the state
 *
 * @return	true for success, false otherwise
 */
bool bsec2_set_state(bsec2_t * const me, uint8_t * state);

/**
 * @brief Function to retrieve the current library configuration
 *
 * @param me     : Pointer to a structure instance of bsec2_t
 * @param config : Pointer to a memory location, to hold the serialized config blob
 *
 * @return	true for success, false otherwise
 */
bool bsec2_get_config(bsec2_t * const me, uint8_t * config);

/**
  * @brief Function to set the configuration of the algorithm from memory
  *
  * @param me    : Pointer to a structure instance of bsec2_t
  * @param state : Pointer to a memory location that contains the configuration
  *
  * @return	true for success, false otherwise
  */
bool bsec2_set_config(bsec2_t * const me, const uint8_t * config);

/**
 * @brief Function to set the temperature offset
 *
 * @param me          : Pointer to a structure instance of bsec2_t
 * @param temp_offset	: Temperature offset in degree Celsius
 */
void bsec2_set_temperature_offset(bsec2_t * const me, float temp_offset);

/**
 * @brief Function to calculate an int64_t timestamp in milliseconds
 *
 * @param me : Pointer to a structure instance of bsec2_t
 */
int64_t bsec2_get_time_ms(bsec2_t * const me);

/**
 * @brief Function to assign the memory block to the bsec instance
 *
 * @param me            : Pointer to a structure instance of bsec2_t
 * @param[in] mem_block : reference to the memory block
 */
void bsec2_allocate_memory(bsec2_t * const me, uint8_t mem_block[BSEC_INSTANCE_SIZE]);

/**
 * @brief Function to de-allocate the dynamically allocated memory
 *
 * * @param me : Pointer to a structure instance of bsec2_t
 */
void bsec2_clear_memory(bsec2_t * const me);

#ifdef __cplusplus
}
#endif

#endif /* BSEC2_H_ */

/***************************** END OF FILE ************************************/
