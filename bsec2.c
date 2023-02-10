/**
  ******************************************************************************
  * @file           : bsec2.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Jan 24, 2023
  * @brief          : todo: write brief
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

/* Includes ------------------------------------------------------------------*/
#include "bsec2.h"
#include <string.h>

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Reads the data from the BME68x sensor and process it
 * @param currTimeNs: Current time in ns
 * @return true if there are new outputs. false otherwise
 */
bool process_data(bsec2_t * const me, int64_t curr_time_ns, const bme68x_data_t * data);

/**
 * @brief Common code for the begin function
 */
bool begin_common(bsec2_t * const me);

/**
 * @brief Set the BME68x sensor configuration to forced mode
 */
void set_bme68x_config_forced(bsec2_t * const me);

/**
 * @brief Set the BME68x sensor configuration to parallel mode
 */
void set_bme68x_config_parallel(bsec2_t * const me);

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Function to initialize an instance of BSEC2
 */
bool bsec2_init(bsec2_t * const me, void * arg, bme68x_intf_t intf) {
	me->ovf_counter = 0;
	me->last_millis = 0;
	me->status = BSEC_OK;
	me->ext_temp_offset = 0.0f;
	me->op_mode = BME68X_SLEEP_MODE;
	me->new_data_callback = NULL;

	memset(&me->sensor, 0, sizeof(me->sensor));
	memset(&me->version, 0, sizeof(me->version));
	memset(&me->bme_conf, 0, sizeof(me->bme_conf));
	memset(&me->outputs, 0, sizeof(me->outputs));

	bme68x_lib_init(&me->sensor, arg, intf);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return false;
	}

	return begin_common(me);
}

/**
 * @brief Function to request/subscribe for desired virtual outputs with the supported sample rates
 */
bool bsec2_update_subscription(bsec2_t * const me, bsec_sensor_t sensor_list[], uint8_t n_sensors,
		float sample_rate) {
	bsec_sensor_configuration_t virtual_sensors[BSEC_NUMBER_OUTPUTS];
	bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
	uint8_t n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

	for (uint8_t i = 0; i < n_sensors; i++) {
		virtual_sensors[i].sensor_id = sensor_list[i];
		virtual_sensors[i].sample_rate = sample_rate;
	}

  /* Subscribe to library virtual sensors outputs */
	me->status = bsec_update_subscription(virtual_sensors, n_sensors, sensor_settings, &n_sensor_settings);

	if (me->status != BSEC_OK) {
		return false;
	}

	return true;
}

/**
 * @brief Callback from the user to read data from the BME68X using parallel mode/forced mode, process and store outputs
 */
bool bsec2_run(bsec2_t * const me) {
	uint8_t n_fields_left = 0;
	bme68x_data_t data;
	int64_t curr_time_ns = bsec2_get_time_ms(me) * INT64_C(1000000);
	me->op_mode = me->bme_conf.op_mode;

	if (curr_time_ns >= me->bme_conf.next_call) {
    /* Provides the information about the current sensor configuration that is
       necessary to fulfill the input requirements, eg: operation mode, timestamp
       at which the sensor data shall be fetched etc */
		me->status = bsec_sensor_control(curr_time_ns, &me->bme_conf);

		if (me->status != BSEC_OK) {
			return false;
		}

		switch (me->bme_conf.op_mode) {
			case BME68X_FORCED_MODE:
				set_bme68x_config_forced(me);
				break;
			case BME68X_PARALLEL_MODE:
				if (me->op_mode != me->bme_conf.op_mode) {
					set_bme68x_config_parallel(me);
				}
				break;
			case BME68X_SLEEP_MODE:
				if (me->op_mode != me->bme_conf.op_mode) {
					bme68x_lib_set_op_mode(&me->sensor, BME68X_SLEEP_MODE);
					me->op_mode = BME68X_SLEEP_MODE;
				}
				break;
		}

		if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
			return false;
		}

		if (me->bme_conf.trigger_measurement && me->bme_conf.op_mode != BME68X_SLEEP_MODE) {
			if (bme68x_lib_fetch_data(&me->sensor)) {
				do {
					n_fields_left = bme68x_lib_get_data(&me->sensor, &data);

	        /* check for valid gas data */
					if (data.status & BME68X_GASM_VALID_MSK) {
						if (!process_data(me, curr_time_ns, &data)) {
							return false;
						}
					}
				} while (n_fields_left);
			}
		}
	}

	return true;
}

void bsec2_attach_callback(bsec2_t * const me, bsec_callback_t callback) {
	me->new_data_callback = callback;
}

/**
 * @brief Function to get the BSEC outputs
 */
const bsec_outputs_t * bsec2_get_outputs(bsec2_t * const me) {
    if (me->outputs.n_outputs) {
        return &me->outputs;
    }

    return NULL;
}

/**
 * @brief Function to get the BSEC output by sensor id
 */
const bsec_data_t bsec2_get_data(bsec2_t * const me, bsec_sensor_t id) {
	const bsec_data_t emp = {0};

	for (uint8_t i = 0; i < me->outputs.n_outputs; i++) {
		if (id == me->outputs.output[i].sensor_id) {
			return me->outputs.output[i];
		}
	}

	return emp;
}

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 */
bool bsec2_get_state(bsec2_t * const me, uint8_t * state) {
	uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;

	me->status = bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, work_buffer,
			BSEC_MAX_WORKBUFFER_SIZE, &n_serialized_state);

	if (me->status != BSEC_OK) {
		return false;
	}

	return true;
}

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 */
bool bsec2_set_state(bsec2_t * const me, uint8_t * state) {
	me->status = bsec_set_state(state, BSEC_MAX_STATE_BLOB_SIZE, work_buffer,
			BSEC_MAX_WORKBUFFER_SIZE);

	if (me->status != BSEC_OK) {
		return false;
	}

	memset(&me->bme_conf, 0, sizeof(me->bme_conf));

	return true;
}

/**
 * @brief Function to retrieve the current library configuration
 */
bool bsec2_get_config(bsec2_t * const me, uint8_t * config) {
	uint32_t n_serialized_settings = 0;

	me->status = bsec_get_configuration(0, config, BSEC_MAX_PROPERTY_BLOB_SIZE,
			work_buffer, BSEC_MAX_WORKBUFFER_SIZE, &n_serialized_settings);

	if (me->status != BSEC_OK) {
		return false;
	}

	return true;
}

/**
  * @brief Function to set the configuration of the algorithm from memory
  */
bool bsec2_set_config(bsec2_t * const me, const uint8_t * config) {
	me->status = bsec_set_configuration(config, BSEC_MAX_PROPERTY_BLOB_SIZE,
			work_buffer, BSEC_MAX_WORKBUFFER_SIZE);

	if (me->status != BSEC_OK) {
		return false;
	}

	memset(&me->bme_conf, 0, sizeof(me->bme_conf));

	return true;
}

/**
 * @brief Function to set the temperature offset
 */
void bsec2_set_temperature_offset(bsec2_t * const me, float temp_offset) {
	me->ext_temp_offset = temp_offset;
}

/**
 * @brief Function to calculate an int64_t timestamp in milliseconds
 */
int64_t bsec2_get_time_ms(bsec2_t * const me) {
	int64_t time_ms = (unsigned long)(esp_timer_get_time() / 1000ULL);

	if (me->last_millis > time_ms) {/* An overflow occurred */
		me->ovf_counter++;
	}

	me->last_millis = time_ms;

	return time_ms + (me->ovf_counter * INT64_C(0xFFFFFFFF));
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Reads the data from the BME68x sensor and process it
 */
bool process_data(bsec2_t * const me, int64_t curr_time_ns, const bme68x_data_t * data) {
	bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
	uint8_t n_inputs = 0;

  /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
	if (BSEC_CHECK_INPUT(me->bme_conf.process_data, BSEC_INPUT_TEMPERATURE)) {
		inputs[n_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
		inputs[n_inputs].signal = me->ext_temp_offset;
		inputs[n_inputs].time_stamp = curr_time_ns;

		n_inputs++;
#ifdef BME68X_USE_FPU
		inputs[n_inputs].signal = data->temperature;
#else
		inputs[n_inputs].signal = data->temperature / 100.0f;
#endif
		inputs[n_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
		inputs[n_inputs].time_stamp = curr_time_ns;
		n_inputs++;
	}

	if (BSEC_CHECK_INPUT(me->bme_conf.process_data, BSEC_INPUT_HUMIDITY)) {
#ifdef BME68X_USE_FPU
		inputs[n_inputs].signal = data->humidity;
#else
		inputs[n_inputs].signal = data-> humidity / 1000.0f
#endif
		inputs[n_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
		inputs[n_inputs].time_stamp = curr_time_ns;
		n_inputs++;
	}

	if (BSEC_CHECK_INPUT(me->bme_conf.process_data, BSEC_INPUT_PRESSURE)) {
		inputs[n_inputs].sensor_id = BSEC_INPUT_PRESSURE;
		inputs[n_inputs].signal = data->pressure;
		inputs[n_inputs].time_stamp = curr_time_ns;
		n_inputs++;
	}

	if (BSEC_CHECK_INPUT(me->bme_conf.process_data, BSEC_INPUT_GASRESISTOR) &&
			(data->status & BME68X_GASM_VALID_MSK)) {
		inputs[n_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
		inputs[n_inputs].signal = data->gas_resistance;
		inputs[n_inputs].time_stamp = curr_time_ns;
		n_inputs++;
	}

	if (BSEC_CHECK_INPUT(me->bme_conf.process_data, BSEC_INPUT_PROFILE_PART) &&
			(data->status & BME68X_GASM_VALID_MSK)) {
		inputs[n_inputs].sensor_id = BSEC_INPUT_PROFILE_PART;
		inputs[n_inputs].signal = (me->op_mode == BME68X_FORCED_MODE) ? 0 : data->gas_index;
		inputs[n_inputs].time_stamp = curr_time_ns;
		n_inputs++;
	}

	if (n_inputs > 0) {
		me->outputs.n_outputs = BSEC_NUMBER_OUTPUTS;
		memset(me->outputs.output, 0, sizeof(me->outputs.output));

    /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
		me->status = bsec_do_steps(inputs, n_inputs, me->outputs.output, &me->outputs.n_outputs);

		if (me->status != BSEC_OK) {
			return false;
		}

		if (me->new_data_callback) {
			me->new_data_callback(* data, me->outputs, * me);
		}
	}

	return true;
}

/**
 * @brief Common code for the begin function
 */
bool begin_common(bsec2_t * const me) {
	me->status = bsec_init();

	if (me->status != BSEC_OK) {
		return false;
	}

	me->status = bsec_get_version(&me->version);

	if (me->status != BSEC_OK) {
		return false;
	}

	memset(&me->bme_conf, 0, sizeof(me->bme_conf));
	memset(&me->outputs, 0, sizeof(me->outputs));

	return true;
}

/**
 * @brief Set the BME68X sensor configuration to forced mode
 */
void set_bme68x_config_forced(bsec2_t * const me) {
  /* Set the filter, odr, temperature, pressure and humidity settings */
	bme68x_lib_set_tph(&me->sensor, me->bme_conf.temperature_oversampling,
			me->bme_conf.pressure_oversampling, me->bme_conf.humidity_oversampling);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	bme68x_lib_set_heater_prof_for(&me->sensor, me->bme_conf.heater_temperature,
			me->bme_conf.heater_duration);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	bme68x_lib_set_op_mode(&me->sensor, BME68X_FORCED_MODE);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	me->op_mode = BME68X_FORCED_MODE;
}

/**
 * @brief Set the BME68X sensor configuration to parallel mode
 */
void set_bme68x_config_parallel(bsec2_t * const me) {
	uint16_t shared_heater_dur = 0;

  /* Set the filter, odr, temperature, pressure and humidity settings */
	bme68x_lib_set_tph(&me->sensor, me->bme_conf.temperature_oversampling,
				me->bme_conf.pressure_oversampling, me->bme_conf.humidity_oversampling);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	shared_heater_dur = BSEC_TOTAL_HEAT_DUR -
			(bme68x_lib_get_meas_dur(&me->sensor, BME68X_PARALLEL_MODE) / INT64_C(1000));

	bme68x_lib_set_heater_prof_par(&me->sensor, me->bme_conf.heater_temperature_profile,
			me->bme_conf.heater_duration_profile, shared_heater_dur, me->bme_conf.heater_profile_len);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	bme68x_lib_set_op_mode(&me->sensor, BME68X_PARALLEL_MODE);

	if (bme68x_lib_check_status(&me->sensor) == BME68X_ERROR) {
		return;
	}

	me->op_mode = BME68X_PARALLEL_MODE;
}
/***************************** END OF FILE ************************************/
