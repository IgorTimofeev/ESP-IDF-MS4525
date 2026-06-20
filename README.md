# ESP-IDF-MS4525

Tiny lib for reading differential pressure, temperature & airspeed from MS4525 pitot/static sensor

# Usage

```c++
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/i2c_master.h>

#include <MS4525.h>

extern "C" {
	using namespace YOBA;

	// Checks for sensor errors, logs them and pauses the program if something fails
	static MS4525Error checkSensorError(const MS4525Error error, const char* prefix) {
		if (error == MS4525Error::none)
			return error;

		ESP_LOGE("main", "%s: %s", prefix, MS4525::errorToString(error));

		// Deal with error
		while (true)
			vTaskDelay(pdMS_TO_TICKS(1000));

		return error;
	}

	void app_main(void) {

		// -------------------------------- I2C initialization --------------------------------

		i2c_master_bus_config_t I2CBusConfig {};
		I2CBusConfig.i2c_port = I2C_NUM_0;
		I2CBusConfig.sda_io_num = GPIO_NUM_5;              // Change to your pin
		I2CBusConfig.scl_io_num = GPIO_NUM_4;              // Change to your pin
		I2CBusConfig.clk_source = I2C_CLK_SRC_DEFAULT;
		I2CBusConfig.glitch_ignore_cnt = 7;
		I2CBusConfig.flags.enable_internal_pullup = true;

		i2c_master_bus_handle_t I2CBusHandle;
		ESP_ERROR_CHECK(i2c_new_master_bus(&I2CBusConfig, &I2CBusHandle));

		// -------------------------------- Sensor initialization --------------------------------

		MS4525 sensor {};

		sensor.setup(I2CBusHandle);

		// Reading & calculating the median value for the sensor bias, since without prior
		// software calibration, most sensor will produce messy results
		float differentialPressureBias = 0;
		auto error = sensor.computeMedianDifferentialPressureBias(differentialPressureBias);
		checkSensorError(error, "failed to compute average pressure bias");

		// Setting computed bias
		sensor.setDifferentialPressureBias(differentialPressureBias);

		ESP_LOGI("main", "median diff pressure bias: %f", differentialPressureBias);

		// -------------------------------- Airspeed reading --------------------------------

		float differentialPressurePSI, temperatureC, indicatedAirspeedMS;

		while (true) {
			// Reading diff pressure & temperature
			error = sensor.readDifferentialPressureAndTemperature(differentialPressurePSI, temperatureC);
			checkSensorError(error, "failed to read diff pressure & temperature");

			// Getting IAS using diff pressure
			indicatedAirspeedMS = MS4525::getIndicatedAirspeedMS(differentialPressurePSI);

			// Printing whole stuff out
			ESP_LOGI("main", "diff pressure: %f PSI, temperature: %f deg C, airspeed: %f m/s", differentialPressurePSI, temperatureC, indicatedAirspeedMS);

			vTaskDelay(pdMS_TO_TICKS(1'000 / 50));
		}
	}
}
```