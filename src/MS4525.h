#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
#include <cmath>
#include <cstring>
#include <array>
#include <ranges>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>
#include <esp_log.h>

#include <driver/i2c_master.h>

namespace YOBA {
	enum class MS4525Error : uint8_t {
		// Everything is okay
		none,
		// Generic I2C wiring or bus configuration issue
		I2C,
		// The module didn't have enough time to decode data. Consider lowering your sampling speed tp 50-100 Hz
		staleData,
		// Some shit with module itself, perhaps registers got overwritten during power loss. Reboot it
		internalFault
	};

	// Defines working range of internal ADC
	// For example in 4525DO-DS5AI001DP sensor the "A" in "5AI" stands for output type A
	enum class MS4525OutputType : uint8_t {
		// [10%; 80%] of ADC range
		a,
		// [5%; 90%] of ADC range
		b
	};

	class MS4525 {
		public:
			constexpr static uint8_t defaultI2CAddress = 0x28;
			constexpr static uint32_t defaultI2CFrequencyHz = 400'000;

			/// Let's consider we have 4525DO-DS5AI001DP sensor:
			///
			/// MS4525 - base model number
			///
			/// DO - digital output (I2C, not analog)
			///
			/// DS - package type and port configuration. Dual-sided (D) pins with straight ports (S) for connecting silicone tubes
			///
			/// 5 — supply voltage (5 means range [4.75 V, 5.25 V]). In my case, the sensor works perfectly fine
			/// even when powered by the ESP32’s 3.3 V LDO. If you still want to power it from 5 V, keep in mind that the MS4525
			/// module will most likely pull the SCL/SDA lines up to 5 V. Therefore, you’ll probably need a level shifter
			/// to avoid damaging the ESP pins
			///
			/// A - output signal type (A or B) defines the working range of the maximum resolution of a 14-bit ADC.
			/// For type A range will be [10; 80]%, for type B it becomes [5; 90]%
			/// The remaining ADC zones are reserved for system error detection
			///
			/// I - interface type (I2C in our case)
			///
			/// 001: The maximum measurable range is 1 PSI (Pound per Square Inch)
			///
			/// D — pressure measurement mode. Differential (D) measures the pressure difference between the upper (Port 1) and lower (Port 2) ports.
			/// Allows for the measurement of both positive pressure and negative pressure (vacuum)
			///
			/// P — port direction. Bidirectional (P) means that sensor's physical scale is symmetrical about zero and ranges from -1 PSI to +1 PSI (from -6894.76 Pa to +6894.76 Pa)
			MS4525Error setup(
				const i2c_master_bus_handle_t& I2CBusHandle,
				const uint8_t I2CAddress = defaultI2CAddress,
				const uint32_t I2CFrequencyHz = defaultI2CFrequencyHz,

				const MS4525OutputType outputType = MS4525OutputType::a,
				const float minPressurePSI = -1.f,
				const float maxPressurePSI = 1.f
			) {
				_outputType = outputType;
				_minPressurePSI = minPressurePSI;
				_maxPressurePSI = maxPressurePSI;

				// Device
				i2c_device_config_t I2CDeviceConfig {};
				I2CDeviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
				I2CDeviceConfig.device_address = I2CAddress;
				I2CDeviceConfig.scl_speed_hz = I2CFrequencyHz;

				const auto ESPError = i2c_master_bus_add_device(I2CBusHandle, &I2CDeviceConfig, &_I2CDeviceHandle);

				if (ESPError != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(ESPError);
					return MS4525Error::I2C;
				}

				return MS4525Error::none;
			}

			MS4525Error readDifferentialPressureAndTemperature(float& differentialPressurePSI, float& temperatureC) const {
				// Reading raw data
				uint8_t rawData[4];

				const auto error = i2c_master_receive(_I2CDeviceHandle, rawData, 4, 500);

				if (error != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(error);
					return MS4525Error::I2C;
				}

				// Decoding status bits
				switch ((rawData[0] >> 6) & 0b0000'0011) {
					// Normal
					case 0: break;
					// Command mode
					case 1: return MS4525Error::internalFault;
					// Stale data
					case 2: return MS4525Error::staleData;
					// Fault
					default: return MS4525Error::internalFault;
				}

				const uint16_t rawPressure = (static_cast<uint16_t>(rawData[0] & 0b0011'1111) << 8) | rawData[1];
				const uint16_t rawTemperature = (static_cast<uint16_t>(rawData[2]) << 0b0000'0011) | ((rawData[3] >> 5) & 0b0000'0111);

				// ESP_LOGI("sens", "rawPressure: %d, rawTemperature: %d", rawPressure, rawTemperature);

				// Computing pressure
				float outputTypeRangeMin, outputTypeRangeMax;

				switch (_outputType) {
					case MS4525OutputType::a:
						outputTypeRangeMin = outputTypeARangeMin;
						outputTypeRangeMax = outputTypeARangeMax;

						break;

					default:
						outputTypeRangeMin = outputTypeBRangeMin;
						outputTypeRangeMax = outputTypeBRangeMax;

						break;
				}

				differentialPressurePSI =
					(static_cast<float>(rawPressure) - outputTypeRangeMin * maxRawPressure)
					* (_maxPressurePSI - _minPressurePSI)
					/ (outputTypeRangeMax * maxRawPressure)
					+ _minPressurePSI
					- _differentialPressureBias;

				// Computing temperature
				temperatureC =
					minTemperatureC
					+ static_cast<float>(rawTemperature)
					* (maxTemperatureC - minTemperatureC)
					/ maxRawTemperature;

				// ESP_LOGI("sens", "pressurePSI: %f, temperatureC: %f", pressurePSI, temperatureC);

				return MS4525Error::none;
			}

			float getDifferentialPressureBias() const {
				return _differentialPressureBias;
			}

			void setDifferentialPressureBias(const float pressureBias) {
				_differentialPressureBias = pressureBias;
			}

			static float getIndicatedAirspeedMS(const float differentialPressurePSI) {
				return
					differentialPressurePSI > 0.0f
					? std::sqrtf(2.0f * differentialPressurePSI * paInOnePSI / airDensityKgM3)
					: 0.0f;
			}

			// Performs multiple sensor readings into stack allocated buffer at the specified sample rate and returns the median value
			template<size_t bufferSize = 200, uint16_t sampleRateHz = 50>
			MS4525Error computeMedianDifferentialPressureBias(float& bias) const {
				std::array<float, bufferSize> buffer {};

				float differentialPressurePSI, temperatureC;

				const auto sampleDurationTicks = pdMS_TO_TICKS(1'000 / sampleRateHz);

				for (float& i : buffer) {
					const auto error = readDifferentialPressureAndTemperature(differentialPressurePSI, temperatureC);

					if (error != MS4525Error::none)
						return error;

					i = differentialPressurePSI;

					vTaskDelay(sampleDurationTicks);
				}

				std::ranges::sort(buffer);

				bias = buffer[buffer.size() / 2];

				return MS4525Error::none;
			}

			static const char* errorToString(const MS4525Error error) {
				switch (error) {
					case MS4525Error::none:
						return "none";

					case MS4525Error::I2C:
						return "I2C or wiring failure";

					case MS4525Error::staleData:
						return "stale data";

					default:
						return "internal fault";
				}
			}

		private:
			constexpr static auto logTag = "MS4525";

			constexpr static float maxRawPressure = 16383.0f;

			constexpr static float maxRawTemperature = 2047.0f;
			constexpr static float minTemperatureC = -50.0f;
			constexpr static float maxTemperatureC = 150.0f;

			constexpr static float outputTypeARangeMin = 0.1f;
			constexpr static float outputTypeARangeMax = 0.8f;

			constexpr static float outputTypeBRangeMin = 0.05f;
			constexpr static float outputTypeBRangeMax = 0.9f;

			constexpr static float paInOnePSI = 6894.757f;
			constexpr static float airDensityKgM3 = 1.225f;

			i2c_master_dev_handle_t _I2CDeviceHandle {};
			MS4525OutputType _outputType = MS4525OutputType::a;
			float _minPressurePSI = 0;
			float _maxPressurePSI = 0;

			float _differentialPressureBias = 0;
	};
}
