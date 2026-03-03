/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include "../../../GlobalVars.h"
#include "../../../consts.h"
#include "CalibrationStep.h"
#include "logging/Logger.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

namespace {
static constexpr int8_t AXIS_X = 0;
static constexpr int8_t AXIS_Y = 1;
static constexpr int8_t AXIS_Z = 2;
static constexpr int8_t AXIS_X_DOWN = 3;
static constexpr int8_t AXIS_Y_DOWN = 4;
static constexpr int8_t AXIS_Z_DOWN = 5;
static constexpr int8_t AXIS_COUNT = 6;
}  // namespace

template <typename SensorRawT>
class AccelSixSideCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	AccelSixSideCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig,
		SlimeVR::Logging::Logger& logger,
		float accelScale
	)
		: CalibrationStep<SensorRawT>{sensorConfig}
		, logger{logger}
		, accelScale{accelScale} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = CalibrationData{};
		calibrationData.value().startMillis = millis();
		sampleLedPulseActive = false;
		ledManager.off();
	}

	TickResult tick() override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return TickResult::CONTINUE;
		}

		auto& data = calibrationData.value();
		if (millis() - data.startMillis > maxCalibrationTimeSeconds * 1e3) {
			logger.warn("Accel six-side calibration timed out.");
			return TickResult::SKIP;
		}

		if (!isAllPositionsRecorded()) {
			// Keep collecting samples for current orientation.
			if (data.storedSampleCount < samplePerPositionCount) {
				return TickResult::CONTINUE;
			}

			// One face complete: compute axis average and move to next orientation.
			if (!storeCurrentAxisAverage()) {
				logger.warn("Accel six-side calibration failed to store axis sample");
				return TickResult::SKIP;
			}

			return TickResult::CONTINUE;
		}

		// All six faces recorded: solve per-axis offset/scale.
		float sixSideOff[3];
		float sixSideScale[3];
		if (!computeAxisCalibration(
				data.axisSamples[AXIS_X],
				data.axisSamples[AXIS_X_DOWN],
				sixSideOff[AXIS_X],
				sixSideScale[AXIS_X]
			)
			|| !computeAxisCalibration(
				data.axisSamples[AXIS_Y],
				data.axisSamples[AXIS_Y_DOWN],
				sixSideOff[AXIS_Y],
				sixSideScale[AXIS_Y]
			)
			|| !computeAxisCalibration(
				data.axisSamples[AXIS_Z],
				data.axisSamples[AXIS_Z_DOWN],
				sixSideOff[AXIS_Z],
				sixSideScale[AXIS_Z]
			)) {
			logger.warn("Accel six-side calibration result was invalid");
			return TickResult::SKIP;
		}

		for (size_t axis = 0; axis < 3; axis++) {
			sensorConfig.A_sixSideOff[axis] = sixSideOff[axis];
			sensorConfig.A_sixSideScale[axis] = sixSideScale[axis];
		}

		sampleLedPulseActive = false;
		ledManager.off();
		return TickResult::DONE;
	}

	void cancel() override final {
		calibrationData.reset();
		sampleLedPulseActive = false;
		ledManager.off();
	}

	bool requiresRest() override final { return false; }

	void processAccelSample(const SensorRawT accelSample[3]) override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return;
		}

		if (!shouldAcceptSample(accelSample)) {
			return;
		}

		auto& data = calibrationData.value();
		data.samples[data.writeIndex] = {
			static_cast<int32_t>(accelSample[0]),
			static_cast<int32_t>(accelSample[1]),
			static_cast<int32_t>(accelSample[2]),
		};
		data.writeIndex = (data.writeIndex + 1) % samplePerPositionCount;
		if (data.storedSampleCount < samplePerPositionCount) {
			data.storedSampleCount++;
		}

		triggerSampleLedPulse();
	}

	void setRestDetected(const bool rest) override final { restDetected = rest; }

private:
	static constexpr uint8_t maxCalibrationTimeSeconds = 120;
	static constexpr size_t samplePerPositionCount = 160;
	static constexpr float allowableVerticalAxisPercentage = 0.05f;
	static constexpr float minAxisDeltaMs2 = CONST_EARTH_GRAVITY * 0.5f;
	static constexpr float minValidScale = 0.1f;
	static constexpr float maxValidScale = 10.0f;
	static constexpr float maxValidOffset = CONST_EARTH_GRAVITY * 2.0f;
	static constexpr uint16_t sampleLedPulseWidthMs = 2;

	bool shouldAcceptSample(const SensorRawT accelSample[3]) {
		if (!calibrationData.has_value()) {
			return false;
		}

		if (!restDetected) {
			return false;
		}

		float absAxes[3]{
			std::abs(static_cast<float>(accelSample[0])),
			std::abs(static_cast<float>(accelSample[1])),
			std::abs(static_cast<float>(accelSample[2])),
		};

		size_t largestAxis;
		if (absAxes[AXIS_X] > absAxes[AXIS_Y] && absAxes[AXIS_X] > absAxes[AXIS_Z]) {
			largestAxis = accelSample[AXIS_X] < 0 ? AXIS_X_DOWN : AXIS_X;
		} else if (absAxes[AXIS_Y] > absAxes[AXIS_Z]) {
			largestAxis = accelSample[AXIS_Y] < 0 ? AXIS_Y_DOWN : AXIS_Y;
		} else {
			largestAxis = accelSample[AXIS_Z] < 0 ? AXIS_Z_DOWN : AXIS_Z;
		}

		const float largestAxisValue = absAxes[largestAxis % 3];
		if (!std::isfinite(largestAxisValue)) {
			return false;
		}

		const float smallAxisPercentage1
			= absAxes[(largestAxis + 1) % 3] / largestAxisValue;
		const float smallAxisPercentage2
			= absAxes[(largestAxis + 2) % 3] / largestAxisValue;

		if ((smallAxisPercentage1 > allowableVerticalAxisPercentage)
			|| (smallAxisPercentage2 > allowableVerticalAxisPercentage)) {
			return false;
		}

		auto& data = calibrationData.value();
		if (data.currentAxis == AXIS_COUNT && !data.positionRecorded[largestAxis]) {
			data.currentAxis = static_cast<int8_t>(largestAxis);
		} else if (data.currentAxis != static_cast<int8_t>(largestAxis)) {
			return false;
		}

		return true;
	}

	bool storeCurrentAxisAverage() {
		auto& data = calibrationData.value();
		if (data.currentAxis < 0 || data.currentAxis >= AXIS_COUNT) {
			return false;
		}
		const int8_t recordedAxis = data.currentAxis;
		const size_t sampleAxis = static_cast<size_t>(recordedAxis) % 3;

		float sum = 0.0f;
		for (size_t i = 0; i < samplePerPositionCount; i++) {
			sum += static_cast<float>(data.samples[i][sampleAxis]);
		}
		data.axisSamples[recordedAxis] = sum / samplePerPositionCount;
		data.positionRecorded[recordedAxis] = true;

		data.storedSampleCount = 0;
		data.writeIndex = 0;
		data.currentAxis = AXIS_COUNT;

		logger.info(
			"Recorded accel six-side position %d with average raw %.2f",
			recordedAxis,
			data.axisSamples[recordedAxis]
		);
		return true;
	}

	bool computeAxisCalibration(
		float axisUpRaw,
		float axisDownRaw,
		float& outOffset,
		float& outScale
	) const {
		// Xoffset = ((Xup + Xdown) / (Xup - Xdown)) * g
		// Xscale = 2g / (Xup - Xdown)
		const float rawDelta = axisUpRaw - axisDownRaw;
		const float deltaMs2 = rawDelta * accelScale;
		if (!std::isfinite(rawDelta) || !std::isfinite(deltaMs2)
			|| std::abs(deltaMs2) < minAxisDeltaMs2) {
			return false;
		}

		outScale = (2.0f * CONST_EARTH_GRAVITY) / deltaMs2;
		outOffset = ((axisUpRaw + axisDownRaw) / rawDelta) * CONST_EARTH_GRAVITY;
		if (!std::isfinite(outScale) || !std::isfinite(outOffset)) {
			return false;
		}

		if (std::abs(outScale) < minValidScale || std::abs(outScale) > maxValidScale
			|| std::abs(outOffset) > maxValidOffset) {
			return false;
		}

		return true;
	}

	void updateSampleLedPulse() {
		if (!sampleLedPulseActive) {
			return;
		}

		const uint32_t now = millis();
		if (now >= sampleLedPulseEndMillis) {
			ledManager.off();
			sampleLedPulseActive = false;
		}
	}

	void triggerSampleLedPulse() {
		const uint32_t now = millis();
		ledManager.on();
		sampleLedPulseEndMillis = now + sampleLedPulseWidthMs;
		sampleLedPulseActive = true;
	}

	bool isAllPositionsRecorded() const {
		const auto& data = calibrationData.value();
		for (size_t i = 0; i < AXIS_COUNT; i++) {
			if (!data.positionRecorded[i]) {
				return false;
			}
		}
		return true;
	}

	struct CalibrationData {
		uint64_t startMillis = 0;
		size_t storedSampleCount = 0;
		size_t writeIndex = 0;
		int8_t currentAxis = AXIS_COUNT;
		std::array<float, AXIS_COUNT> axisSamples{};
		std::array<bool, AXIS_COUNT>
			positionRecorded{false, false, false, false, false, false};
		std::array<std::array<int32_t, 3>, samplePerPositionCount> samples{};
	};

	std::optional<CalibrationData> calibrationData;
	SlimeVR::Logging::Logger& logger;
	float accelScale;
	bool restDetected = false;
	uint32_t sampleLedPulseEndMillis = 0;
	bool sampleLedPulseActive = false;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
