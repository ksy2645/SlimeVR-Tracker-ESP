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
#include "../../../motionprocessing/RestDetection.h"
#include "CalibrationStep.h"
#include "logging/Logger.h"
#include "magneto1.4.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

namespace {
static constexpr int8_t AXIS_COUNT = 6;
}  // namespace

template <typename SensorRawT>
class AccelSixSideMagnetoCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	AccelSixSideMagnetoCalibrationStep(
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
		auto& data = calibrationData.value();
		data.startMillis = millis();
		data.sampleCollectionStartMillis
			= data.startMillis + calibrationPreparationSeconds * 1e3;
		statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, true);
		sampleLedPulseActive = false;
		ledManager.off();
		logger.info(
			"Put the device into 6 unique orientations (all sides), leave it still "
			"and do not hold/touch for %u seconds each",
			static_cast<unsigned>(legacyRestDetectionMinTimeSeconds)
		);
	}

	TickResult tick() override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return TickResult::CONTINUE;
		}

		auto& data = calibrationData.value();
		if (!data.collectionStarted) {
			if (millis() < data.sampleCollectionStartMillis) {
				return TickResult::CONTINUE;
			}

			data.collectionStarted = true;
			initializeRestDetection(data);
			ledManager.off();
			ledManager.pattern(100, 100, 6);
			ledManager.off();
			logger.info("Gathering accelerometer data...");
			logger.info(
				"Waiting for position %u, you can leave the device as is...",
				static_cast<unsigned>(data.positionsRecorded + 1)
			);
		}

		if (data.positionsRecorded < expectedPositionCount) {
			maybeLogCollectionProgress(data);
			return TickResult::CONTINUE;
		}

		float magnetoFit[4][3];
		data.magneto.current_calibration(magnetoFit);
		if (!isMagnetoFitValid(magnetoFit)) {
			logger.warn(
				"Accel six-side legacy magneto fit rejected as invalid, restarting "
				"sample collection"
			);
			resetCollectionForRetry(data);
			logger.info(
				"Waiting for position %u, you can leave the device as is...",
				static_cast<unsigned>(data.positionsRecorded + 1)
			);
			return TickResult::CONTINUE;
		}
		storeCalibration(magnetoFit);

		logger.info(
			"Accel six-side legacy magneto calibration completed with %u samples "
			"(discarded %u invalid samples)",
			static_cast<unsigned>(data.magnetoSampleCount),
			static_cast<unsigned>(data.discardedSampleCount)
		);
		stopCalibrationFeedback();
		return TickResult::DONE;
	}

	void cancel() override final {
		calibrationData.reset();
		stopCalibrationFeedback();
	}

	bool requiresRest() override final { return false; }

	void processAccelSample(const SensorRawT accelSample[3]) override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return;
		}

		auto& data = calibrationData.value();
		if (!data.collectionStarted) {
			return;
		}

		if (!data.restDetection.has_value()) {
			return;
		}

		const sensor_real_t scaledData[3] = {
			static_cast<sensor_real_t>(accelSample[0]) * accelScale,
			static_cast<sensor_real_t>(accelSample[1]) * accelScale,
			static_cast<sensor_real_t>(accelSample[2]) * accelScale,
		};
		data.restDetection.value().updateAcc(data.accelSamplePeriodSeconds, scaledData);

		if (data.waitForMotion) {
			if (!data.restDetection.value().getRestDetected()) {
				data.waitForMotion = false;
			}
			return;
		}

		if (!data.restDetection.value().getRestDetected()) {
			data.currentPositionSampleCount = 0;
			return;
		}

		if (!isAccelSampleValid(accelSample)) {
			data.discardedSampleCount++;
			return;
		}

		const size_t idx = data.currentPositionSampleCount;
		if (idx < samplePerPositionCount) {
			data.positionSamples[idx] = {accelSample[0], accelSample[1], accelSample[2]};
			data.currentPositionSampleCount++;
			triggerSampleLedPulse();
		}

		if (data.currentPositionSampleCount < samplePerPositionCount) {
			return;
		}

		for (size_t i = 0; i < samplePerPositionCount; i++) {
			data.magneto.sample(
				static_cast<double>(data.positionSamples[i][0]),
				static_cast<double>(data.positionSamples[i][1]),
				static_cast<double>(data.positionSamples[i][2])
			);
		}
		data.magnetoSampleCount += samplePerPositionCount;
		data.positionsRecorded++;
		data.currentPositionSampleCount = 0;

		if (data.positionsRecorded < expectedPositionCount) {
			ledManager.pattern(50, 50, 2);
			ledManager.off();
			logger.info(
				"Recorded, waiting for position %u...",
				static_cast<unsigned>(data.positionsRecorded + 1)
			);
			data.waitForMotion = true;
		}
	}

	void setRestDetected(const bool rest) override final { (void)rest; }

private:
	struct CalibrationData;

	static constexpr size_t samplePerPositionCount = 96;
	static constexpr uint8_t expectedPositionCount = AXIS_COUNT;
	static constexpr uint8_t calibrationPreparationSeconds = 3;
	static constexpr float legacyRestDetectionMinTimeSeconds = 3.0f;
	static constexpr float legacyRestDetectionAccThreshold = 0.25f;
	static constexpr float minAccelSampleNormMs2 = 4.0f;
	static constexpr float maxAccelSampleNormMs2 = 12.0f;
	static constexpr float maxLegacyFitElementAbs = 20.0f;
	static constexpr float minLegacyFitMeanDiagonalAbs = 1e-4f;
	static constexpr float maxLegacyFitMeanDiagonalAbs = 10.0f;
	static constexpr uint16_t progressLogIntervalMs = 3000;
	static constexpr uint16_t sampleLedPulseWidthMs = 10;

	void initializeRestDetection(CalibrationData& data) {
		RestDetectionParams restParams;
		restParams.restMinTime = legacyRestDetectionMinTimeSeconds;
		restParams.restThAcc = legacyRestDetectionAccThreshold;
		data.accelSamplePeriodSeconds = sensorConfig.A_Ts;
		data.restDetection.emplace(restParams, sensorConfig.G_Ts, sensorConfig.A_Ts);
	}

	void storeCalibration(const float magnetoFit[4][3]) {
		for (size_t i = 0; i < 3; i++) {
			sensorConfig.A_sixSideLegacyB[i] = magnetoFit[0][i];
			sensorConfig.A_sixSideLegacyAinv[0][i] = magnetoFit[1][i];
			sensorConfig.A_sixSideLegacyAinv[1][i] = magnetoFit[2][i];
			sensorConfig.A_sixSideLegacyAinv[2][i] = magnetoFit[3][i];
		}

		logger.info(
			"Accel six-side legacy magneto B: %f %f %f",
			sensorConfig.A_sixSideLegacyB[0],
			sensorConfig.A_sixSideLegacyB[1],
			sensorConfig.A_sixSideLegacyB[2]
		);
		logger.info("Accel six-side legacy magneto Ainv:");
		for (size_t i = 0; i < 3; i++) {
			logger.info(
				"  %f %f %f",
				sensorConfig.A_sixSideLegacyAinv[i][0],
				sensorConfig.A_sixSideLegacyAinv[i][1],
				sensorConfig.A_sixSideLegacyAinv[i][2]
			);
		}
	}

	bool isAccelSampleValid(const SensorRawT accelSample[3]) const {
		const float x = static_cast<float>(accelSample[0]) * accelScale;
		const float y = static_cast<float>(accelSample[1]) * accelScale;
		const float z = static_cast<float>(accelSample[2]) * accelScale;
		if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
			return false;
		}

		const float norm = std::sqrt(x * x + y * y + z * z);
		return std::isfinite(norm) && norm >= minAccelSampleNormMs2
			&& norm <= maxAccelSampleNormMs2;
	}

	static bool isMagnetoFitValid(const float magnetoFit[4][3]) {
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (uint8_t i = 0; i < 3; i++) {
			if (!std::isfinite(magnetoFit[0][i])) {
				return false;
			}
			for (uint8_t j = 0; j < 3; j++) {
				const float value = magnetoFit[i + 1][j];
				if (!std::isfinite(value) || std::abs(value) > maxLegacyFitElementAbs) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(magnetoFit[i + 1][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal)
			|| meanDiagonal < minLegacyFitMeanDiagonalAbs
			|| meanDiagonal > maxLegacyFitMeanDiagonalAbs) {
			return false;
		}

		const float diagonalTolerance = std::max(meanDiagonal * 0.6f, 0.05f);
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(diagonalAbs[i] - meanDiagonal) > diagonalTolerance) {
				return false;
			}
		}

		return true;
	}

	static void resetCollectionForRetry(CalibrationData& data) {
		data.positionsRecorded = 0;
		data.currentPositionSampleCount = 0;
		data.waitForMotion = true;
		data.magnetoSampleCount = 0;
		data.magneto = MagnetoCalibration{};
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

	void stopCalibrationFeedback() {
		sampleLedPulseActive = false;
		statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
		ledManager.off();
	}

	void maybeLogCollectionProgress(CalibrationData& data) {
		const uint32_t now = millis();
		if (now - data.lastProgressLogMillis < progressLogIntervalMs) {
			return;
		}

		data.lastProgressLogMillis = now;
		const bool restNow
			= data.restDetection.has_value() && data.restDetection.value().getRestDetected();

		logger.info(
			"Accel six-side legacy magneto progress: positions %u/%u, pos-samples %u/%u, "
			"waitForMotion=%u, rest=%u",
			static_cast<unsigned>(data.positionsRecorded),
			static_cast<unsigned>(expectedPositionCount),
			static_cast<unsigned>(data.currentPositionSampleCount),
			static_cast<unsigned>(samplePerPositionCount),
			static_cast<unsigned>(data.waitForMotion ? 1 : 0),
			static_cast<unsigned>(restNow ? 1 : 0)
		);
	}

	struct CalibrationData {
		uint64_t startMillis = 0;
		uint64_t sampleCollectionStartMillis = 0;
		bool collectionStarted = false;
		uint16_t positionsRecorded = 0;
		uint16_t currentPositionSampleCount = 0;
		bool waitForMotion = true;
		float accelSamplePeriodSeconds = 0.0f;
		size_t magnetoSampleCount = 0;
		size_t discardedSampleCount = 0;
		uint32_t lastProgressLogMillis = 0;
		std::optional<RestDetection> restDetection;
		std::array<std::array<SensorRawT, 3>, samplePerPositionCount> positionSamples{};
		MagnetoCalibration magneto;
	};

	std::optional<CalibrationData> calibrationData;
	SlimeVR::Logging::Logger& logger;
	float accelScale;
	uint32_t sampleLedPulseEndMillis = 0;
	bool sampleLedPulseActive = false;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
