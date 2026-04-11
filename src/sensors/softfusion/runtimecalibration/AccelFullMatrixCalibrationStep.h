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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

#include "../../../GlobalVars.h"
#include "CalibrationStep.h"
#include "logging/Logger.h"
#include "magneto1.4.h"

#ifndef RUNTIME_CALIBRATION_ENABLE_ACCEL_FULL_MATRIX_VERBOSE_LOGS
#define RUNTIME_CALIBRATION_ENABLE_ACCEL_FULL_MATRIX_VERBOSE_LOGS 0
#endif

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename SensorRawT>
class AccelFullMatrixCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	struct CalibrationOutput {
		float A_B[3];
		float A_Ainv[3][3];
		float referenceAccelNormRaw;
		size_t sampleCount;
	};

	AccelFullMatrixCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig,
		SlimeVR::Logging::Logger& logger,
		float gyroScaleRadPerSecPerRaw
	)
		: CalibrationStep<SensorRawT>{sensorConfig}
		, logger{logger}
		, maxReferenceCaptureGyroNormRaw{convertGyroNormDpsToRawThreshold(
			  gyroScaleRadPerSecPerRaw,
			  maxReferenceCaptureGyroNormDps
		  )}
		, maxSampleCollectionHoldStillGyroNormRaw{convertGyroNormDpsToRawThreshold(
			  gyroScaleRadPerSecPerRaw,
			  maxSampleCollectionHoldStillGyroNormDps
		  )} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = CalibrationData{};
		calibrationData.value().startMillis = millis();
		calibrationData.value().stage = Stage::CAPTURE_REFERENCE;
		statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, true);
		latestCalibration.reset();
		sampleLedPulseActive = false;
		hasRecentGyroSample = false;
		lastGyroSampleMillis = 0;
		gyroNormRawEma = 0.0f;
		// Hold-still phase indicator
		ledManager.on();
		logger.info(
			"Accel full-matrix calibration started: keep tracker still to capture "
			"gravity norm, then hold still at many different orientations"
		);
	}

	TickResult tick() override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return TickResult::CONTINUE;
		}

		auto& data = calibrationData.value();
		const uint64_t elapsedMillis = millis() - data.startMillis;
		if (elapsedMillis > maxCalibrationTimeSeconds * 1e3) {
			logger.warn(
				"Accel full-matrix calibration timed out (%u/%u samples, "
				"coverage +X:%u -X:%u +Y:%u -Y:%u +Z:%u -Z:%u)",
				static_cast<unsigned>(data.storedSampleCount),
				static_cast<unsigned>(minCalibrationSampleCount),
				static_cast<unsigned>(data.poseGroupCounts[0]),
				static_cast<unsigned>(data.poseGroupCounts[1]),
				static_cast<unsigned>(data.poseGroupCounts[2]),
				static_cast<unsigned>(data.poseGroupCounts[3]),
				static_cast<unsigned>(data.poseGroupCounts[4]),
				static_cast<unsigned>(data.poseGroupCounts[5])
			);
			sampleLedPulseActive = false;
			statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
			ledManager.off();
			return TickResult::SKIP;
		}

		if (data.stage == Stage::CAPTURE_REFERENCE) {
			if (data.referenceSampleCount < referenceSampleCount) {
				return TickResult::CONTINUE;
			}

			data.referenceAccelNormRaw = data.referenceAccelNormRawAccum
									   / static_cast<float>(data.referenceSampleCount);
			if (!std::isfinite(data.referenceAccelNormRaw)
				|| data.referenceAccelNormRaw < minReferenceAccelNormRaw) {
				logger.warn("Accel full-matrix reference norm capture failed");
				sampleLedPulseActive = false;
				statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
				ledManager.off();
				return TickResult::SKIP;
			}

			data.stage = Stage::COLLECT_CALIBRATION_SAMPLES;
			data.lastAcceptedSample.reset();
			data.poseAnchorDirection.reset();
			data.poseSampleCount = 0;
			data.poseSamplesCommitted = false;
			data.holdDirection.reset();
			data.holdAccelNormRaw = 0.0f;
			data.holdStartMillis = 0;
			// Phase switch feedback
			ledManager.pattern(60, 60, 2);
			ledManager.off();
			logger.info(
				"Accel full-matrix reference norm ready: %.2f raw (tolerance +/- %.2f, "
				"%u samples per pose)",
				data.referenceAccelNormRaw,
				std::max(
					data.referenceAccelNormRaw * sampleNormToleranceRatio,
					sampleNormToleranceRawMin
				),
				static_cast<unsigned>(samplesPerPose)
			);
			return TickResult::CONTINUE;
		}

		maybeLogCollectionProgress(data);

		if (data.storedSampleCount < minCalibrationSampleCount) {
			return TickResult::CONTINUE;
		}
		if (!hasDirectionCoverage(data)) {
#if RUNTIME_CALIBRATION_ENABLE_ACCEL_FULL_MATRIX_VERBOSE_LOGS
			const uint32_t now = millis();
			if (now - data.lastCoverageLogMillis >= 3000) {
				data.lastCoverageLogMillis = now;
				const uint8_t missXP = data.poseGroupCounts[0] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[0]);
				const uint8_t missXN = data.poseGroupCounts[1] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[1]);
				const uint8_t missYP = data.poseGroupCounts[2] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[2]);
				const uint8_t missYN = data.poseGroupCounts[3] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[3]);
				const uint8_t missZP = data.poseGroupCounts[4] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[4]);
				const uint8_t missZN = data.poseGroupCounts[5] >= minPoseGroupsPerDirection
										 ? 0
										 : (minPoseGroupsPerDirection
											- data.poseGroupCounts[5]);
				logger.info(
					"Accel full-matrix waiting for direction coverage "
					"(samples %u/%u, +X:%u -X:%u +Y:%u -Y:%u +Z:%u -Z:%u, "
					"missing +X:%u -X:%u +Y:%u -Y:%u +Z:%u -Z:%u)",
					static_cast<unsigned>(data.storedSampleCount),
					static_cast<unsigned>(minCalibrationSampleCount),
					static_cast<unsigned>(data.poseGroupCounts[0]),
					static_cast<unsigned>(data.poseGroupCounts[1]),
					static_cast<unsigned>(data.poseGroupCounts[2]),
					static_cast<unsigned>(data.poseGroupCounts[3]),
					static_cast<unsigned>(data.poseGroupCounts[4]),
					static_cast<unsigned>(data.poseGroupCounts[5]),
					static_cast<unsigned>(missXP),
					static_cast<unsigned>(missXN),
					static_cast<unsigned>(missYP),
					static_cast<unsigned>(missYN),
					static_cast<unsigned>(missZP),
					static_cast<unsigned>(missZN)
				);
			}
#endif
			return TickResult::CONTINUE;
		}

		float A_BAinv[4][3];
		computeCalibration(A_BAinv);

		if (!isCalibrationMatrixSane(A_BAinv)) {
			logger.warn("Accel full-matrix matrix sanity check failed");
			sampleLedPulseActive = false;
			statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
			ledManager.off();
			return TickResult::SKIP;
		}

		if (!isCalibrationQualityValid(A_BAinv)) {
			logger.warn("Accel full-matrix quality check failed");
			sampleLedPulseActive = false;
			statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
			ledManager.off();
			return TickResult::SKIP;
		}

		CalibrationOutput out{};
		for (uint8_t i = 0; i < 3; i++) {
			out.A_B[i] = A_BAinv[0][i];
			out.A_Ainv[0][i] = A_BAinv[1][i];
			out.A_Ainv[1][i] = A_BAinv[2][i];
			out.A_Ainv[2][i] = A_BAinv[3][i];
		}
		out.referenceAccelNormRaw = data.referenceAccelNormRaw;
		out.sampleCount = data.storedSampleCount;
		latestCalibration = out;
		data.stage = Stage::COMPLETE;

		logger.info(
			"Accel full-matrix calibration completed (%u samples, reference norm %.2f)",
			static_cast<unsigned>(out.sampleCount),
			out.referenceAccelNormRaw
		);
		logger.info(
			"Accel calibration A_B: %f %f %f",
			out.A_B[0],
			out.A_B[1],
			out.A_B[2]
		);
		logger.info("Accel calibration A_Ainv:");
		logger.info("  %f %f %f", out.A_Ainv[0][0], out.A_Ainv[0][1], out.A_Ainv[0][2]);
		logger.info("  %f %f %f", out.A_Ainv[1][0], out.A_Ainv[1][1], out.A_Ainv[1][2]);
		logger.info("  %f %f %f", out.A_Ainv[2][0], out.A_Ainv[2][1], out.A_Ainv[2][2]);

		sampleLedPulseActive = false;
		statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
		ledManager.off();
		return TickResult::DONE;
	}

	void cancel() override final {
		calibrationData.reset();
		sampleLedPulseActive = false;
		statusManager.setStatus(SlimeVR::Status::ACCEL_CALIBRATING, false);
		ledManager.off();
	}

	bool requiresRest() override final { return false; }

	void setRestDetected(const bool rest) override final { restDetected = rest; }

	void processGyroSample(const SensorRawT gyroSample[3]) override final {
		const float gx = static_cast<float>(gyroSample[0]);
		const float gy = static_cast<float>(gyroSample[1]);
		const float gz = static_cast<float>(gyroSample[2]);
		const float gyroNormRaw = std::sqrt(gx * gx + gy * gy + gz * gz);
		if (!std::isfinite(gyroNormRaw)) {
			return;
		}

		const uint32_t now = millis();
		lastGyroSampleMillis = now;
		if (!hasRecentGyroSample) {
			gyroNormRawEma = gyroNormRaw;
			hasRecentGyroSample = true;
			return;
		}

		gyroNormRawEma
			= gyroEmaAlpha * gyroNormRaw + (1.0f - gyroEmaAlpha) * gyroNormRawEma;
	}

	void processAccelSample(const SensorRawT accelSample[3]) override final {
		updateSampleLedPulse();

		if (!calibrationData.has_value()) {
			return;
		}

		auto& data = calibrationData.value();
		if (data.stage == Stage::CAPTURE_REFERENCE) {
			if (!shouldAcceptReferenceSample(accelSample)) {
				return;
			}

			const float accelNormRawValue = accelNormRaw(accelSample);
			data.referenceAccelNormRawAccum += accelNormRawValue;
			data.referenceSampleCount++;
			return;
		}

		if (data.stage != Stage::COLLECT_CALIBRATION_SAMPLES) {
			return;
		}

		if (!shouldAcceptCalibrationSample(accelSample)) {
			return;
		}
		const uint8_t poseIndex
			= data.poseSampleCount > 0 ? data.poseSampleCount - 1 : 0;
		if (poseIndex < samplesPerPose) {
			data.poseSamples[poseIndex]
				= {accelSample[0], accelSample[1], accelSample[2]};
		}
		if (!data.poseSamplesCommitted && data.poseSampleCount >= samplesPerPose) {
			commitPoseSamplesWeighted(data);
			data.poseSamplesCommitted = true;
		}
		triggerSampleLedPulse();
	}

	bool hasResult() const { return latestCalibration.has_value(); }

	const std::optional<CalibrationOutput>& getResult() const {
		return latestCalibration;
	}

private:
	struct CalibrationData;

	enum class Stage {
		CAPTURE_REFERENCE,
		COLLECT_CALIBRATION_SAMPLES,
		COMPLETE,
	};

	enum class RejectReason : uint8_t {
		ZERO_SAMPLE = 0,
		RAW_ABS_LIMIT,
		NORM_NONFINITE,
		UNIT_DIRECTION,
		NORM_TOLERANCE,
		HOLD_STILL,
		POSE_DOT_NONFINITE,
		SPIKE_DELTA,
		COUNT,
	};

	static constexpr size_t referenceSampleCount = 120;
	static constexpr size_t calibrationWindowSampleCount = 240;
	static constexpr size_t minCalibrationSampleCount = 80;
	static constexpr uint16_t maxCalibrationTimeSeconds = 420;
	static constexpr float minReferenceAccelNormRaw = 100.0f;
	static constexpr uint8_t samplesPerPose = 9;
	static constexpr float minSamePoseDirectionDot
		= std::cos(6 * PI / 180);  // 0.9945219f;  // cos(6 deg)
	static constexpr float maxNextPoseDirectionDot
		= std::cos(20 * PI / 180);  // 0.9396926f;  // cos(20 deg)
	// ~313 raw tolerance when referenceAccelNormRaw is around 8245
	static constexpr float sampleNormToleranceRatio = 0.038f;
	static constexpr float sampleNormToleranceRawMin = 60.0f;
	static constexpr bool allowGyroStillReferenceWithoutRest = true;
	static constexpr bool requireGyroStillForSampling = true;
	// FSR/dps changes auto-apply without touching this file.
	// Used during CAPTURE_REFERENCE stage.
	static constexpr float maxReferenceCaptureGyroNormDps = 9.8f;
	// Used during COLLECT_CALIBRATION_SAMPLES hold-still gate.
	static constexpr float maxSampleCollectionHoldStillGyroNormDps = 7.7f;
	static constexpr uint16_t gyroSampleStaleTimeoutMs = 250;
	static constexpr float gyroEmaAlpha = 0.25f;
	static constexpr uint16_t progressLogIntervalMs = 3000;
	static constexpr uint16_t rejectLogIntervalMs = 3000;
	static constexpr bool requireHoldStillForSampling = true;
	static constexpr uint16_t minHoldStillDurationMs = 200;
	static constexpr float minHoldStillDirectionDot
		= std::cos(4 * PI / 180);  // 0.9975641f;  // cos(4 deg)
	static constexpr float maxHoldStillAccelNormDeltaRaw = 80.0f;
	static constexpr int32_t maxSpikeDeltaRaw = 300;
	static constexpr int32_t maxRawAbsValue = 32000;
	static constexpr float maxMagnitudeDeviationRatio = 0.12f;
	static constexpr uint8_t minPoseGroupsPerDirection = 3;
	static constexpr uint16_t sampleLedPulseWidthMs = 30;

	static float accelNormRaw(const SensorRawT sample[3]) {
		const float x = static_cast<float>(sample[0]);
		const float y = static_cast<float>(sample[1]);
		const float z = static_cast<float>(sample[2]);
		return std::sqrt(x * x + y * y + z * z);
	}

	static bool toUnitDirection(
		const SensorRawT sample[3],
		std::array<float, 3>& outUnitDirection
	) {
		const float accelNormRawValue = accelNormRaw(sample);
		if (!std::isfinite(accelNormRawValue) || accelNormRawValue <= 1e-6f) {
			return false;
		}

		const float invNorm = 1.0f / accelNormRawValue;
		outUnitDirection = {
			static_cast<float>(sample[0]) * invNorm,
			static_cast<float>(sample[1]) * invNorm,
			static_cast<float>(sample[2]) * invNorm,
		};
		return true;
	}

	bool shouldAcceptReferenceSample(const SensorRawT sample[3]) const {
		if (sample[0] == 0 && sample[1] == 0 && sample[2] == 0) {
			return false;
		}

		const float accelNormRawValue = accelNormRaw(sample);
		if (!std::isfinite(accelNormRawValue)) {
			return false;
		}

		if (restDetected) {
			return true;
		}

		if (!allowGyroStillReferenceWithoutRest) {
			return false;
		}

		return hasFreshGyroSample() && gyroNormRawEma <= maxReferenceCaptureGyroNormRaw;
	}

	bool shouldAcceptCalibrationSample(const SensorRawT sample[3]) {
		if (!calibrationData.has_value()) {
			return false;
		}
		auto& data = calibrationData.value();

		if (sample[0] == 0 && sample[1] == 0 && sample[2] == 0) {
			recordReject(data, RejectReason::ZERO_SAMPLE);
			return false;
		}

		if (std::abs(static_cast<int32_t>(sample[0])) > maxRawAbsValue
			|| std::abs(static_cast<int32_t>(sample[1])) > maxRawAbsValue
			|| std::abs(static_cast<int32_t>(sample[2])) > maxRawAbsValue) {
			recordReject(data, RejectReason::RAW_ABS_LIMIT);
			return false;
		}

		const float accelNormRawValue = accelNormRaw(sample);
		if (!std::isfinite(accelNormRawValue)) {
			recordReject(data, RejectReason::NORM_NONFINITE);
			return false;
		}

		std::array<float, 3> currentDirection{};
		if (!toUnitDirection(sample, currentDirection)) {
			recordReject(data, RejectReason::UNIT_DIRECTION);
			return false;
		}

		const float allowedNormDelta = std::max(
			data.referenceAccelNormRaw * sampleNormToleranceRatio,
			sampleNormToleranceRawMin
		);
		const float normDeltaAbs
			= std::abs(accelNormRawValue - data.referenceAccelNormRaw);
		data.lastAccelNormRaw = accelNormRawValue;
		data.lastAccelNormDeltaAbsRaw = normDeltaAbs;
		data.lastAllowedAccelNormDeltaRaw = allowedNormDelta;
		data.lastAccelNormRejected = normDeltaAbs > allowedNormDelta;
		if (data.lastAccelNormRejected) {
			recordReject(data, RejectReason::NORM_TOLERANCE);
			return false;
		}

		if (requireHoldStillForSampling
			&& !isHoldStillSatisfied(data, currentDirection, accelNormRawValue)) {
			recordReject(data, RejectReason::HOLD_STILL);
			return false;
		}

		if (!data.poseAnchorDirection.has_value() || data.poseSampleCount == 0) {
			beginPoseGroup(data, sample, currentDirection);
			return true;
		}

		float poseDot = data.poseAnchorDirection.value()[0] * currentDirection[0]
					  + data.poseAnchorDirection.value()[1] * currentDirection[1]
					  + data.poseAnchorDirection.value()[2] * currentDirection[2];
		if (!std::isfinite(poseDot)) {
			recordReject(data, RejectReason::POSE_DOT_NONFINITE);
			return false;
		}
		poseDot = std::clamp(poseDot, -1.0f, 1.0f);

		// Current pose group is complete; wait until orientation changes enough.
		if (data.poseSampleCount >= samplesPerPose) {
			if (poseDot > maxNextPoseDirectionDot) {
				data.waitForNextPoseCount++;
				// Explicitly clear previous-pose continuity while waiting for next pose.
				data.lastAcceptedSample.reset();
				return false;
			}

			beginPoseGroup(data, sample, currentDirection);
			data.poseRestartCount++;
			return true;
		}

		// Orientation drifted too far before completing this pose group: restart group.
		if (poseDot < minSamePoseDirectionDot) {
			beginPoseGroup(data, sample, currentDirection);
			data.poseRestartCount++;
			return true;
		}

		if (!data.lastAcceptedSample.has_value()) {
			beginPoseGroup(data, sample, currentDirection);
			data.poseRestartCount++;
			return true;
		}

		const auto& prev = data.lastAcceptedSample.value();
		const int32_t dx
			= std::abs(static_cast<int32_t>(sample[0]) - static_cast<int32_t>(prev[0]));
		const int32_t dy
			= std::abs(static_cast<int32_t>(sample[1]) - static_cast<int32_t>(prev[1]));
		const int32_t dz
			= std::abs(static_cast<int32_t>(sample[2]) - static_cast<int32_t>(prev[2]));
		if (dx > maxSpikeDeltaRaw || dy > maxSpikeDeltaRaw || dz > maxSpikeDeltaRaw) {
			recordReject(data, RejectReason::SPIKE_DELTA);
			return false;
		}

		data.poseSampleCount++;
		data.lastAcceptedSample = {sample[0], sample[1], sample[2]};
		return true;
	}

	void computeCalibration(float outA_BAinv[4][3]) {
		auto& data = calibrationData.value();
		MagnetoCalibration magneto;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			magneto.sample(
				static_cast<double>(data.samples[i][0]),
				static_cast<double>(data.samples[i][1]),
				static_cast<double>(data.samples[i][2])
			);
		}
		magneto.current_calibration(outA_BAinv);
	}

	static float calibratedMagnitude(
		const std::array<SensorRawT, 3>& rawSample,
		const float A_BAinv[4][3]
	) {
		const float tx = static_cast<float>(rawSample[0]) - A_BAinv[0][0];
		const float ty = static_cast<float>(rawSample[1]) - A_BAinv[0][1];
		const float tz = static_cast<float>(rawSample[2]) - A_BAinv[0][2];

		const float x = A_BAinv[1][0] * tx + A_BAinv[1][1] * ty + A_BAinv[1][2] * tz;
		const float y = A_BAinv[2][0] * tx + A_BAinv[2][1] * ty + A_BAinv[2][2] * tz;
		const float z = A_BAinv[3][0] * tx + A_BAinv[3][1] * ty + A_BAinv[3][2] * tz;

		return std::sqrt(x * x + y * y + z * z);
	}

	static bool isCalibrationMatrixSane(const float A_BAinv[4][3]) {
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (size_t i = 0; i < 3; i++) {
			if (!std::isfinite(A_BAinv[0][i])) {
				return false;
			}

			for (size_t j = 0; j < 3; j++) {
				const float v = A_BAinv[i + 1][j];
				if (!std::isfinite(v) || std::abs(v) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(A_BAinv[i + 1][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
			return false;
		}

		const float diagonalTolerance = std::max(meanDiagonal * 0.6f, 0.05f);
		for (size_t i = 0; i < 3; i++) {
			if (std::abs(diagonalAbs[i] - meanDiagonal) > diagonalTolerance) {
				return false;
			}
		}

		return true;
	}

	bool isCalibrationQualityValid(const float A_BAinv[4][3]) const {
		const auto& data = calibrationData.value();
		if (data.storedSampleCount < minCalibrationSampleCount) {
			return false;
		}

		float magnitudeSum = 0.0f;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			magnitudeSum += calibratedMagnitude(data.samples[i], A_BAinv);
		}
		const float meanMagnitude
			= magnitudeSum / static_cast<float>(data.storedSampleCount);
		if (!std::isfinite(meanMagnitude) || meanMagnitude < 1e-3f) {
			return false;
		}

		size_t invalidCount = 0;
		const float maxDeviation = meanMagnitude * maxMagnitudeDeviationRatio;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			const float magnitude = calibratedMagnitude(data.samples[i], A_BAinv);
			if (!std::isfinite(magnitude)
				|| std::abs(magnitude - meanMagnitude) > maxDeviation) {
				invalidCount++;
			}
		}

		return invalidCount < (data.storedSampleCount / 5);
	}

	static uint8_t directionBucketFromUnitDirection(
		const std::array<float, 3>& unitDirection
	) {
		const float absX = std::abs(unitDirection[0]);
		const float absY = std::abs(unitDirection[1]);
		const float absZ = std::abs(unitDirection[2]);
		uint8_t axis = 0;
		float maxAbs = absX;
		if (absY > maxAbs) {
			maxAbs = absY;
			axis = 1;
		}
		if (absZ > maxAbs) {
			axis = 2;
		}

		if (axis == 0) {
			return unitDirection[0] >= 0.0f ? 0 : 1;  // +X, -X
		}
		if (axis == 1) {
			return unitDirection[1] >= 0.0f ? 2 : 3;  // +Y, -Y
		}
		return unitDirection[2] >= 0.0f ? 4 : 5;  // +Z, -Z
	}

	bool hasFreshGyroSample() const {
		if (!hasRecentGyroSample) {
			return false;
		}

		const uint32_t now = millis();
		return now - lastGyroSampleMillis <= gyroSampleStaleTimeoutMs;
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

	struct CalibrationData {
		uint64_t startMillis = 0;
		Stage stage = Stage::CAPTURE_REFERENCE;
		float referenceAccelNormRawAccum = 0.0f;
		float referenceAccelNormRaw = 0.0f;
		size_t referenceSampleCount = 0;
		size_t storedSampleCount = 0;
		size_t writeIndex = 0;
		std::array<std::array<SensorRawT, 3>, calibrationWindowSampleCount> samples{};
		std::array<std::array<SensorRawT, 3>, samplesPerPose> poseSamples{};
		std::optional<std::array<float, 3>> poseAnchorDirection;
		uint8_t currentPoseBucket = 0;
		std::array<uint16_t, 6> poseGroupCounts{};
		std::array<uint32_t, static_cast<size_t>(RejectReason::COUNT)> rejectCounts{};
		uint32_t waitForNextPoseCount = 0;
		uint32_t poseRestartCount = 0;
		uint8_t poseSampleCount = 0;
		bool poseSamplesCommitted = false;
		std::optional<std::array<float, 3>> holdDirection;
		float holdAccelNormRaw = 0.0f;
		uint32_t holdStartMillis = 0;
		uint32_t lastCoverageLogMillis = 0;
		uint32_t lastProgressLogMillis = 0;
		uint32_t lastRejectLogMillis = 0;
		float lastAccelNormRaw = 0.0f;
		float lastAccelNormDeltaAbsRaw = 0.0f;
		float lastAllowedAccelNormDeltaRaw = 0.0f;
		bool lastAccelNormRejected = false;
		std::optional<std::array<SensorRawT, 3>> lastAcceptedSample;
	};

	bool hasDirectionCoverage(const CalibrationData& data) const {
		for (uint8_t i = 0; i < data.poseGroupCounts.size(); i++) {
			if (data.poseGroupCounts[i] < minPoseGroupsPerDirection) {
				return false;
			}
		}
		return true;
	}

	static const char* directionBucketName(uint8_t bucket) {
		switch (bucket) {
			case 0:
				return "+X";
			case 1:
				return "-X";
			case 2:
				return "+Y";
			case 3:
				return "-Y";
			case 4:
				return "+Z";
			case 5:
				return "-Z";
			default:
				return "?";
		}
	}

	static const char* rejectReasonName(RejectReason reason) {
		switch (reason) {
			case RejectReason::ZERO_SAMPLE:
				return "zero";
			case RejectReason::RAW_ABS_LIMIT:
				return "raw_abs";
			case RejectReason::NORM_NONFINITE:
				return "norm_nan";
			case RejectReason::UNIT_DIRECTION:
				return "unit_dir";
			case RejectReason::NORM_TOLERANCE:
				return "norm_tol";
			case RejectReason::HOLD_STILL:
				return "hold";
			case RejectReason::POSE_DOT_NONFINITE:
				return "pose_dot_nan";
			case RejectReason::SPIKE_DELTA:
				return "spike";
			case RejectReason::COUNT:
				return "count";
		}
		return "?";
	}

	void recordReject(CalibrationData& data, RejectReason reason) {
		const size_t idx = static_cast<size_t>(reason);
		if (idx < data.rejectCounts.size()) {
			data.rejectCounts[idx]++;
		}
		const bool clearGroupOnReject = data.poseSampleCount < samplesPerPose;
		if (clearGroupOnReject) {
			clearPoseGroup(data);
		}
	}

	void maybeLogCollectionProgress(CalibrationData& data) {
		const uint32_t now = millis();
		if (now - data.lastProgressLogMillis < progressLogIntervalMs) {
			return;
		}

		data.lastProgressLogMillis = now;
		const uint32_t currentSamples = static_cast<uint32_t>(data.storedSampleCount);
		const uint32_t requiredSamples
			= static_cast<uint32_t>(minCalibrationSampleCount);
		const uint32_t remaining
			= currentSamples >= requiredSamples ? 0 : (requiredSamples - currentSamples);
#if RUNTIME_CALIBRATION_ENABLE_ACCEL_FULL_MATRIX_VERBOSE_LOGS
		logger.info(
			"Accel full-matrix progress: %u/%u samples (remaining %u), "
			"coverage (+X:%u -X:%u +Y:%u -Y:%u +Z:%u -Z:%u), gyro=%.1f%s, "
			"norm=%.2f ref=%.2f d=%.2f tol=%.2f%s",
			static_cast<unsigned>(currentSamples),
			static_cast<unsigned>(requiredSamples),
			static_cast<unsigned>(remaining),
			static_cast<unsigned>(data.poseGroupCounts[0]),
			static_cast<unsigned>(data.poseGroupCounts[1]),
			static_cast<unsigned>(data.poseGroupCounts[2]),
			static_cast<unsigned>(data.poseGroupCounts[3]),
			static_cast<unsigned>(data.poseGroupCounts[4]),
			static_cast<unsigned>(data.poseGroupCounts[5]),
			gyroNormRawEma,
			hasFreshGyroSample() ? "" : " (stale)",
			data.lastAccelNormRaw,
			data.referenceAccelNormRaw,
			data.lastAccelNormDeltaAbsRaw,
			data.lastAllowedAccelNormDeltaRaw,
			data.lastAccelNormRejected ? " (norm_reject)" : ""
		);

		if (now - data.lastRejectLogMillis >= rejectLogIntervalMs) {
			data.lastRejectLogMillis = now;
			logger.info(
				"AccelFM rejects: %s=%u %s=%u %s=%u %s=%u %s=%u %s=%u %s=%u %s=%u "
				"wait_next=%u restarts=%u in_pose=%u",
				rejectReasonName(RejectReason::ZERO_SAMPLE),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::ZERO_SAMPLE)]
				),
				rejectReasonName(RejectReason::RAW_ABS_LIMIT),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::RAW_ABS_LIMIT)]
				),
				rejectReasonName(RejectReason::NORM_NONFINITE),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::NORM_NONFINITE)]
				),
				rejectReasonName(RejectReason::UNIT_DIRECTION),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::UNIT_DIRECTION)]
				),
				rejectReasonName(RejectReason::NORM_TOLERANCE),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::NORM_TOLERANCE)]
				),
				rejectReasonName(RejectReason::HOLD_STILL),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::HOLD_STILL)]
				),
				rejectReasonName(RejectReason::POSE_DOT_NONFINITE),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(
						RejectReason::POSE_DOT_NONFINITE
					)]
				),
				rejectReasonName(RejectReason::SPIKE_DELTA),
				static_cast<unsigned>(
					data.rejectCounts[static_cast<size_t>(RejectReason::SPIKE_DELTA)]
				),
				static_cast<unsigned>(data.waitForNextPoseCount),
				static_cast<unsigned>(data.poseRestartCount),
				static_cast<unsigned>(data.poseSampleCount)
			);
		}
#else
		logger.info(
			"Accel full-matrix progress: %u/%u samples (remaining %u), "
			"coverage (+X:%u -X:%u +Y:%u -Y:%u +Z:%u -Z:%u)",
			static_cast<unsigned>(currentSamples),
			static_cast<unsigned>(requiredSamples),
			static_cast<unsigned>(remaining),
			static_cast<unsigned>(data.poseGroupCounts[0]),
			static_cast<unsigned>(data.poseGroupCounts[1]),
			static_cast<unsigned>(data.poseGroupCounts[2]),
			static_cast<unsigned>(data.poseGroupCounts[3]),
			static_cast<unsigned>(data.poseGroupCounts[4]),
			static_cast<unsigned>(data.poseGroupCounts[5])
		);
#endif
	}

	static void beginPoseGroup(
		CalibrationData& data,
		const SensorRawT sample[3],
		const std::array<float, 3>& currentDirection
	) {
		data.poseAnchorDirection = currentDirection;
		data.currentPoseBucket = directionBucketFromUnitDirection(currentDirection);
		data.poseSampleCount = 1;
		data.poseSamplesCommitted = false;
		data.lastAcceptedSample = {sample[0], sample[1], sample[2]};
	}

	static void clearPoseGroup(CalibrationData& data) {
		data.poseAnchorDirection.reset();
		data.poseSampleCount = 0;
		data.poseSamplesCommitted = false;
		data.lastAcceptedSample.reset();
	}

	static constexpr float poseWeight(uint8_t index) {
		const int center = static_cast<int>(samplesPerPose / 2);
		const int distanceFromCenter = std::abs(static_cast<int>(index) - center);
		return static_cast<float>((center + 1) - distanceFromCenter);
	}

	static SensorRawT toSensorRaw(float value) {
		if constexpr (std::is_integral_v<SensorRawT>) {
			long rounded = std::lround(static_cast<double>(value));
			const long minValue
				= static_cast<long>(std::numeric_limits<SensorRawT>::min());
			const long maxValue
				= static_cast<long>(std::numeric_limits<SensorRawT>::max());
			rounded = std::clamp(rounded, minValue, maxValue);
			return static_cast<SensorRawT>(rounded);
		}

		return static_cast<SensorRawT>(value);
	}

	static void appendStoredSample(
		CalibrationData& data,
		SensorRawT x,
		SensorRawT y,
		SensorRawT z
	) {
		data.samples[data.writeIndex] = {x, y, z};
		data.writeIndex = (data.writeIndex + 1) % calibrationWindowSampleCount;
		if (data.storedSampleCount < calibrationWindowSampleCount) {
			data.storedSampleCount++;
		}
	}

	static void commitPoseSamplesWeighted(CalibrationData& data) {
		float weightedMean[3]{0.0f, 0.0f, 0.0f};
		float totalWeight = 0.0f;
		for (uint8_t i = 0; i < samplesPerPose; i++) {
			const float w = poseWeight(i);
			totalWeight += w;
			weightedMean[0] += w * static_cast<float>(data.poseSamples[i][0]);
			weightedMean[1] += w * static_cast<float>(data.poseSamples[i][1]);
			weightedMean[2] += w * static_cast<float>(data.poseSamples[i][2]);
		}
		if (totalWeight <= 1e-6f || !std::isfinite(totalWeight)) {
			return;
		}

		weightedMean[0] /= totalWeight;
		weightedMean[1] /= totalWeight;
		weightedMean[2] /= totalWeight;
		// Keep one representative sample per pose group:
		// weighted average with lower weights near the pose-group edges.
		appendStoredSample(
			data,
			toSensorRaw(weightedMean[0]),
			toSensorRaw(weightedMean[1]),
			toSensorRaw(weightedMean[2])
		);
		data.poseGroupCounts[data.currentPoseBucket]++;
	}

	bool isHoldStillSatisfied(
		CalibrationData& data,
		const std::array<float, 3>& currentDirection,
		const float accelNormRawValue
	) {
		const uint32_t now = millis();
		if (requireGyroStillForSampling
			&& (!hasFreshGyroSample()
				|| gyroNormRawEma > maxSampleCollectionHoldStillGyroNormRaw)) {
			data.holdDirection = currentDirection;
			data.holdAccelNormRaw = accelNormRawValue;
			data.holdStartMillis = now;
			return false;
		}

		if (!data.holdDirection.has_value()) {
			data.holdDirection = currentDirection;
			data.holdAccelNormRaw = accelNormRawValue;
			data.holdStartMillis = now;
			return false;
		}

		const auto& holdDirection = data.holdDirection.value();
		float holdDot = holdDirection[0] * currentDirection[0]
					  + holdDirection[1] * currentDirection[1]
					  + holdDirection[2] * currentDirection[2];
		if (!std::isfinite(holdDot)) {
			data.holdDirection = currentDirection;
			data.holdAccelNormRaw = accelNormRawValue;
			data.holdStartMillis = now;
			return false;
		}

		holdDot = std::clamp(holdDot, -1.0f, 1.0f);
		const float holdAccelNormDelta
			= std::abs(accelNormRawValue - data.holdAccelNormRaw);
		if (holdDot < minHoldStillDirectionDot
			|| holdAccelNormDelta > maxHoldStillAccelNormDeltaRaw) {
			data.holdDirection = currentDirection;
			data.holdAccelNormRaw = accelNormRawValue;
			data.holdStartMillis = now;
			return false;
		}

		if (now - data.holdStartMillis < minHoldStillDurationMs) {
			return false;
		}

		// Keep hold anchor fixed while sampling; reset only on violation.
		return true;
	}

	std::optional<CalibrationData> calibrationData;
	std::optional<CalibrationOutput> latestCalibration;
	SlimeVR::Logging::Logger& logger;
	float maxReferenceCaptureGyroNormRaw = 0.0f;
	float maxSampleCollectionHoldStillGyroNormRaw = 0.0f;
	bool restDetected = false;
	bool hasRecentGyroSample = false;
	uint32_t lastGyroSampleMillis = 0;
	float gyroNormRawEma = 0.0f;
	uint32_t sampleLedPulseEndMillis = 0;
	bool sampleLedPulseActive = false;

	// Converts gyro norm threshold (dps) into a raw threshold
	// using current sensor scale.
	static float convertGyroNormDpsToRawThreshold(
		float gyroScaleRadPerSecPerRaw,
		float gyroNormDpsThreshold
	) {
		if (!std::isfinite(gyroScaleRadPerSecPerRaw)
			|| gyroScaleRadPerSecPerRaw <= 1e-9f) {
			return 0.0f;
		}
		const float gyroDpsPerRaw = gyroScaleRadPerSecPerRaw * (180.0f / PI);
		if (!std::isfinite(gyroDpsPerRaw) || gyroDpsPerRaw <= 1e-9f) {
			return 0.0f;
		}
		const float rawThreshold = gyroNormDpsThreshold / gyroDpsPerRaw;
		if (!std::isfinite(rawThreshold)) {
			return 0.0f;
		}
		return rawThreshold > 0.0f ? rawThreshold : 0.0f;
	}
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
