/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include <vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#define RC_ACCEL_MODE_DISABLED 0
#define RC_ACCEL_MODE_LEGACY_MAGNETO_SIX_SIDE 1
#define RC_ACCEL_MODE_SIX_SIDE 2
#define RC_ACCEL_MODE_FULL_MATRIX 3

// Select exactly one accel calibration mode by enum-like value.
#ifndef RC_ACCEL_MODE
#define RC_ACCEL_MODE RC_ACCEL_MODE_FULL_MATRIX
#endif

#if RC_ACCEL_MODE < RC_ACCEL_MODE_DISABLED || RC_ACCEL_MODE > RC_ACCEL_MODE_FULL_MATRIX
#error "RC_ACCEL_MODE is out of valid range"
#endif

#define RC_ACCEL_MODE_IS_DISABLED (RC_ACCEL_MODE == RC_ACCEL_MODE_DISABLED)
#define RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE \
	(RC_ACCEL_MODE == RC_ACCEL_MODE_LEGACY_MAGNETO_SIX_SIDE)
#define RC_ACCEL_MODE_IS_SIX_SIDE (RC_ACCEL_MODE == RC_ACCEL_MODE_SIX_SIDE)
#define RC_ACCEL_MODE_IS_FULL_MATRIX (RC_ACCEL_MODE == RC_ACCEL_MODE_FULL_MATRIX)

#include "../../../GlobalVars.h"
#include "../../../calibration.h"
#include "../../../configuration/Configuration.h"
#include "AccelBiasCalibrationStep.h"
#include "AccelFullMatrixCalibrationStep.h"
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
#include "AccelSixSideMagnetoCalibrationStep.h"
#else
#include "AccelSixSideCalibrationStep.h"
#endif
#include "GyroBiasCalibrationStep.h"
#include "MagCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/CalibrationBase.h"

#ifndef RUNTIME_CALIBRATION_ENABLE_RUNTIME_ACCEL_DEBUG_LOGS
#define RUNTIME_CALIBRATION_ENABLE_RUNTIME_ACCEL_DEBUG_LOGS 0
#endif

namespace SlimeVR::Sensors::RuntimeCalibration {

#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
template <typename SensorRawT>
using SelectedAccelSixSideCalibrationStep
	= AccelSixSideMagnetoCalibrationStep<SensorRawT>;
#else
template <typename SensorRawT>
using SelectedAccelSixSideCalibrationStep = AccelSixSideCalibrationStep<SensorRawT>;
#endif

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = true;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles}
		, accelFullMatrixCalibrationStep{calibration, logger, static_cast<float>(Consts::GScale)}
		, accelSixSideCalibrationStep{calibration, logger, static_cast<float>(Consts::AScale)}
		, magCalibrationStep{calibration, logger} {
		calibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.runtimeCalibration.ImuType == IMU::Type)
			&& (sensorCalibration.data.runtimeCalibration.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.runtimeCalibration;
		activeCalibration = sensorCalibration.data.runtimeCalibration;
		const bool calibrationEnabled
			= toggles.getToggle(SensorToggles::CalibrationEnabled);
		if (!calibrationEnabled) {
			activeCalibration.gyroPointsCalibrated = 0;
			for (size_t i = 0; i < 3; i++) {
				activeCalibration.G_off1[i] = 0;
				activeCalibration.G_off2[i] = 0;
			}

			for (size_t i = 0; i < 3; i++) {
				activeCalibration.accelCalibrated[i] = false;
				activeCalibration.A_off[i] = 0;
				activeCalibration.A_sixSideOff[i] = 0;
				activeCalibration.A_sixSideScale[i] = 1.0f;
				activeCalibration.A_sixSideLegacyB[i] = 0.0f;
				activeCalibration.A_B[i] = 0.0f;
				for (size_t j = 0; j < 3; j++) {
					activeCalibration.A_Ainv[i][j] = (i == j) ? 1.0f : 0.0f;
					activeCalibration.A_sixSideLegacyAinv[i][j]
						= (i == j) ? 1.0f : 0.0f;
				}
			}
			activeCalibration.accelFullMatrixCalibrated = false;
			activeAccelFullMatrixValid = false;
			activeAccelSixSideValid = false;
			pendingMagCalibrationRequest = false;
		} else {
			calculateZROChange();

			if (!isAccelFullMatrixCalibrationMissing(calibration)
				&& !isAccelFullMatrixCalibrationValid(calibration)) {
				logger.warn(
					"Invalid accel full-matrix calibration data found, clearing it"
				);
				resetAccelFullMatrixCalibration(calibration);
			}
			syncAccelFullMatrixCalibrationToActive();

			if (isAccelSixSideCalibrationMissing(calibration)) {
				resetAccelSixSideCalibration(calibration);
			} else if (!isAccelSixSideCalibrationValid(calibration)) {
				logger.warn(
					"Invalid accel six-side calibration data found, clearing it"
				);
				resetAccelSixSideCalibration(calibration);
			}
			syncAccelSixSideCalibrationToActive();

			if (!isMagCalibrationMissing(calibration)
				&& !isMagCalibrationValid(calibration)) {
				logger.warn("Invalid mag calibration data found, clearing it");
				resetMagCalibration(calibration);
			}

			if (!isMagNormReferenceValid(calibration.M_refNorm)) {
				calibration.M_refNorm = 0.0f;
			}

			syncMagCalibrationToActive();

			const bool isMagCalibrationReadya = isMagCalibrationReady();
			pendingMagCalibrationRequest
				= shouldRequestMagCalibration(isMagCalibrationReadya);
			if (!isMagCalibrationReadya) {
				fusion.setMagFusionEnabled(false);
			}
		}

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();
		syncAccelFullMatrixCalibrationToActive();
		syncMagCalibrationToActive();

		printCalibration();
	}

	void checkStartupCalibration() final {
#if RC_ACCEL_MODE_IS_FULL_MATRIX || RC_ACCEL_MODE_IS_SIX_SIDE \
	|| RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
		initializeStartupAccelSixSideTrigger();
#endif
	}

	void startCalibration(int calibrationType) final {
		if (!isCalibrationRequestEnabled()) {
			return;
		}

		cancelActiveCalibrationIfRunning();
		handleExternalCalibrationRequest(calibrationType);
	}

	void tick() final {
		updateStartupAccelSixSideTrigger();
		maybeStartAccelFullMatrixCalibration();
		maybeStartAccelSixSideCalibration();
		maybeStartMagCalibration();
		if (!isMagCalibrationReady()) {
			fusion.setMagFusionEnabled(false);
		}

		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		sensor_real_t rawAccelCounts[3] = {
			accelSample[0],
			accelSample[1],
			accelSample[2],
		};
		sensor_real_t rawAccelMs2[3] = {0.0f, 0.0f, 0.0f};
		for (size_t i = 0; i < 3; i++) {
			rawAccelMs2[i] = accelSample[i] * Consts::AScale;
		}

#if RC_ACCEL_MODE_IS_FULL_MATRIX
		if (activeAccelFullMatrixValid) {
			float tmp[3];
			tmp[0] = rawAccelCounts[0] - activeCalibration.A_B[0];
			tmp[1] = rawAccelCounts[1] - activeCalibration.A_B[1];
			tmp[2] = rawAccelCounts[2] - activeCalibration.A_B[2];

			accelSample[0] = (activeCalibration.A_Ainv[0][0] * tmp[0]
							  + activeCalibration.A_Ainv[0][1] * tmp[1]
							  + activeCalibration.A_Ainv[0][2] * tmp[2])
						   * Consts::AScale;
			accelSample[1] = (activeCalibration.A_Ainv[1][0] * tmp[0]
							  + activeCalibration.A_Ainv[1][1] * tmp[1]
							  + activeCalibration.A_Ainv[1][2] * tmp[2])
						   * Consts::AScale;
			accelSample[2] = (activeCalibration.A_Ainv[2][0] * tmp[0]
							  + activeCalibration.A_Ainv[2][1] * tmp[1]
							  + activeCalibration.A_Ainv[2][2] * tmp[2])
						   * Consts::AScale;
		} else {
			for (size_t i = 0; i < 3; i++) {
				accelSample[i] = rawAccelMs2[i];
			}
		}
#elif RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
		if (activeAccelSixSideValid) {
			float tmp[3];
			tmp[0] = rawAccelCounts[0] - activeCalibration.A_sixSideLegacyB[0];
			tmp[1] = rawAccelCounts[1] - activeCalibration.A_sixSideLegacyB[1];
			tmp[2] = rawAccelCounts[2] - activeCalibration.A_sixSideLegacyB[2];

			accelSample[0] = (activeCalibration.A_sixSideLegacyAinv[0][0] * tmp[0]
							  + activeCalibration.A_sixSideLegacyAinv[0][1] * tmp[1]
							  + activeCalibration.A_sixSideLegacyAinv[0][2] * tmp[2])
						   * Consts::AScale;
			accelSample[1] = (activeCalibration.A_sixSideLegacyAinv[1][0] * tmp[0]
							  + activeCalibration.A_sixSideLegacyAinv[1][1] * tmp[1]
							  + activeCalibration.A_sixSideLegacyAinv[1][2] * tmp[2])
						   * Consts::AScale;
			accelSample[2] = (activeCalibration.A_sixSideLegacyAinv[2][0] * tmp[0]
							  + activeCalibration.A_sixSideLegacyAinv[2][1] * tmp[1]
							  + activeCalibration.A_sixSideLegacyAinv[2][2] * tmp[2])
						   * Consts::AScale;
		} else {
			for (size_t i = 0; i < 3; i++) {
				accelSample[i] = rawAccelMs2[i];
			}
		}
#elif RC_ACCEL_MODE_IS_SIX_SIDE
		if (activeAccelSixSideValid) {
			for (size_t i = 0; i < 3; i++) {
				accelSample[i] = rawAccelMs2[i] * activeCalibration.A_sixSideScale[i]
							   - activeCalibration.A_sixSideOff[i];
			}
		} else {
			for (size_t i = 0; i < 3; i++) {
				accelSample[i] = rawAccelMs2[i];
			}
		}
#else
		for (size_t i = 0; i < 3; i++) {
			accelSample[i] = rawAccelMs2[i];
		}
#endif

#if RUNTIME_CALIBRATION_ENABLE_RUNTIME_ACCEL_DEBUG_LOGS
		const bool restDetected = fusion.getRestDetected();
#if RC_ACCEL_MODE_IS_FULL_MATRIX
		if (activeAccelFullMatrixValid) {
			captureAccelDebugSnapshot(
				accelFullMatrixRestSnapshot,
				rawAccelCounts,
				rawAccelMs2,
				accelSample,
				restDetected
			);
			maybeLogAccelFullMatrixDebug();
		} else {
			captureAccelDebugSnapshot(
				accelSixSideRestSnapshot,
				rawAccelCounts,
				rawAccelMs2,
				accelSample,
				restDetected
			);
			maybeLogAccelSixSideDebug();
		}
#else
		captureAccelDebugSnapshot(
			accelSixSideRestSnapshot,
			rawAccelCounts,
			rawAccelMs2,
			accelSample,
			restDetected
		);
		maybeLogAccelSixSideDebug();
#endif
#endif
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		gyroSample[0] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[0] - activeCalibration.G_off1[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[1] - activeCalibration.G_off1[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[2] - activeCalibration.G_off1[2])
		);
	}

	void scaleMagSample(sensor_real_t magSample[3]) final {
		float tmp[3];
		for (uint8_t i = 0; i < 3; i++) {
			tmp[i] = (magSample[i] - activeCalibration.M_B[i]);
		}

		magSample[0]
			= (activeCalibration.M_Ainv[0][0] * tmp[0]
			   + activeCalibration.M_Ainv[0][1] * tmp[1]
			   + activeCalibration.M_Ainv[0][2] * tmp[2]);
		magSample[1]
			= (activeCalibration.M_Ainv[1][0] * tmp[0]
			   + activeCalibration.M_Ainv[1][1] * tmp[1]
			   + activeCalibration.M_Ainv[1][2] * tmp[2]);
		magSample[2]
			= (activeCalibration.M_Ainv[2][0] * tmp[0]
			   + activeCalibration.M_Ainv[2][1] * tmp[1]
			   + activeCalibration.M_Ainv[2][2] * tmp[2]);
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }

	float getMagTimestep() final { return activeCalibration.M_Ts; }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (isCalibrating) {
			currentStep->setRestDetected(fusion.getRestDetected());
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideMagSample(const RawSensorT magSample[3]) final {
		if (isCalibrating) {
			currentStep->processMagSample(magSample);
		}
	}

	void provideTempSample(float tempSample) final {
		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void onMagEnabled() final {
		pendingMagCalibrationRequest = shouldRequestMagCalibration();
	}

	bool shouldUpdateMagFusion() const {
		if (isCalibrating && currentStep == &magCalibrationStep) {
			return false;
		}
		return isMagCalibrationReady();
	}

	bool shouldUseMagSample(const sensor_real_t* magSample) const {
		if (!isMagNormReferenceValid(activeCalibration.M_refNorm)) {
			return true;
		}

		const float magnitude = std::sqrt(
			static_cast<float>(
				magSample[0] * magSample[0] + magSample[1] * magSample[1]
				+ magSample[2] * magSample[2]
			)
		);
		if (!std::isfinite(magnitude)) {
			return false;
		}

		const float allowedDeviation = std::max(
			activeCalibration.M_refNorm * magNormRejectRelativeThreshold,
			magNormRejectAbsoluteThreshold
		);
		return std::abs(magnitude - activeCalibration.M_refNorm) <= allowedDeviation;
	}

	void calculateZROChange() {
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		activeZROChange = 0.1f / maxDiff
						/ (activeCalibration.gyroMeasurementTemperature2
						   - activeCalibration.gyroMeasurementTemperature1);
	}

	float getZROChange() final { return activeZROChange; }

	bool clearGyroCalibration() final {
		calibration.gyroPointsCalibrated = 0;
		calibration.gyroMeasurementTemperature1 = 0.0f;
		calibration.gyroMeasurementTemperature2 = 0.0f;
		activeCalibration.gyroPointsCalibrated = 0;
		activeCalibration.gyroMeasurementTemperature1 = 0.0f;
		activeCalibration.gyroMeasurementTemperature2 = 0.0f;
		for (size_t i = 0; i < 3; i++) {
			calibration.G_off1[i] = 0.0f;
			calibration.G_off2[i] = 0.0f;
			activeCalibration.G_off1[i] = 0.0f;
			activeCalibration.G_off2[i] = 0.0f;
		}
		activeZROChange = IMU::TemperatureZROChange;

		if (nextCalibrationStep == CalibrationStepEnum::GYRO_BIAS) {
			currentStep->cancel();
			computeNextCalibrationStep();
			isCalibrating = false;
		}

		saveCalibration();
		logger.info("Gyro calibration cleared");
		return true;
	}

	bool clearMagCalibration() final {
		resetMagCalibration(calibration);
		syncMagCalibrationToActive();
		fusion.setMagFusionEnabled(false);

		pendingMagCalibrationRequest = shouldRequestMagCalibration();

		if (nextCalibrationStep == CalibrationStepEnum::MAG_CALIB) {
			currentStep->cancel();
			computeNextCalibrationStep();
			isCalibrating = false;
		}

		saveCalibration();
		logger.info("Mag calibration cleared");
		return true;
	}

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
		ACCEL_SIX_SIDE,
		ACCEL_FULL_MATRIX,
		MAG_CALIB,
	};

	static constexpr float minValidMagReferenceNorm = 1.0f;
	static constexpr float maxValidMagReferenceNorm = 100000.0f;
	static constexpr float magNormRejectRelativeThreshold = 0.07f;
	static constexpr float magNormRejectAbsoluteThreshold = 20.0f;
	static constexpr uint8_t startupAccelSixSideWindowSeconds = 3;
	static constexpr float startupAccelSixSideMinAbsGravityZ = 0.75f;
	static constexpr uint16_t startupAccelSixSideReadyLedPulseMs = 800;
	static constexpr uint16_t startupAccelSixSideReadyLedOnMs = 400;
	static constexpr uint16_t startupAccelSixSideBootZCollectionMs = 800;

	void resetStartupAccelSixSideBootGateSampling() { startupAccelSixSideBootZ = 0.0f; }

	void clearPendingAccelCalibrationRequests() {
		pendingAccelFullMatrixCalibrationRequest = false;
		pendingAccelSixSideCalibrationRequest = false;
	}

	void initializeStartupAccelSixSideTrigger() {
		startupAccelSixSideWindowStartMillis = millis();
		startupAccelSixSideReadyLastLedMs = startupAccelSixSideWindowStartMillis;
		startupAccelSixSideReadyLedPulseEndMillis = 0;
		startupAccelSixSideReadyLedPulseActive = false;
		clearPendingAccelCalibrationRequests();
		startupAccelSixSideMonitoringEnabled = false;
		startupAccelSixSideBootZGatePending
			= toggles.getToggle(SensorToggles::CalibrationEnabled);
		resetStartupAccelSixSideBootGateSampling();
	}

	bool isCalibrationRequestEnabled() {
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			logger.warn("Calibration is disabled");
			return false;
		}
		return true;
	}

	void cancelActiveCalibrationIfRunning() {
		if (!isCalibrating) {
			return;
		}

		currentStep->cancel();
		isCalibrating = false;
	}

	void requestAccelSixSideCalibration() {
		pendingAccelSixSideCalibrationRequest = true;
		pendingAccelFullMatrixCalibrationRequest = false;
		disableStartupAccelSixSideTrigger();
		logger.info("Requested accel six-side calibration");
	}

	void requestAccelFullMatrixCalibration() {
		pendingAccelFullMatrixCalibrationRequest = true;
		pendingAccelSixSideCalibrationRequest = false;
		disableStartupAccelSixSideTrigger();
		logger.info("Requested accel full-matrix calibration");
	}

	void requestMagCalibration() {
		pendingMagCalibrationRequest = true;
		magCalibrationAttemptedSinceBoot = false;
		logger.info("Requested mag calibration");
	}

	void handleExternalCalibrationRequest(int calibrationType) {
		switch (calibrationType) {
			case CALIBRATION_TYPE_EXTERNAL_ACCEL:
				requestAccelSixSideCalibration();
				break;
			case CALIBRATION_TYPE_EXTERNAL_ACCEL_FULL_MATRIX:
				requestAccelFullMatrixCalibration();
				break;
			case CALIBRATION_TYPE_EXTERNAL_MAG:
				requestMagCalibration();
				break;
			default:
				logger.warn("Unsupported calibration type: %d", calibrationType);
				break;
		}
	}

	void computeNextCalibrationStep() {
		if (!calibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (calibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (calibration.gyroPointsCalibrated == 1) {
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				}

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
			case CalibrationStepEnum::ACCEL_SIX_SIDE:
				if (!isAccelSixSideCalibrationMissing(calibration)
					&& !isAccelSixSideCalibrationValid(calibration)) {
					logger.warn(
						"Accel six-side calibration output was invalid, resetting it"
					);
					resetAccelSixSideCalibration(calibration);
				}
				// sync
				syncAccelSixSideCalibrationToActive();
				computeNextCalibrationStep();
				break;
			case CalibrationStepEnum::ACCEL_FULL_MATRIX: {
				const bool hasAccelFullMatrixResult
					= accelFullMatrixCalibrationStep.hasResult();
				if (!hasAccelFullMatrixResult) {
					logger.warn(
						"Accel full-matrix calibration finished without result"
					);
					resetAccelFullMatrixCalibration(calibration);
				} else {
					const auto& out
						= accelFullMatrixCalibrationStep.getResult().value();
					calibration.accelFullMatrixCalibrated = true;
					for (uint8_t i = 0; i < 3; i++) {
						calibration.A_B[i] = out.A_B[i];
						for (uint8_t j = 0; j < 3; j++) {
							calibration.A_Ainv[i][j] = out.A_Ainv[i][j];
						}
					}
				}

				syncAccelFullMatrixCalibrationToActive();
				computeNextCalibrationStep();

				if (print) {
					if (!hasAccelFullMatrixResult) {
						logger.warn("Accel full-matrix calibration failed");
					}
				}
				break;
			}
			case CalibrationStepEnum::MAG_CALIB:
				if (!isMagCalibrationValid(calibration)) {
					logger.warn(
						"Mag calibration output was invalid, keeping mag disabled"
					);
					resetMagCalibration(calibration);
					fusion.setMagFusionEnabled(false);
				}
				syncMagCalibrationToActive();
				pendingMagCalibrationRequest = false;
				computeNextCalibrationStep();

				if (print) {
					printCalibration(CalibrationPrintFlags::MAG_CALIB);
				}
				break;
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void maybeStartMagCalibration() {
		if (!pendingMagCalibrationRequest) {
			return;
		}

		if (pendingAccelSixSideCalibrationRequest
			|| nextCalibrationStep == CalibrationStepEnum::ACCEL_SIX_SIDE
			|| pendingAccelFullMatrixCalibrationRequest
			|| nextCalibrationStep == CalibrationStepEnum::ACCEL_FULL_MATRIX) {
			return;
		}

		if (magCalibrationAttemptedSinceBoot) {
			pendingMagCalibrationRequest = false;
			return;
		}

		if (!toggles.getToggle(SensorToggles::MagEnabled)) {
			return;
		}

		if (isMagCalibrationReady()) {
			pendingMagCalibrationRequest = false;
			return;
		}

		if (isCalibrating
			|| nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
			return;
		}

		nextCalibrationStep = CalibrationStepEnum::MAG_CALIB;
		currentStep = &magCalibrationStep;
		pendingMagCalibrationRequest = false;
		magCalibrationAttemptedSinceBoot = true;
	}

	void maybeStartAccelFullMatrixCalibration() {
#if !RC_ACCEL_MODE_IS_FULL_MATRIX
		pendingAccelFullMatrixCalibrationRequest = false;
		return;
#endif

		if (!pendingAccelFullMatrixCalibrationRequest) {
			return;
		}

		if (isCalibrating
			|| nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
			return;
		}

		nextCalibrationStep = CalibrationStepEnum::ACCEL_FULL_MATRIX;
		currentStep = &accelFullMatrixCalibrationStep;
		pendingAccelFullMatrixCalibrationRequest = false;
		disableStartupAccelSixSideTrigger();
		logger.info("Starting accel full-matrix calibration");
	}

	void maybeStartAccelSixSideCalibration() {
#if !(RC_ACCEL_MODE_IS_SIX_SIDE || RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE)
		pendingAccelSixSideCalibrationRequest = false;
		return;
#endif

		if (!pendingAccelSixSideCalibrationRequest) {
			return;
		}

		if (pendingAccelFullMatrixCalibrationRequest
			|| nextCalibrationStep == CalibrationStepEnum::ACCEL_FULL_MATRIX) {
			return;
		}

		if (isCalibrating
			|| nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
			return;
		}

		nextCalibrationStep = CalibrationStepEnum::ACCEL_SIX_SIDE;
		currentStep = &accelSixSideCalibrationStep;
		pendingAccelSixSideCalibrationRequest = false;
		disableStartupAccelSixSideTrigger();
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
		logger.info("Starting accel six-side calibration (legacy magneto1.4)");
#else
		logger.info("Starting accel six-side calibration");
#endif
	}

	void updateStartupAccelSixSideTrigger() {
		if (!startupAccelSixSideBootZGatePending
			&& !startupAccelSixSideMonitoringEnabled) {
			return;
		}

		const uint64_t elapsed = millis() - startupAccelSixSideWindowStartMillis;
		if (elapsed > startupAccelSixSideWindowSeconds * 1e3) {
			disableStartupAccelSixSideTrigger();
			return;
		}

		if (pendingAccelSixSideCalibrationRequest
			|| pendingAccelFullMatrixCalibrationRequest) {
			return;
		}

		sensor_real_t gravityRaw[3];
		SensorFusion::calcGravityVec(fusion.getQuaternion(), gravityRaw);
		const float gravityZ = static_cast<float>(gravityRaw[2]);
		if (!std::isfinite(gravityZ)) {
			return;
		}

		if (startupAccelSixSideBootZGatePending) {
			if (elapsed < startupAccelSixSideBootZCollectionMs) {
				return;
			}

			startupAccelSixSideBootZ = gravityZ;

			startupAccelSixSideBootZGatePending = false;
			if (startupAccelSixSideBootZ <= -startupAccelSixSideMinAbsGravityZ) {
				startupAccelSixSideMonitoringEnabled = true;
				logger.info(
					"Startup six-side trigger enabled (boot negative Z detected, "
					"Z=%.3f)",
					startupAccelSixSideBootZ
				);
			} else {
				startupAccelSixSideMonitoringEnabled = false;
				logger.info(
					"Startup six-side trigger skipped (boot Z was not negative, "
					"Z=%.3f)",
					startupAccelSixSideBootZ
				);
			}
			return;
		}

		if (!startupAccelSixSideMonitoringEnabled) {
			return;
		}

		const uint64_t now = millis();
		if (startupAccelSixSideReadyLedPulseActive) {
			if (now >= startupAccelSixSideReadyLedPulseEndMillis) {
				ledManager.off();
				startupAccelSixSideReadyLedPulseActive = false;
			}
		} else {
			const uint64_t ledElapsed = now - startupAccelSixSideReadyLastLedMs;
			if (ledElapsed >= startupAccelSixSideReadyLedPulseMs) {
				startupAccelSixSideReadyLastLedMs = now;
				startupAccelSixSideReadyLedPulseEndMillis
					= now + startupAccelSixSideReadyLedOnMs;
				startupAccelSixSideReadyLedPulseActive = true;
				ledManager.on();
			}
		}
		// logger.info("gravityZ: %f", gravityZ);

		if (gravityZ >= startupAccelSixSideMinAbsGravityZ) {
#if RC_ACCEL_MODE_IS_FULL_MATRIX
			pendingAccelFullMatrixCalibrationRequest = true;
			disableStartupAccelSixSideTrigger();
			logger.info(
				"Startup flip detected (positive Z), scheduling accel full-matrix "
				"calibration"
			);
#else
			pendingAccelSixSideCalibrationRequest = true;
			disableStartupAccelSixSideTrigger();
			logger.info(
				"Startup flip detected (positive Z), scheduling accel six-side "
				"calibration"
			);
#endif
		}
	}

	void disableStartupAccelSixSideTrigger() {
		startupAccelSixSideReadyLedPulseActive = false;
		startupAccelSixSideReadyLedPulseEndMillis = 0;
		startupAccelSixSideReadyLastLedMs = millis();
		ledManager.off();
		startupAccelSixSideBootZGatePending = false;
		startupAccelSixSideMonitoringEnabled = false;
		resetStartupAccelSixSideBootGateSampling();
	}

	void syncAccelSixSideCalibrationToActive() {
		for (uint8_t i = 0; i < 3; i++) {
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
			activeCalibration.A_sixSideLegacyB[i] = calibration.A_sixSideLegacyB[i];
			for (uint8_t j = 0; j < 3; j++) {
				activeCalibration.A_sixSideLegacyAinv[i][j]
					= calibration.A_sixSideLegacyAinv[i][j];
			}
#else
			activeCalibration.A_sixSideOff[i] = calibration.A_sixSideOff[i];
			activeCalibration.A_sixSideScale[i] = calibration.A_sixSideScale[i];
#endif
		}
		activeAccelSixSideValid = toggles.getToggle(SensorToggles::CalibrationEnabled)
							   && !isAccelSixSideCalibrationMissing(activeCalibration)
							   && isAccelSixSideCalibrationValid(activeCalibration);
	}

	void syncAccelFullMatrixCalibrationToActive() {
		activeCalibration.accelFullMatrixCalibrated
			= calibration.accelFullMatrixCalibrated;
		for (uint8_t i = 0; i < 3; i++) {
			activeCalibration.A_B[i] = calibration.A_B[i];
			for (uint8_t j = 0; j < 3; j++) {
				activeCalibration.A_Ainv[i][j] = calibration.A_Ainv[i][j];
			}
		}
		activeAccelFullMatrixValid
			= toggles.getToggle(SensorToggles::CalibrationEnabled)
		   && isAccelFullMatrixCalibrationValid(activeCalibration);
	}

	const bool isAccelFullMatrixCalibrationMissing(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		return !config.accelFullMatrixCalibrated;
	}

	bool isAccelFullMatrixCalibrationValid(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		if (!config.accelFullMatrixCalibrated) {
			return false;
		}

		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (uint8_t i = 0; i < 3; i++) {
			if (!std::isfinite(config.A_B[i])) {
				return false;
			}

			for (uint8_t j = 0; j < 3; j++) {
				const float v = config.A_Ainv[i][j];
				if (!std::isfinite(v) || std::abs(v) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(config.A_Ainv[i][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
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

	static void resetAccelFullMatrixCalibration(
		Configuration::RuntimeCalibrationSensorConfig& calibration
	) {
		calibration.accelFullMatrixCalibrated = false;
		for (uint8_t i = 0; i < 3; i++) {
			calibration.A_B[i] = 0.0f;
			for (uint8_t j = 0; j < 3; j++) {
				calibration.A_Ainv[i][j] = (i == j) ? 1.0f : 0.0f;
			}
		}
	}

	const bool isAccelSixSideCalibrationMissing(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		constexpr float epsilon = 1e-6f;
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
		bool offsetsAreZero = true;
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(config.A_sixSideLegacyB[i]) > epsilon) {
				offsetsAreZero = false;
				break;
			}
		}

		bool matrixIsIdentity = true;
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 3; j++) {
				const float expected = i == j ? 1.0f : 0.0f;
				if (std::abs(config.A_sixSideLegacyAinv[i][j] - expected) > epsilon) {
					matrixIsIdentity = false;
					break;
				}
			}
		}

		return offsetsAreZero && matrixIsIdentity;
#else
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(config.A_sixSideOff[i]) > epsilon
				|| std::abs(config.A_sixSideScale[i] - 1.0f) > epsilon) {
				return false;
			}
		}
		return true;
#endif
	}

	bool isAccelSixSideCalibrationValid(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (uint8_t i = 0; i < 3; i++) {
			if (!std::isfinite(config.A_sixSideLegacyB[i])) {
				return false;
			}
			for (uint8_t j = 0; j < 3; j++) {
				const float value = config.A_sixSideLegacyAinv[i][j];
				if (!std::isfinite(value) || std::abs(value) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(config.A_sixSideLegacyAinv[i][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
			return false;
		}

		const float diagonalTolerance = std::max(meanDiagonal * 0.6f, 0.05f);
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(diagonalAbs[i] - meanDiagonal) > diagonalTolerance) {
				return false;
			}
		}

		return true;
#elif RC_ACCEL_MODE_IS_SIX_SIDE
		for (uint8_t i = 0; i < 3; i++) {
			const float scale = config.A_sixSideScale[i];
			const float offset = config.A_sixSideOff[i];
			if (!std::isfinite(scale) || !std::isfinite(offset)) {
				return false;
			}
			if (std::abs(scale) < 1e-4f || std::abs(scale) > 10.0f) {
				return false;
			}
			if (std::abs(offset) > 200.0f) {
				return false;
			}
		}
		return true;
#else
		return true;
#endif
	}

	static void resetAccelSixSideCalibration(
		Configuration::RuntimeCalibrationSensorConfig& calibration
	) {
		for (uint8_t i = 0; i < 3; i++) {
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
			calibration.A_sixSideLegacyB[i] = 0.0f;
			for (uint8_t j = 0; j < 3; j++) {
				calibration.A_sixSideLegacyAinv[i][j] = (i == j) ? 1.0f : 0.0f;
			}
#else
			calibration.A_sixSideOff[i] = 0.0f;
			calibration.A_sixSideScale[i] = 1.0f;
#endif
		}
	}

#if RUNTIME_CALIBRATION_ENABLE_RUNTIME_ACCEL_DEBUG_LOGS
	struct AccelDebugSnapshot {
		bool valid = false;
		uint64_t captureMillis = 0;
		sensor_real_t rawCounts[3] = {0.0f, 0.0f, 0.0f};
		sensor_real_t rawMs2[3] = {0.0f, 0.0f, 0.0f};
		sensor_real_t scaledMs2[3] = {0.0f, 0.0f, 0.0f};
	};

	static void copyAccelVector(const sensor_real_t src[3], sensor_real_t dst[3]) {
		for (uint8_t i = 0; i < 3; i++) {
			dst[i] = src[i];
		}
	}

	static float accelVectorNorm(const sensor_real_t v[3]) {
		return std::sqrt(static_cast<float>(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
	}

	static bool isFiniteAccelVector(const sensor_real_t v[3]) {
		return std::isfinite(static_cast<float>(v[0]))
			&& std::isfinite(static_cast<float>(v[1]))
			&& std::isfinite(static_cast<float>(v[2]));
	}

	static bool isValidAccelDebugSample(
		const sensor_real_t rawCounts[3],
		const sensor_real_t rawMs2[3],
		const sensor_real_t scaledMs2[3]
	) {
		if (!isFiniteAccelVector(rawCounts) || !isFiniteAccelVector(rawMs2)
			|| !isFiniteAccelVector(scaledMs2)) {
			return false;
		}

		const bool allMinusOne = rawCounts[0] == static_cast<sensor_real_t>(-1.0f)
							  && rawCounts[1] == static_cast<sensor_real_t>(-1.0f)
							  && rawCounts[2] == static_cast<sensor_real_t>(-1.0f);
		if (allMinusOne) {
			return false;
		}

		const float rawNorm = accelVectorNorm(rawMs2);
		if (!std::isfinite(rawNorm) || rawNorm < 1.0f || rawNorm > 30.0f) {
			return false;
		}

		return true;
	}

	static void captureAccelDebugSnapshot(
		AccelDebugSnapshot& snapshot,
		const sensor_real_t rawCounts[3],
		const sensor_real_t rawMs2[3],
		const sensor_real_t scaledMs2[3],
		bool restDetected
	) {
		if (!restDetected) {
			return;
		}
		if (!isValidAccelDebugSample(rawCounts, rawMs2, scaledMs2)) {
			return;
		}

		snapshot.valid = true;
		snapshot.captureMillis = millis();
		copyAccelVector(rawCounts, snapshot.rawCounts);
		copyAccelVector(rawMs2, snapshot.rawMs2);
		copyAccelVector(scaledMs2, snapshot.scaledMs2);
	}

	void maybeLogAccelSixSideDebug() {
		const uint64_t now = millis();
		if (now - lastAccelSixSideDebugLogMillis < accelSixSideDebugLogIntervalMs) {
			return;
		}
		if (!accelSixSideRestSnapshot.valid) {
			return;
		}
		lastAccelSixSideDebugLogMillis = now;

		const sensor_real_t* logRawCounts = accelSixSideRestSnapshot.rawCounts;
		const sensor_real_t* logRawMs2 = accelSixSideRestSnapshot.rawMs2;
		const sensor_real_t* logScaledMs2 = accelSixSideRestSnapshot.scaledMs2;
		const bool logRestDetected = true;
		const uint32_t snapshotAgeMs
			= static_cast<uint32_t>(now - accelSixSideRestSnapshot.captureMillis);
		accelSixSideRestSnapshot.valid = false;

		const float rawNorm = accelVectorNorm(logRawMs2);
		const float scaledNorm = accelVectorNorm(logScaledMs2);
		const float normDelta = scaledNorm - rawNorm;
		const float normDeltaPct
			= rawNorm > 1e-6f ? (normDelta / rawNorm) * 100.0f : 0.0f;

		logger.info(
			"Accel6 dbg raw[cnt]=%.2f %.2f %.2f raw[m/s2]=%.3f %.3f %.3f "
			"scaled[m/s2]=%.3f %.3f %.3f rest=%u age_ms=%u |a|raw=%.3f |a|scaled=%.3f "
			"d|a|=%.3f d%%=%.2f",

			logRawCounts[0],
			logRawCounts[1],
			logRawCounts[2],
			logRawMs2[0],
			logRawMs2[1],
			logRawMs2[2],
			logScaledMs2[0],
			logScaledMs2[1],
			logScaledMs2[2],
			logRestDetected ? 1u : 0u,
			static_cast<unsigned>(snapshotAgeMs),
			rawNorm,
			scaledNorm,
			normDelta,
			normDeltaPct
		);
	}

	void maybeLogAccelFullMatrixDebug() {
		const uint64_t now = millis();
		if (now - lastAccelFullMatrixDebugLogMillis
			< accelFullMatrixDebugLogIntervalMs) {
			return;
		}
		if (!accelFullMatrixRestSnapshot.valid) {
			return;
		}
		lastAccelFullMatrixDebugLogMillis = now;

		const sensor_real_t* logRawCounts = accelFullMatrixRestSnapshot.rawCounts;
		const sensor_real_t* logRawMs2 = accelFullMatrixRestSnapshot.rawMs2;
		const sensor_real_t* logScaledMs2 = accelFullMatrixRestSnapshot.scaledMs2;

		const bool logRestDetected = true;
		const uint32_t snapshotAgeMs
			= static_cast<uint32_t>(now - accelFullMatrixRestSnapshot.captureMillis);
		accelFullMatrixRestSnapshot.valid = false;

		const float rawNorm = accelVectorNorm(logRawMs2);
		const float scaledNorm = accelVectorNorm(logScaledMs2);
		const float normDelta = scaledNorm - rawNorm;
		const float normDeltaPct
			= rawNorm > 1e-6f ? (normDelta / rawNorm) * 100.0f : 0.0f;

		logger.info(
			"AccelFM dbg raw[cnt]=%.2f %.2f %.2f raw[m/s2]=%.3f %.3f %.3f "
			"scaled[m/s2]=%.3f %.3f %.3f rest=%u age_ms=%u |a|raw=%.3f "
			"|a|scaled=%.3f d|a|=%.3f d%%=%.2f",
			logRawCounts[0],
			logRawCounts[1],
			logRawCounts[2],
			logRawMs2[0],
			logRawMs2[1],
			logRawMs2[2],
			logScaledMs2[0],
			logScaledMs2[1],
			logScaledMs2[2],
			logRestDetected ? 1u : 0u,
			static_cast<unsigned>(snapshotAgeMs),
			rawNorm,
			scaledNorm,
			normDelta,
			normDeltaPct
		);
		// logger.info(
		// 	"AccelFM dbg Ainv row0=%.6f %.6f %.6f",
		// 	activeCalibration.A_Ainv[0][0],
		// 	activeCalibration.A_Ainv[0][1],
		// 	activeCalibration.A_Ainv[0][2]
		// );
		// logger.info(
		// 	"AccelFM dbg Ainv row1=%.6f %.6f %.6f",
		// 	activeCalibration.A_Ainv[1][0],
		// 	activeCalibration.A_Ainv[1][1],
		// 	activeCalibration.A_Ainv[1][2]
		// );
		// logger.info(
		// 	"AccelFM dbg Ainv row2=%.6f %.6f %.6f",
		// 	activeCalibration.A_Ainv[2][0],
		// 	activeCalibration.A_Ainv[2][1],
		// 	activeCalibration.A_Ainv[2][2]
		// );
	}
#endif

	bool shouldRequestMagCalibration(bool magCalibrationReady) const {
		return toggles.getToggle(SensorToggles::MagEnabled) && !magCalibrationReady
			&& !magCalibrationAttemptedSinceBoot;
	}

	bool shouldRequestMagCalibration() const {
		return shouldRequestMagCalibration(isMagCalibrationReady());
	}

	bool isMagCalibrationMissing(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		constexpr float epsilon = 1e-6f;
		bool offsetsAreZero = true;
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(config.M_B[i]) > epsilon) {
				offsetsAreZero = false;
				break;
			}
		}

		bool matrixIsIdentity = true;
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 3; j++) {
				const float expected = i == j ? 1.0f : 0.0f;
				if (std::abs(config.M_Ainv[i][j] - expected) > epsilon) {
					matrixIsIdentity = false;
					break;
				}
			}
		}

		return offsetsAreZero && matrixIsIdentity;
	}

	bool isMagCalibrationValid(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (uint8_t i = 0; i < 3; i++) {
			if (!std::isfinite(config.M_B[i])) {
				return false;
			}

			for (uint8_t j = 0; j < 3; j++) {
				const float v = config.M_Ainv[i][j];
				if (!std::isfinite(v) || std::abs(v) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(config.M_Ainv[i][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
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

	static bool isMagNormReferenceValid(float refNorm) {
		return std::isfinite(refNorm) && refNorm >= minValidMagReferenceNorm
			&& refNorm <= maxValidMagReferenceNorm;
	}

	bool isMagCalibrationReady() const {
		return !isMagCalibrationMissing(calibration)
			&& isMagCalibrationValid(calibration);
	}

	void syncMagCalibrationToActive() {
		for (uint8_t i = 0; i < 3; i++) {
			activeCalibration.M_B[i] = calibration.M_B[i];
			for (uint8_t j = 0; j < 3; j++) {
				activeCalibration.M_Ainv[i][j] = calibration.M_Ainv[i][j];
			}
		}
		activeCalibration.M_refNorm = calibration.M_refNorm;
	}

	static void resetMagCalibration(
		Configuration::RuntimeCalibrationSensorConfig& calibration
	) {
		for (uint8_t i = 0; i < 3; i++) {
			calibration.M_B[i] = 0.0f;
			for (uint8_t j = 0; j < 3; j++) {
				calibration.M_Ainv[i][j] = (i == j) ? 1.0f : 0.0f;
			}
		}
		calibration.M_refNorm = 0.0f;
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		calibration.data.runtimeCalibration = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
		MAG_CALIB = 16,
		ACCEL_EXTEND = 32,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS
		| CalibrationPrintFlags::MAG_CALIB | CalibrationPrintFlags::ACCEL_EXTEND;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature1,
					calibration.G_off1[0],
					calibration.G_off1[1],
					calibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature2,
					calibration.G_off2[0],
					calibration.G_off2[1],
					calibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_EXTEND)) {
			if (isAccelSixSideCalibrationMissing(calibration)) {
				logger.info("Accel six-side not calibrated");
			} else {
#if RC_ACCEL_MODE_IS_LEGACY_MAGNETO_SIX_SIDE
				logger.info(
					"Accel six-side (legacy magneto) B: %f %f %f",
					calibration.A_sixSideLegacyB[0],
					calibration.A_sixSideLegacyB[1],
					calibration.A_sixSideLegacyB[2]
				);
				logger.info("Accel six-side (legacy magneto) Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					logger.info(
						"  %f %f %f",
						calibration.A_sixSideLegacyAinv[i][0],
						calibration.A_sixSideLegacyAinv[i][1],
						calibration.A_sixSideLegacyAinv[i][2]
					);
				}
#else
				logger.info(
					"Accel six-side scale: %f %f %f",
					calibration.A_sixSideScale[0],
					calibration.A_sixSideScale[1],
					calibration.A_sixSideScale[2]
				);
				logger.info(
					"Accel six-side offset: %f %f %f",
					calibration.A_sixSideOff[0],
					calibration.A_sixSideOff[1],
					calibration.A_sixSideOff[2]
				);
#endif
			}

			if (isAccelFullMatrixCalibrationValid(calibration)) {
				logger.info(
					"Accel full-matrix A_B: %f %f %f",
					calibration.A_B[0],
					calibration.A_B[1],
					calibration.A_B[2]
				);
				logger.info("Accel full-matrix A_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					logger.info(
						"  %f %f %f",
						calibration.A_Ainv[i][0],
						calibration.A_Ainv[i][1],
						calibration.A_Ainv[i][2]
					);
				}
			} else {
				logger.info("Accel full-matrix not calibrated");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::MAG_CALIB)) {
			if (isMagCalibrationReady()) {
				logger.info(
					"Mag calibration offsets: %f %f %f",
					calibration.M_B[0],
					calibration.M_B[1],
					calibration.M_B[2]
				);
				logger.info("Mag calibration M_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					logger.info(
						"  %f %f %f",
						calibration.M_Ainv[i][0],
						calibration.M_Ainv[i][1],
						calibration.M_Ainv[i][2]
					);
				}
				logger.info("Mag calibration ref norm: %f", calibration.M_refNorm);
			} else {
				logger.info("Mag not calibrated");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{activeCalibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		calibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{calibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		calibration,
		static_cast<float>(Consts::AScale)
	};
	AccelFullMatrixCalibrationStep<RawSensorT> accelFullMatrixCalibrationStep;
	SelectedAccelSixSideCalibrationStep<RawSensorT> accelSixSideCalibrationStep;
	MagCalibrationStep<RawSensorT> magCalibrationStep;
	NullCalibrationStep<RawSensorT> nullCalibrationStep{calibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;
	bool pendingMagCalibrationRequest = false;
	bool magCalibrationAttemptedSinceBoot = false;
	bool startupAccelSixSideBootZGatePending = false;
	bool startupAccelSixSideMonitoringEnabled = false;
	float startupAccelSixSideBootZ = 0.0f;
	bool pendingAccelFullMatrixCalibrationRequest = false;
	bool pendingAccelSixSideCalibrationRequest = false;
	uint64_t startupAccelSixSideWindowStartMillis = 0;
	uint64_t startupAccelSixSideReadyLastLedMs = 0;
	uint64_t startupAccelSixSideReadyLedPulseEndMillis = 0;
	bool startupAccelSixSideReadyLedPulseActive = false;
#if RUNTIME_CALIBRATION_ENABLE_RUNTIME_ACCEL_DEBUG_LOGS
	static constexpr uint32_t accelSixSideDebugLogIntervalMs = 3000;
	uint64_t lastAccelSixSideDebugLogMillis = 0;
	static constexpr uint32_t accelFullMatrixDebugLogIntervalMs = 3000;
	uint64_t lastAccelFullMatrixDebugLogMillis = 0;
	AccelDebugSnapshot accelSixSideRestSnapshot{};
	AccelDebugSnapshot accelFullMatrixRestSnapshot{};
#endif

	SlimeVR::Configuration::RuntimeCalibrationSensorConfig calibration{
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},

		.M_B = {0.0, 0.0, 0.0},
		.M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		.M_refNorm = 0.0f,

		.A_sixSideOff = {0.0, 0.0, 0.0},
		.A_sixSideScale = {1.0, 1.0, 1.0},

		.accelFullMatrixCalibrated = false,
		.A_B = {0.0, 0.0, 0.0},
		.A_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},

		.A_sixSideLegacyB = {0.0, 0.0, 0.0},
		.A_sixSideLegacyAinv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}
	};

	float activeZROChange = 0;

	Configuration::RuntimeCalibrationSensorConfig activeCalibration = calibration;
	bool activeAccelFullMatrixValid = false;
	bool activeAccelSixSideValid = false;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
