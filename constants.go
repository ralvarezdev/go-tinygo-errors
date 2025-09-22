package go_tinygo_errors

import (
	tinygoerrors "github.com/ralvarezdev/tinygo-errors"
)

const (
	// ErrorCodeBuffersStartNumber is the starting number for error code buffers
	ErrorCodeBuffersStartNumber = 4000

	// ErrorCodeBNO08XStartNumber is the starting number for BNO08X-related error codes.
	ErrorCodeBNO08XStartNumber uint16 = 5000

	// ErrorCodePullUpResistorStartNumber is the starting number for pull-up resistor-related error codes.
	ErrorCodePullUpResistorStartNumber uint16 = 5200

	// ErrorCodeESCMotorStartNumber is the starting number for ESC motor-related error codes.
	ErrorCodeESCMotorStartNumber uint16 = 5210

	// ErrorCodeServoStartNumber is the starting number for servo-related error codes.
	ErrorCodeServoStartNumber uint16 = 5230
)

const (
	ErrorCodeBuffersInvalidBufferSize tinygoerrors.ErrorCode = ErrorCodeBuffersStartNumber + iota
	ErrorCodeBuffersTooMuchPrecisionDigitsForFloat64
)

const (
	ErrorCodeBNO08XPacketBufferIndexOutOfRange = tinygoerrors.ErrorCode(iota + ErrorCodeBNO08XStartNumber)
	ErrorCodeBNO08XInvalidChannelNumber
	ErrorCodeBNO08XNilPacketReader
	ErrorCodeBNO08XNilPacketWriter
	ErrorCodeBNO08XNilUARTBus
	ErrorCodeBNO08XFailedToConfigureUART
	ErrorCodeBNO08XFailedToResetUARTRVC
	ErrorCodeBNO08XUARTRVCNilFrame
	ErrorCodeBNO08XUARTRVCFrameTooShort
	ErrorCodeBNO08XUARTRVCInvalidChecksum
	ErrorCodeBNO08XUARTRVCByteTimeout
	ErrorCodeBNO08XUARTRVCFailedToReadByte
	ErrorCodeBNO08XUARTRVCUARTTimeout
	ErrorCodeBNO08XFailedToCreatePacket
	ErrorCodeBNO08XFailedToSetUARTFormat
	ErrorCodeBNO08XFailedToCreatePacketReader
	ErrorCodeBNO08XFailedToCreatePacketWriter
	ErrorCodeBNO08XUARTByteTimeout
	ErrorCodeBNO08XUARTFailedToReadByte
	ErrorCodeBNO08XFailedToResetBNO08X
	ErrorCodeBNO08XNilPacketBuffer
	ErrorCodeBNO08XFailedToGetExpectedReportLength
	ErrorCodeBNO08XInvalidReportLength
	ErrorCodeBNO08XFailedToGetReportID
	ErrorCodeBNO08XInsertCommandRequestReportBufferTooShort
	ErrorCodeBNO08XInsertCommandRequestReportTooManyArguments
	ErrorCodeBNO08XUnknownReportID
	ErrorCodeBNO08XFailedToCreatePacketFromBuffer
	ErrorCodeBNO08XFailedToCreateReportFromPacketBuffer
	ErrorCodeBNO08XInvalidReportIDToParseReport
	ErrorCodeBNO08XInvalidReportStabilityClassificationUint8
	ErrorCodeBNO08XInvalidReportActivityUint8
	ErrorCodeBNO08XInvalidReportIDForFourDimensionalParsing
	ErrorCodeBNO08XInvalidReportIDForThreeDimensionalParsing
	ErrorCodeBNO08XInvalidReportAccuracyStatusUint8
	ErrorCodeBNO08XSensorReportDataTooShort
	ErrorCodeBNO08XNoPacketAvailable
	ErrorCodeBNO08XInvalidReportDataLength
	ErrorCodeBNO08XUARTEndMissing
	ErrorCodeBNO08XPacketHeaderBufferTooShort
	ErrorCodeBNO08XNilDestinationBuffer
	ErrorCodeBNO08XInvalidStartOrEndIndex
	ErrorCodeBNO08XNilPacketData
	ErrorCodeBNO08XCommandRequestReportNilBuffer
	ErrorCodeBNO08XNilReportData
	ErrorCodeBNO08XNilCommandRequestReportParameters
	ErrorCodeBNO08XPacketDataTooShort
	ErrorCodeBNO08XI2CFailedToProbeDevice
	ErrorCodeBNO08XI2CFailedToProbeDeviceRepeatly
	ErrorCodeBNO08XNilI2CBus
	ErrorCodeBNO08XInvalidI2CAddress
	ErrorCodeBNO08XFailedToConfigureI2C
	ErrorCodeBNO08XI2CFailedToWritePacketHeaderBuffer
	ErrorCodeBNO08XI2CFailedToWritePacketPacketBuffer
	ErrorCodeBNO08XPacketBufferTooShortForPacketHeader
	ErrorCodeBNO08XPacketBufferTooShortForPacket
	ErrorCodeBNO08XI2CFailedToReadRequestedDataLength
	ErrorCodeBNO08XInvalidPacketSize
	ErrorCodeBNO08XNilSPIBus
	ErrorCodeBNO08XFailedToConfigureSPI
	ErrorCodeBNO08XFailedToWakeUpSPI
	ErrorCodeBNO08XSPIFailedToWritePacketHeaderBuffer
	ErrorCodeBNO08XSPIFailedToWritePacketPacketBuffer
	ErrorCodeBNO08XSPIFailedToReadRequestedDataLength
	ErrorCodeBNO08XFailedToEnableDependencyFeature
	ErrorCodeBNO08XFailedToEnableFeature
	ErrorCodeBNO08XFailedToBeginCalibration
	ErrorCodeBNO08XNilSubcommandParams
	ErrorCodeBNO08XFailedToInsertCommandRequestReport
	ErrorCodeBNO08XFailedToSendMeCommandRequestPacket
	ErrorCodeBNO08XFailedToSendCommandRequestPacketToSaveCalibrationData
	ErrorCodeBNO08XFailedToSaveCalibrationData
	ErrorCodeBNO08XFailedToSendResetCommandRequestPacket
	ErrorCodeBNO08XFailedToReadSensorID
	ErrorCodeBNO08XFailedToSendProductIDRequestPacket
	ErrorCodeBNO08XWaitingForPacketTimedOut
	ErrorCodeBNO08XMismatchedPacketDataLength
	ErrorCodeBNO08XFailedToSaveDynamicCalibrationData
	ErrorCodeBNO08XFailedToParseGetFeatureReport
	ErrorCodeBNO08XFailedToParseSensorID
	ErrorCodeBNO08XFailedToParseRawMagnetometerReport
	ErrorCodeBNO08XFailedToParseStepCounterReport
	ErrorCodeBNO08XFailedToParseShakeReport
	ErrorCodeBNO08XFailedToParseStabilityClassifierReport
	ErrorCodeBNO08XFailedToParseActivityClassifierReport
	ErrorCodeBNO08XFailedToParseMagnetometerReport
	ErrorCodeBNO08XFailedToParseRotationVectorReport
	ErrorCodeBNO08XFailedToParseGeomagneticRotationVectorReport
	ErrorCodeBNO08XFailedToParseGameRotationVectorReport
	ErrorCodeBNO08XFailedToParseAccelerometerReport
	ErrorCodeBNO08XFailedToParseLinearAccelerationReport
	ErrorCodeBNO08XFailedToParseGravityReport
	ErrorCodeBNO08XFailedToParseGyroscopeReport
	ErrorCodeBNO08XFailedToParseRawGyroscopeReport
	ErrorCodeBNO08XFailedToParseRawAccelerometerReport
	ErrorCodeBNO08XFailedToGetReportLengthForTheGivenReportID
	ErrorCodeBNO08XUnprocessableBatchBytes
	ErrorCodeBNO08XI2CFailedToReadPacketHeader
	ErrorCodeBNO08XSPIFailedToReadPacketHeader
	ErrorCodeBNO08XUnhandledUARTControlSHTPProtocol
	ErrorCodeBNO08XNilBNO08XInstance
	ErrorCodeBNO08XSetFeatureEnableReportDataNilBuffer
	ErrorCodeBNO08XSetFeatureEnableReportDataBufferTooShort
	ErrorCodeBNO08XNilPacketHeaderBuffer
	ErrorCodeBNO08XReportHeaderBufferTooShort
	ErrorCodeBNO08XNilWaitForPacketFunction
	ErrorCodeBNO08XInvalidMode
	ErrorCodeBNO08XUnknownModeAttemptingSoftwareReset
	ErrorCodeBNO08XUARTRVCInvalidYawDegreesValue
	ErrorCodeBNO08XUARTRVCInvalidPitchDegreesValue
	ErrorCodeBNO08XUARTRVCInvalidRollDegreesValue
)

const (
	ErrorCodePullUpResistorNilHandler = tinygoerrors.ErrorCode(iota + ErrorCodePullUpResistorStartNumber)
)

const (
	ErrorCodeESCMotorFailedToConfigurePWM tinygoerrors.ErrorCode = tinygoerrors.ErrorCode(iota + ErrorCodeESCMotorStartNumber)
	ErrorCodeESCMotorZeroFrequency
	ErrorCodeESCMotorSpeedOutOfRange
	ErrorCodeESCMotorNilHandler
	ErrorCodeESCMotorInvalidNeutralPulseWidth
	ErrorCodeESCMotorInvalidMinPulseWidth
	ErrorCodeESCMotorInvalidMaxPulseWidth
	ErrorCodeESCMotorInvalidMaxForwardSpeed
	ErrorCodeESCMotorInvalidMaxBackwardSpeed
	ErrorCodeESCMotorFailedToGetPWMChannel
)

const (
	ErrorCodeServoFailedToConfigurePWM tinygoerrors.ErrorCode = tinygoerrors.ErrorCode(iota + ErrorCodeServoStartNumber)
	ErrorCodeServoZeroFrequency
	ErrorCodeServoAngleOutOfRange
	ErrorCodeServoInvalidMinPulseWidth
	ErrorCodeServoInvalidMaxPulseWidth
	ErrorCodeServoNilHandler
	ErrorCodeServoUnknownDirection
	ErrorCodeServoFailedToGetPWMChannel
	ErrorCodeServoInvalidActuationRange
	ErrorCodeServoInvalidCenterAngle
)

var (
	// ErrorCodeMessages maps error codes to their corresponding error messages.
	ErrorCodeMessages = map[tinygoerrors.ErrorCode]string{
		// Buffers errors
		ErrorCodeBuffersInvalidBufferSize:                "invalid buffer size",
		ErrorCodeBuffersTooMuchPrecisionDigitsForFloat64: "too much precision digits for float64",

		// BNO08X errors
		ErrorCodeBNO08XPacketBufferIndexOutOfRange:                           "bno08x packet buffer index out of range",
		ErrorCodeBNO08XInvalidChannelNumber:                                  "bno08x invalid channel number",
		ErrorCodeBNO08XNilPacketReader:                                       "bno08x packet reader cannot be nil",
		ErrorCodeBNO08XNilPacketWriter:                                       "bno08x packet writer cannot be nil",
		ErrorCodeBNO08XNilUARTBus:                                            "bno08x uart bus cannot be nil",
		ErrorCodeBNO08XFailedToConfigureUART:                                 "bno08x failed to configure uart",
		ErrorCodeBNO08XFailedToResetUARTRVC:                                  "bno08x failed to reset uart rvc",
		ErrorCodeBNO08XUARTRVCNilFrame:                                       "bno08x uart rvc frame cannot be nil",
		ErrorCodeBNO08XUARTRVCFrameTooShort:                                  "bno08x uart rvc frame too short",
		ErrorCodeBNO08XUARTRVCInvalidChecksum:                                "bno08x uart rvc invalid checksum",
		ErrorCodeBNO08XUARTRVCByteTimeout:                                    "bno08x uart rvc byte timeout",
		ErrorCodeBNO08XUARTRVCFailedToReadByte:                               "bno08x uart rvc failed to read byte",
		ErrorCodeBNO08XUARTRVCUARTTimeout:                                    "bno08x uart rvc uart timeout",
		ErrorCodeBNO08XFailedToCreatePacket:                                  "bno08x failed to create packet",
		ErrorCodeBNO08XFailedToSetUARTFormat:                                 "bno08x failed to set uart format",
		ErrorCodeBNO08XFailedToCreatePacketReader:                            "bno08x failed to create packet reader",
		ErrorCodeBNO08XFailedToCreatePacketWriter:                            "bno08x failed to create packet writer",
		ErrorCodeBNO08XUARTByteTimeout:                                       "bno08x uart byte timeout",
		ErrorCodeBNO08XUARTFailedToReadByte:                                  "bno08x uart failed to read byte",
		ErrorCodeBNO08XFailedToResetBNO08X:                                   "bno08x failed to reset",
		ErrorCodeBNO08XNilPacketBuffer:                                       "bno08x packet buffer cannot be nil",
		ErrorCodeBNO08XFailedToGetExpectedReportLength:                       "bno08x failed to get expected report length",
		ErrorCodeBNO08XInvalidReportLength:                                   "bno08x invalid report length",
		ErrorCodeBNO08XFailedToGetReportID:                                   "bno08x failed to get report id",
		ErrorCodeBNO08XInsertCommandRequestReportBufferTooShort:              "bno08x insert command request report buffer too short",
		ErrorCodeBNO08XInsertCommandRequestReportTooManyArguments:            "bno08x insert command request report too many arguments",
		ErrorCodeBNO08XUnknownReportID:                                       "bno08x unknown report id",
		ErrorCodeBNO08XFailedToCreatePacketFromBuffer:                        "bno08x failed to create packet from buffer",
		ErrorCodeBNO08XFailedToCreateReportFromPacketBuffer:                  "bno08x failed to create report from packet buffer",
		ErrorCodeBNO08XInvalidReportIDToParseReport:                          "bno08x invalid report id to parse report",
		ErrorCodeBNO08XInvalidReportStabilityClassificationUint8:             "bno08x invalid report stability classification uint8",
		ErrorCodeBNO08XInvalidReportActivityUint8:                            "bno08x invalid report activity uint8",
		ErrorCodeBNO08XInvalidReportIDForFourDimensionalParsing:              "bno08x invalid report id for four dimensional parsing",
		ErrorCodeBNO08XInvalidReportIDForThreeDimensionalParsing:             "bno08x invalid report id for three dimensional parsing",
		ErrorCodeBNO08XInvalidReportAccuracyStatusUint8:                      "bno08x invalid report accuracy status uint8",
		ErrorCodeBNO08XSensorReportDataTooShort:                              "bno08x sensor report data too short",
		ErrorCodeBNO08XNoPacketAvailable:                                     "bno08x no packet available",
		ErrorCodeBNO08XInvalidReportDataLength:                               "bno08x invalid report data length",
		ErrorCodeBNO08XUARTEndMissing:                                        "bno08x uart end missing",
		ErrorCodeBNO08XPacketHeaderBufferTooShort:                            "bno08x packet header buffer too short",
		ErrorCodeBNO08XNilDestinationBuffer:                                  "bno08x destination buffer cannot be nil",
		ErrorCodeBNO08XInvalidStartOrEndIndex:                                "bno08x invalid start or end index",
		ErrorCodeBNO08XNilPacketData:                                         "bno08x packet data cannot be nil",
		ErrorCodeBNO08XCommandRequestReportNilBuffer:                         "bno08x command request report buffer cannot be nil",
		ErrorCodeBNO08XNilReportData:                                         "bno08x report data cannot be nil",
		ErrorCodeBNO08XNilCommandRequestReportParameters:                     "bno08x command request report parameters cannot be nil",
		ErrorCodeBNO08XPacketDataTooShort:                                    "bno08x packet data too short",
		ErrorCodeBNO08XI2CFailedToProbeDevice:                                "bno08x i2c failed to probe device",
		ErrorCodeBNO08XI2CFailedToProbeDeviceRepeatly:                        "bno08x i2c failed to probe device repeatedly",
		ErrorCodeBNO08XNilI2CBus:                                             "bno08x i2c bus cannot be nil",
		ErrorCodeBNO08XInvalidI2CAddress:                                     "bno08x invalid i2c address",
		ErrorCodeBNO08XFailedToConfigureI2C:                                  "bno08x failed to configure i2c",
		ErrorCodeBNO08XI2CFailedToWritePacketHeaderBuffer:                    "bno08x i2c failed to write packet header buffer",
		ErrorCodeBNO08XI2CFailedToWritePacketPacketBuffer:                    "bno08x i2c failed to write packet packet buffer",
		ErrorCodeBNO08XPacketBufferTooShortForPacketHeader:                   "bno08x packet buffer too short for packet header",
		ErrorCodeBNO08XPacketBufferTooShortForPacket:                         "bno08x packet buffer too short for packet",
		ErrorCodeBNO08XI2CFailedToReadRequestedDataLength:                    "bno08x i2c failed to read requested data length",
		ErrorCodeBNO08XInvalidPacketSize:                                     "bno08x invalid packet size",
		ErrorCodeBNO08XNilSPIBus:                                             "bno08x spi bus cannot be nil",
		ErrorCodeBNO08XFailedToConfigureSPI:                                  "bno08x failed to configure spi",
		ErrorCodeBNO08XFailedToWakeUpSPI:                                     "bno08x failed to wake up spi",
		ErrorCodeBNO08XSPIFailedToWritePacketHeaderBuffer:                    "bno08x spi failed to write packet header buffer",
		ErrorCodeBNO08XSPIFailedToWritePacketPacketBuffer:                    "bno08x spi failed to write packet packet buffer",
		ErrorCodeBNO08XSPIFailedToReadRequestedDataLength:                    "bno08x spi failed to read requested data length",
		ErrorCodeBNO08XFailedToEnableDependencyFeature:                       "bno08x failed to enable dependency feature",
		ErrorCodeBNO08XFailedToEnableFeature:                                 "bno08x failed to enable feature",
		ErrorCodeBNO08XFailedToBeginCalibration:                              "bno08x failed to begin calibration",
		ErrorCodeBNO08XNilSubcommandParams:                                   "bno08x subcommand params cannot be nil",
		ErrorCodeBNO08XFailedToInsertCommandRequestReport:                    "bno08x failed to insert command request report",
		ErrorCodeBNO08XFailedToSendMeCommandRequestPacket:                    "bno08x failed to send me command request packet",
		ErrorCodeBNO08XFailedToSendCommandRequestPacketToSaveCalibrationData: "bno08x failed to send command request packet to save calibration data",
		ErrorCodeBNO08XFailedToSaveCalibrationData:                           "bno08x failed to save calibration data",
		ErrorCodeBNO08XFailedToSendResetCommandRequestPacket:                 "bno08x failed to send reset command request packet",
		ErrorCodeBNO08XFailedToReadSensorID:                                  "bno08x failed to read sensor id",
		ErrorCodeBNO08XFailedToSendProductIDRequestPacket:                    "bno08x failed to send product id request packet",
		ErrorCodeBNO08XWaitingForPacketTimedOut:                              "bno08x waiting for packet timed out",
		ErrorCodeBNO08XMismatchedPacketDataLength:                            "bno08x mismatched packet data length",
		ErrorCodeBNO08XFailedToSaveDynamicCalibrationData:                    "bno08x failed to save dynamic calibration data",
		ErrorCodeBNO08XFailedToParseGetFeatureReport:                         "bno08x failed to parse get feature report",
		ErrorCodeBNO08XFailedToParseSensorID:                                 "bno08x failed to parse sensor id",
		ErrorCodeBNO08XFailedToParseRawMagnetometerReport:                    "bno08x failed to parse raw magnetometer report",
		ErrorCodeBNO08XFailedToParseStepCounterReport:                        "bno08x failed to parse step counter report",
		ErrorCodeBNO08XFailedToParseShakeReport:                              "bno08x failed to parse shake report",
		ErrorCodeBNO08XFailedToParseStabilityClassifierReport:                "bno08x failed to parse stability classifier report",
		ErrorCodeBNO08XFailedToParseActivityClassifierReport:                 "bno08x failed to parse activity classifier report",
		ErrorCodeBNO08XFailedToParseMagnetometerReport:                       "bno08x failed to parse magnetometer report",
		ErrorCodeBNO08XFailedToParseRotationVectorReport:                     "bno08x failed to parse rotation vector report",
		ErrorCodeBNO08XFailedToParseGeomagneticRotationVectorReport:          "bno08x failed to parse geomagnetic rotation vector report",
		ErrorCodeBNO08XFailedToParseGameRotationVectorReport:                 "bno08x failed to parse game rotation vector report",
		ErrorCodeBNO08XFailedToParseAccelerometerReport:                      "bno08x failed to parse accelerometer report",
		ErrorCodeBNO08XFailedToParseLinearAccelerationReport:                 "bno08x failed to parse linear acceleration report",
		ErrorCodeBNO08XFailedToParseGravityReport:                            "bno08x failed to parse gravity report",
		ErrorCodeBNO08XFailedToParseGyroscopeReport:                          "bno08x failed to parse gyroscope report",
		ErrorCodeBNO08XFailedToParseRawGyroscopeReport:                       "bno08x failed to parse raw gyroscope report",
		ErrorCodeBNO08XFailedToParseRawAccelerometerReport:                   "bno08x failed to parse raw accelerometer report",
		ErrorCodeBNO08XFailedToGetReportLengthForTheGivenReportID:            "bno08x failed to get report length for the given report id",
		ErrorCodeBNO08XUnprocessableBatchBytes:                               "bno08x unprocessable batch bytes",
		ErrorCodeBNO08XI2CFailedToReadPacketHeader:                           "bno08x i2c failed to read packet header",
		ErrorCodeBNO08XSPIFailedToReadPacketHeader:                           "bno08x spi failed to read packet header",
		ErrorCodeBNO08XUnhandledUARTControlSHTPProtocol:                      "bno08x unhandled uart control shtp protocol",
		ErrorCodeBNO08XNilBNO08XInstance:                                     "bno08x instance cannot be nil",
		ErrorCodeBNO08XSetFeatureEnableReportDataNilBuffer:                   "bno08x set feature enable report data buffer cannot be nil",
		ErrorCodeBNO08XSetFeatureEnableReportDataBufferTooShort:              "bno08x set feature enable report data buffer too short",
		ErrorCodeBNO08XNilPacketHeaderBuffer:                                 "bno08x packet header buffer cannot be nil",
		ErrorCodeBNO08XReportHeaderBufferTooShort:                            "bno08x report header buffer too short",
		ErrorCodeBNO08XNilWaitForPacketFunction:                              "bno08x wait for packet function cannot be nil",
		ErrorCodeBNO08XInvalidMode:                                           "bno08x invalid mode",
		ErrorCodeBNO08XUnknownModeAttemptingSoftwareReset:                    "bno08x unknown mode attempting software reset",
		ErrorCodeBNO08XUARTRVCInvalidYawDegreesValue:                         "bno08x uart rvc invalid yaw degrees value",
		ErrorCodeBNO08XUARTRVCInvalidPitchDegreesValue:                       "bno08x uart rvc invalid pitch degrees value",
		ErrorCodeBNO08XUARTRVCInvalidRollDegreesValue:                        "bno08x uart rvc invalid roll degrees value",

		// Pull-up resistor errors
		ErrorCodePullUpResistorNilHandler: "pull-up resistor handler cannot be nil",

		// ESC motor errors
		ErrorCodeESCMotorFailedToConfigurePWM:    "esc motor failed to configure pwm",
		ErrorCodeESCMotorZeroFrequency:           "esc motor frequency cannot be zero",
		ErrorCodeESCMotorSpeedOutOfRange:         "esc motor speed out of range",
		ErrorCodeESCMotorNilHandler:              "esc motor handler cannot be nil",
		ErrorCodeESCMotorInvalidNeutralPulseWidth: "esc motor invalid neutral pulse width",
		ErrorCodeESCMotorInvalidMinPulseWidth:    "esc motor invalid min pulse width",
		ErrorCodeESCMotorInvalidMaxPulseWidth:    "esc motor invalid max pulse width",
		ErrorCodeESCMotorInvalidMaxForwardSpeed:   "esc motor invalid max forward speed",
		ErrorCodeESCMotorInvalidMaxBackwardSpeed:  "esc motor invalid max backward speed",
		ErrorCodeESCMotorFailedToGetPWMChannel:    "esc motor failed to get pwm channel",

		// Servo errors
		ErrorCodeServoFailedToConfigurePWM:    "servo failed to configure pwm",
		ErrorCodeServoZeroFrequency:           "servo frequency cannot be zero",
		ErrorCodeServoAngleOutOfRange:         "servo angle out of range",
		ErrorCodeServoInvalidMinPulseWidth:    "servo invalid min pulse width",
		ErrorCodeServoInvalidMaxPulseWidth:    "servo invalid max pulse width",
		ErrorCodeServoUnknownDirection:        "servo unknown direction",
		ErrorCodeServoFailedToGetPWMChannel:   "servo failed to get pwm channel",
		ErrorCodeServoInvalidActuationRange:   "servo invalid actuation range",
		ErrorCodeServoInvalidCenterAngle:      "servo invalid center angle",
		ErrorCodeServoNilHandler:              "servo handler cannot be nil",
	}
)
