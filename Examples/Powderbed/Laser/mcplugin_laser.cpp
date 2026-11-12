/*++

Copyright (C) 2020 Autodesk Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the Autodesk Inc. nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AUTODESK INC. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "libmcplugin_statefactory.hpp" 
#include "libmcplugin_interfaceexception.hpp"
#include "libmcplugin_state.hpp" 

#include "libmcdriver_scanlab_dynamic.hpp"
#include "libmcenv_drivercast.hpp"

#include <cmath>
#include <vector>

using namespace LibMCPlugin::Impl;

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4250)
#endif

/*************************************************************************************************************************
 Import functionality for Driver into current plugin
**************************************************************************************************************************/
LIBMC_IMPORTDRIVERCLASSES(ScanLab, ScanLab_RTC6)


/*************************************************************************************************************************
 Class definition of CLaserData
**************************************************************************************************************************/
class CLaserData : public virtual CPluginData {
protected:
	// Globally store driver wrappers in the plugin
	PDriverCast_ScanLab_RTC6 m_DriverCast_ScanLab;

public:
	PDriver_ScanLab_RTC6 acquireScanLabDriver(LibMCEnv::PStateEnvironment pStateEnvironment) {
		return m_DriverCast_ScanLab.acquireDriver(pStateEnvironment, "scanlab");
	}
};


/*************************************************************************************************************************
 State type definition
**************************************************************************************************************************/
typedef CState<CLaserData> CLaserState;


/*************************************************************************************************************************
 Custom functionality implementation
**************************************************************************************************************************/
void InitialiseScanlabDriver(LibMCEnv::PStateEnvironment pStateEnvironment, PDriver_ScanLab_RTC6 pDriver)
{
	pStateEnvironment->LogMessage("Initialising Scanlab Driver");

	if (pStateEnvironment->GetBoolParameter("cardconfig", "simulatelaser")) {
		pStateEnvironment->LogMessage("Laser Simulation enabled!...");
		pDriver->SetToSimulationMode();
	}

	if (!pDriver->IsSimulationMode()) {
		pDriver->LoadSDK("rtc6dllx64");

		std::string sCardConfigGroup = "cardconfig";
		uint32_t nRTCIndex = 1;

		auto sIP = pStateEnvironment->GetStringParameter(sCardConfigGroup, "ipaddress");
		auto sNetmask = pStateEnvironment->GetStringParameter(sCardConfigGroup, "netmask");
		auto nTimeout = pStateEnvironment->GetIntegerParameter(sCardConfigGroup, "timeout");
		auto nSerial = pStateEnvironment->GetIntegerParameter(sCardConfigGroup, "serial");
		auto dLaserOnDelay = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laserondelay");
		auto dLaserOffDelay = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laseroffdelay");
		auto dMarkDelay = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "markdelay");
		auto dJumpDelay = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "jumpdelay");
		auto dPolygonDelay = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "polygondelay");

		auto dLaserOriginX = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laseroriginx");
		auto dLaserOriginY = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laseroriginy");
		auto dLaserFieldMinX = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laserfield_minx");
		auto dLaserFieldMinY = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laserfield_miny");
		auto dLaserFieldMaxX = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laserfield_maxx");
		auto dLaserFieldMaxY = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "laserfield_maxy");

		auto dTransformationAngle = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "transformation_angle");
		auto dTransformationOffsetX = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "transformation_offsetx");
		auto dTransformationOffsetY = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "transformation_offsety");
		auto dTransformationScaleX = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "transformation_scalex");
		auto dTransformationScaleY = pStateEnvironment->GetDoubleParameter(sCardConfigGroup, "transformation_scaley");


		auto sCorrectionResourceName = pStateEnvironment->GetStringParameter("correction", "resourcename");
		auto nTableIndex = (uint32_t)pStateEnvironment->GetIntegerParameter("correction", "tableindex");
		auto nDimension = (uint32_t)pStateEnvironment->GetIntegerParameter("correction", "dimension");
		auto nTableNumberHeadA = (uint32_t)pStateEnvironment->GetIntegerParameter("correction", "tablenumbera");
		auto nTableNumberHeadB = (uint32_t)pStateEnvironment->GetIntegerParameter("correction", "tablenumberb");
		auto dCorrectionFactor = pStateEnvironment->GetDoubleParameter("correction", "correctionfactor");


		pStateEnvironment->LogMessage("Acquiring ScanLab card #" + std::to_string(nSerial));
		pDriver->Initialise(sIP, sNetmask, (uint32_t)nTimeout, (uint32_t)nSerial);

		std::string sFirmwareResource;
		if (sIP.empty()) {
			pStateEnvironment->LogMessage("Loading RTC PCI firmware...");
			sFirmwareResource = "rtc6out";
		}
		else {
			pStateEnvironment->LogMessage("Loading RTC Ethernet firmware...");
			sFirmwareResource = "rtc6eth";
		}

		pDriver->LoadFirmware(sFirmwareResource, "rtc6rbf", "rtc6dat");

		std::vector<uint8_t> CorrectionFileBuffer;
		pStateEnvironment->LogMessage("Loading correction file...");
		pStateEnvironment->LoadResourceData(sCorrectionResourceName, CorrectionFileBuffer);

		pDriver->SetCorrectionFile(CorrectionFileBuffer, nTableIndex, nDimension, nTableNumberHeadA, nTableNumberHeadB);

		auto pContext = pDriver->GetContext();
		if (dCorrectionFactor > 0.0) {
			pContext->SetCorrectionFactors(dCorrectionFactor, dCorrectionFactor);
		}

		auto dLaserPowerAt0Percent = pStateEnvironment->GetDoubleParameter("powercalibration", "calibrationsetpoint0");
		auto dLaserPowerAt100Percent = pStateEnvironment->GetDoubleParameter("powercalibration", "calibrationsetpoint100");

		double dPulseHalfPeriodInMicroseconds = pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulsehalfperiod");
		double dPulseLengthInMicroseconds = pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulselength");
		double dStandbyHalfPeriodInMicroseconds = pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbyhalfperiod");
		double dStandbyLengthInMicroseconds = pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbylength");

		pStateEnvironment->LogMessage("Setting up laser fields and transformation.");

		double dBitsPerMM = pContext->GetCorrectionFactor();
		pStateEnvironment->LogMessage("Setting laser field for laser " + std::to_string(nRTCIndex) + ": " + std::to_string(dLaserFieldMinX) + "/" + std::to_string(dLaserFieldMinY) + " - " + std::to_string(dLaserFieldMaxX) + "/" + std::to_string(dLaserFieldMaxY));

		pStateEnvironment->LogMessage("  (in laser coordinates: " + std::to_string(dLaserFieldMinX - dLaserOriginX) + "/" + std::to_string(dLaserFieldMinY - dLaserOriginY) + " - " + std::to_string(dLaserFieldMaxX - dLaserOriginX) + "/" + std::to_string(dLaserFieldMaxY - dLaserOriginY));
		pStateEnvironment->LogMessage("Setting laser origin");
		pContext->SetLaserOrigin(dLaserOriginX, dLaserOriginY);

		pStateEnvironment->LogMessage("Setting laser field");
		// Don't set laser field for now because we have to take the rotation into account. The laser field will be set in the transformation matrix without range checking...
		//pContext->SetLaserField(dLaserFieldMinX - dLaserOriginX, dLaserFieldMinY - dLaserOriginY, dLaserFieldMaxX - dLaserOriginX, dLaserFieldMaxY - dLaserOriginY);

		//pStateEnvironment->LogMessage("Enable Range Checking");
		//pContext->EnableRangeChecking();

		double dAngleInRadian = dTransformationAngle / 180.0 * 3.14159265359;
		pStateEnvironment->LogMessage("Set Transformation Offset: " + std::to_string((int32_t)round(dTransformationOffsetX * dBitsPerMM)) + " / " + std::to_string((int32_t)round(dTransformationOffsetY * dBitsPerMM)));
		pContext->SetTransformationOffset((int32_t)round(dTransformationOffsetX * dBitsPerMM), (int32_t)round(dTransformationOffsetY * dBitsPerMM));


		double dM11 = dTransformationScaleX * cos(dAngleInRadian);
		double dM12 = -dTransformationScaleY * sin(dAngleInRadian);
		double dM21 = dTransformationScaleX * sin(dAngleInRadian);
		double dM22 = dTransformationScaleY * cos(dAngleInRadian);

		pStateEnvironment->LogMessage("Set Transformation Matrix: " + std::to_string(dM11) + " / " + std::to_string(dM12) + " / " + std::to_string(dM21) + " / " + std::to_string(dM22));
		pContext->SetTransformationMatrix(dM11, dM12, dM21, dM22);


		pStateEnvironment->LogMessage("Configuring laser...");
		pStateEnvironment->LogMessage("Setting Signal half period to " + std::to_string(dPulseHalfPeriodInMicroseconds) + "microseconds and pulse length " + std::to_string(dPulseLengthInMicroseconds));
		pStateEnvironment->LogMessage("Setting Standby half period to " + std::to_string(dStandbyHalfPeriodInMicroseconds) + "microseconds and pulse length " + std::to_string(dStandbyLengthInMicroseconds));

		pDriver->SetLaserSignalTimingDefaults(dPulseHalfPeriodInMicroseconds, dPulseLengthInMicroseconds, dStandbyHalfPeriodInMicroseconds, dStandbyLengthInMicroseconds);

		// Which line of the card the laser is wired to, and how it is wired, differs between machines, so
		// all of it is machine settings rather than hardcoded values: "laseronsignallowactive" inverts the
		// LASER_ON signal (set_laser_control bit 3), "laser1signallowactive" the laser port pin LASER_1
		// (bit 4), and "modpinlaseron" selects the line that carries the pulse modulation. A laser that
		// emits while its line is low or floating needs the polarity of that line inverted, otherwise it
		// stays on whenever the card drives the line into its inactive state.
		bool bLaserOnSignalLowActive = pStateEnvironment->GetBoolParameter("cardconfig", "laseronsignallowactive");
		bool bLaser1SignalLowActive = pStateEnvironment->GetBoolParameter("cardconfig", "laser1signallowactive");
		pStateEnvironment->LogMessage("Laser signal polarity: LASER_ON " + std::string((bLaserOnSignalLowActive) ? "LOW" : "HIGH") +
			" active, LASER_1 " + std::string((bLaser1SignalLowActive) ? "LOW" : "HIGH") + " active.");

		pDriver->ConfigureLaserMode(LibMCDriver_ScanLab::eLaserMode::CO2, LibMCDriver_ScanLab::eLaserPort::LaserPulseModulation, dLaserPowerAt100Percent, false, false, bLaserOnSignalLowActive, bLaser1SignalLowActive, false, false);

		// Route the pulse modulation onto the laser line this machine is wired to, and release the laser
		// signal: it gates the modulation (while asserted, the routed pin carries the pulse train), but the
		// card's power-up default is on - without clearing it here, the laser would emit whenever nothing
		// else gates the modulation, i.e. permanently. RTC6 config_laser_signals: bit 0 modulates the
		// LASER_ON pin, bit 1 the LASER_1 pin; the routing is a power-up default too and has to be applied
		// on every start.
		if (!pDriver->IsSimulationMode()) {
			bool bModPinLaserOn = pStateEnvironment->GetBoolParameter("cardconfig", "modpinlaseron");
			pDriver->GetContext()->ConfigLaserSignals((bModPinLaserOn) ? 1 : 2);
			pStateEnvironment->LogMessage("Laser pulse modulation routed to " + std::string((bModPinLaserOn) ? "LASER_ON pin." : "LASER_1 pin."));
			pDriver->GetContext()->SetLaserSignal(false);
		}

		pStateEnvironment->LogMessage("Configuring delays...");
		pDriver->ConfigureDelays(dLaserOnDelay, dLaserOffDelay, dMarkDelay, dJumpDelay, dPolygonDelay);

		
		pStateEnvironment->LogMessage("Updating Laser Power Calibration table...");
		std::vector<LibMCDriver_ScanLab::sLaserCalibrationPoint> calibrationPoints;
		for (uint32_t nPercentage = 1; nPercentage <= 99; nPercentage++) {
			std::string sCalibrationParameterName = "calibrationsetpoint" + std::to_string(nPercentage);
			if (pStateEnvironment->HasParameter("powercalibration", sCalibrationParameterName)) {
				double dLaserPowerInWatts = pStateEnvironment->GetDoubleParameter("powercalibration", sCalibrationParameterName);
				pStateEnvironment->LogMessage("Calibration Power: " + std::to_string(dLaserPowerInWatts) + " Watts = " + std::to_string(nPercentage) + "%");

				calibrationPoints.push_back({ (double)nPercentage, (double)dLaserPowerInWatts });
			}
		}

		pContext->SetPiecewiseLinearLaserPowerCalibration(dLaserPowerAt0Percent, dLaserPowerAt100Percent, calibrationPoints);
		

		for (double dWatts = 0; dWatts <= 18.0; dWatts += 0.5) {
			double dPercent = pContext->MapPowerWattsToPercent(dWatts);
			pStateEnvironment->LogMessage("Power: " + std::to_string(dWatts) + " Watts = " + std::to_string(dPercent) + "%");
		}
	}
	else {
		pStateEnvironment->LogMessage("Laser in simulation mode - skipping hardware initialisation");
	}
}


struct sScannerRecordingSettings {
	bool m_bEnabled = false;
	LibMCDriver_ScanLab::eRTCRecordingFrequency m_Frequency = LibMCDriver_ScanLab::eRTCRecordingFrequency::InvalidFrequency;
	double m_dFrequencyInKHz = 0.0;
	bool m_bTargetPosition = false;
	bool m_bActualPosition = false;
	bool m_bLaserOn = false;
	bool m_bPulseLength = false;
	bool m_bAnalogOut1 = false;
	bool m_bStoreInExecution = false;
	bool m_bStoreInJournal = false;
	bool m_bStoreInBuild = false;
};

LibMCDriver_ScanLab::eRTCRecordingFrequency MapScannerRecordingFrequency(int64_t nFrequencyInKHz)
{
	switch (nFrequencyInKHz) {
		case 100: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record100kHz;
		case 50: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record50kHz;
		case 25: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record25kHz;
		case 20: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record20kHz;
		case 10: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record10kHz;
		case 5: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record5kHz;
		case 4: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record4kHz;
		case 2: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record2kHz;
		case 1: return LibMCDriver_ScanLab::eRTCRecordingFrequency::Record1kHz;
		default:
			throw std::runtime_error("invalid scanner recording frequency: " + std::to_string(nFrequencyInKHz) + " kHz (allowed: 100, 50, 25, 20, 10, 5, 4, 2 or 1)");
	}
}

// Reads the "scannerrecording" parameter group. The remaining settings are only read and validated if recording is enabled.
sScannerRecordingSettings ReadScannerRecordingSettings(LibMCEnv::PStateEnvironment pStateEnvironment)
{
	const std::string sGroup = "scannerrecording";

	sScannerRecordingSettings settings;
	settings.m_bEnabled = pStateEnvironment->GetBoolParameter(sGroup, "enabled");
	if (!settings.m_bEnabled)
		return settings;

	int64_t nFrequencyInKHz = pStateEnvironment->GetIntegerParameter(sGroup, "frequency");
	settings.m_Frequency = MapScannerRecordingFrequency(nFrequencyInKHz);
	settings.m_dFrequencyInKHz = (double)nFrequencyInKHz;

	settings.m_bTargetPosition = pStateEnvironment->GetBoolParameter(sGroup, "targetposition");
	settings.m_bActualPosition = pStateEnvironment->GetBoolParameter(sGroup, "actualposition");
	settings.m_bLaserOn = pStateEnvironment->GetBoolParameter(sGroup, "laseron");
	settings.m_bPulseLength = pStateEnvironment->GetBoolParameter(sGroup, "pulselength");
	settings.m_bAnalogOut1 = pStateEnvironment->GetBoolParameter(sGroup, "analogout1");
	settings.m_bStoreInExecution = pStateEnvironment->GetBoolParameter(sGroup, "storeinexecution");
	settings.m_bStoreInJournal = pStateEnvironment->GetBoolParameter(sGroup, "storeinjournal");
	settings.m_bStoreInBuild = pStateEnvironment->GetBoolParameter(sGroup, "storeinbuild");

	if (!(settings.m_bTargetPosition || settings.m_bActualPosition || settings.m_bLaserOn || settings.m_bPulseLength || settings.m_bAnalogOut1))
		throw std::runtime_error("scanner recording is enabled, but no signal is selected");
	if (!(settings.m_bStoreInExecution || settings.m_bStoreInJournal || settings.m_bStoreInBuild))
		throw std::runtime_error("scanner recording is enabled, but no storage (execution, journal or build) is selected");

	return settings;
}

// Same list sequence as CDriver_ScanLab_RTC6::DrawLayer, with set_trigger8 recording around the layer.
// Only returns once the list has finished and all samples have been read from the card.
LibMCDriver_ScanLab::PRTCRecording DrawLayerWithRecording(PDriver_ScanLab_RTC6 pDriver, LibMCEnv::PBuild pBuildJob, uint32_t nLayerIndex, const sScannerRecordingSettings& settings)
{
	auto pContext = pDriver->GetContext();
	auto pRecording = pContext->PrepareRecording(false, settings.m_bActualPosition, settings.m_bActualPosition);

	if (settings.m_bTargetPosition) {
		pRecording->AddChannel("targetx", LibMCDriver_ScanLab::eRTCChannelType::ChannelTargetX);
		pRecording->AddChannel("targety", LibMCDriver_ScanLab::eRTCChannelType::ChannelTargetY);
	}
	if (settings.m_bActualPosition) {
		pRecording->AddChannel("actualx", LibMCDriver_ScanLab::eRTCChannelType::ChannelCurrentXRaw);
		pRecording->AddChannel("actualy", LibMCDriver_ScanLab::eRTCChannelType::ChannelCurrentYRaw);
	}
	if (settings.m_bLaserOn)
		pRecording->AddChannel("laseron", LibMCDriver_ScanLab::eRTCChannelType::ChannelLaserOn);
	if (settings.m_bPulseLength)
		pRecording->AddChannel("pulselength", LibMCDriver_ScanLab::eRTCChannelType::ChannelPulseLength);
	if (settings.m_bAnalogOut1)
		pRecording->AddChannel("analogout1", LibMCDriver_ScanLab::eRTCChannelType::ChannelAnalogOut1);

	auto pAccessor = pBuildJob->CreateToolpathAccessor();
	auto pLayer = pAccessor->LoadLayer(nLayerIndex);

	pContext->SetStartList(1, 0);
	pRecording->EnableRecording(settings.m_Frequency);

	pContext->AddLayerToList(pLayer, false);

	pRecording->DisableRecording();
	pContext->SetEndOfList();

	pRecording->ExecuteListWithRecording();

	return pRecording;
}

void StoreScannerRecording(LibMCEnv::PStateEnvironment pStateEnvironment, LibMCDriver_ScanLab::PRTCRecording pRecording, LibMCEnv::PBuild pBuildJob, const std::string& sExecutionUUID, uint32_t nLayerIndex, const sScannerRecordingSettings& settings)
{
	uint64_t nRecordCount = 0;
	for (const std::string sChannelName : { "targetx", "actualx", "laseron", "pulselength", "analogout1" }) {
		if (pRecording->HasChannel(sChannelName)) {
			nRecordCount = pRecording->GetRecordCount(sChannelName);
			break;
		}
	}

	auto pDataTable = pStateEnvironment->CreateDataTable();

	std::vector<double> timeInMS(nRecordCount);
	for (uint64_t nIndex = 0; nIndex < nRecordCount; nIndex++)
		timeInMS.at(nIndex) = (double)nIndex / settings.m_dFrequencyInKHz;
	pDataTable->AddColumn("time", "Time (ms)", LibMCEnv::eDataTableColumnType::DoubleColumn);
	pDataTable->SetDoubleColumnValues("time", timeInMS);

	if (settings.m_bTargetPosition)
		pRecording->AddTargetPositionsToDataTable(pDataTable, "targetx", "Target X (mm)", "targety", "Target Y (mm)");
	if (settings.m_bActualPosition)
		pRecording->AddBacktransformedXYPositionsToDataTable(pDataTable, "actualx", "Actual X (mm)", "actualy", "Actual Y (mm)");
	if (settings.m_bLaserOn)
		pRecording->AddRecordsToDataTable("laseron", pDataTable, "laseron", "Laser On");
	if (settings.m_bPulseLength)
		pRecording->AddRecordsToDataTable("pulselength", pDataTable, "pulselength", "Pulse Length (raw)");
	if (settings.m_bAnalogOut1)
		pRecording->AddRecordsToDataTable("analogout1", pDataTable, "analogout1", "Analog Out 1 (raw)");

	std::string sIdentifier = "scannerrecording_layer" + std::to_string(nLayerIndex);
	std::string sName = "Scanner Recording Layer " + std::to_string(nLayerIndex);

	if (settings.m_bStoreInExecution) {
		if ((!sExecutionUUID.empty()) && pBuildJob->HasExecution(sExecutionUUID)) {
			auto pExecution = pBuildJob->FindExecution(sExecutionUUID);
			// The UI viewer loads the plain identifier; a re-exposed layer gets a suffix.
			std::string sExecutionIdentifier = sIdentifier;
			uint32_t nRun = 1;
			while (pExecution->HasAttachmentIdentifier(sExecutionIdentifier)) {
				nRun++;
				sExecutionIdentifier = sIdentifier + "_" + std::to_string(nRun);
			}

			pExecution->StoreDataTable(sExecutionIdentifier, sName, pDataTable, nullptr, "");
		}
		else {
			pStateEnvironment->LogWarning("Scanner recording of layer " + std::to_string(nLayerIndex) + " not stored with a build execution: no execution is running");
		}
	}

	if (settings.m_bStoreInJournal) {
		auto pStream = pStateEnvironment->CreateTemporaryStream(sIdentifier, "text/csv");
		pDataTable->WriteCSVToStream(pStream, nullptr);
		pStream->Finish();
	}

	if (settings.m_bStoreInBuild) {
		// Build attachment identifiers must be unique, and a build can be exposed more than once.
		std::string sBuildIdentifier = sIdentifier;
		uint32_t nRun = 1;
		while (pBuildJob->HasAttachmentIdentifier(sBuildIdentifier)) {
			nRun++;
			sBuildIdentifier = sIdentifier + "_" + std::to_string(nRun);
		}

		pBuildJob->StoreDataTable(sBuildIdentifier, sName, pDataTable, nullptr, "");
	}

	pStateEnvironment->LogMessage("Stored scanner recording of layer " + std::to_string(nLayerIndex) + " (" + std::to_string(nRecordCount) + " samples)");
}


/*************************************************************************************************************************
 Class definition of CLaserState_Init
**************************************************************************************************************************/
// Standard resting state after power on. It does no work itself; initialisation is triggered
// by transitioning into "init" (on start-up, or after a fatal error has been cleared).
// The card settings are derived from main, which mirrors them from the machine configuration
// during its own init, so the laser must not initialise before main has applied it.
class CLaserState_WaitForInit : public virtual CLaserState {
public:
	CLaserState_WaitForInit(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "waitforinit";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		if (!pStateEnvironment->GetBoolParameter("cardconfig", "configurationapplied")) {
			pStateEnvironment->SetNextState("waitforinit");
			return;
		}

		pStateEnvironment->SetNextState("init");
	}
};


class CLaserState_Init : public virtual CLaserState {
public:
	CLaserState_Init(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "init";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		pStateEnvironment->LogMessage("Initializing laser subsystem...");

		// Discard any stale signals (e.g. a recovery request) left over from a previous run
		// or error before re-establishing a clean baseline.
		pStateEnvironment->ClearAllUnhandledSignals();

		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType == "scanlab") {
			InitialiseScanlabDriver(pStateEnvironment, m_pPluginData->acquireScanLabDriver(pStateEnvironment));
		}
		else {
			throw std::runtime_error("invalid scanner card type: " + sCardType);
		}

		pStateEnvironment->SetNextState("idle");
	}
};


/*************************************************************************************************************************
 Ends a running timed laser emission: drops the laser enable pin and restores the configured pulse
 modulation timing. Shared by the powermeasurement state and the fatal-error recovery, it is a no-op
 while no timed emission is active.
**************************************************************************************************************************/
static void EndPowerMeasurementEmission(PPluginData pPluginData, LibMCEnv::PStateEnvironment pStateEnvironment)
{
	if (!pStateEnvironment->GetBoolParameter("powermeasurement", "active"))
		return;

	pStateEnvironment->SetBoolParameter("powermeasurement", "active", false);
	pStateEnvironment->SetIntegerParameter("powermeasurement", "remainingms", 0);

	std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
	if (sCardType == "scanlab") {
		auto pLaserData = std::dynamic_pointer_cast<CLaserData>(pPluginData);
		if (pLaserData != nullptr) {
			auto pDriver = pLaserData->acquireScanLabDriver(pStateEnvironment);
			if (!pDriver->IsSimulationMode()) {
				auto pContext = pDriver->GetContext();

				// Put the timings used for normal marking back onto the card.
				pContext->SetStandbyInMicroSeconds(pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbyhalfperiod"),
					pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbylength"));
				pContext->SetLaserPulsesInMicroSeconds(pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulsehalfperiod"),
					pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulselength"));

				pContext->SetLaserSignal(false);
				pContext->SetLaserPinOut(false, false);
			}
		}
	}

	pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", false);
	pStateEnvironment->LogMessage("Timed laser emission finished.");
}


/*************************************************************************************************************************
 Direct LASER1 pulse modulation ("Power Measurement" page)

 The RTC6 generates the modulation itself: with CO2 laser mode active (set_laser_mode(0), applied during laser
 initialisation) the control command set_laser_pulses_ctrl programs a fixed-rate pulse train, and the pin the
 modulation is routed to carries it while the laser signal is asserted. This control always routes it onto the
 LASER_1 pin, never onto the laser enable line. Only the pulse length changes from setting to setting - the half
 period is derived once per run and stays fixed, so the modulation frequency never follows the duty cycle:

	half_period   = round(64 MHz / (2 * frequency_hz))   (RTC6 Standard compatibility mode, 1/64 us counts)
	duty          = clamp(duty_centipoints / 10000, 0, 1)
	pulse_length  = round(2 * half_period * duty)

 0 % maps to pulse_length = 0, which leaves LASER1 inactive for the duration of the run because the standby
 pulses are disabled while the modulation runs (RTC6 manual Rev. 1.0.20, pp. 185-188 and p. 667); 100 % maps to
 the full period, which leaves LASER1 asserted throughout the active window.

 Assumptions to verify on the bench: the counts are derived for the 64 MHz timer of RTC6 Standard compatibility
 mode, which nothing in this path changes, and the half period is kept inside the 15-bit range of the laser pulse
 control (frequencies below ~1 kHz would need a different mode).
**************************************************************************************************************************/
static const double kRtcTimerClockHz = 64000000.0;
static const int64_t kLaserPwmMaxHalfPeriodCounts = 32767;
static const int64_t kLaserModulationMinFrequencyHz = 1;
static const int64_t kLaserModulationMaxFrequencyHz = 500000;
static const int64_t kLaserModulationMinRunningMS = 100;
static const int64_t kLaserModulationMaxRunningMS = 300000;

struct sLaserPwmTiming {
	int64_t m_nHalfPeriodCounts;
	int64_t m_nPulseLengthCounts;
};

static sLaserPwmTiming CalculateLaserPwmTiming(int64_t nFrequencyHz, int64_t nDutyCentiPercent)
{
	if ((nFrequencyHz < kLaserModulationMinFrequencyHz) || (nFrequencyHz > kLaserModulationMaxFrequencyHz))
		throw std::runtime_error("modulation frequency out of range: " + std::to_string(nFrequencyHz) + " Hz (allowed: " +
			std::to_string(kLaserModulationMinFrequencyHz) + " to " + std::to_string(kLaserModulationMaxFrequencyHz) + " Hz)");

	double dDuty = (double)nDutyCentiPercent / 10000.0;
	if (!(dDuty > 0.0))
		dDuty = 0.0;
	if (dDuty > 1.0)
		dDuty = 1.0;

	const double dHalfPeriodCounts = std::round(kRtcTimerClockHz / (2.0 * (double)nFrequencyHz));
	if (!(dHalfPeriodCounts >= 1.0))
		throw std::runtime_error("modulation frequency too high for the RTC6 laser pulse control: " + std::to_string(nFrequencyHz) + " Hz");

	sLaserPwmTiming timing;
	timing.m_nHalfPeriodCounts = (int64_t)dHalfPeriodCounts;
	if (timing.m_nHalfPeriodCounts > kLaserPwmMaxHalfPeriodCounts)
		throw std::runtime_error("modulation frequency too low for the RTC6 laser pulse control: " + std::to_string(nFrequencyHz) +
			" Hz (the half period exceeds its " + std::to_string(kLaserPwmMaxHalfPeriodCounts) + " count limit below " +
			std::to_string((int64_t)std::ceil(kRtcTimerClockHz / (2.0 * (double)kLaserPwmMaxHalfPeriodCounts))) + " Hz)");

	const double dPulseLengthCounts = std::round(2.0 * (double)timing.m_nHalfPeriodCounts * dDuty);
	timing.m_nPulseLengthCounts = (int64_t)dPulseLengthCounts;
	if (timing.m_nPulseLengthCounts > 2 * timing.m_nHalfPeriodCounts)
		timing.m_nPulseLengthCounts = 2 * timing.m_nHalfPeriodCounts;

	return timing;
}

// Routes the pulse modulation onto the LASER_1 pin, silences the standby pulses (with an active pulse length of
// 0 the card would otherwise fall back to them during a run), and programs the requested pulse train. The laser
// signal itself is left to the caller: it gates the emission, so it is only asserted once the timings on the pin
// are the requested ones.
static void ApplyLaserModulationToCard(PDriver_ScanLab_RTC6 pDriver, const sLaserPwmTiming& PwmTiming)
{
	auto pContext = pDriver->GetContext();

	pContext->ConfigLaserSignals(2);
	pContext->SetStandbyInBits(0, 0);
	pContext->SetLaserPulsesInBits((uint32_t)PwmTiming.m_nHalfPeriodCounts, (uint32_t)PwmTiming.m_nPulseLengthCounts);
}


/*************************************************************************************************************************
 Ends a running direct LASER1 modulation: releases the laser signal first so that nothing keeps emitting, then puts
 the marking timings, the modulation routing and the standby pulses back the way they were. Shared by the
 lasermodulation state and the fatal-error recovery, it is a no-op while no modulation runs.
**************************************************************************************************************************/
static void EndLaserModulation(PPluginData pPluginData, LibMCEnv::PStateEnvironment pStateEnvironment)
{
	if (!pStateEnvironment->GetBoolParameter("lasermodulation", "active"))
		return;

	pStateEnvironment->SetBoolParameter("lasermodulation", "active", false);
	pStateEnvironment->SetIntegerParameter("lasermodulation", "remainingms", 0);

	std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
	if (sCardType == "scanlab") {
		auto pLaserData = std::dynamic_pointer_cast<CLaserData>(pPluginData);
		if (pLaserData != nullptr) {
			auto pDriver = pLaserData->acquireScanLabDriver(pStateEnvironment);
			if (!pDriver->IsSimulationMode()) {
				auto pContext = pDriver->GetContext();

				pContext->SetLaserSignal(false);

				// Restore the timings normal marking runs with, and the modulation routing this machine
				// is wired for (this control always modulates the LASER_1 pin).
				pContext->SetStandbyInMicroSeconds(pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbyhalfperiod"),
					pStateEnvironment->GetDoubleParameter("lasersignaltiming", "standbylength"));
				pContext->SetLaserPulsesInMicroSeconds(pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulsehalfperiod"),
					pStateEnvironment->GetDoubleParameter("lasersignaltiming", "pulselength"));
				pContext->ConfigLaserSignals((pStateEnvironment->GetBoolParameter("cardconfig", "modpinlaseron")) ? 1 : 2);

				pContext->SetLaserPinOut(false, false);
			}
		}
	}

	pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", false);
	pStateEnvironment->LogMessage("Direct LASER1 modulation stopped.");
}


/*************************************************************************************************************************
 Class definition of CLaserState_Idle
**************************************************************************************************************************/
class CLaserState_Idle : public virtual CLaserState {
public:
	CLaserState_Idle(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "idle";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		pStateEnvironment->SetNextState("idle");

		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType == "scanlab") {
			auto pDriver = m_pPluginData->acquireScanLabDriver(pStateEnvironment);
			pDriver->QueryParameters();

			// Periodically read laser feedback pins
			bool bLaserOK = true;
			bool bTemperatureOK = true;
			uint32_t nLaserDigitalInput = 0;

			if (pDriver->IsSimulationMode()) {
				// In simulation mode, default to true
				bLaserOK = true;
				bTemperatureOK = true;
			}
			else {
				// Read actual pin values from RTC Context
				// DI0 is LaserOK, DI1 is TemperatureOK
				auto pContext = pDriver->GetContext();
				bLaserOK = pContext->ReadDigitalInputBit(0);
				bTemperatureOK = pContext->ReadDigitalInputBit(1);
				nLaserDigitalInput = pContext->ReadDigitalInputs();
			}

			// Store the values in the laserfeedback parameter group
			pStateEnvironment->SetBoolParameter("laserfeedback", "laserok", bLaserOK);
			pStateEnvironment->SetBoolParameter("laserfeedback", "temperatureok", bTemperatureOK);
			pStateEnvironment->SetIntegerParameter("laserfeedback", "laserdigitalinput", nLaserDigitalInput);
		}

		auto pExposureSignal = pStateEnvironment->GetUnhandledSignal("signal_exposure");
		if (pExposureSignal != nullptr) {
			pStateEnvironment->StoreSignal("exposuresignal", pExposureSignal);
			pStateEnvironment->SetNextState("exposure");
		}

		auto pLaserOnOffSignal = pStateEnvironment->GetUnhandledSignal("signal_changelaseronoff");
		if (pLaserOnOffSignal != nullptr) {
			pStateEnvironment->SetNextState("laserpowerchange");
		}

		auto pStopPowerMeasurement = pStateEnvironment->ClaimSignalFromQueue("signal_stoppowermeasurement");
		if (pStopPowerMeasurement != nullptr) {
			// Nothing is emitting while idle: acknowledge so that the stop request does not linger in the queue.
			pStopPowerMeasurement->SetBoolResult("success", true);
			pStopPowerMeasurement->SignalHandled();
		}

		auto pPowerMeasurementSignal = pStateEnvironment->GetUnhandledSignal("signal_startpowermeasurement");
		if ((pPowerMeasurementSignal != nullptr) && (pExposureSignal == nullptr) && (pLaserOnOffSignal == nullptr)) {
			pStateEnvironment->StoreSignal("powermeasurementsignal", pPowerMeasurementSignal);
			pStateEnvironment->SetNextState("powermeasurement");
		}

		auto pStopLaserModulation = pStateEnvironment->ClaimSignalFromQueue("signal_stoplasermodulation");
		if (pStopLaserModulation != nullptr) {
			// Nothing is modulating while idle: acknowledge so that the stop request does not linger in the queue.
			pStopLaserModulation->SetBoolResult("success", true);
			pStopLaserModulation->SignalHandled();
		}

		auto pUpdateLaserModulation = pStateEnvironment->ClaimSignalFromQueue("signal_updatelasermodulation");
		if (pUpdateLaserModulation != nullptr) {
			// A live edit that arrives once the run has already ended has nothing to re-program: reject it.
			pUpdateLaserModulation->SetBoolResult("success", false);
			pUpdateLaserModulation->SignalHandled();
		}

		auto pLaserModulationSignal = pStateEnvironment->GetUnhandledSignal("signal_startlasermodulation");
		if ((pLaserModulationSignal != nullptr) && (pExposureSignal == nullptr) && (pLaserOnOffSignal == nullptr)) {
			pStateEnvironment->StoreSignal("lasermodulationsignal", pLaserModulationSignal);
			pStateEnvironment->SetNextState("lasermodulation");
		}
	}
};


/*************************************************************************************************************************
 Class definition of CLaserState_Exposure
**************************************************************************************************************************/
class CLaserState_Exposure : public virtual CLaserState {
public:
	CLaserState_Exposure(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "exposure";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		auto pSignalHandler = pStateEnvironment->RetrieveSignal("exposuresignal");

		pStateEnvironment->LogMessage("Exposure...");
		auto pBuildJob = pStateEnvironment->GetBuildJob(pSignalHandler->GetString("jobuuid"));
		auto nLayerIndex = (uint32_t)pSignalHandler->GetInteger("layerindex");
		auto nExposureTimeout = (uint32_t)pSignalHandler->GetInteger("timeout");

		pStateEnvironment->LogMessage("Exposing layer " + std::to_string(nLayerIndex) + " (timeout " + std::to_string(nExposureTimeout) + " ms)");

		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType == "scanlab") {
			auto pDriver = m_pPluginData->acquireScanLabDriver(pStateEnvironment);

			if (pDriver->IsSimulationMode()) {
				if (pStateEnvironment->GetBoolParameter("scannerrecording", "enabled"))
					pStateEnvironment->LogMessage("Scanner recording is not available in simulation mode");
				pStateEnvironment->LogMessage("Simulation mode delay of 2 seconds..");
				pStateEnvironment->Sleep(2000);
				pDriver->DrawLayer(pBuildJob->GetStorageUUID(), nLayerIndex);
			}
			else {
				auto recordingSettings = ReadScannerRecordingSettings(pStateEnvironment);

				// The laser on signal is only asserted while the layer is drawn: while it is active the
				// idle standby (tickle) pulses already make the laser emit.
				// DrawLayer / DrawLayerWithRecording block until the card has finished the list.
				auto pContext = pDriver->GetContext();
				auto nLeadTime = pStateEnvironment->GetIntegerParameter("lasersignaltiming", "enableleadtime");

				LibMCDriver_ScanLab::PRTCRecording pRecording;
				pContext->SetLaserSignal(true);
				pContext->SetLaserPinOut(true, false);
				pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", true);
				try {
					if (nLeadTime > 0)
						pStateEnvironment->Sleep((uint32_t)nLeadTime);

					if (recordingSettings.m_bEnabled)
						pRecording = DrawLayerWithRecording(pDriver, pBuildJob, nLayerIndex, recordingSettings);
					else
						pDriver->DrawLayer(pBuildJob->GetStorageUUID(), nLayerIndex);
				}
				catch (...) {
					pContext->SetLaserSignal(false);
					pContext->SetLaserPinOut(false, false);
					pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", false);
					throw;
				}
				pContext->SetLaserSignal(false);
				pContext->SetLaserPinOut(false, false);
				pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", false);

				if (pRecording.get() != nullptr) {
					try {
						StoreScannerRecording(pStateEnvironment, pRecording, pBuildJob, pSignalHandler->GetString("executionuuid"), nLayerIndex, recordingSettings);
					}
					catch (std::exception& e) {
						pStateEnvironment->LogWarning("Could not store scanner recording of layer " + std::to_string(nLayerIndex) + ": " + e.what());
					}
				}
			}
		}
		pSignalHandler->SetBoolResult("success", true);
		pSignalHandler->SignalHandled();

		pStateEnvironment->SetNextState("idle");
	}
};


/*************************************************************************************************************************
 Class definition of CLaserState_LaserPowerChange
**************************************************************************************************************************/
class CLaserState_LaserPowerChange : public virtual CLaserState {
public:
	CLaserState_LaserPowerChange(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "laserpowerchange";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		auto pLaserOnOffSignal = pStateEnvironment->GetUnhandledSignal("signal_changelaseronoff");
		if (pLaserOnOffSignal == nullptr)
			throw std::runtime_error("could not find laser on/off signal");

		bool bLaserOn = pLaserOnOffSignal->GetBool("laseron");
		pStateEnvironment->LogMessage("Changing laser on/off state to " + std::string((bLaserOn) ? "ON" : "OFF"));

		pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", bLaserOn);

		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType == "scanlab") {
			auto pRTCDriver = m_pPluginData->acquireScanLabDriver(pStateEnvironment);

			if (pRTCDriver->IsSimulationMode()) {
				pStateEnvironment->LogMessage("Simulation mode delay of 1 seconds..");
				pStateEnvironment->Sleep(1000);
			}
			else {
				auto pContext = pRTCDriver->GetContext();
				pContext->SetLaserSignal(bLaserOn);
				pContext->SetLaserPinOut(bLaserOn, false);
			}
		}

		pLaserOnOffSignal->SetBoolResult("success", true);
		pLaserOnOffSignal->SignalHandled();

		pStateEnvironment->SetNextState("idle");
	}
};


/*************************************************************************************************************************
 Class definition of CLaserState_PowerMeasurement

 Runs a timed manual laser emission for power measurements. On the first pass it applies the requested
 pulse modulation to the card and asserts the laser signal; with the modulation routed onto the laser
 line this machine is wired to (config_laser_signals, set up during laser initialisation), that line is
 gated high for the duration and carries the emission pulse train. Every pass then checks for an early
 stop request and updates the remaining-time parameter until the timer expires, then releases the signal
 and returns to idle.
**************************************************************************************************************************/
class CLaserState_PowerMeasurement : public virtual CLaserState {
public:
	CLaserState_PowerMeasurement(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "powermeasurement";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		pStateEnvironment->SetNextState("powermeasurement");

		if (!pStateEnvironment->GetBoolParameter("powermeasurement", "active")) {
			StartEmission(pStateEnvironment);
			return;
		}

		auto pStopSignal = pStateEnvironment->ClaimSignalFromQueue("signal_stoppowermeasurement");
		if (pStopSignal != nullptr) {
			pStopSignal->SetBoolResult("success", true);
			pStopSignal->SignalHandled();
			pStateEnvironment->LogMessage("Timed laser emission stopped early by request.");
			EndPowerMeasurementEmission(m_pPluginData, pStateEnvironment);
			pStateEnvironment->SetNextState("idle");
			return;
		}

		auto pLaserModulationSignal = pStateEnvironment->ClaimSignalFromQueue("signal_startlasermodulation");
		if (pLaserModulationSignal != nullptr) {
			// The timed emission holds the card's pulse timings and the laser signal: reject the request rather
			// than letting it start on its own as soon as this emission ends.
			pLaserModulationSignal->SetBoolResult("success", false);
			pLaserModulationSignal->SignalHandled();
			pStateEnvironment->LogWarning("Direct LASER1 modulation refused: a timed laser emission is running.");
		}

		int64_t nRemaining = pStateEnvironment->GetIntegerParameter("powermeasurement", "endtime") -
			(int64_t)pStateEnvironment->GetGlobalTimerInMilliseconds();

		if (nRemaining <= 0) {
			EndPowerMeasurementEmission(m_pPluginData, pStateEnvironment);
			pStateEnvironment->SetNextState("idle");
			return;
		}

		if (nRemaining > 1000)
			pStateEnvironment->SetIntegerParameter("powermeasurement", "remainingms", nRemaining);

		pStateEnvironment->Sleep((uint32_t)((nRemaining < 50) ? nRemaining : 50));
	}

private:

	void StartEmission(LibMCEnv::PStateEnvironment pStateEnvironment) {
		auto pSignalHandler = pStateEnvironment->RetrieveSignal("powermeasurementsignal");
		if (pSignalHandler == nullptr)
			throw std::runtime_error("could not find timed laser emission signal");

		// Never emit manually while an automated build execution is driving the laser.
		std::string sMainState = pStateEnvironment->GetMachineState("main");
		if ((sMainState == "initexecution") || (sMainState == "singleexposure") || (sMainState == "execution_waiting") ||
			(sMainState == "execution_exposing") || (sMainState == "finishexecution") || (sMainState == "cancelexecution")) {
			pStateEnvironment->LogWarning("Timed laser emission refused: a build execution is running.");
			pSignalHandler->SetBoolResult("success", false);
			pSignalHandler->SignalHandled();
			pStateEnvironment->SetNextState("idle");
			return;
		}

		// One manual emission control at a time: the timed emission and the direct modulation would otherwise
		// both program the card's pulse timings and fight over the laser signal.
		if (pStateEnvironment->GetBoolParameter("lasermodulation", "active")) {
			pStateEnvironment->LogWarning("Timed laser emission refused: the direct LASER1 modulation is running.");
			pSignalHandler->SetBoolResult("success", false);
			pSignalHandler->SignalHandled();
			pStateEnvironment->SetNextState("idle");
			return;
		}

		int64_t nDurationMS = pSignalHandler->GetInteger("durationms");
		double dPowerWatts = pSignalHandler->GetDouble("powerw");
		int64_t nFrequencyHz = pSignalHandler->GetInteger("frequencyhz");
		int64_t nDutyCentiPercent = pSignalHandler->GetInteger("dutycyclecenti");

		if (nDurationMS < 10) nDurationMS = 10;
		if (nDurationMS > 600000) nDurationMS = 600000;
		if (!(dPowerWatts >= 0.0)) dPowerWatts = 0.0;
		if (nFrequencyHz < 1) nFrequencyHz = 1;
		if (nFrequencyHz > 500000) nFrequencyHz = 500000;
		if (nDutyCentiPercent < 0) nDutyCentiPercent = 0;
		if (nDutyCentiPercent > 10000) nDutyCentiPercent = 10000;

		pStateEnvironment->SetIntegerParameter("powermeasurement", "durationms", nDurationMS);
		pStateEnvironment->SetDoubleParameter("powermeasurement", "powerw", dPowerWatts);
		pStateEnvironment->SetIntegerParameter("powermeasurement", "frequencyhz", nFrequencyHz);
		pStateEnvironment->SetIntegerParameter("powermeasurement", "dutycyclecenti", nDutyCentiPercent);

		double dPeriodInMicroSeconds = 1000000.0 / (double)nFrequencyHz;

		// The CO2 laser's power is pulse-width modulated: the power setting maps to a pulse width fraction
		// through the calibration table, and the PWM duty cycle multiplies it. The result is written to the
		// laser pulse timing, which the card emits on the laser pin the modulation is routed to.
		double dPowerFraction = 1.0;
		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType != "scanlab")
			throw std::runtime_error("invalid scanner card type: " + sCardType);

		auto pDriver = m_pPluginData->acquireScanLabDriver(pStateEnvironment);
		if (!pDriver->IsSimulationMode()) {
			dPowerFraction = pDriver->GetContext()->MapPowerWattsToPercent(dPowerWatts) / 100.0;
		}
		else {
			double dMaxPower = pStateEnvironment->GetDoubleParameter("powercalibration", "calibrationsetpoint100");
			dPowerFraction = (dMaxPower > 0.0) ? (dPowerWatts / dMaxPower) : 1.0;
		}

		if (dPowerFraction < 0.0) dPowerFraction = 0.0;
		if (dPowerFraction > 1.0) dPowerFraction = 1.0;

		double dPulseWidthInMicroSeconds = dPeriodInMicroSeconds * ((double)nDutyCentiPercent / 10000.0) * dPowerFraction;

		pStateEnvironment->LogMessage("Timed laser emission: " + std::to_string(nDurationMS) + " ms, " +
			std::to_string(dPowerWatts) + " W, " + std::to_string(nFrequencyHz) + " Hz, " +
			std::to_string((double)nDutyCentiPercent / 100.0) + " % duty (period " + std::to_string(dPeriodInMicroSeconds) +
			" us, pulse width " + std::to_string(dPulseWidthInMicroSeconds) + " us)");

		if (!pDriver->IsSimulationMode()) {
			auto pContext = pDriver->GetContext();
			pContext->SetLaserPulsesInMicroSeconds(dPeriodInMicroSeconds * 0.5, dPulseWidthInMicroSeconds);

			// Arming the emission: while the laser signal is asserted, the card emits the pulse modulation
			// on the routed laser pin.
			pContext->SetLaserSignal(true);
			pContext->SetLaserPinOut(true, false);
		}
		else {
			pStateEnvironment->LogMessage("Laser in simulation mode - no hardware emission");
		}

		pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", true);
		pStateEnvironment->SetIntegerParameter("powermeasurement", "endtime",
			(int64_t)pStateEnvironment->GetGlobalTimerInMilliseconds() + nDurationMS);
		pStateEnvironment->SetIntegerParameter("powermeasurement", "remainingms", nDurationMS);
		pStateEnvironment->SetBoolParameter("powermeasurement", "active", true);

		pSignalHandler->SetBoolResult("success", true);
		pSignalHandler->SignalHandled();
	}
};


/*************************************************************************************************************************
 Class definition of CLaserState_LaserModulation

 Runs the direct LASER1 pulse modulation of the "Power Measurement" page. On the first pass it derives the pulse
 timings from the requested frequency and duty cycle, programs them, and asserts the laser signal so that the
 LASER_1 pin carries the train. While the run is active it re-programs the timings whenever a frequency or duty
 cycle field changes, answers stop requests, hands control back as soon as an exposure is requested, and ends by
 itself when the configured time limit expires.
**************************************************************************************************************************/
static bool BuildExecutionDrivesLaser(LibMCEnv::PStateEnvironment pStateEnvironment)
{
	std::string sMainState = pStateEnvironment->GetMachineState("main");
	return (sMainState == "initexecution") || (sMainState == "singleexposure") || (sMainState == "execution_waiting") ||
		(sMainState == "execution_exposing") || (sMainState == "finishexecution") || (sMainState == "cancelexecution");
}

class CLaserState_LaserModulation : public virtual CLaserState {
public:
	CLaserState_LaserModulation(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "lasermodulation";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		pStateEnvironment->SetNextState("lasermodulation");

		if (!pStateEnvironment->GetBoolParameter("lasermodulation", "active")) {
			StartModulation(pStateEnvironment);
			return;
		}

		// A running exposure always wins over the manual control: stop emitting and return to idle, where the
		// exposure signal is picked up. The request itself is left in the queue for the idle state to claim.
		if (pStateEnvironment->GetUnhandledSignal("signal_exposure") != nullptr) {
			pStateEnvironment->LogWarning("Direct LASER1 modulation stopped: an exposure was requested.");
			EndLaserModulation(m_pPluginData, pStateEnvironment);
			pStateEnvironment->SetNextState("idle");
			return;
		}

		auto pUpdateSignal = pStateEnvironment->ClaimSignalFromQueue("signal_updatelasermodulation");
		if (pUpdateSignal != nullptr) {
			bool bAccepted = true;
			try {
				ApplyRequestedSettings(pStateEnvironment, pUpdateSignal->GetInteger("frequencyhz"), pUpdateSignal->GetInteger("dutycyclecenti"));
			}
			catch (std::exception& e) {
				bAccepted = false;
				pStateEnvironment->LogWarning("Could not update the LASER1 modulation: " + std::string(e.what()));
			}
			pUpdateSignal->SetBoolResult("success", bAccepted);
			pUpdateSignal->SignalHandled();
			return;
		}

		auto pStopSignal = pStateEnvironment->ClaimSignalFromQueue("signal_stoplasermodulation");
		if (pStopSignal != nullptr) {
			pStopSignal->SetBoolResult("success", true);
			pStopSignal->SignalHandled();
			pStateEnvironment->LogMessage("Direct LASER1 modulation stopped early by request.");
			EndLaserModulation(m_pPluginData, pStateEnvironment);
			pStateEnvironment->SetNextState("idle");
			return;
		}

		auto pPowerMeasurementSignal = pStateEnvironment->ClaimSignalFromQueue("signal_startpowermeasurement");
		if (pPowerMeasurementSignal != nullptr) {
			// The modulation holds the card's pulse timings and the laser signal: reject the request rather than
			// letting the timed emission start on its own as soon as this run ends.
			pPowerMeasurementSignal->SetBoolResult("success", false);
			pPowerMeasurementSignal->SignalHandled();
			pStateEnvironment->LogWarning("Timed laser emission refused: the direct LASER1 modulation is running.");
		}

		int64_t nRemaining = pStateEnvironment->GetIntegerParameter("lasermodulation", "endtime") -
			(int64_t)pStateEnvironment->GetGlobalTimerInMilliseconds();

		if (nRemaining <= 0) {
			pStateEnvironment->LogMessage("Direct LASER1 modulation reached its time limit.");
			EndLaserModulation(m_pPluginData, pStateEnvironment);
			pStateEnvironment->SetNextState("idle");
			return;
		}

		if (nRemaining > 1000)
			pStateEnvironment->SetIntegerParameter("lasermodulation", "remainingms", nRemaining);

		pStateEnvironment->Sleep((uint32_t)((nRemaining < 50) ? nRemaining : 50));
	}

private:

	void StartModulation(LibMCEnv::PStateEnvironment pStateEnvironment) {
		auto pSignalHandler = pStateEnvironment->RetrieveSignal("lasermodulationsignal");
		if (pSignalHandler == nullptr)
			throw std::runtime_error("could not find direct laser modulation signal");

		// Never modulate manually while an automated build execution drives the laser, and never alongside the
		// timed emission: both program the card's pulse timings and gate them with the same laser signal.
		if (BuildExecutionDrivesLaser(pStateEnvironment)) {
			pStateEnvironment->LogWarning("Direct LASER1 modulation refused: a build execution is running.");
			pSignalHandler->SetBoolResult("success", false);
			pSignalHandler->SignalHandled();
			pStateEnvironment->SetNextState("idle");
			return;
		}

		if (pStateEnvironment->GetBoolParameter("powermeasurement", "active")) {
			pStateEnvironment->LogWarning("Direct LASER1 modulation refused: a timed laser emission is running.");
			pSignalHandler->SetBoolResult("success", false);
			pSignalHandler->SignalHandled();
			pStateEnvironment->SetNextState("idle");
			return;
		}

		int64_t nFrequencyHz = pSignalHandler->GetInteger("frequencyhz");
		int64_t nDutyCentiPercent = pSignalHandler->GetInteger("dutycyclecenti");
		int64_t nMaxRunningMS = pSignalHandler->GetInteger("maxrunningms");
		if (nMaxRunningMS < kLaserModulationMinRunningMS) nMaxRunningMS = kLaserModulationMinRunningMS;
		if (nMaxRunningMS > kLaserModulationMaxRunningMS) nMaxRunningMS = kLaserModulationMaxRunningMS;

		sLaserPwmTiming PwmTiming;
		try {
			PwmTiming = CalculateLaserPwmTiming(nFrequencyHz, nDutyCentiPercent);
		}
		catch (std::exception& e) {
			pStateEnvironment->LogWarning("Direct LASER1 modulation refused: " + std::string(e.what()));
			pSignalHandler->SetBoolResult("success", false);
			pSignalHandler->SignalHandled();
			pStateEnvironment->SetNextState("idle");
			return;
		}

		pSignalHandler->SetBoolResult("success", true);
		pSignalHandler->SignalHandled();

		pStateEnvironment->SetIntegerParameter("lasermodulation", "maxrunningms", nMaxRunningMS);
		ProgramModulation(pStateEnvironment, nFrequencyHz, nDutyCentiPercent, PwmTiming);
	}

	// Re-programs a running modulation from the settings of a live form edit.
	void ApplyRequestedSettings(LibMCEnv::PStateEnvironment pStateEnvironment, int64_t nFrequencyHz, int64_t nDutyCentiPercent) {
		sLaserPwmTiming PwmTiming = CalculateLaserPwmTiming(nFrequencyHz, nDutyCentiPercent);
		ProgramModulation(pStateEnvironment, nFrequencyHz, nDutyCentiPercent, PwmTiming);
	}

	// Writes the derived counts to the card and mirrors them, together with the new deadline, in the parameters
	// the "Power Measurement" page displays. A start asserts the laser signal; a live update leaves it alone.
	void ProgramModulation(LibMCEnv::PStateEnvironment pStateEnvironment, int64_t nFrequencyHz, int64_t nDutyCentiPercent, const sLaserPwmTiming& PwmTiming) {
		const bool bWasActive = pStateEnvironment->GetBoolParameter("lasermodulation", "active");

		std::string sCardType = pStateEnvironment->GetStringParameter("cardconfig", "cardtype");
		if (sCardType != "scanlab")
			throw std::runtime_error("invalid scanner card type: " + sCardType);

		pStateEnvironment->LogMessage("LASER1 modulation: " + std::to_string(nFrequencyHz) + " Hz, " +
			std::to_string((double)nDutyCentiPercent / 100.0) + " % duty -> set_laser_pulses(" +
			std::to_string(PwmTiming.m_nHalfPeriodCounts) + ", " + std::to_string(PwmTiming.m_nPulseLengthCounts) + ")");

		// Mirror the request first: if programming the card then fails, the modulation counts as running, and
		// the error handling releases it (and with it the laser signal) instead of leaving it invisible.
		pStateEnvironment->SetIntegerParameter("lasermodulation", "frequencyhz", nFrequencyHz);
		pStateEnvironment->SetIntegerParameter("lasermodulation", "dutycyclecenti", nDutyCentiPercent);
		pStateEnvironment->SetIntegerParameter("lasermodulation", "halfperiodcounts", PwmTiming.m_nHalfPeriodCounts);
		pStateEnvironment->SetIntegerParameter("lasermodulation", "pulselengthcounts", PwmTiming.m_nPulseLengthCounts);

		if (!bWasActive) {
			int64_t nMaxRunningMS = pStateEnvironment->GetIntegerParameter("lasermodulation", "maxrunningms");
			pStateEnvironment->SetIntegerParameter("lasermodulation", "endtime",
				(int64_t)pStateEnvironment->GetGlobalTimerInMilliseconds() + nMaxRunningMS);
			pStateEnvironment->SetIntegerParameter("lasermodulation", "remainingms", nMaxRunningMS);
			pStateEnvironment->SetBoolParameter("lasermodulation", "active", true);
			pStateEnvironment->SetBoolParameter("lasersignaltiming", "laseronsignal", true);
		}

		auto pDriver = m_pPluginData->acquireScanLabDriver(pStateEnvironment);
		if (!pDriver->IsSimulationMode()) {
			ApplyLaserModulationToCard(pDriver, PwmTiming);

			if (!bWasActive) {
				// Arming the emission: as in the exposure and timed emission paths, the laser signal gates the
				// modulation, so the routed pin only carries the pulse train from here on.
				auto pContext = pDriver->GetContext();
				pContext->SetLaserSignal(true);
				pContext->SetLaserPinOut(true, false);
			}
		}
		else {
			pStateEnvironment->LogMessage("Laser in simulation mode - no hardware modulation");
		}
	}
};


/*************************************************************************************************************************
 Class definition of CLaserState_FatalError
**************************************************************************************************************************/
class CLaserState_FatalError : public virtual CLaserState {
public:
	CLaserState_FatalError(const std::string& sStateName, PPluginData pPluginData)
		: CLaserState(getStateName(), sStateName, pPluginData) {
	}

	static const std::string getStateName() {
		return "fatalerror";
	}

	void Execute(LibMCEnv::PStateEnvironment pStateEnvironment) override {
		if (pStateEnvironment.get() == nullptr)
			throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDPARAM);

		// Safety: if the error hit while a timed emission was running, drop the laser enable pin
		// before entering the recovery wait.
		if (pStateEnvironment->GetBoolParameter("powermeasurement", "active")) {
			try {
				EndPowerMeasurementEmission(m_pPluginData, pStateEnvironment);
			}
			catch (std::exception& e) {
				pStateEnvironment->LogWarning("Could not stop timed laser emission during error handling: " + std::string(e.what()));
				pStateEnvironment->SetBoolParameter("powermeasurement", "active", false);
				pStateEnvironment->SetIntegerParameter("powermeasurement", "remainingms", 0);
			}
		}

		// Same for a running direct LASER1 modulation: nothing may keep emitting while the subsystem is broken.
		if (pStateEnvironment->GetBoolParameter("lasermodulation", "active")) {
			try {
				EndLaserModulation(m_pPluginData, pStateEnvironment);
			}
			catch (std::exception& e) {
				pStateEnvironment->LogWarning("Could not stop the direct LASER1 modulation during error handling: " + std::string(e.what()));
				pStateEnvironment->SetBoolParameter("lasermodulation", "active", false);
				pStateEnvironment->SetIntegerParameter("lasermodulation", "remainingms", 0);
			}
		}

		// Stay in the fatal-error state until an external recovery request is received, then
		// go back to "waitforinit" so the machine re-runs its full initialisation sequence.
		auto pSignal = pStateEnvironment->ClaimSignalFromQueue("signal_initialize_again");
		if (pSignal) {
			pSignal->SetBoolResult("success", true);
			pSignal->SignalHandled();

			pStateEnvironment->LogMessage("Laser recovery requested, re-initializing...");
			pStateEnvironment->SetNextState("waitforinit");
		}
		else {
			pStateEnvironment->SetNextState("fatalerror");
		}
	}
};


/*************************************************************************************************************************
 Class definition of CStateFactory
**************************************************************************************************************************/

CStateFactory::CStateFactory(const std::string& sInstanceName) {
	m_pPluginData = std::make_shared<CLaserData>();
}

IState* CStateFactory::CreateState(const std::string& sStateName) {
	IState* pStateInstance = nullptr;

	if (createStateInstanceByName<CLaserState_WaitForInit>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_Init>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_Idle>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_Exposure>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_LaserPowerChange>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_PowerMeasurement>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_LaserModulation>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;
	if (createStateInstanceByName<CLaserState_FatalError>(sStateName, pStateInstance, m_pPluginData))
		return pStateInstance;

	throw ELibMCPluginInterfaceException(LIBMCPLUGIN_ERROR_INVALIDSTATENAME, "invalid state name: " + sStateName);
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif
