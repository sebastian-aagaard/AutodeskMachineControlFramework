/*++

Copyright (C) 2026 Autodesk Inc.

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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AUTODESK INC. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


Abstract: This is the class implementation of CSensorthermPyrometer

*/

#include "libmcdriver_sensortherm_pyrometer.hpp"
#include "libmcdriver_sensortherm_interfaceexception.hpp"

// Include custom headers here.
#include "libmcdriver_sensortherm_driver_sensortherm.hpp"
#include <cstdio>
#include <vector>

using namespace LibMCDriver_Sensortherm::Impl;


/*************************************************************************************************************************
 Local constants
**************************************************************************************************************************/

// The connect call of the operating system has no timeout of its own, so a pyrometer that
// is down can block for the TCP connect timeout of the host. Connecting is retried at most
// every five seconds, so that a state machine does not spend its whole cycle budget on a
// pyrometer that is currently not reachable.
const int64_t SENSORTHERM_CONNECTRETRYMICROSECONDS = 5000000;

// Maximum number of answer bytes to read for one command.
const size_t SENSORTHERM_MAXANSWERLENGTH = 1024;

// Maximum number of stream packets to parse in one update, to keep the call bounded.
const uint32_t SENSORTHERM_MAXPACKETSUPDATE = 512;

// The continuous stream mode of a pyrometer sends one packet every 25 microseconds, which
// are about 40000 packets or 800 kilobytes per second. That is far more than one packet at
// a time, so the stream socket is read in blocks of a manageable size.
const uint32_t SENSORTHERM_STREAMCHUNKSIZE = 16384;
const uint32_t SENSORTHERM_MAXCHUNKSUPDATE = 16;


/*************************************************************************************************************************
 Class definition of CSensorthermPyrometer
**************************************************************************************************************************/

CSensorthermPyrometer::CSensorthermPyrometer(const std::string & sIdentifier, const std::string & sIPAddress, const LibMCDriver_Sensortherm_uint32 nCommandPort, const LibMCDriver_Sensortherm_uint32 nStreamPort, const LibMCDriver_Sensortherm_uint32 nTimeOutInMs, const std::string & sDeviceAddress, const ESensorthermByteOrder eByteOrder, const double dTemperatureFactor, const LibMCDriver_Sensortherm::eSensorthermStreamMode eStreamMode, const LibMCEnv::PDriverEnvironment pDriverEnvironment)
	: m_pDriverEnvironment(pDriverEnvironment), m_sIdentifier(sIdentifier), m_sIPAddress(sIPAddress), m_sDeviceAddress(sDeviceAddress),
	  m_nCommandPort(nCommandPort), m_nStreamPort(nStreamPort), m_nTimeOutInMs(nTimeOutInMs),
	  m_eByteOrder(eByteOrder), m_dTemperatureFactor(dTemperatureFactor), m_eStreamMode(eStreamMode),
	  m_bIsStreaming(false), m_nSampleCount(0), m_nLastConnectAttemptMicroSeconds(0), m_nDiscardedSamples(0)
{
	if (pDriverEnvironment.get() == nullptr)
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM);

	if (m_sIPAddress.empty())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM, "no IP address given for pyrometer connection '" + m_sIdentifier + "'");

	if ((m_sDeviceAddress.length() != 2) || (m_sDeviceAddress[0] < '0') || (m_sDeviceAddress[0] > '9') ||
		(m_sDeviceAddress[1] < '0') || (m_sDeviceAddress[1] > '9'))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM, "the device address of a Sensortherm pyrometer must be a two digit decimal number, got '" + m_sDeviceAddress + "'");

	if (m_nTimeOutInMs == 0)
		m_nTimeOutInMs = 1000;
}


CSensorthermPyrometer::~CSensorthermPyrometer()
{
	try {
		Disconnect();
	} catch (...) {
	}
}

void CSensorthermPyrometer::Disconnect()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	m_pStreamConnection = nullptr;
	m_pCommandConnection = nullptr;
	m_bIsStreaming = false;
	m_vStreamCarry.clear();
}


std::string CSensorthermPyrometer::GetIdentifier()
{
	return m_sIdentifier;
}


std::string CSensorthermPyrometer::GetIPAddress()
{
	return m_sIPAddress;
}


LibMCDriver_Sensortherm_uint32 CSensorthermPyrometer::GetCommandPort()
{
	return m_nCommandPort;
}


LibMCDriver_Sensortherm_uint32 CSensorthermPyrometer::GetStreamPort()
{
	return m_nStreamPort;
}


bool CSensorthermPyrometer::IsConnected()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return (m_pCommandConnection.get() != nullptr) && m_pCommandConnection->IsConnected();
}


int64_t CSensorthermPyrometer::MicroSecondsNow()
{
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}


bool CSensorthermPyrometer::EnsureCommandConnection()
{
	if ((m_pCommandConnection.get() != nullptr) && m_pCommandConnection->IsConnected())
		return true;

	const int64_t nNow = MicroSecondsNow();
	if ((m_pCommandConnection.get() != nullptr) && ((nNow - m_nLastConnectAttemptMicroSeconds) < SENSORTHERM_CONNECTRETRYMICROSECONDS))
		return false;

	m_nLastConnectAttemptMicroSeconds = nNow;

	try {
		m_pCommandConnection = m_pDriverEnvironment->CreateTCPIPConnection(m_sIPAddress, m_nCommandPort, m_nTimeOutInMs);
	} catch (std::exception &) {
		m_pCommandConnection = nullptr;
		return false;
	}

	return (m_pCommandConnection.get() != nullptr) && m_pCommandConnection->IsConnected();
}


bool CSensorthermPyrometer::EnsureStreamConnection()
{
	if ((m_pStreamConnection.get() != nullptr) && m_pStreamConnection->IsConnected())
		return true;

	try {
		m_pStreamConnection = m_pDriverEnvironment->CreateTCPIPConnection(m_sIPAddress, m_nStreamPort, m_nTimeOutInMs);
	} catch (std::exception &) {
		m_pStreamConnection = nullptr;
		return false;
	}

	return (m_pStreamConnection.get() != nullptr) && m_pStreamConnection->IsConnected();
}


void CSensorthermPyrometer::StoreSample(const sSensorthermSample & sSample)
{
	m_Sample = sSample;
	m_Sample.nHostMicroSeconds = MicroSecondsNow();
	m_LastSampleTime = std::chrono::steady_clock::now();
	m_nSampleCount++;
}

std::string CSensorthermPyrometer::ExecuteCommand(const std::string & sCommand)
{
	if (m_pCommandConnection.get() == nullptr)
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_DRIVERNOTCONNECTED, "the pyrometer '" + m_sIdentifier + "' is not connected");

	const std::string sRequest = m_sDeviceAddress + sCommand + "\r";

	std::vector<uint8_t> vRequest(sRequest.begin(), sRequest.end());
	m_pCommandConnection->SendBuffer(vRequest);

	const std::string sAnswer = ReadAnswer();

	if (sAnswer == "no")
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAMETERVALUE, "the pyrometer '" + m_sIdentifier + "' rejected the command '" + sCommand + "'");

	return sAnswer;
}


std::string CSensorthermPyrometer::ReadAnswer()
{
	std::string sAnswer;

	const int64_t nDeadLine = MicroSecondsNow() + ((int64_t)m_nTimeOutInMs * 1000);

	while (true) {

		const int64_t nRemaining = nDeadLine - MicroSecondsNow();
		if (nRemaining <= 0)
			throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_PROTOCOLTIMEOUT, "the pyrometer '" + m_sIdentifier + "' did not answer within " + std::to_string(m_nTimeOutInMs) + " ms");

		const uint32_t nTimeOutInMs = (uint32_t)((nRemaining + 999) / 1000);

		if (!m_pCommandConnection->WaitForData(nTimeOutInMs))
			throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_PROTOCOLTIMEOUT, "the pyrometer '" + m_sIdentifier + "' did not answer within " + std::to_string(m_nTimeOutInMs) + " ms");

		auto pPacket = m_pCommandConnection->ReceiveData(64, nTimeOutInMs);
		std::vector<uint8_t> vData;
		pPacket->GetData(vData);

		for (size_t nIndex = 0; nIndex < vData.size(); nIndex++) {
			if (vData[nIndex] == '\r')
				return sAnswer;
			sAnswer.push_back((char)vData[nIndex]);
			if (sAnswer.length() > SENSORTHERM_MAXANSWERLENGTH)
				throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_PROTOCOLRESPONSE, "the pyrometer '" + m_sIdentifier + "' sent an answer that is longer than " + std::to_string(SENSORTHERM_MAXANSWERLENGTH) + " bytes");
		}
	}
}


std::string CSensorthermPyrometer::SendCommand(const std::string & sCommand)
{
	std::lock_guard<std::mutex> lock(m_DataMutex);

	if (!EnsureCommandConnection())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_DRIVERNOTCONNECTED, "the pyrometer '" + m_sIdentifier + "' at " + m_sIPAddress + " could not be reached on port " + std::to_string(m_nCommandPort));

	return ExecuteCommand(sCommand);
}


bool CSensorthermPyrometer::PollSample()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);

	if (!EnsureCommandConnection())
		return false;

	std::string sAnswer;
	try {
		sAnswer = ExecuteCommand("bup");
	} catch (ELibMCDriver_SensorthermInterfaceException & E) {
		if (E.getErrorCode() != LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAMETERVALUE)
			throw;
		return false;
	}

	sSensorthermSample sSample;
	if (!parsePollPayload(sAnswer, m_dTemperatureFactor, sSample))
		return false;

	StoreSample(sSample);
	return true;
}


bool CSensorthermPyrometer::UpdateStream()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);

	if (!m_bIsStreaming)
		return false;

	if (!EnsureStreamConnection())
		return false;

	bool bReceived = false;

	try {

		std::vector<uint8_t> vBuffer = std::move(m_vStreamCarry);
		m_vStreamCarry.clear();

		for (uint32_t nIndex = 0; nIndex < SENSORTHERM_MAXCHUNKSUPDATE; nIndex++) {

			if (!m_pStreamConnection->WaitForData(0))
				break;

			auto pChunk = m_pStreamConnection->ReceiveData(SENSORTHERM_STREAMCHUNKSIZE, m_nTimeOutInMs);

			std::vector<uint8_t> vChunk;
			pChunk->GetData(vChunk);

			if (vChunk.empty())
				break;

			vBuffer.insert(vBuffer.end(), vChunk.begin(), vChunk.end());
		}

		// Only whole packets carry information, the stream has no framing of its own. The
		// oldest packets are the ones that are already stale, so the parse window is the
		// newest block of packets and everything before it is counted as discarded.
		const size_t nCompleteBytes = (vBuffer.size() / SENSORTHERM_STREAMPACKETSIZE) * SENSORTHERM_STREAMPACKETSIZE;
		const size_t nParseBytes = (size_t)SENSORTHERM_MAXPACKETSUPDATE * SENSORTHERM_STREAMPACKETSIZE;

		size_t nOffset = 0;
		if (nCompleteBytes > nParseBytes) {
			m_nDiscardedSamples += (nCompleteBytes - nParseBytes) / SENSORTHERM_STREAMPACKETSIZE;
			nOffset = nCompleteBytes - nParseBytes;
		}

		for (; nOffset + SENSORTHERM_STREAMPACKETSIZE <= nCompleteBytes; nOffset += SENSORTHERM_STREAMPACKETSIZE) {

			sSensorthermSample sSample;
			if (!parseStreamPacketAt(vBuffer, nOffset, m_eByteOrder, m_dTemperatureFactor, sSample))
				continue;

			StoreSample(sSample);
			bReceived = true;
		}

		// keep the bytes of a packet that was cut in half by the end of the read
		m_vStreamCarry.assign(vBuffer.begin() + nCompleteBytes, vBuffer.end());

	} catch (std::exception &) {
		m_pStreamConnection = nullptr;
		m_vStreamCarry.clear();
	}

	return bReceived;
}


void CSensorthermPyrometer::StartStreaming()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);

	if (!EnsureCommandConnection())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_DRIVERNOTCONNECTED, "the pyrometer '" + m_sIdentifier + "' at " + m_sIPAddress + " could not be reached on port " + std::to_string(m_nCommandPort));

	ExecuteCommand((m_eStreamMode == LibMCDriver_Sensortherm::eSensorthermStreamMode::ControllerSampled) ? "ipd01" : "ipd00");
	ExecuteCommand("ipx1");

	if (!EnsureStreamConnection())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_DRIVERNOTCONNECTED, "the pyrometer '" + m_sIdentifier + "' accepted the stream command, but its data stream port " + std::to_string(m_nStreamPort) + " could not be reached");

	m_vStreamCarry.clear();
	m_nDiscardedSamples = 0;
	m_bIsStreaming = true;
}


void CSensorthermPyrometer::StopStreaming()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);

	if ((m_pCommandConnection.get() != nullptr) && m_pCommandConnection->IsConnected()) {
		try {
			ExecuteCommand("ipx0");
		} catch (std::exception &) {
		}
	}

	m_bIsStreaming = false;
	m_vStreamCarry.clear();
}


bool CSensorthermPyrometer::IsStreaming()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_bIsStreaming;
}


LibMCDriver_Sensortherm::eSensorthermStreamMode CSensorthermPyrometer::GetStreamMode()
{
	return m_eStreamMode;
}


void CSensorthermPyrometer::SetStreamMode(const LibMCDriver_Sensortherm::eSensorthermStreamMode eStreamMode)
{
	m_eStreamMode = eStreamMode;
}

LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetTwoColorTemperature()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	if (!m_Sample.bIsValid)
		return std::numeric_limits<double>::quiet_NaN();
	return m_Sample.dTwoColorTemperature;
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetChannel1Temperature()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	if (!m_Sample.bIsValid)
		return std::numeric_limits<double>::quiet_NaN();
	return m_Sample.dChannel1Temperature;
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetChannel2Temperature()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	if (!m_Sample.bIsValid)
		return std::numeric_limits<double>::quiet_NaN();
	return m_Sample.dChannel2Temperature;
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetSignalStrength()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.dSignalStrength;
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetControlOutput()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.dControlOutput;
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetSetpoint()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.dSetpoint;
}


LibMCDriver_Sensortherm_int64 CSensorthermPyrometer::GetSampleCount()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return (LibMCDriver_Sensortherm_int64)m_nSampleCount;
}


LibMCDriver_Sensortherm_int64 CSensorthermPyrometer::GetDeviceTimeStamp()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return (LibMCDriver_Sensortherm_int64)((double)m_Sample.nDeviceTicks / SENSORTHERM_TICKS_PER_MILLISECOND);
}


LibMCDriver_Sensortherm_int64 CSensorthermPyrometer::GetAgeInMilliseconds()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	if (m_nSampleCount == 0)
		return -1;
	return (MicroSecondsNow() - m_Sample.nHostMicroSeconds) / 1000;
}


bool CSensorthermPyrometer::GetDigitalInput(const LibMCDriver_Sensortherm_uint32 nIndex)
{
	if ((nIndex < 1) || (nIndex > 4))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDINDEX, "a Sensortherm pyrometer has four digital inputs, index " + std::to_string(nIndex) + " is not valid");

	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.IsInputActive(nIndex);
}


bool CSensorthermPyrometer::GetDigitalOutput(const LibMCDriver_Sensortherm_uint32 nIndex)
{
	if ((nIndex < 1) || (nIndex > 4))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDINDEX, "a Sensortherm pyrometer has four digital outputs, index " + std::to_string(nIndex) + " is not valid");

	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.IsOutputActive(nIndex);
}


bool CSensorthermPyrometer::IsControlActive()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.IsControlActive();
}


bool CSensorthermPyrometer::IsFahrenheit()
{
	std::lock_guard<std::mutex> lock(m_DataMutex);
	return m_Sample.IsFahrenheit();
}


LibMCDriver_Sensortherm_uint32 CSensorthermPyrometer::GetFaultStatus()
{
	const std::string sAnswer = SendCommand("fs");

	uint32_t nValue = 0;
	if (!parseHexNumber(sAnswer, nValue))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_PROTOCOLRESPONSE, "the pyrometer '" + m_sIdentifier + "' sent an invalid fault status '" + sAnswer + "'");

	return nValue;
}


std::string CSensorthermPyrometer::GetSerialNumber()
{
	return SendCommand("sn");
}


std::string CSensorthermPyrometer::GetOrderNumber()
{
	return SendCommand("bn");
}


std::string CSensorthermPyrometer::GetDeviceType()
{
	return SendCommand("ty");
}


LibMCDriver_Sensortherm_double CSensorthermPyrometer::GetEmissivity()
{
	const std::string sAnswer = SendCommand("eg");

	uint32_t nValue = 0;
	if (!parseHexNumber(sAnswer, nValue))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_PROTOCOLRESPONSE, "the pyrometer '" + m_sIdentifier + "' sent an invalid emissivity '" + sAnswer + "'");

	return (double)nValue * SENSORTHERM_PERCENT_FACTOR;
}


void CSensorthermPyrometer::SetEmissivity(const LibMCDriver_Sensortherm_double dEmissivity)
{

	if ((dEmissivity < 5.0) || (dEmissivity > 120.0))
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM, "the emissivity must be between 5.0 and 120.0 percent, got " + std::to_string(dEmissivity));

	const uint32_t nValue = (uint32_t)(dEmissivity * 10.0);

	char pBuffer[16];
	snprintf(pBuffer, sizeof(pBuffer), "eg%04X", nValue);

	SendCommand(std::string(pBuffer));
}

void CSensorthermPyrometer::PublishParameters(LibMCEnv::PDriverStatusUpdateSession pDriverUpdateInstance)
{
	if (pDriverUpdateInstance.get() == nullptr)
		return;

	std::lock_guard<std::mutex> lock(m_DataMutex);

	const bool bConnected = (m_pCommandConnection.get() != nullptr) && m_pCommandConnection->IsConnected();

	const double dNotMeasured = std::numeric_limits<double>::quiet_NaN();

	pDriverUpdateInstance->SetBoolParameter("connected", bConnected);
	pDriverUpdateInstance->SetBoolParameter("measuring", m_Sample.bIsValid);
	pDriverUpdateInstance->SetBoolParameter("streaming", m_bIsStreaming);
	pDriverUpdateInstance->SetIntegerParameter("sample_count", (LibMCEnv_int64)m_nSampleCount);
	pDriverUpdateInstance->SetIntegerParameter("discarded_samples", (LibMCEnv_int64)m_nDiscardedSamples);
	pDriverUpdateInstance->SetIntegerParameter("sample_age_ms", (m_nSampleCount == 0) ? -1 : (MicroSecondsNow() - m_Sample.nHostMicroSeconds) / 1000);
	pDriverUpdateInstance->SetIntegerParameter("device_timestamp_ms", (LibMCEnv_int64)((double)m_Sample.nDeviceTicks / SENSORTHERM_TICKS_PER_MILLISECOND));

	pDriverUpdateInstance->SetDoubleParameter("two_color_temperature", m_Sample.bIsValid ? m_Sample.dTwoColorTemperature : dNotMeasured);
	pDriverUpdateInstance->SetDoubleParameter("channel1_temperature", m_Sample.bIsValid ? m_Sample.dChannel1Temperature : dNotMeasured);
	pDriverUpdateInstance->SetDoubleParameter("channel2_temperature", m_Sample.bIsValid ? m_Sample.dChannel2Temperature : dNotMeasured);
	pDriverUpdateInstance->SetDoubleParameter("signal_strength", m_Sample.bIsValid ? m_Sample.dSignalStrength : dNotMeasured);
	pDriverUpdateInstance->SetDoubleParameter("control_output", m_Sample.bIsValid ? m_Sample.dControlOutput : dNotMeasured);
	pDriverUpdateInstance->SetDoubleParameter("setpoint", m_Sample.bIsValid ? m_Sample.dSetpoint : dNotMeasured);

	pDriverUpdateInstance->SetBoolParameter("control_active", m_Sample.IsControlActive());
	pDriverUpdateInstance->SetBoolParameter("fahrenheit", m_Sample.IsFahrenheit());

	uint32_t nInputs = 0;
	uint32_t nOutputs = 0;
	for (uint32_t nIndex = 1; nIndex <= 4; nIndex++) {
		if (m_Sample.IsInputActive(nIndex))
			nInputs |= (1u << (nIndex - 1));
		if (m_Sample.IsOutputActive(nIndex))
			nOutputs |= (1u << (nIndex - 1));
	}

	pDriverUpdateInstance->SetIntegerParameter("digital_inputs", (LibMCEnv_int64)nInputs);
	pDriverUpdateInstance->SetIntegerParameter("digital_outputs", (LibMCEnv_int64)nOutputs);
}






