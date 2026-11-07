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


Abstract: This is the class implementation of CDriver_Sensortherm

*/

#include "libmcdriver_sensortherm_driver_sensortherm.hpp"
#include "libmcdriver_sensortherm_interfaceexception.hpp"

// Include custom headers here.
#define __STRINGIZE(x) #x
#define __STRINGIZE_VALUE_OF(x) __STRINGIZE(x)

using namespace LibMCDriver_Sensortherm::Impl;


/*************************************************************************************************************************
 Class definition of CDriver_Sensortherm
**************************************************************************************************************************/

CDriver_Sensortherm::CDriver_Sensortherm(const std::string& sName, LibMCEnv::PDriverEnvironment pDriverEnvironment)
	: m_sName(sName), m_pDriverEnvironment(pDriverEnvironment), m_sDefaultDeviceAddress("00"),
	  m_nDefaultCommandPort(SENSORTHERM_DEFAULT_COMMANDPORT), m_nDefaultStreamPort(SENSORTHERM_DEFAULT_STREAMPORT),
	  m_nDefaultTimeOutInMs(1000), m_eDefaultByteOrder(ESensorthermByteOrder::BigEndian),
	  m_dDefaultTemperatureFactor(SENSORTHERM_TEMPERATURE_FACTOR),
	  m_bIsAutoConnecting(false), m_bIsAutoStreaming(false), m_sAutoConnectionIdentifier("pyrometer"),
	  m_eDefaultStreamMode(LibMCDriver_Sensortherm::eSensorthermStreamMode::Continuous)
{
	if (pDriverEnvironment.get() == nullptr)
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM);
}


CDriver_Sensortherm::~CDriver_Sensortherm()
{
	try {
		CloseAllConnections();
	} catch (...) {
	}
}

void CDriver_Sensortherm::Configure(const std::string& sConfigurationString)
{

	if (!sConfigurationString.empty()) {

		auto pXMLDocument = m_pDriverEnvironment->ParseXMLString(sConfigurationString);

		if (pXMLDocument->GetDefaultNamespace() != SENSORTHERM_CONFIGURATIONSCHEMA)
			throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDCONFIGURATIONSCHEMA,
				"the pyrometer driver configuration must use the namespace '" + SENSORTHERM_CONFIGURATIONSCHEMA + "'");

		auto pRootNode = pXMLDocument->GetRootNode();

		if (pRootNode->HasChild("", "connection")) {

			auto pConnectionNode = pRootNode->FindChild("", "connection", true);

			m_sDefaultIPAddress = pConnectionNode->GetAttributeValueDef("", "ipaddress", m_sDefaultIPAddress);
			m_nDefaultCommandPort = (LibMCDriver_Sensortherm_uint32)pConnectionNode->GetAttributeIntegerValueDef("", "commandport", 1, 65535, m_nDefaultCommandPort);
			m_nDefaultStreamPort = (LibMCDriver_Sensortherm_uint32)pConnectionNode->GetAttributeIntegerValueDef("", "streamport", 1, 65535, m_nDefaultStreamPort);
			m_nDefaultTimeOutInMs = (LibMCDriver_Sensortherm_uint32)pConnectionNode->GetAttributeIntegerValueDef("", "timeout", 10, 600000, m_nDefaultTimeOutInMs);
			m_sDefaultDeviceAddress = pConnectionNode->GetAttributeValueDef("", "deviceaddress", m_sDefaultDeviceAddress);
			m_dDefaultTemperatureFactor = pConnectionNode->GetAttributeDoubleValueDef("", "temperaturefactor", 0.0001, 1000.0, m_dDefaultTemperatureFactor);
			m_sAutoConnectionIdentifier = pConnectionNode->GetAttributeValueDef("", "identifier", m_sAutoConnectionIdentifier);
			m_bIsAutoConnecting = pConnectionNode->GetAttributeBoolValueDef("", "connect", m_bIsAutoConnecting) && !m_sDefaultIPAddress.empty();
			m_bIsAutoStreaming = pConnectionNode->GetAttributeBoolValueDef("", "streaming", m_bIsAutoStreaming);

			const std::string sByteOrder = pConnectionNode->GetAttributeValueDef("", "byteorder", "big");
			if (sByteOrder == "big")
				m_eDefaultByteOrder = ESensorthermByteOrder::BigEndian;
			else if (sByteOrder == "little")
				m_eDefaultByteOrder = ESensorthermByteOrder::LittleEndian;
			else
				throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDDRIVERPROTOCOL,
					"the byte order of a pyrometer connection must be 'big' or 'little', got '" + sByteOrder + "'");

			const int64_t nStreamMode = pConnectionNode->GetAttributeIntegerValueDef("", "streammode", 0, 1, (int64_t)m_eDefaultStreamMode);
			m_eDefaultStreamMode = (nStreamMode == 1)
				? LibMCDriver_Sensortherm::eSensorthermStreamMode::ControllerSampled
				: LibMCDriver_Sensortherm::eSensorthermStreamMode::Continuous;

		}

	}

	m_pDriverEnvironment->LogMessage("Configuring the Sensortherm pyrometer driver '" + m_sName + "'");

	m_pDriverEnvironment->RegisterBoolParameter("connected", "The pyrometer is reachable", false);
	m_pDriverEnvironment->RegisterBoolParameter("measuring", "The pyrometer reported a valid measurement", false);
	m_pDriverEnvironment->RegisterBoolParameter("streaming", "The pyrometer data stream is active", false);
	m_pDriverEnvironment->RegisterIntegerParameter("sample_count", "Number of samples received from the pyrometer", 0);
	m_pDriverEnvironment->RegisterIntegerParameter("discarded_samples", "Number of stream samples skipped to keep the newest sample current", 0);
	m_pDriverEnvironment->RegisterIntegerParameter("sample_age_ms", "Age of the newest pyrometer sample in milliseconds", -1);
	m_pDriverEnvironment->RegisterIntegerParameter("device_timestamp_ms", "Device time stamp of the newest pyrometer sample in milliseconds", 0);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("two_color_temperature", "Two color temperature of the pyrometer in degree Celsius", 0.0, 0.1);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("channel1_temperature", "Channel 1 temperature of the pyrometer in degree Celsius", 0.0, 0.1);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("channel2_temperature", "Channel 2 temperature of the pyrometer in degree Celsius", 0.0, 0.1);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("signal_strength", "Signal strength of the pyrometer in percent", 0.0, 0.1);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("control_output", "Manipulated variable of the pyrometer controller in percent", 0.0, 0.1);
	m_pDriverEnvironment->RegisterDoubleParameterWithUnits("setpoint", "Setpoint of the pyrometer controller in degree Celsius", 0.0, 0.1);
	m_pDriverEnvironment->RegisterBoolParameter("control_active", "The controller of the pyrometer is active", false);
	m_pDriverEnvironment->RegisterBoolParameter("fahrenheit", "The pyrometer reports Fahrenheit", false);
	m_pDriverEnvironment->RegisterIntegerParameter("digital_inputs", "Bit field of the digital inputs of the pyrometer", 0);
	m_pDriverEnvironment->RegisterIntegerParameter("digital_outputs", "Bit field of the digital outputs of the pyrometer", 0);

	if (m_bIsAutoConnecting) {
		// The connection is created lazily, so that a pyrometer that is currently down does
		// not keep the machine from starting.
		if (!ConnectionExists(m_sAutoConnectionIdentifier)) {
			Connect(m_sAutoConnectionIdentifier, m_sDefaultIPAddress, 0, 0, "", 0);
			m_pDriverEnvironment->LogMessage("The pyrometer driver '" + m_sName + "' polls the pyrometer at " + m_sDefaultIPAddress + " by itself");
		}
	}

}


std::string CDriver_Sensortherm::GetName()
{
	return m_sName;
}


std::string CDriver_Sensortherm::GetType()
{
	return "sensortherm-h4";
}


void CDriver_Sensortherm::GetVersion(LibMCDriver_Sensortherm_uint32& nMajor, LibMCDriver_Sensortherm_uint32& nMinor, LibMCDriver_Sensortherm_uint32& nMicro, std::string& sBuild)
{
	nMajor = LIBMCDRIVER_SENSORTHERM_VERSION_MAJOR;
	nMinor = LIBMCDRIVER_SENSORTHERM_VERSION_MINOR;
	nMicro = LIBMCDRIVER_SENSORTHERM_VERSION_MICRO;
	sBuild = __STRINGIZE_VALUE_OF(__GITHASH);
}

void CDriver_Sensortherm::QueryParameters()
{
	QueryParametersEx(m_pDriverEnvironment->CreateStatusUpdateSession());
}


void CDriver_Sensortherm::QueryParametersEx(LibMCEnv::PDriverStatusUpdateSession pDriverUpdateInstance)
{
	if (pDriverUpdateInstance.get() == nullptr)
		return;

	std::lock_guard<std::mutex> lock(m_ConnectionMutex);

	// A driver that was configured to poll on its own queries the newest sample here. A
	// pyrometer that is driven by a state machine is not touched.
	if (m_bIsAutoConnecting) {
		auto iIter = m_Connections.find(m_sAutoConnectionIdentifier);
		if (iIter != m_Connections.end()) {
			try {
				if (m_bIsAutoStreaming && !iIter->second->IsStreaming())
					iIter->second->StartStreaming();
				if (!iIter->second->UpdateStream())
					iIter->second->PollSample();
			} catch (std::exception &) {
			}
		}
	}

	// The driver parameter group is a flat namespace, so only the connection with the
	// alphabetically first identifier is mirrored into it. Further pyrometers are read
	// over their connection instance.
	for (auto iIter : m_Connections) {
		iIter.second->PublishParameters(pDriverUpdateInstance);
		break;
	}
}


ISensorthermPyrometer * CDriver_Sensortherm::Connect(const std::string & sIdentifier, const std::string & sIPAddress, const LibMCDriver_Sensortherm_uint32 nCommandPort, const LibMCDriver_Sensortherm_uint32 nStreamPort, const std::string & sDeviceAddress, const LibMCDriver_Sensortherm_uint32 nTimeOutInMs)
{

	if (sIdentifier.empty())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM, "a pyrometer connection needs an identifier");

	std::lock_guard<std::mutex> lock(m_ConnectionMutex);

	auto iIter = m_Connections.find(sIdentifier);
	if (iIter != m_Connections.end())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_DUPLICATECONNECTION, "a pyrometer connection with the identifier '" + sIdentifier + "' already exists");

	std::string sAddress = sIPAddress.empty() ? m_sDefaultIPAddress : sIPAddress;
	if (sAddress.empty())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_INVALIDPARAM, "no IP address is known for the pyrometer connection '" + sIdentifier + "'. Give one in the machine definition or in the Connect call");

	std::string sAddressOfDevice = sDeviceAddress.empty() ? m_sDefaultDeviceAddress : sDeviceAddress;

	const LibMCDriver_Sensortherm_uint32 nCommand = (nCommandPort == 0) ? m_nDefaultCommandPort : nCommandPort;
	const LibMCDriver_Sensortherm_uint32 nStream = (nStreamPort == 0) ? m_nDefaultStreamPort : nStreamPort;
	const LibMCDriver_Sensortherm_uint32 nTimeOut = (nTimeOutInMs == 0) ? m_nDefaultTimeOutInMs : nTimeOutInMs;

	auto pPyrometer = new CSensorthermPyrometer(sIdentifier, sAddress, nCommand, nStream, nTimeOut, sAddressOfDevice, m_eDefaultByteOrder, m_dDefaultTemperatureFactor, m_eDefaultStreamMode, m_pDriverEnvironment);

	// The driver keeps the connection alive until it is closed, independent of how many
	// handles the state machines hold.
	pPyrometer->IncRefCount();
	m_Connections.insert(std::make_pair(sIdentifier, pPyrometer));

	m_pDriverEnvironment->LogMessage("Created a pyrometer connection '" + sIdentifier + "' to " + sAddress + " (command port " + std::to_string(nCommand) + ", stream port " + std::to_string(nStream) + ")");

	return pPyrometer;
}


ISensorthermPyrometer * CDriver_Sensortherm::FindConnection(const std::string & sIdentifier)
{
	std::lock_guard<std::mutex> lock(m_ConnectionMutex);

	auto pPyrometer = FindConnectionInstance(sIdentifier);
	pPyrometer->IncRefCount();

	return pPyrometer;
}


bool CDriver_Sensortherm::ConnectionExists(const std::string & sIdentifier)
{
	std::lock_guard<std::mutex> lock(m_ConnectionMutex);
	return (m_Connections.find(sIdentifier) != m_Connections.end());
}


void CDriver_Sensortherm::CloseAllConnections()
{
	std::lock_guard<std::mutex> lock(m_ConnectionMutex);

	for (auto iIter : m_Connections) {
		iIter.second->Disconnect();
		iIter.second->DecRefCount();
	}

	m_Connections.clear();
}


CSensorthermPyrometer * CDriver_Sensortherm::FindConnectionInstance(const std::string & sIdentifier)
{
	auto iIter = m_Connections.find(sIdentifier);
	if (iIter == m_Connections.end())
		throw ELibMCDriver_SensorthermInterfaceException(LIBMCDRIVER_SENSORTHERM_ERROR_CONNECTIONNOTFOUND, "no pyrometer connection with the identifier '" + sIdentifier + "' exists");

	return iIter->second;
}



