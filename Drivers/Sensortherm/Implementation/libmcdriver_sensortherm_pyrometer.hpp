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


Abstract: This is the class declaration of CSensorthermPyrometer

*/


#ifndef __LIBMCDRIVER_SENSORTHERM_PYROMETER
#define __LIBMCDRIVER_SENSORTHERM_PYROMETER

#include "libmcdriver_sensortherm_interfaces.hpp"
#include "libmcdriver_sensortherm_protocol.hpp"

// Parent classes
#include "libmcdriver_sensortherm_base.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4250)
#endif

// Include custom headers here.
#include <chrono>
#include <mutex>
#include <vector>

namespace LibMCDriver_Sensortherm {
namespace Impl {

class CDriver_Sensortherm;


/*************************************************************************************************************************
 Class declaration of CSensorthermPyrometer
**************************************************************************************************************************/

/**
* One connection to a Sensortherm H4 series pyrometer. Both sockets are opened lazily, so
* that a machine description may reference a pyrometer that is currently switched off or
* being cabled up. All measurements are protected by a mutex, because the data stream can
* be read from the driver thread while a state machine queries a value.
*/
class CSensorthermPyrometer : public virtual ISensorthermPyrometer, public virtual CBase {
private:

	LibMCEnv::PDriverEnvironment m_pDriverEnvironment;

	std::string m_sIdentifier;
	std::string m_sIPAddress;
	std::string m_sDeviceAddress;

	LibMCDriver_Sensortherm_uint32 m_nCommandPort;
	LibMCDriver_Sensortherm_uint32 m_nStreamPort;
	LibMCDriver_Sensortherm_uint32 m_nTimeOutInMs;

	ESensorthermByteOrder m_eByteOrder;
	double m_dTemperatureFactor;
	LibMCDriver_Sensortherm::eSensorthermStreamMode m_eStreamMode;

	LibMCEnv::PTCPIPConnection m_pCommandConnection;
	LibMCEnv::PTCPIPConnection m_pStreamConnection;

	int64_t m_nLastConnectAttemptMicroSeconds;

	bool m_bIsStreaming;

	std::mutex m_DataMutex;
	sSensorthermSample m_Sample;
	uint64_t m_nSampleCount;
	std::chrono::steady_clock::time_point m_LastSampleTime;

	// Bytes of a stream packet that did not arrive complete. The data stream is a plain
	// sequence of packets without any framing, so a partial packet has to be completed by
	// the next read in order to keep the packet alignment.
	std::vector<uint8_t> m_vStreamCarry;

	// Number of stream packets that were dropped rather than parsed, because the pyrometer
	// sends faster than the driver is polled. Dropping the oldest packets is what keeps the
	// reported sample current instead of further and further in the past.
	uint64_t m_nDiscardedSamples;

	bool EnsureCommandConnection();

	bool EnsureStreamConnection();

	/**
	* Sends one command and reads the answer. The device address is prefixed and the
	* carriage return terminator is appended here.
	*/
	std::string ExecuteCommand(const std::string & sCommand);

	/**
	* Reads up to the next carriage return, which terminates every answer.
	*/
	std::string ReadAnswer();

	void StoreSample(const sSensorthermSample & sSample);

	static int64_t MicroSecondsNow();

public:

	CSensorthermPyrometer(const std::string & sIdentifier, const std::string & sIPAddress, const LibMCDriver_Sensortherm_uint32 nCommandPort, const LibMCDriver_Sensortherm_uint32 nStreamPort, const LibMCDriver_Sensortherm_uint32 nTimeOutInMs, const std::string & sDeviceAddress, const ESensorthermByteOrder eByteOrder, const double dTemperatureFactor, const LibMCDriver_Sensortherm::eSensorthermStreamMode eStreamMode, const LibMCEnv::PDriverEnvironment pDriverEnvironment);

	virtual ~CSensorthermPyrometer();

	void Disconnect() override;

	std::string GetIdentifier() override;

	std::string GetIPAddress() override;

	LibMCDriver_Sensortherm_uint32 GetCommandPort() override;

	LibMCDriver_Sensortherm_uint32 GetStreamPort() override;

	bool IsConnected() override;

	std::string SendCommand(const std::string & sCommand) override;

	bool PollSample() override;

	bool UpdateStream() override;

	void StartStreaming() override;

	void StopStreaming() override;

	bool IsStreaming() override;

	LibMCDriver_Sensortherm::eSensorthermStreamMode GetStreamMode() override;

	void SetStreamMode(const LibMCDriver_Sensortherm::eSensorthermStreamMode eStreamMode) override;

	LibMCDriver_Sensortherm_double GetTwoColorTemperature() override;

	LibMCDriver_Sensortherm_double GetChannel1Temperature() override;

	LibMCDriver_Sensortherm_double GetChannel2Temperature() override;

	LibMCDriver_Sensortherm_double GetSignalStrength() override;

	LibMCDriver_Sensortherm_double GetControlOutput() override;

	LibMCDriver_Sensortherm_double GetSetpoint() override;

	LibMCDriver_Sensortherm_int64 GetSampleCount() override;

	LibMCDriver_Sensortherm_int64 GetDeviceTimeStamp() override;

	LibMCDriver_Sensortherm_int64 GetAgeInMilliseconds() override;

	bool GetDigitalInput(const LibMCDriver_Sensortherm_uint32 nIndex) override;

	bool GetDigitalOutput(const LibMCDriver_Sensortherm_uint32 nIndex) override;

	bool IsControlActive() override;

	bool IsFahrenheit() override;

	LibMCDriver_Sensortherm_uint32 GetFaultStatus() override;

	std::string GetSerialNumber() override;

	std::string GetOrderNumber() override;

	std::string GetDeviceType() override;

	LibMCDriver_Sensortherm_double GetEmissivity() override;

	void SetEmissivity(const LibMCDriver_Sensortherm_double dEmissivity) override;

	/**
	* Publishes the connection state and the newest sample into the driver parameters.
	* Called by the driver on every parameter query.
	*/
	void PublishParameters(LibMCEnv::PDriverStatusUpdateSession pDriverUpdateInstance);

};

} // namespace Impl
} // namespace LibMCDriver_Sensortherm

#ifdef _MSC_VER
#pragma warning(pop)
#endif
#endif // __LIBMCDRIVER_SENSORTHERM_PYROMETER

