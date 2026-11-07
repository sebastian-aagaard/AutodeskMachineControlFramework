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


Abstract: This is the class declaration of CDriver_Sensortherm

*/


#ifndef __LIBMCDRIVER_SENSORTHERM_DRIVER_SENSORTHERM
#define __LIBMCDRIVER_SENSORTHERM_DRIVER_SENSORTHERM

#include "libmcdriver_sensortherm_interfaces.hpp"
#include "libmcdriver_sensortherm_pyrometer.hpp"

// Parent classes
#include "libmcdriver_sensortherm_driver.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4250)
#endif

// Include custom headers here.
#include <map>
#include <mutex>

namespace LibMCDriver_Sensortherm {
namespace Impl {


/*************************************************************************************************************************
 Local constants
**************************************************************************************************************************/

// Configuration schema of the driver section in a machine definition.
const std::string SENSORTHERM_CONFIGURATIONSCHEMA = "http://schemas.autodesk.com/amc/sensorthermh4/2026/02";


/*************************************************************************************************************************
 Class declaration of CDriver_Sensortherm
**************************************************************************************************************************/

class CDriver_Sensortherm : public virtual IDriver_Sensortherm, public virtual CDriver {
private:

	std::string m_sName;

	LibMCEnv::PDriverEnvironment m_pDriverEnvironment;

	// Connection settings taken from the machine definition. A value that a caller of
	// Connect leaves empty or zero falls back to these defaults.
	std::string m_sDefaultIPAddress;
	std::string m_sDefaultDeviceAddress;
	LibMCDriver_Sensortherm_uint32 m_nDefaultCommandPort;
	LibMCDriver_Sensortherm_uint32 m_nDefaultStreamPort;
	LibMCDriver_Sensortherm_uint32 m_nDefaultTimeOutInMs;
	ESensorthermByteOrder m_eDefaultByteOrder;
	double m_dDefaultTemperatureFactor;
	LibMCDriver_Sensortherm::eSensorthermStreamMode m_eDefaultStreamMode;

	// A machine definition can give the connection settings and let the driver poll the
	// pyrometer on its own, which makes the measurements visible without a state machine.
	bool m_bIsAutoConnecting;
	bool m_bIsAutoStreaming;
	std::string m_sAutoConnectionIdentifier;

	std::mutex m_ConnectionMutex;
	std::map<std::string, CSensorthermPyrometer*> m_Connections;

	CSensorthermPyrometer * FindConnectionInstance(const std::string & sIdentifier);

public:

	CDriver_Sensortherm(const std::string& sName, LibMCEnv::PDriverEnvironment pDriverEnvironment);

	virtual ~CDriver_Sensortherm();

	void Configure(const std::string& sConfigurationString) override;

	std::string GetName() override;

	std::string GetType() override;

	void GetVersion(LibMCDriver_Sensortherm_uint32& nMajor, LibMCDriver_Sensortherm_uint32& nMinor, LibMCDriver_Sensortherm_uint32& nMicro, std::string& sBuild) override;

	void QueryParameters() override;

	void QueryParametersEx(LibMCEnv::PDriverStatusUpdateSession pDriverUpdateInstance) override;

	ISensorthermPyrometer * Connect(const std::string & sIdentifier, const std::string & sIPAddress, const LibMCDriver_Sensortherm_uint32 nCommandPort, const LibMCDriver_Sensortherm_uint32 nStreamPort, const std::string & sDeviceAddress, const LibMCDriver_Sensortherm_uint32 nTimeOutInMs) override;

	ISensorthermPyrometer * FindConnection(const std::string & sIdentifier) override;

	bool ConnectionExists(const std::string & sIdentifier) override;

	void CloseAllConnections() override;

};

} // namespace Impl
} // namespace LibMCDriver_Sensortherm

#ifdef _MSC_VER
#pragma warning(pop)
#endif
#endif // __LIBMCDRIVER_SENSORTHERM_DRIVER_SENSORTHERM
