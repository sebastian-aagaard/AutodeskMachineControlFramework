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


Abstract: Wire format of the Sensortherm H4 series (HELIOS H422 / H433) pyrometer.

The device speaks two protocols on two TCP ports:

  - Command port (device default 1024): ASCII commands. Every command starts with the
    two digit device address, is followed by the command letters and its parameters,
    and is terminated by a carriage return. Reads answer with the requested value,
    writes answer with "ok", rejected parameters answer with "no". The measurement
    data can be polled with the buffer poll command "bup", which answers with the
    hexadecimal payload of the active buffer mode.

  - Data stream port (device default 1025): binary. After the stream has been started
    with "ipdXX" (mode) and "ipx1" (start), the device sends packets of exactly
    SENSORTHERM_STREAMPACKETSIZE bytes with a fixed layout.

Encodings that the manufacturer does not document (the byte order of the stream
packet, and the temperature factor) are parameters of the driver configuration.

*/

#ifndef __LIBMCDRIVER_SENSORTHERM_PROTOCOL
#define __LIBMCDRIVER_SENSORTHERM_PROTOCOL

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

// Header only and without driver dependencies, so that amc_unittest can test it.

namespace LibMCDriver_Sensortherm {
namespace Impl {

/*************************************************************************************************************************
 Sensortherm H4 protocol constants
**************************************************************************************************************************/

// Default TCP port the pyrometer is commanded on.
const uint32_t SENSORTHERM_DEFAULT_COMMANDPORT = 1024;
// Default TCP port the pyrometer streams its measurement data on.
const uint32_t SENSORTHERM_DEFAULT_STREAMPORT = 1025;

// Size of one binary data stream packet.
const size_t SENSORTHERM_STREAMPACKETSIZE = 20;

// Device time stamp ticks per millisecond (126000 ticks per ms).
const double SENSORTHERM_TICKS_PER_MILLISECOND = 126000.0;

// Temperatures are reported in tenth of a degree.
const double SENSORTHERM_TEMPERATURE_FACTOR = 0.1;

// Signal strengths and controller outputs are reported in tenth of a percent.
const double SENSORTHERM_PERCENT_FACTOR = 0.1;

// A temperature word of 0xF001 marks a measurement overflow.
const uint16_t SENSORTHERM_OVERFLOW_MARKER = 0xF001;

// Byte offsets inside one data stream packet.
const size_t SENSORTHERM_OFFSET_TIMESTAMP = 0x00;
const size_t SENSORTHERM_OFFSET_TWOCOLORTEMPERATURE = 0x04;
const size_t SENSORTHERM_OFFSET_CHANNEL1TEMPERATURE = 0x06;
const size_t SENSORTHERM_OFFSET_CHANNEL2TEMPERATURE = 0x08;
const size_t SENSORTHERM_OFFSET_SIGNALSTRENGTH = 0x0A;
const size_t SENSORTHERM_OFFSET_SETPOINT = 0x0C;
const size_t SENSORTHERM_OFFSET_CONTROLOUTPUT = 0x0E;
const size_t SENSORTHERM_OFFSET_IO = 0x10;
const size_t SENSORTHERM_OFFSET_STATUS = 0x11;
const size_t SENSORTHERM_OFFSET_SOFTWAREVERSION = 0x12;

// Bits of the status byte of a data stream packet.
const uint8_t SENSORTHERM_STATUS_PILOTLIGHT = 0x08;
const uint8_t SENSORTHERM_STATUS_CONTROLACTIVE = 0x10;
const uint8_t SENSORTHERM_STATUS_FAHRENHEIT = 0x20;


enum class ESensorthermByteOrder {
	BigEndian = 0,
	LittleEndian = 1
};

/*************************************************************************************************************************
 Struct declaration of sSensorthermSample
**************************************************************************************************************************/

/**
* One measurement sample of a pyrometer. The values are already converted into physical
* units (degree Celsius and percent). A temperature of NaN means that the pyrometer
* reported a measurement overflow for that channel.
*/
struct sSensorthermSample {
	bool bIsValid;
	uint32_t nDeviceTicks;
	double dTwoColorTemperature;
	double dChannel1Temperature;
	double dChannel2Temperature;
	double dSignalStrength;
	double dControlOutput;
	double dSetpoint;
	uint8_t nIOByte;
	uint8_t nStatusByte;
	uint16_t nSoftwareVersion;
	int64_t nHostMicroSeconds;

	sSensorthermSample()
		: bIsValid(false), nDeviceTicks(0), dTwoColorTemperature(0.0), dChannel1Temperature(0.0),
		  dChannel2Temperature(0.0), dSignalStrength(0.0), dControlOutput(0.0), dSetpoint(0.0),
		  nIOByte(0), nStatusByte(0), nSoftwareVersion(0), nHostMicroSeconds(0)
	{
	}

	bool IsPilotLightActive() const { return (nStatusByte & SENSORTHERM_STATUS_PILOTLIGHT) != 0; }
	bool IsControlActive() const { return (nStatusByte & SENSORTHERM_STATUS_CONTROLACTIVE) != 0; }
	bool IsFahrenheit() const { return (nStatusByte & SENSORTHERM_STATUS_FAHRENHEIT) != 0; }

	bool IsInputActive(const uint32_t nIndex) const {
		return (nIOByte & (1u << (nIndex - 1))) != 0;
	}

	bool IsOutputActive(const uint32_t nIndex) const {
		return (nIOByte & (1u << ((nIndex - 1) + 4))) != 0;
	}
};


/*************************************************************************************************************************
 Protocol helper functions
**************************************************************************************************************************/

inline uint16_t readUInt16(const std::vector<uint8_t> & pBuffer, const size_t nIndex, const ESensorthermByteOrder eByteOrder) {
	if (nIndex + 1 >= pBuffer.size())
		return 0;
	if (eByteOrder == ESensorthermByteOrder::LittleEndian)
		return (uint16_t)((uint16_t)pBuffer[nIndex] | ((uint16_t)pBuffer[nIndex + 1] << 8));
	return (uint16_t)(((uint16_t)pBuffer[nIndex] << 8) | (uint16_t)pBuffer[nIndex + 1]);
}

inline uint32_t readUInt32(const std::vector<uint8_t> & pBuffer, const size_t nIndex, const ESensorthermByteOrder eByteOrder) {
	if (nIndex + 3 >= pBuffer.size())
		return 0;
	if (eByteOrder == ESensorthermByteOrder::LittleEndian)
		return	(uint32_t)pBuffer[nIndex] | ((uint32_t)pBuffer[nIndex + 1] << 8) |
				((uint32_t)pBuffer[nIndex + 2] << 16) | ((uint32_t)pBuffer[nIndex + 3] << 24);
	return	((uint32_t)pBuffer[nIndex] << 24) | ((uint32_t)pBuffer[nIndex + 1] << 16) |
			((uint32_t)pBuffer[nIndex + 2] << 8) | (uint32_t)pBuffer[nIndex + 3];
}

/**
* Converts a 16 bit device temperature into degree Celsius. The documented overflow
* marker becomes a NaN, so that a caller can separate "not measurable" from a reading.
*/
inline double convertTemperature(const uint16_t nRaw, const double dTemperatureFactor) {
	if (nRaw == SENSORTHERM_OVERFLOW_MARKER)
		return std::numeric_limits<double>::quiet_NaN();
	return (double)nRaw * dTemperatureFactor;
}

/**
* Parses one binary data stream packet. Returns false if the buffer is too small.
*/
inline bool parseStreamPacketAt(const std::vector<uint8_t> & pBuffer, const size_t nOffset, const ESensorthermByteOrder eByteOrder, const double dTemperatureFactor, sSensorthermSample & sSample) {

	if (nOffset + SENSORTHERM_STREAMPACKETSIZE > pBuffer.size())
		return false;

	sSample.bIsValid = true;
	sSample.nDeviceTicks = readUInt32(pBuffer, nOffset + SENSORTHERM_OFFSET_TIMESTAMP, eByteOrder);
	sSample.dTwoColorTemperature = convertTemperature(readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_TWOCOLORTEMPERATURE, eByteOrder), dTemperatureFactor);
	sSample.dChannel1Temperature = convertTemperature(readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_CHANNEL1TEMPERATURE, eByteOrder), dTemperatureFactor);
	sSample.dChannel2Temperature = convertTemperature(readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_CHANNEL2TEMPERATURE, eByteOrder), dTemperatureFactor);
	sSample.dSignalStrength = (double)readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_SIGNALSTRENGTH, eByteOrder) * SENSORTHERM_PERCENT_FACTOR;
	sSample.dSetpoint = convertTemperature(readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_SETPOINT, eByteOrder), dTemperatureFactor);
	sSample.dControlOutput = (double)readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_CONTROLOUTPUT, eByteOrder) * SENSORTHERM_PERCENT_FACTOR;
	sSample.nIOByte = pBuffer[nOffset + SENSORTHERM_OFFSET_IO];
	sSample.nStatusByte = pBuffer[nOffset + SENSORTHERM_OFFSET_STATUS];
	sSample.nSoftwareVersion = readUInt16(pBuffer, nOffset + SENSORTHERM_OFFSET_SOFTWAREVERSION, eByteOrder);

	return true;
}

inline bool parseStreamPacket(const std::vector<uint8_t> & pBuffer, const ESensorthermByteOrder eByteOrder, const double dTemperatureFactor, sSensorthermSample & sSample) {

	return parseStreamPacketAt(pBuffer, 0, eByteOrder, dTemperatureFactor, sSample);
}

inline bool parseHexNumber(const std::string & sHex, uint32_t & nValue) {

	if (sHex.empty())
		return false;

	uint32_t nResult = 0;
	for (size_t nIndex = 0; nIndex < sHex.length(); nIndex++) {
		const char c = sHex[nIndex];
		uint32_t nDigit = 0;
		if ((c >= '0') && (c <= '9'))
			nDigit = (uint32_t)(c - '0');
		else if ((c >= 'a') && (c <= 'f'))
			nDigit = (uint32_t)(c - 'a') + 10;
		else if ((c >= 'A') && (c <= 'F'))
			nDigit = (uint32_t)(c - 'A') + 10;
		else
			return false;
		nResult = (nResult << 4) | nDigit;
	}
	nValue = nResult;
	return true;
}

inline bool readHexWord(const std::string & sPayload, const size_t nIndex, const size_t nWidth, uint32_t & nValue) {
	if (nIndex + nWidth > sPayload.length())
		return false;
	return parseHexNumber(sPayload.substr(nIndex, nWidth), nValue);
}

/**
* Parses the answer of the buffer poll command "bup". The payload depends on the buffer
* mode that is active on the device:
*
*   mode 00: AAAA                                   display temperature
*   mode 01: AAAA BBBB CCCC                         quotient, channel 1, channel 2
*   mode 02: AAAA BBBB CCCC DDDD EEEE FFFF GG HH II JJ
*   mode 03: AAAA BBBB CCCC DDDD EEEE FFFF GG HH II JJ KKKK LLLL MMMM MMMM
*
* AAAA to CCCC are temperatures in tenth of a degree, DDDD is the current ramp setpoint,
* EEEE the analog controller output and FFFF the signal strength in tenth of a percent,
* GG to JJ are the four data status bytes. The status bytes are interpreted with the bit
* layout the manufacturer documents for the status byte of the binary data stream.
*
* Returns false if the payload is not a poll payload.
*/
inline bool parsePollPayload(const std::string & sPayload, const double dTemperatureFactor, sSensorthermSample & sSample) {

	std::string sData;
	for (size_t nIndex = 0; nIndex < sPayload.length(); nIndex++) {
		const char c = sPayload[nIndex];
		if ((c == ' ') || (c == '\t') || (c == '\r') || (c == '\n'))
			continue;
		sData.push_back(c);
	}

	if ((sData.length() != 4) && (sData.length() != 12) && (sData.length() != 32) && (sData.length() != 40))
		return false;

	uint32_t nValue = 0;

	sSample = sSensorthermSample();
	sSample.bIsValid = true;

	if (!readHexWord(sData, 0, 4, nValue))
		return false;
	sSample.dTwoColorTemperature = convertTemperature((uint16_t)nValue, dTemperatureFactor);

	if (sData.length() >= 12) {
		if (!readHexWord(sData, 4, 4, nValue))
			return false;
		sSample.dChannel1Temperature = convertTemperature((uint16_t)nValue, dTemperatureFactor);
		if (!readHexWord(sData, 8, 4, nValue))
			return false;
		sSample.dChannel2Temperature = convertTemperature((uint16_t)nValue, dTemperatureFactor);
	} else {
		sSample.dChannel1Temperature = std::numeric_limits<double>::quiet_NaN();
		sSample.dChannel2Temperature = std::numeric_limits<double>::quiet_NaN();
	}

	if (sData.length() >= 32) {
		if (!readHexWord(sData, 12, 4, nValue))
			return false;
		sSample.dSetpoint = convertTemperature((uint16_t)nValue, dTemperatureFactor);
		if (!readHexWord(sData, 16, 4, nValue))
			return false;
		sSample.dControlOutput = (double)nValue * SENSORTHERM_PERCENT_FACTOR;
		if (!readHexWord(sData, 20, 4, nValue))
			return false;
		sSample.dSignalStrength = (double)nValue * SENSORTHERM_PERCENT_FACTOR;
		if (!readHexWord(sData, 24, 2, nValue))
			return false;
		sSample.nStatusByte = (uint8_t)nValue;
		if (!readHexWord(sData, 26, 2, nValue))
			return false;
		sSample.nIOByte = (uint8_t)nValue;
	}

	return true;
}

} // namespace Impl
} // namespace LibMCDriver_Sensortherm

#endif // __LIBMCDRIVER_SENSORTHERM_PROTOCOL


