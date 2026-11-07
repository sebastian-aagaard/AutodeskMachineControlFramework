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

*/

#ifndef __AMC_UNITTESTS_SENSORTHERM_PROTOCOL
#define __AMC_UNITTESTS_SENSORTHERM_PROTOCOL

#include "amc_unittests.hpp"
#include "Drivers/Sensortherm/Implementation/libmcdriver_sensortherm_protocol.hpp"

#include <cmath>
#include <limits>
#include <vector>


namespace AMCUnitTest {

	using namespace LibMCDriver_Sensortherm::Impl;

	class CUnitTestGroup_Sensortherm_Protocol : public CUnitTestGroup {
	public:

		std::string getTestGroupName() override {
			return "Sensortherm_Protocol";
		}

		void registerTests() override {
			registerTest("StreamPacket", "A big endian data stream packet is decoded", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_Sensortherm_Protocol::testStreamPacket, this));
			registerTest("StreamPacketByteOrder", "The little endian byte order decodes the same packet", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_Sensortherm_Protocol::testStreamPacketByteOrder, this));
			registerTest("StreamPacketOffset", "A packet at an offset into a block of stream data decodes", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_Sensortherm_Protocol::testStreamPacketOffset, this));
			registerTest("PollPayload", "The buffer modes of the poll command are decoded", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_Sensortherm_Protocol::testPollPayload, this));

			registerTest("InvalidPayload", "Answers that are no measurement data are rejected", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_Sensortherm_Protocol::testInvalidPayload, this));
		}

		void initializeTests() override {
		}

	private:

		typedef sSensorthermSample CSample;

		static std::vector<uint8_t> buildStreamPacket()
		{
			std::vector<uint8_t> pBuffer(SENSORTHERM_STREAMPACKETSIZE, 0);

			pBuffer[0x00] = 0x00; pBuffer[0x01] = 0x00; pBuffer[0x02] = 0x04; pBuffer[0x03] = 0xE4;
			pBuffer[0x04] = 0x0B; pBuffer[0x05] = 0xB8;
			pBuffer[0x06] = 0x09; pBuffer[0x07] = 0xC4;
			pBuffer[0x08] = 0xF0; pBuffer[0x09] = 0x01;
			pBuffer[0x0A] = 0x03; pBuffer[0x0B] = 0xE8;
			pBuffer[0x0C] = 0x0B; pBuffer[0x0D] = 0xB8;
			pBuffer[0x0E] = 0x01; pBuffer[0x0F] = 0xF4;
			pBuffer[0x10] = 0x13;
			pBuffer[0x11] = 0x10;
			pBuffer[0x12] = 0x01; pBuffer[0x13] = 0x02;

			return pBuffer;
		}

		static bool nearlyEqual(const double dA, const double dB)
		{
			return std::abs(dA - dB) < 1.0e-9;
		}

		void testStreamPacket()
		{
			auto pBuffer = buildStreamPacket();

			CSample sSample;
			assertTrue(parseStreamPacket(pBuffer, ESensorthermByteOrder::BigEndian, SENSORTHERM_TEMPERATURE_FACTOR, sSample), "packet parses");

			assertTrue(sSample.nDeviceTicks == 1252, "device ticks");
			assertTrue(nearlyEqual(sSample.dTwoColorTemperature, 300.0), "two color temperature");
			assertTrue(nearlyEqual(sSample.dChannel1Temperature, 250.0), "channel 1 temperature");
			assertTrue(std::isnan(sSample.dChannel2Temperature), "channel 2 overflow becomes not a number");
			assertTrue(nearlyEqual(sSample.dSignalStrength, 100.0), "signal strength");
			assertTrue(nearlyEqual(sSample.dSetpoint, 300.0), "setpoint");
			assertTrue(nearlyEqual(sSample.dControlOutput, 50.0), "control output");
			assertTrue(sSample.nSoftwareVersion == 0x0102, "software version");

			assertTrue(sSample.IsControlActive(), "control is active");
			assertTrue(!sSample.IsPilotLightActive(), "pilot light is off");
			assertTrue(!sSample.IsFahrenheit(), "temperatures are celsius");

			assertTrue(sSample.IsInputActive(1) && sSample.IsInputActive(2), "inputs 1 and 2 are high");
			assertTrue(!sSample.IsInputActive(3) && !sSample.IsInputActive(4), "inputs 3 and 4 are low");
			assertTrue(sSample.IsOutputActive(1), "output 1 is on");
			assertTrue(!sSample.IsOutputActive(2), "output 2 is off");
		}

		void testStreamPacketByteOrder()
		{
			auto pBigEndian = buildStreamPacket();

			std::vector<uint8_t> pLittleEndian(pBigEndian.size());
			for (size_t nIndex = 0; nIndex + 1 < pBigEndian.size(); nIndex += 2) {
				pLittleEndian[nIndex] = pBigEndian[nIndex + 1];
				pLittleEndian[nIndex + 1] = pBigEndian[nIndex];
			}

			CSample sSample;
			assertTrue(parseStreamPacket(pLittleEndian, ESensorthermByteOrder::LittleEndian, SENSORTHERM_TEMPERATURE_FACTOR, sSample), "little endian packet parses");
			assertTrue(nearlyEqual(sSample.dTwoColorTemperature, 300.0), "little endian decodes the same temperature");

			std::vector<uint8_t> pTooShort(10, 0);
			assertFalse(parseStreamPacket(pTooShort, ESensorthermByteOrder::BigEndian, SENSORTHERM_TEMPERATURE_FACTOR, sSample), "a short packet is rejected");
		}

		void testStreamPacketOffset()
		{
			auto pPacket = buildStreamPacket();

			// A read from the stream socket does not have to end with a packet boundary, so
			// the driver has to be able to parse a packet at an offset into a block of data.
			std::vector<uint8_t> pBuffer(pPacket);
			pBuffer.insert(pBuffer.end(), pPacket.begin(), pPacket.end());

			CSample sSample;
			assertTrue(parseStreamPacketAt(pBuffer, SENSORTHERM_STREAMPACKETSIZE, ESensorthermByteOrder::BigEndian,
										   SENSORTHERM_TEMPERATURE_FACTOR, sSample), "the second packet of a block parses");
			assertTrue(nearlyEqual(sSample.dTwoColorTemperature, 300.0), "offset packet two color temperature");
			assertTrue(nearlyEqual(sSample.dChannel1Temperature, 250.0), "offset packet channel 1 temperature");
			assertTrue(nearlyEqual(sSample.dSetpoint, 300.0), "offset packet setpoint");

			pBuffer.resize(2 * SENSORTHERM_STREAMPACKETSIZE + 10);
			assertFalse(parseStreamPacketAt(pBuffer, 2 * SENSORTHERM_STREAMPACKETSIZE, ESensorthermByteOrder::BigEndian,
											SENSORTHERM_TEMPERATURE_FACTOR, sSample), "a truncated packet at an offset is rejected");
		}

		void testPollPayload()
		{
			CSample sSample;

			assertTrue(parsePollPayload("0BB8", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "buffer mode 00 parses");
			assertTrue(nearlyEqual(sSample.dTwoColorTemperature, 300.0), "buffer mode 00 temperature");
			assertTrue(std::isnan(sSample.dChannel1Temperature), "buffer mode 00 has no channel values");

			assertTrue(parsePollPayload("0BB8 09C4 F001", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "buffer mode 01 parses");
			assertTrue(nearlyEqual(sSample.dChannel1Temperature, 250.0), "buffer mode 01 channel 1");
			assertTrue(std::isnan(sSample.dChannel2Temperature), "buffer mode 01 channel 2 overflow");

			assertTrue(parsePollPayload("0BB8 09C4 0A2C 1770 01F4 03E8 10 13 00 00", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "buffer mode 02 parses");
			assertTrue(nearlyEqual(sSample.dSetpoint, 600.0), "buffer mode 02 setpoint");
			assertTrue(nearlyEqual(sSample.dControlOutput, 50.0), "buffer mode 02 control output");
			assertTrue(nearlyEqual(sSample.dSignalStrength, 100.0), "buffer mode 02 signal strength");
			assertTrue(sSample.nStatusByte == 0x10, "buffer mode 02 status byte");
			assertTrue(sSample.IsControlActive(), "buffer mode 02 control state");
			assertTrue(sSample.nIOByte == 0x13, "buffer mode 02 io byte");
			assertTrue(sSample.IsInputActive(1) && sSample.IsOutputActive(1), "buffer mode 02 io state");
		}

		void testInvalidPayload()
		{
			CSample sSample;

			assertFalse(parsePollPayload("ok", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "an acknowledge is no measurement");
			assertFalse(parsePollPayload("no", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "a rejection is no measurement");
			assertFalse(parsePollPayload("", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "an empty answer is no measurement");
			assertFalse(parsePollPayload("0BB80", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "a wrong length is no measurement");
			assertFalse(parsePollPayload("0BB8 09C4 XYZW", SENSORTHERM_TEMPERATURE_FACTOR, sSample), "invalid hexadecimal is rejected");

			uint32_t nValue = 0;
			assertTrue(parseHexNumber("0f00", nValue) && (nValue == 0x0F00), "lower case hexadecimal");
			assertFalse(parseHexNumber("xyz", nValue), "no hexadecimal");
			assertFalse(parseHexNumber("", nValue), "empty hexadecimal");

			assertTrue(std::isnan(convertTemperature(SENSORTHERM_OVERFLOW_MARKER, SENSORTHERM_TEMPERATURE_FACTOR)), "the overflow marker becomes not a number");
			assertTrue(nearlyEqual(convertTemperature(1234, SENSORTHERM_TEMPERATURE_FACTOR), 123.4), "a temperature is scaled");
		}

	};

}

#endif // __AMC_UNITTESTS_SENSORTHERM_PROTOCOL
