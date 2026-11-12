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


#ifndef __AMCTEST_UNITTEST_TOOLPATHLAYERDATA
#define __AMCTEST_UNITTEST_TOOLPATHLAYERDATA

#include "amc_unittests.hpp"
#include "amc_toolpathlayerdata.hpp"
#include "libmc_exceptiontypes.hpp"

#include <string>
#include <vector>

namespace AMCUnitTest {


	// A 3MF Toolpath profile maps a modifier factor to a profile attribute with
	//     value = minvalue + factor * (maxvalue - minvalue)
	// so that the layer data never carries absolute machine values. The factors are written either onto every
	// geometry element or, as the less granular form the specification allows, once onto the <segment>
	// element. Lib3MF reports only the geometry level factors through its typed layer reader, so the segment
	// level form has to be picked up from the layer part itself. The JSON fixtures below are the layer
	// viewable strings that lib3mf produces for such layer parts.
	class CUnitTestGroup_ToolpathLayerData : public CUnitTestGroup {
	public:
		std::string getTestGroupName() override {
			return "ToolpathLayerData";
		}

		void registerTests() override {
			registerTest("SegmentLevelConstantFactor", "a constant factor on a segment element is found", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathLayerData::testSegmentLevelConstantFactor, this));
			registerTest("AllSegmentTypesAreOrdered", "every segment of the layer part is reported in document order", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathLayerData::testAllSegmentTypesAreOrdered, this));
			registerTest("GeometryLevelFactorsAreSkipped", "factors on the geometry elements are left to the typed reader", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathLayerData::testGeometryLevelFactorsAreSkipped, this));
			registerTest("IncompleteAndInvalidFactors", "incomplete factor pairs are ignored and invalid factors are rejected", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathLayerData::testIncompleteAndInvalidFactors, this));
		}

		void initializeTests() override {
		}

	private:

		// A hatch segment that carries one constant factor for the whole segment, which is the case of
		// <segment type="hatch" profileid="1" partid="2" f="0.602143">.
		static std::string getSegmentLevelFactorLayerJSON()
		{
			return "{\"layer\":{\"xmlns\":\"http://schemas.3mf.io/3dmanufacturing/toolpath/2026/03\",\"_parts\":{\"_part\":{\"id\":2,\"uuid\":\"36da8553-e928-4f98-cfb4-97a74f602bde\",\"_profiles\":{\"_profile\":{\"id\":1,\"uuid\":\"c898e0ce-2775-43ad-af74-710e46b15df5\",\"_data\":{},\"_segments\":{\"_segment\":{\"type\":\"hatch\",\"profileid\":1,\"partid\":2,\"f\":0.602143,\"_hatch\":{\"x1\":99572,\"y1\":9888,\"x2\":99665,\"y2\":9796,\"_hatch\":{\"x1\":99597,\"y1\":9722,\"x2\":99431,\"y2\":9888,\"_hatch\":{\"x1\":99289,\"y1\":9888,\"x2\":99529,\"y2\":9648,\"_hatch\":{\"x1\":99461,\"y1\":9574,\"x2\":99148,\"y2\":9888}}}}}}}}}}}}";
		}

		// A hatch segment with a constant factor, a loop segment with a linear factor, a polyline segment with
		// a constant g factor, and a delay and a sync segment, which carry no geometry but occupy an index.
		static std::string getMixedSegmentTypesLayerJSON()
		{
			return "{\"layer\":{\"xmlns\":\"http://schemas.3mf.io/3dmanufacturing/toolpath/2026/03\",\"_parts\":{\"_part\":{\"id\":2,\"uuid\":\"36da8553-e928-4f98-cfb4-97a74f602bde\",\"_profiles\":{\"_profile\":{\"id\":1,\"uuid\":\"c898e0ce-2775-43ad-af74-710e46b15df5\",\"_data\":{},\"_segments\":{\"_segment\":{\"type\":\"hatch\",\"profileid\":1,\"partid\":1,\"f\":0.602143,\"_hatch\":{\"x1\":99572,\"y1\":9888,\"x2\":99665,\"y2\":9796,\"_hatch\":{\"x1\":99597,\"y1\":9722,\"x2\":99431,\"y2\":9888,\"_segment\":{\"type\":\"loop\",\"profileid\":1,\"partid\":1,\"f1\":0.1,\"f2\":0.9,\"_point\":{\"x\":100,\"y\":100,\"_point\":{\"x\":200,\"y\":100,\"_point\":{\"x\":200,\"y\":200,\"_segment\":{\"type\":\"polyline\",\"profileid\":1,\"partid\":1,\"g\":0.5,\"_point\":{\"x\":10,\"y\":10,\"_point\":{\"x\":20,\"y\":20,\"_segment\":{\"type\":\"delay\",\"profileid\":1,\"partid\":1,\"_segment\":{\"type\":\"sync\",\"profileid\":1,\"partid\":1,\"lasersync\":1}}}}}}}}}}}}}}}}}}}";
		}

		// The same geometry, but with the factors on the hatch elements. Those factors are reported by the
		// typed layer reader and MUST NOT be reported as segment level factors.
		static std::string getGeometryLevelFactorLayerJSON()
		{
			return "{\"layer\":{\"xmlns\":\"http://schemas.3mf.io/3dmanufacturing/toolpath/2026/03\",\"_parts\":{\"_part\":{\"id\":2,\"uuid\":\"36da8553-e928-4f98-cfb4-97a74f602bde\",\"_profiles\":{\"_profile\":{\"id\":1,\"uuid\":\"c898e0ce-2775-43ad-af74-710e46b15df5\",\"_data\":{},\"_segments\":{\"_segment\":{\"type\":\"hatch\",\"profileid\":1,\"partid\":2,\"_hatch\":{\"x1\":99572,\"y1\":9888,\"x2\":99665,\"y2\":9796,\"_hatch\":{\"x1\":99597,\"y1\":9722,\"x2\":99431,\"y2\":9888,\"_hatch\":{\"x1\":99289,\"y1\":9888,\"x2\":99529,\"y2\":9648,\"_hatch\":{\"x1\":99461,\"y1\":9574,\"x2\":99148,\"y2\":9888,\"f\":0.602143},\"f\":0.602143},\"f\":0.602143},\"f\":0.602143}}}}}}}}}";
		}

		static std::string getLayerJSONWithSegmentAttributes(const std::string& sSegmentAttributes)
		{
			return "{\"layer\":{\"_segments\":{\"_segment\":{" + sSegmentAttributes + ",\"profileid\":1,\"partid\":1,\"_hatch\":{\"x1\":1,\"y1\":2,\"x2\":3,\"y2\":4}}}}}";
		}

		static void parse(const std::string& sLayerJSON, std::vector<AMC::sToolpathSegmentFactorAttributes>& segmentLevelFactors, bool& bHasAnyFactorAttribute)
		{
			AMC::CToolpathLayerData::parseSegmentLevelFactorsFromLayerJSON(sLayerJSON, segmentLevelFactors, bHasAnyFactorAttribute);
		}


		void testSegmentLevelConstantFactor()
		{
			std::vector<AMC::sToolpathSegmentFactorAttributes> segmentLevelFactors;
			bool bHasAnyFactorAttribute = true;
			parse(getSegmentLevelFactorLayerJSON(), segmentLevelFactors, bHasAnyFactorAttribute);

			assertTrue(segmentLevelFactors.size() == 1, "the layer part must have exactly one segment");
			assertTrue(bHasAnyFactorAttribute, "the segment must be reported to carry a factor attribute");

			// Factor slots are F, G, H and E, in this order.
			const AMC::sToolpathSegmentFactorAttributes& segmentFactors = segmentLevelFactors.at(0);
			assertTrue(segmentFactors.m_bHasConstantFactor[0], "the segment must carry a constant f factor");
			assertDoubleRange(segmentFactors.m_dConstantFactor[0], 0.602142, 0.602144, "the f factor must be the value of the segment element");
			assertFalse(segmentFactors.m_bHasLinearFactor[0], "a constant factor must not be reported as a linear factor");
			assertFalse(segmentFactors.m_bHasConstantFactor[1], "the segment must not carry a g factor");
			assertFalse(segmentFactors.m_bHasConstantFactor[2], "the segment must not carry a h factor");
			assertFalse(segmentFactors.m_bHasConstantFactor[3], "the segment must not carry an e factor");
		}


		void testAllSegmentTypesAreOrdered()
		{
			std::vector<AMC::sToolpathSegmentFactorAttributes> segmentLevelFactors;
			bool bHasAnyFactorAttribute = false;
			parse(getMixedSegmentTypesLayerJSON(), segmentLevelFactors, bHasAnyFactorAttribute);

			// Segments without geometry still occupy a segment index, otherwise the factors of the following
			// segments would be applied to the wrong segment.
			assertTrue(segmentLevelFactors.size() == 5, "all five segments must be reported");
			assertTrue(bHasAnyFactorAttribute, "the layer part must be reported to carry factor attributes");

			assertTrue(segmentLevelFactors.at(0).m_bHasConstantFactor[0], "the hatch segment must carry a constant f factor");
			assertFalse(segmentLevelFactors.at(1).m_bHasConstantFactor[0], "the loop segment must not carry a constant f factor");
			assertTrue(segmentLevelFactors.at(1).m_bHasLinearFactor[0], "the loop segment must carry a linear f factor");
			assertDoubleRange(segmentLevelFactors.at(1).m_dStartLinearFactor[0], 0.099999, 0.100001, "the start of the loop ramp must be the f1 factor");
			assertDoubleRange(segmentLevelFactors.at(1).m_dEndLinearFactor[0], 0.899999, 0.900001, "the end of the loop ramp must be the f2 factor");
			assertTrue(segmentLevelFactors.at(2).m_bHasConstantFactor[1], "the polyline segment must carry a constant g factor");
			assertDoubleRange(segmentLevelFactors.at(2).m_dConstantFactor[1], 0.499999, 0.500001, "the g factor must be the value of the segment element");
			assertFalse(segmentLevelFactors.at(3).m_bHasConstantFactor[0], "the delay segment must not carry a factor");
			assertFalse(segmentLevelFactors.at(4).m_bHasConstantFactor[0], "the sync segment must not carry a factor");
		}


		void testGeometryLevelFactorsAreSkipped()
		{
			std::vector<AMC::sToolpathSegmentFactorAttributes> segmentLevelFactors;
			bool bHasAnyFactorAttribute = true;
			parse(getGeometryLevelFactorLayerJSON(), segmentLevelFactors, bHasAnyFactorAttribute);

			assertTrue(segmentLevelFactors.size() == 1, "the layer part must have exactly one segment");
			assertFalse(segmentLevelFactors.at(0).m_bHasConstantFactor[0], "the factor of a hatch element must not be reported as a segment factor");
			assertFalse(bHasAnyFactorAttribute, "a layer that carries its factors on the geometry elements must not report any segment factor");
		}


		void testIncompleteAndInvalidFactors()
		{
			std::vector<AMC::sToolpathSegmentFactorAttributes> segmentLevelFactors;
			bool bHasAnyFactorAttribute = false;

			// A linear factor needs both of its values. A single value is not enough to ramp, but it does tell
			// that the layer part carries segment level factors at all.
			parse(getLayerJSONWithSegmentAttributes("\"type\":\"polyline\",\"f1\":0.2"), segmentLevelFactors, bHasAnyFactorAttribute);
			assertTrue(segmentLevelFactors.size() == 1, "the layer part must have exactly one segment");
			assertFalse(segmentLevelFactors.at(0).m_bHasLinearFactor[0], "an incomplete linear factor must not be used");
			assertTrue(bHasAnyFactorAttribute, "an incomplete linear factor must still be reported as a factor attribute");

			// A factor is a normalized scale and MUST be between 0 and 1.
			bool bDidThrow = false;
			try {
				parse(getLayerJSONWithSegmentAttributes("\"type\":\"hatch\",\"f\":1.5"), segmentLevelFactors, bHasAnyFactorAttribute);
			}
			catch (ELibMCCustomException& E) {
				bDidThrow = (E.getErrorCode() == LIBMC_ERROR_INVALIDTOOLPATHMODIFIERFACTOR);
			}
			assertTrue(bDidThrow, "a factor out of range must be rejected as an invalid modifier factor");

			// A layer part that cannot be read must not be mistaken for a layer that carries modifiers.
			segmentLevelFactors.clear();
			bHasAnyFactorAttribute = true;
			parse("", segmentLevelFactors, bHasAnyFactorAttribute);
			assertTrue(segmentLevelFactors.empty() && (!bHasAnyFactorAttribute), "no layer part must report no factor at all");

			segmentLevelFactors.clear();
			bHasAnyFactorAttribute = true;
			parse("{\"layer\":{", segmentLevelFactors, bHasAnyFactorAttribute);
			assertTrue(segmentLevelFactors.empty() && (!bHasAnyFactorAttribute), "an unparseable layer part must report no factor at all");
		}

	};

}

#endif //__AMCTEST_UNITTEST_TOOLPATHLAYERDATA
