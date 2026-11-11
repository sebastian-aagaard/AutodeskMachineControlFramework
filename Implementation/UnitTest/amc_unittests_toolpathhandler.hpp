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


#ifndef __AMCTEST_UNITTEST_TOOLPATHHANDLER
#define __AMCTEST_UNITTEST_TOOLPATHHANDLER

#include "amc_unittests.hpp"
#include "amc_unittests_libmcdata.hpp"
#include "amc_toolpathhandler.hpp"
#include "common_utils.hpp"
#include "libmc_exceptiontypes.hpp"

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace AMCUnitTest {

	class CUnitTestGroup_ToolpathHandler : public CUnitTestGroup {
	public:
		std::string getTestGroupName() override {
			return "ToolpathHandler";
		}

		void registerTests() override {
			registerTest("AcquireNotLoaded", "acquiring a toolpath that is not loaded returns null", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathHandler::testAcquireNotLoaded, this));
			registerTest("UnloadThrowsNotLoaded", "unloading a toolpath that is not loaded throws", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathHandler::testUnloadThrowsNotLoaded, this));
			registerTest("UnloadAllEmpty", "unloading all entities on an empty handler succeeds", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathHandler::testUnloadAllEmpty, this));
			registerTest("ConcurrentAccess", "concurrent acquire, find and unload do not race", eUnitTestCategory::utMandatoryPass, std::bind(&CUnitTestGroup_ToolpathHandler::testConcurrentAccess, this));
		}

		void initializeTests() override {
		}

	private:

		struct SToolpathHandlerFixture {
			LibMCData::PWrapper m_pDataWrapper;
			LibMCData::PDataModel m_pDataModel;
			AMC::PToolpathHandler m_pToolpathHandler;
			std::string m_sBasePath;
		};

		SToolpathHandlerFixture createFixture(const std::string& sSuffix)
		{
			SToolpathHandlerFixture fixture;

			std::string sRootPath = "temp";
			if (!AMCCommon::CUtils::fileOrPathExistsOnDisk(sRootPath))
				AMCCommon::CUtils::createDirectoryOnDisk(sRootPath);

			fixture.m_sBasePath = sRootPath + "/toolpathhandler_" + sSuffix + "_" + AMCCommon::CUtils::createUUID();
			if (!AMCCommon::CUtils::fileOrPathExistsOnDisk(fixture.m_sBasePath))
				AMCCommon::CUtils::createDirectoryOnDisk(fixture.m_sBasePath);

			std::string sDatabaseFile = fixture.m_sBasePath + "/datamodel.db";

			fixture.m_pDataWrapper = AMCUnitTest::loadLibMCDataInProcess();
			fixture.m_pDataModel = fixture.m_pDataWrapper->CreateDataModelInstance();
			fixture.m_pDataModel->InitialiseDatabase(fixture.m_sBasePath, LibMCData::eDataBaseType::SqLite, sDatabaseFile);

			fixture.m_pToolpathHandler = std::make_shared<AMC::CToolpathHandler>(fixture.m_pDataModel);

			return fixture;
		}

		void testAcquireNotLoaded()
		{
			auto fixture = createFixture("acquire_notloaded");

			auto pToolpathEntity = fixture.m_pToolpathHandler->acquireToolpathEntity(AMCCommon::CUtils::createUUID());
			assertNull(pToolpathEntity.get(), "acquiring an unloaded toolpath should return nullptr");
		}

		void testUnloadThrowsNotLoaded()
		{
			auto fixture = createFixture("unload_throws");

			bool bDidThrow = false;
			try {
				fixture.m_pToolpathHandler->unloadToolpathEntity(AMCCommon::CUtils::createUUID());
			}
			catch (ELibMCCustomException& E) {
				bDidThrow = (E.getErrorCode() == LIBMC_ERROR_TOOLPATHENTITYNOTLOADED);
			}
			assertTrue(bDidThrow, "unloading an unloaded toolpath should throw TOOLPATHENTITYNOTLOADED");
		}

		void testUnloadAllEmpty()
		{
			auto fixture = createFixture("unloadall_empty");

			fixture.m_pToolpathHandler->unloadAllEntities();
			auto pToolpathEntity = fixture.m_pToolpathHandler->acquireToolpathEntity(AMCCommon::CUtils::createUUID());
			assertNull(pToolpathEntity.get(), "unloading all entities on an empty handler should succeed");
		}

		void testConcurrentAccess()
		{
			auto fixture = createFixture("concurrent");

			// Hammer the entity map from several threads while it is being cleared, mirroring
			// HTTP request threads looking up toolpath entities while an execution finishes
			// and unloads all of them. Any unsynchronized map access would crash or corrupt.
			std::atomic<bool> bStop(false);
			std::atomic<int> nExceptions(0);
			std::vector<std::thread> vThreads;

			for (int nIndex = 0; nIndex < 4; nIndex++) {
				vThreads.emplace_back([&fixture, &bStop, &nExceptions, nIndex]() {
					try {
						while (!bStop.load()) {
							auto pToolpathEntity = fixture.m_pToolpathHandler->acquireToolpathEntity("01a10000-0000-0000-0000-000000000000");
							(void)pToolpathEntity;
							if (nIndex == 0)
								fixture.m_pToolpathHandler->unloadAllEntities();
							if (nIndex == 1) {
								try {
									fixture.m_pToolpathHandler->unloadToolpathEntity("01a10000-0000-0000-0000-000000000000");
								}
								catch (ELibMCCustomException&) {
								}
							}
						}
					}
					catch (...) {
						nExceptions++;
					}
				});
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			bStop.store(true);
			for (auto& thread : vThreads)
				thread.join();

			assertTrue(nExceptions.load() == 0, "concurrent toolpath handler access should not raise unexpected exceptions");
		}

	};

}

#endif //__AMCTEST_UNITTEST_TOOLPATHHANDLER

