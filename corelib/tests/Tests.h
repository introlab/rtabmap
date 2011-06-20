/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TESTS_H
#define TESTS_H

#include <cppunit/TestFixture.h>
#include <cppunit/TestCaller.h>

#include <cppunit/extensions/HelperMacros.h>

#include "utilite/UDirectory.h"
#include "utilite/ULogger.h"

class Tests : public CppUnit::TestFixture  {

  CPPUNIT_TEST_SUITE( Tests );
  CPPUNIT_TEST( testAvpd );
  CPPUNIT_TEST( testDBDriverFactory );
  CPPUNIT_TEST( testSqlite3Database );
  CPPUNIT_TEST( testBayesFilter );
  CPPUNIT_TEST( testKeypointMemory );
  CPPUNIT_TEST( testCamera );
  CPPUNIT_TEST( testVWDictionary );
  CPPUNIT_TEST( testVerifyHypotheses );
  CPPUNIT_TEST_SUITE_END();

private:
public:
  void setUp()
  {
	  if(!UDirectory::exists("./LogTestLibCore"))
	  {
		  UDirectory::makeDir("./LogTestLibCore");
	  }

	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //Util::Logger::setLevel(Util::Logger::kDebug);
  }

  void tearDown() 
  {
	  ULogger::reset();
	  ULogger::setType(ULogger::kTypeConsole);
	  ULogger::setLevel(ULogger::kError);
	  //Util::Logger::setLevel(Util::Logger::kDebug);
  }

  void testAvpd();
  void testCamera();
  void testDBDriverFactory();
  void testSqlite3Database();
  void testBayesFilter();
  void testKeypointMemory();
  void testVWDictionary();
  void testVerifyHypotheses();
};

#endif
