/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  
  Author(s):  Peter Kazanzides
  Created on: 2018-12-16
  
  (C) Copyright 2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>


class cmnLoggerTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE(cmnLoggerTest);
    {
        CPPUNIT_TEST(TestLoggerFileName);
    }
    CPPUNIT_TEST_SUITE_END();
    
 public:
    void setUp(void) {
    }
    
    void tearDown(void) {
    }
    
    void TestLoggerFileName(void);
};


CPPUNIT_TEST_SUITE_REGISTRATION(cmnLoggerTest);
