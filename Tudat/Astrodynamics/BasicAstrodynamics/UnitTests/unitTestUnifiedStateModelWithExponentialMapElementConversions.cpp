/*    Copyright (c) 2010-2018, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 *
 */

#define BOOST_TEST_MAIN

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

#include "Tudat/Astrodynamics/BasicAstrodynamics/orbitalElementConversions.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/unitConversions.h"
#include "Tudat/Basics/testMacros.h"
#include "Tudat/Mathematics/BasicMathematics/mathematicalConstants.h"
#include "Tudat/Mathematics/BasicMathematics/basicMathematicsFunctions.h"

#include "Tudat/Astrodynamics/BasicAstrodynamics/unifiedStateModelWithExponentialMapElementConversions.h"
#include "Tudat/Basics/basicTypedefs.h"
#include "Tudat/Astrodynamics/BasicAstrodynamics/stateVectorIndices.h"

namespace tudat
{
namespace unit_tests
{



//! Test the functionality of the time conversion functions.
BOOST_AUTO_TEST_SUITE( test_USMEM_Element_Conversions )

//! Unit test for conversion Keplerian orbital elements to Unified State Model elements.
BOOST_AUTO_TEST_CASE( testconvertKeplerianToUnifiedStateModelWithExponentialMapElements )
{
    using namespace orbital_element_conversions;
    using namespace unit_conversions;
    using mathematical_constants::PI;

    // Setting fraction tolerance for correctness evaluation
    double tolerance = 1.0E-14;

    // Declare gravitational parameter of central body
    const double centralBodyGravitationalParameter = 1.32712440018e20; // [m^3/s^2]

    // Initializing default Keplerian orbit
    Eigen::Vector6d keplerianElements = Eigen::Vector6d::Zero( 6 );
    keplerianElements( semiMajorAxisIndex ) = 1.5e11;
    keplerianElements( eccentricityIndex ) = 0.1;
    keplerianElements( inclinationIndex ) = convertDegreesToRadians( 50.0 );
    keplerianElements( argumentOfPeriapsisIndex ) = convertDegreesToRadians( 350.0 );
    keplerianElements( longitudeOfAscendingNodeIndex ) = convertDegreesToRadians( 15.0 );
    keplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 10.0 );

    // Unified state model element vector declaration
    Eigen::Vector6d expectedUnifiedStateModelElements = Eigen::Vector6d::Zero( );
    Eigen::Vector6d computedUnifiedStateModelElements = Eigen::Vector6d::Zero( );

    // Case 1: Elliptical prograde orbit (default case).
    {
        // Default case, so no modification necessary.

        // Expected unified state model elements [m/s,m/s,m/s,-,-,-].
        // (Results obtained using Matlab code).
        expectedUnifiedStateModelElements( CHodographExponentialMapIndex ) = 29894.5892222602;
        expectedUnifiedStateModelElements( Rf1HodographExponentialMapIndex ) = -260.548512780222;
        expectedUnifiedStateModelElements( Rf2HodographExponentialMapIndex ) = 2978.08312848463;
        expectedUnifiedStateModelElements( e1ExponentialMapIndex ) = -0.419002703925548;
        expectedUnifiedStateModelElements( e2ExponentialMapIndex ) = -0.0551627524676706;
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) = -0.118296904421275;

        // Compute unified state model elements.
        computedUnifiedStateModelElements =
                convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                               centralBodyGravitationalParameter );

        // Check if computed unified state model elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedUnifiedStateModelElements,
                                           computedUnifiedStateModelElements, tolerance );

    }




    // Case 2: Hyperbolic retrograde orbit.
    {
        // Modify Keplerian elements [m,-,rad,rad,rad,rad], i.e. overwrite them.
        keplerianElements( semiMajorAxisIndex ) = -1.5e11;
        keplerianElements( eccentricityIndex ) = 2.0;
        keplerianElements( inclinationIndex ) = convertDegreesToRadians( 170.0 );
        keplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 10.0 );

        // Set expected unified state model elements [m/s,m/s,m/s,-,-,-].
        // (Results obtained using Matlab code).
        expectedUnifiedStateModelElements( CHodographExponentialMapIndex ) = 17173.1340579794;
        expectedUnifiedStateModelElements( Rf1HodographExponentialMapIndex ) = -2993.47450825659;
        expectedUnifiedStateModelElements( Rf2HodographExponentialMapIndex ) = 34215.5701963558;
        expectedUnifiedStateModelElements( e1ExponentialMapIndex ) = -0.987672114350896; //Minor error?
        expectedUnifiedStateModelElements( e2ExponentialMapIndex ) = -0.130029500651719;
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) = -0.0113761072309622;

        // Compute unified state model elements.
        computedUnifiedStateModelElements =
                convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                               centralBodyGravitationalParameter );

        // Check if computed unified state model elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedUnifiedStateModelElements,
                                           computedUnifiedStateModelElements, tolerance );

    }


    // Case 3: Parabolic retrograde orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        keplerianElements( semiLatusRectumIndex ) = 1.5e11;
        keplerianElements( eccentricityIndex ) = 1.0;
        keplerianElements( inclinationIndex ) = convertDegreesToRadians( 170.0 );
        keplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 170.0 );

        // Set expected unified state model elements [m/s,m/s,m/s,-,-,-].
        // (Results obtained using Matlab code).
        expectedUnifiedStateModelElements( CHodographExponentialMapIndex ) = 29744.7407136119;
        expectedUnifiedStateModelElements( Rf1HodographExponentialMapIndex ) = -2592.42496973134;
        expectedUnifiedStateModelElements( Rf2HodographExponentialMapIndex ) = 29631.5529950138;
        expectedUnifiedStateModelElements( e1ExponentialMapIndex ) = -0.299561523151596;
        expectedUnifiedStateModelElements( e2ExponentialMapIndex ) = 0.95008776981561;
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) = -0.0870727897926938;

        // Compute unified state model elements.
        computedUnifiedStateModelElements =
                convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                               centralBodyGravitationalParameter );

        // Check if computed unified state model elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedUnifiedStateModelElements,
                                           computedUnifiedStateModelElements, tolerance );
    }

    // Case 4: Circular prograde orbit with non-zero argument of pericenter, test for error.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        keplerianElements( eccentricityIndex ) = 0.0;
            // Eccentricity is zero, while argument of pericenter is non-zero -> should give error
        keplerianElements( inclinationIndex ) = convertDegreesToRadians( 50.0 );

        // Declare variable indicating whether an exception has been thrown.
        bool isExceptionFound = false;

        // Try computing the unified state model elements and catch the expected runtime error.
        try
        {
            computedUnifiedStateModelElements =
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                                 centralBodyGravitationalParameter );
        }
        catch( std::runtime_error )
        {
            isExceptionFound = true;
        }

        // Check if runtime error has occured
        BOOST_CHECK( isExceptionFound );
    }

    // Case 5: 0 inclination orbit, test for error because longitude of ascending node is non-zero
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        keplerianElements( eccentricityIndex ) = 0.1;
        keplerianElements( inclinationIndex ) = 0.0;

        // Declare variable indicating whether an exception has been thrown.
        bool isExceptionFound = false;

        // Try computing the unified state model elements and catch the expected runtime error.
        try
        {
            computedUnifiedStateModelElements =
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                                 centralBodyGravitationalParameter );
        }
        catch( std::runtime_error )
        {
            isExceptionFound = true;
        }

        // Check if runtime error has occured
        BOOST_CHECK( isExceptionFound );
    }

    // Case 6: 180 inclination orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        keplerianElements( inclinationIndex ) = PI; // = 180 deg

        // Set expected unified state model elements [m/s,m/s,m/s,-,-,-]. (Results were calculated by
        // hand).
        expectedUnifiedStateModelElements( CHodographExponentialMapIndex ) = 29894.5892222602;
        expectedUnifiedStateModelElements( Rf1HodographExponentialMapIndex ) = -260.548512780222;
        expectedUnifiedStateModelElements( Rf2HodographExponentialMapIndex ) = 2978.08312848463;
        expectedUnifiedStateModelElements( e1ExponentialMapIndex ) = -0.300705799504273;
        expectedUnifiedStateModelElements( e2ExponentialMapIndex ) = 0.953716950748227;
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) = -6.11740603377039e-17;

        // Compute unified state model elements.
        computedUnifiedStateModelElements =
                convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                               centralBodyGravitationalParameter );

        // Because two elements are near-zero, a close fraction/percentage check will fail.
        // Therefore, 1.0 is added to the elements to avoid this
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) =
                expectedUnifiedStateModelElements( e3ExponentialMapIndex ) + 1.0;
        computedUnifiedStateModelElements( e3ExponentialMapIndex ) =
                computedUnifiedStateModelElements( e3ExponentialMapIndex ) + 1.0;

        // Check if computed elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedUnifiedStateModelElements,
                                           computedUnifiedStateModelElements, tolerance );
    }

    // Case 7: 0 eccentricity and inclination orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        keplerianElements( eccentricityIndex ) = 0.0;
        keplerianElements( inclinationIndex ) = 0.0;
        keplerianElements( longitudeOfAscendingNodeIndex ) = 0.0; // Default value because of zero inclination
        keplerianElements( argumentOfPeriapsisIndex ) = 0.0; // Default value because of zero eccentricity

        // Expected unified state model elements [m/s,m/s,m/s,-,-,-].
        // (Results obtained using code archive B. Romgens (2011)).
        expectedUnifiedStateModelElements( CHodographExponentialMapIndex ) = 29744.7407136119;
        expectedUnifiedStateModelElements( Rf1HodographExponentialMapIndex ) = 0;
        expectedUnifiedStateModelElements( Rf2HodographExponentialMapIndex ) = 0;
        expectedUnifiedStateModelElements( e1ExponentialMapIndex ) = 0;
        expectedUnifiedStateModelElements( e2ExponentialMapIndex ) = 0;
        expectedUnifiedStateModelElements( e3ExponentialMapIndex ) = 0.996194698091746;

        // Compute unified state model elements.
        computedUnifiedStateModelElements =
                convertKeplerianToUnifiedStateModelWithExponentialMapElements( keplerianElements,
                                                               centralBodyGravitationalParameter );

        // Check if computed elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedUnifiedStateModelElements,
                                           computedUnifiedStateModelElements, tolerance );
    }

    // Case 8: 200 degree inclination orbit, test for error.
    {
        keplerianElements( inclinationIndex ) = convertDegreesToRadians( 200.0 );
        bool isExceptionFound = false;

        // Try to convert Kepler to unified state model Elements
        try
        {
            computedUnifiedStateModelElements = convertKeplerianToUnifiedStateModelWithExponentialMapElements
                    ( keplerianElements, centralBodyGravitationalParameter );
        }
        // Catch the expected runtime error, and set the boolean flag to true.
        catch ( std::runtime_error )
        {
            isExceptionFound = true;
        }

        // Check value of flag.
        BOOST_CHECK( isExceptionFound );
    }
}


//! Unit test for the conversion of unified state model elements to Keplerian elements
BOOST_AUTO_TEST_CASE( testconvertUnifiedStateModelWithExponentialMapToKeplerianElements )
{
    /* Used procedure:
      Because the Kepler to unified state model elements are verified, a subsequent conversion back
      to Keplerian elements should yield the same outcome as the input Keplerian state. This
      principle is used for verification.
     */

    using namespace orbital_element_conversions;
    using namespace unit_conversions;
    using mathematical_constants::PI;

    // Setting fraction tolerance for correctness evaluation
    double tolerance = 1.0E-14;

    // Declare gravitational parameter of central body
    const double centralBodyGravitationalParameter = 1.32712440018e20; // [m^3/s^2]

    // Initializing default Keplerian orbit
    Eigen::Vector6d expectedKeplerianElements = Eigen::Vector6d::Zero( 6 );
    expectedKeplerianElements( semiMajorAxisIndex ) = 1.5e11;
    expectedKeplerianElements( eccentricityIndex ) = 0.1;
    expectedKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 50.0 );
    expectedKeplerianElements( argumentOfPeriapsisIndex ) = convertDegreesToRadians( 350.0 );
    expectedKeplerianElements( longitudeOfAscendingNodeIndex ) = convertDegreesToRadians( 15.0 );
    expectedKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 170.0 );

    // Declaring computed output vector.
    Eigen::Vector6d computedKeplerianElements = Eigen::Vector6d::Zero( 6 );



    // Case 1: Elliptical prograde orbit (default case).
    {
        // Default case, so no modification necessary.

        // Convert to unified state model elements and back.
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }

    // Case 2: Hyperbolic retrograde orbit.
    {
        // Modify Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( semiMajorAxisIndex ) = -1.5e11;
        expectedKeplerianElements( eccentricityIndex ) = 2.0;
        expectedKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 160.0 );
        expectedKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 10.0 ); // 170 is above limit

        // Convert to unified state model elements and back.
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }

    // Case 3: Parabolic retrograde orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( semiLatusRectumIndex ) = 3.5e11;
        expectedKeplerianElements( eccentricityIndex ) = 1.0;
        expectedKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 90.0 );

        // Convert to unified state model elements and back.
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
       }

    // Case 4: Circular prograde orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( semiMajorAxisIndex ) = 3.5e11;
        expectedKeplerianElements( eccentricityIndex ) = 0.0;
        expectedKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 70.0 );
        expectedKeplerianElements( argumentOfPeriapsisIndex ) = 0.0; // For e = 0, undefined.

        // Convert to unified state model elements and back.
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }

    // Case 5: 0 inclination orbit,
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( eccentricityIndex ) = 0.3;
        expectedKeplerianElements( inclinationIndex ) = 0.0;
        expectedKeplerianElements( longitudeOfAscendingNodeIndex ) = 0.0; // Set to zero as for
        // non-inclined orbit planes, this parameter is undefined

        // Convert to unified state model elements and back.
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }

    // Case 6: 180 inclination orbit, test for error.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( semiMajorAxisIndex ) = 1.5e15;
        expectedKeplerianElements( inclinationIndex ) = PI;
        expectedKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 240.0 );

        // Declare variable indicating whether an exception has been thrown.
        bool isExceptionFound = false;

        // Try convert to unified state model elements and back and catch the expected runtime error.
        try
        {
            computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                 centralBodyGravitationalParameter ),
                        centralBodyGravitationalParameter );
        }
        catch( std::runtime_error )
        {
            isExceptionFound = true;
        }

        // Check if runtime error has occured
        BOOST_CHECK( isExceptionFound );
    }

    // Case 7: 0 eccentricity and inclination orbit.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( eccentricityIndex ) = 0.0;
            // argument of pericenter was set to 0 in case 4, so no error.
        expectedKeplerianElements( inclinationIndex ) = 0.0;
            // longitude of ascending node was set to 0 in case 5, so no error.

        // Convert to unified state model elements and back
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }

    // Case 8: true anomaly exceeding 180 degrees.
    {
        // Set Keplerian elements [m,-,rad,rad,rad,rad].
        expectedKeplerianElements( semiMajorAxisIndex ) = 1.5e11;
        expectedKeplerianElements( eccentricityIndex ) = 0.1;
        expectedKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 50.0 );
        expectedKeplerianElements( argumentOfPeriapsisIndex ) = convertDegreesToRadians( 350.0 );
        expectedKeplerianElements( longitudeOfAscendingNodeIndex ) = convertDegreesToRadians( 15.0 );
        expectedKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 240.0 );

        // Convert to unified state model elements and back
        computedKeplerianElements = convertUnifiedStateModelWithExponentialMapToKeplerianElements(
                    convertKeplerianToUnifiedStateModelWithExponentialMapElements( expectedKeplerianElements,
                                                                   centralBodyGravitationalParameter ),
                    centralBodyGravitationalParameter );

        // Check if computed Keplerian elements match the expected values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION( expectedKeplerianElements,
                                           computedKeplerianElements, tolerance );
    }
}
BOOST_AUTO_TEST_SUITE_END( )

} // end namespace unit_tests
} // end namespace tudat
