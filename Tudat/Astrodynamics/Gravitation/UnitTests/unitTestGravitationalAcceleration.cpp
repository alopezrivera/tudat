/*    Copyright (c) 2010-2012, Delft University of Technology
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without modification, are
 *    permitted provided that the following conditions are met:
 *      - Redistributions of source code must retain the above copyright notice, this list of
 *        conditions and the following disclaimer.
 *      - Redistributions in binary form must reproduce the above copyright notice, this list of
 *        conditions and the following disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *      - Neither the name of the Delft University of Technology nor the names of its contributors
 *        may be used to endorse or promote products derived from this software without specific
 *        prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 *    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *    MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 *    OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *    Changelog
 *      YYMMDD    Author            Comment
 *      120226    K. Kumar          File created.
 *      120725    K. Kumar          Added unit test based on MATLAB's gravityzonal() function for
 *                                  zonal terms up to J4.
 *      120815    K. Kumar          Added unit tests for individual zonal terms based on
 *                                  (Melman, 2012) and (Ronse, 2012).
 *
 *    References
 *      Easy calculation. Gravitational Acceleration Tutorial,
 *          http://easycalculation.com/physics/classical-physics
 *          /learn-gravitational-acceleration.php, last accessed: 26th February, 2012.
 *      MathWorks. gravityzonal, MATLAB 2012b, 2012.
 *      Melman, J. Propagate software, J.C.P.Melman@tudelft.nl, 2012.
 *      Ronse, A. A parametric study of space debris impact footprints, MSc thesis, Delft
 *          University of Technlogy, Delft, The Netherlands, in preparation.
 *
 *    Notes
 *
 */

#define BOOST_TEST_MAIN

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

#include <TudatCore/Astrodynamics/BasicAstrodynamics/astrodynamicsFunctions.h>
#include <TudatCore/Astrodynamics/BasicAstrodynamics/orbitalElementConversions.h>
#include <TudatCore/Basics/testMacros.h>
#include <TudatCore/Mathematics/BasicMathematics/mathematicalConstants.h>

#include "Tudat/Astrodynamics/Gravitation/gravitationalAccelerationModel.h"
#include "Tudat/Astrodynamics/Gravitation/UnitTests/planetTestData.h"
#include "Tudat/Mathematics/NumericalIntegrators/rungeKuttaCoefficients.h"
#include "Tudat/Mathematics/NumericalIntegrators/rungeKuttaVariableStepSizeIntegrator.h"

namespace tudat
{
namespace unit_tests
{

using namespace boost::assign;
using tudat::astrodynamics::acceleration_models::computeGravitationalAccelerationZonalSum;
using tudat::astrodynamics::acceleration_models::computeGravitationalAcceleration;
using tudat::astrodynamics::acceleration_models::computeGravitationalAccelerationDueToJ2;
using tudat::astrodynamics::acceleration_models::computeGravitationalAccelerationDueToJ3;
using tudat::astrodynamics::acceleration_models::computeGravitationalAccelerationDueToJ4;

typedef std::map< int, double > KeyIntValueDoubleMap;

BOOST_AUTO_TEST_SUITE( test_gravitational_acceleration )

//! Test if gravitational acceleration is computed correctly.
BOOST_AUTO_TEST_CASE( testGravitationalAcceleration )
{
    // Test 1: compute gravitational acceleration exerted on surface of Earth
    //         (Easy calculation, 2012).
    {
        // Set gravitational parameter of Earth [m^3 s^-2].
        const double gravitationalParameterOfEarth = 6.6726e-11 * 5.9742e24;

        // Set position vector of Earth [m].
        const Eigen::Vector3d positionOfEarth = Eigen::Vector3d::Zero( );

        // Set position vector on Earth surface [m].
        const Eigen::Vector3d positionOnEarthSurface( 6.3781e6, 0.0, 0.0 );

        // Compute gravitational accelerating acting on Earth's surface [N].
        const Eigen::Vector3d gravitationalAccelerationExertedAtEarthSurface
                = astrodynamics::acceleration_models::computeGravitationalAcceleration(
                    positionOnEarthSurface, gravitationalParameterOfEarth, positionOfEarth );

        // Check if computed gravitational force matches expected value.
        BOOST_CHECK_CLOSE_FRACTION( 9.8, gravitationalAccelerationExertedAtEarthSurface.norm( ),
                                    1.0e-4 );
    }

    // Test 2: compute gravitational acceleration exerted on Lunar surface
    //         (Easy calculation, 2012).
    {
        // Set universal gravitational constant [m^3 kg^-1 s^-2].
        const double universalGravitationalConstant = 6.6726e-11;

        // Set mass of Moon [kg].
        const double massOfMoon = 7.36e22;

        // Set position vector of Moon [m].
        const Eigen::Vector3d positionOfMoon( 12.65, 0.23, -45.78 );

        // Set position vector on surface of Moon [m].
        const Eigen::Vector3d positionOfLunarSurface( 0.0, 1735771.89, 0.0 );

        // Compute gravitational accelerating acting on Lunar surface [N].
        const Eigen::Vector3d gravitationalAccelerationExertedAtLunarSurface
                = astrodynamics::acceleration_models::computeGravitationalAcceleration(
                    universalGravitationalConstant, positionOfLunarSurface,
                    massOfMoon, positionOfMoon );

        // Check if computed gravitational force matches expected value.
        BOOST_CHECK_CLOSE_FRACTION( 1.63, gravitationalAccelerationExertedAtLunarSurface.norm( ),
                                    1.0e-6 );
    }
}

//! Test if gravitational acceleration sum due to zonal terms is computed correctly using MATLAB.
BOOST_AUTO_TEST_CASE( testGravitationalAccelarationSumZonalMatlab )
{
    // These tests check if total acceleration due to zonal terms is computed correctly by
    // comparing to output generated using gravityzonal() function in MATLAB (Mathworks, 2012).
    // The planet data used is obtained from the documentation for the gravityzonal() function.

    // Get planet test data.
    std::vector< PlanetTestData > planetData = getPlanetMatlabTestData( );

    // Loop over all planet test data and recompute the results using Tudat code. Check that the
    // values computed match MATLAB's output (Mathworks, 2012).
    for ( unsigned int planet = 0; planet < planetData.size( ); planet++ )
    {
        for ( unsigned int body1 = 0; body1 < planetData.at( planet ).body1Positions.size( );
              body1++ )
        {
            for ( unsigned int body2 = 0; body2 < planetData.at( planet ).body2Positions.size( );
                  body2++ )
            {
                // Compute central gravitational acceleration term [m s^-2].
                const Eigen::Vector3d computedCentralAcceleration
                        = computeGravitationalAcceleration(
                            planetData.at( planet ).body2Positions.at( body2 ),
                            planetData.at( planet ).gravitationalParameter,
                            planetData.at( planet ).body1Positions.at( body1 ) );

                // Check that the computed central gravitational acceleration matches the expected
                // values.
                TUDAT_CHECK_MATRIX_CLOSE_FRACTION(
                            planetData.at( planet ).expectedAcceleration[ body1 ][ body2 ][
                            central ],
                            computedCentralAcceleration,
                            1.0e-15 );

                // Declare zonal coefficients used.
                KeyIntValueDoubleMap zonalCoefficients;

                // Loop over all available zonal gravity field coefficients.
                for ( KeyIntValueDoubleMap::iterator zonalCoefficientIterator
                      = planetData.at( planet ).zonalCoefficients.begin( );
                      zonalCoefficientIterator
                      != planetData.at( planet ).zonalCoefficients.end( );
                      zonalCoefficientIterator++ )
                {
                    // Add current zonal coefficient to local list.
                    zonalCoefficients[ zonalCoefficientIterator->first ]
                            = zonalCoefficientIterator->second;

                    // Compute gravitational acceleration sum [m s^-2].
                    const Eigen::Vector3d computedAccelerationSum
                            = computeGravitationalAccelerationZonalSum(
                                planetData.at( planet ).body2Positions.at( body2 ),
                                planetData.at( planet ).gravitationalParameter,
                                zonalCoefficients,
                                planetData.at( planet ).effectiveRadius,
                                planetData.at( planet ).body1Positions.at( body1 ) );

                    // Check that computed gravitational acceleration sum matches expected values.
                    TUDAT_CHECK_MATRIX_CLOSE_FRACTION(
                                planetData.at( planet ).expectedAcceleration[ body1 ][ body2 ][
                                zonalCoefficientIterator->first ],
                                computedAccelerationSum,
                                1.0e-15 );
                }
            }
        }
    }
}

//! Test if gravitational acceleration due to zonal terms is computed correctly (Melman, 2012).
BOOST_AUTO_TEST_CASE( testGravitationalAccelarationZonalMelman )
{
    typedef Eigen::Vector3d ( *GravitationalAccelerationPointer )(
                const Eigen::Vector3d&, const double, const double, const double,
                const Eigen::Vector3d& );

    // These tests check if acceleration due to zonal terms is computed correctly by comparing to
    // output generated by (Melman, 2012).

    // Get planet test data.
    PlanetTestData earthData = getEarthMelmanTestData( );

    // Set map of function pointers for zonal coefficients.
    std::map< int, GravitationalAccelerationPointer > zonalGravitationalAccelerationPointers
            = map_list_of( 2, &computeGravitationalAccelerationDueToJ2 )
                         ( 3, &computeGravitationalAccelerationDueToJ3 )
                         ( 4, &computeGravitationalAccelerationDueToJ4 );

    // Loop over all planet test data and recompute the results using Tudat code. Check that the
    // values computed match results obtained by (Melman, 2012).
    for ( unsigned int body2 = 0; body2 < earthData.body2Positions.size( ); body2++ )
    {
        // Loop over all available zonal gravity field coefficients.
        for ( KeyIntValueDoubleMap::iterator zonalCoefficientIterator
              = earthData.zonalCoefficients.begin( );
              zonalCoefficientIterator != earthData.zonalCoefficients.end( );
              zonalCoefficientIterator++ )
        {
            // Compute gravitational acceleration due to given zonal term [m s^-2].
            const Eigen::Vector3d computedZonalGravitationalAcceleration
                    = zonalGravitationalAccelerationPointers[ zonalCoefficientIterator->first ](
                        earthData.body2Positions.at( body2 ),
                        earthData.gravitationalParameter,
                        zonalCoefficientIterator->second,
                        earthData.effectiveRadius,
                        earthData.body1Positions.at( 0 ) );

            // Check that computed gravitational acceleration sum matches expected values.
            TUDAT_CHECK_MATRIX_CLOSE_FRACTION( earthData.expectedAcceleration[ 0 ][ body2 ][
                                               zonalCoefficientIterator->first ],
                                               computedZonalGravitationalAcceleration,
                                               1.0e-14 );
        }
    }
}

//! Test if gravitational acceleration due to zonal terms is computed correctly (Ronse, 2012).
BOOST_AUTO_TEST_CASE( testGravitationalAccelarationZonalRonse )
{
    typedef Eigen::Vector3d ( *GravitationalAccelerationPointer )(
                const Eigen::Vector3d&, const double, const double, const double,
                const Eigen::Vector3d& );

    // These tests check if acceleration due to zonal terms is computed correctly by comparing to
    // output generated by (Ronse, 2012).

    // Get planet test data.
    PlanetTestData earthData = getEarthRonseTestData( );

    // Set map of function pointers for zonal coefficients.
    std::map< int, GravitationalAccelerationPointer > zonalGravitationalAccelerationPointers
            = map_list_of( 2, &computeGravitationalAccelerationDueToJ2 )
                         ( 3, &computeGravitationalAccelerationDueToJ3 )
                         ( 4, &computeGravitationalAccelerationDueToJ4 );

    // Loop over all planet test data and recompute the results using Tudat code. Check that the
    // values computed match results obtained by (Ronse, 2012).
    for ( unsigned int body2 = 0; body2 < earthData.body2Positions.size( ); body2++ )
    {
        // Compute central gravitational acceleration term [m s^-2].
        const Eigen::Vector3d computedCentralAcceleration
                = computeGravitationalAcceleration(
                    earthData.body2Positions.at( body2 ),
                    earthData.gravitationalParameter,
                    earthData.body1Positions.at( 0 ) );

        // Check that the computed central gravitational acceleration matches the expected
        // values.
        TUDAT_CHECK_MATRIX_CLOSE_FRACTION(
                    earthData.expectedAcceleration[ 0 ][ body2 ][ central ],
                    computedCentralAcceleration,
                    1.0e-15 );

        // Loop over all available zonal gravity field coefficients.
        for ( KeyIntValueDoubleMap::iterator zonalCoefficientIterator
              = earthData.zonalCoefficients.begin( );
              zonalCoefficientIterator != earthData.zonalCoefficients.end( );
              zonalCoefficientIterator++ )
        {
            // Compute gravitational acceleration due to given zonal term [m s^-2].
            const Eigen::Vector3d computedZonalGravitationalAcceleration
                    = zonalGravitationalAccelerationPointers[ zonalCoefficientIterator->first ](
                        earthData.body2Positions.at( body2 ),
                        earthData.gravitationalParameter,
                        zonalCoefficientIterator->second,
                        earthData.effectiveRadius,
                        earthData.body1Positions.at( 0 ) );

            // Check that computed gravitational acceleration sum matches expected values.
            TUDAT_CHECK_MATRIX_CLOSE_FRACTION( earthData.expectedAcceleration[ 0 ][ body2 ][
                                               zonalCoefficientIterator->first ],
                                               computedZonalGravitationalAcceleration,
                                               1.0e-13 );
        }
    }
}

BOOST_AUTO_TEST_SUITE_END( )

} // namespace unit_tests
} // namespace tudat
