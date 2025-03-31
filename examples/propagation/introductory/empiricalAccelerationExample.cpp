/*    Copyright (c) 2010-2017, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include <tudat/simulation/simulation.h>

//#include "propagationAndOptimization/applicationOutput.h"


//! Execute propagation of orbit of LunarOrbiter around the Earth.
int main()
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            USING STATEMENTS              //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    using namespace tudat;
    using namespace tudat::simulation_setup;
    using namespace tudat::ephemerides;
    using namespace tudat::propagators;
    using namespace tudat::numerical_integrators;
    using namespace tudat::orbital_element_conversions;
    using namespace tudat::basic_astrodynamics;
    using namespace tudat::basic_mathematics;
    using namespace tudat::unit_conversions;
    using namespace tudat::ephemerides;
    using namespace tudat::spice_interface;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////     CREATE ENVIRONMENT AND VEHICLE       //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Load Spice kernels.
    spice_interface::loadStandardSpiceKernels( );


    // Create body objects.
    std::vector< std::string > bodiesToCreate;
    bodiesToCreate.push_back( "Earth" );
    bodiesToCreate.push_back( "Moon" );
    bodiesToCreate.push_back( "Sun" );

    // Set simulation end epoch.
    const double simulationStartEpoch = 0.0;
    const double simulationEndEpoch = 14.0 * tudat::physical_constants::JULIAN_DAY;

    BodyListSettings bodySettings =
            getDefaultBodySettings( bodiesToCreate, "Moon", "ECLIPJ2000" );

    // Create Earth object
    SystemOfBodies bodyMap = createSystemOfBodies( bodySettings );

    // Create spacecraft object.
    bodyMap.createEmptyBody( "LunarOrbiter" );

    // Define limits
    int maxEccentricityCase = 2;
    int maxPropagationCase = 10;

    for( unsigned int eccentricityCase = 0; eccentricityCase < maxEccentricityCase; eccentricityCase++ )
    {
        for( unsigned int propagationCase = 0; propagationCase < maxPropagationCase; propagationCase++ )
        {

            std::cout<<"Eccentricity case: [" << eccentricityCase + 1 << "/" << maxEccentricityCase << "] -- Propagation case: [" << propagationCase + 1 << "/" << maxPropagationCase << "]" <<std::endl;

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////            CREATE ACCELERATIONS          //////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Define propagator settings variables.
            SelectedAccelerationMap accelerationMap;
            std::vector< std::string > bodiesToPropagate;
            std::vector< std::string > centralBodies;

            bodiesToPropagate.push_back( "LunarOrbiter" );
            centralBodies.push_back( "Moon" );

            // Define propagation settings.
            std::map< std::string, std::vector< std::shared_ptr< AccelerationSettings > > > accelerationsOfLunarOrbiter;

            accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< AccelerationSettings >(
                                                                 basic_astrodynamics::point_mass_gravity ) );

            double empiricalAccelerationNorm = 1.0E-8;
            if( propagationCase == 1 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitX( ) ) );
            }
            else if( propagationCase == 2 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitY( ) ) );
            }
            else if( propagationCase == 3 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitZ( ) ) );
            }
            else if( propagationCase == 4 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitX( ) ) );
            }
            else if( propagationCase == 5 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitY( ) ) );
            }
            else if( propagationCase == 6 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitZ( ) ) );
            }
            else if( propagationCase == 7 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitX( ) ) );
            }
            else if( propagationCase == 8 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitY( ) ) );
            }
            else if( propagationCase == 9 )
            {
                accelerationsOfLunarOrbiter[ "Moon" ].push_back( std::make_shared< EmpiricalAccelerationSettings >(
                                                                     Eigen::Vector3d::Zero( ),
                                                                     Eigen::Vector3d::Zero( ),
                                                                     empiricalAccelerationNorm * Eigen::Vector3d::UnitZ( ) ) );
            }
            accelerationMap[  "LunarOrbiter" ] = accelerationsOfLunarOrbiter;


            // Create acceleration models and propagation settings.
            basic_astrodynamics::AccelerationMap accelerationModelMap = createAccelerationModelsMap(
                        bodyMap, accelerationMap, bodiesToPropagate, centralBodies );

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////             CREATE PROPAGATION SETTINGS            ////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Set initial conditions for the LunarOrbiter satellite that will be propagated in this simulation.
            // The initial conditions are given in Keplerian elements and later on converted to Cartesian
            // elements.

            // Set Keplerian elements for LunarOrbiter.
            Eigen::Vector6d lunarOrbiterInitialStateInKeplerianElements;
            lunarOrbiterInitialStateInKeplerianElements( semiMajorAxisIndex ) = 2000.0E3;
            if( eccentricityCase == 0 )
            {
                lunarOrbiterInitialStateInKeplerianElements( eccentricityIndex ) = 0.01;
            }
            else if( eccentricityCase == 1 )
            {
                lunarOrbiterInitialStateInKeplerianElements( eccentricityIndex ) = 0.05;
            }
            lunarOrbiterInitialStateInKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 89.3 );
            lunarOrbiterInitialStateInKeplerianElements( argumentOfPeriapsisIndex )
                    = convertDegreesToRadians( 235.7 );
            lunarOrbiterInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex )
                    = convertDegreesToRadians( 23.4 );
            lunarOrbiterInitialStateInKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 139.87 );

            // Convert LunarOrbiter state from Keplerian elements to Cartesian elements.
            double moonGravitationalParameter = bodyMap.at( "Moon" )->getGravityFieldModel( )->getGravitationalParameter( );
            Eigen::VectorXd systemInitialState = convertKeplerianToCartesianElements(
                        lunarOrbiterInitialStateInKeplerianElements,
                        moonGravitationalParameter );

            // Define list of dependent variables to save.
            std::vector< std::shared_ptr< SingleDependentVariableSaveSettings > > dependentVariablesList;
            dependentVariablesList.push_back(
                        std::make_shared< SingleDependentVariableSaveSettings >(
                            keplerian_state_dependent_variable, "LunarOrbiter", "Moon") );

            auto integratorSettings = rungeKuttaFixedStepSettings(
                    10.0, rungeKutta4Classic);

            auto outputProcessingSettings = std::make_shared<SingleArcPropagatorProcessingSettings>(
                    false, false, 1, TUDAT_NAN );

            auto propagatorSettings = translationalStatePropagatorSettings(
                    centralBodies, accelerationModelMap, bodiesToPropagate, systemInitialState, simulationStartEpoch, integratorSettings,
                    propagationTimeTerminationSettings(simulationEndEpoch), cowell, dependentVariablesList, outputProcessingSettings);

            auto dynamicsSimulator = createDynamicsSimulator<double, double>(
                    bodyMap, propagatorSettings);
            auto propagationResults =
                    std::dynamic_pointer_cast<SingleArcSimulationResults<double, double>>(dynamicsSimulator->getPropagationResults());

            // Store results
            std::string outputSubFolder = "output_empiricalAccelerationExample/";
            input_output::writeDataMapToTextFile(propagationResults->getDependentVariableHistory(),
                                                 "dependent_variable_history_" +
                                                 boost::lexical_cast< std::string >( propagationCase ) + "_" +
                                                 boost::lexical_cast< std::string >( eccentricityCase ) + ".csv",
                                                 outputSubFolder,
                                                 "",
                                                 std::numeric_limits< double >::digits10,
                                                 std::numeric_limits< double >::digits10,
                                                 ",");
            input_output::writeIdMapToTextFile(propagationResults->getDependentVariableId(),
                                               "dependent_variable_names_" +
                                               boost::lexical_cast< std::string >( propagationCase ) + "_" +
                                               boost::lexical_cast< std::string >( eccentricityCase ) + ".csv",
                                               outputSubFolder,
                                               ",");

        }
    }

    return EXIT_SUCCESS;
}
