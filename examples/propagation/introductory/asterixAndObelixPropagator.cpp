/*    Copyright (c) 2010-2019, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include "tudat/simulation/simulation.h"

#include "tudat/io/applicationOutput.h"

//! Execute propagation of orbits of Asterix and Obelix around the Earth.
int main( )
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            USING STATEMENTS              //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    using namespace tudat;
    using namespace tudat::simulation_setup;
    using namespace tudat::propagators;
    using namespace tudat::numerical_integrators;
    using namespace tudat::basic_astrodynamics;
    using namespace tudat::basic_mathematics;
    using namespace tudat::orbital_element_conversions;
    using namespace tudat::unit_conversions;
    using namespace tudat::input_output;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////     CREATE ENVIRONMENT AND VEHICLES      //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Load Spice kernels.
    spice_interface::loadStandardSpiceKernels( );

    // Set simulation start epoch.
    const double simulationStartEpoch = 0.0;

    // Set simulation end epoch.
    const double simulationEndEpoch = tudat::physical_constants::JULIAN_DAY;

    // Set numerical integration fixed step size.
    const double fixedStepSize = 60.0;

    // Create body settings
    std::vector< std::string > bodiesToCreate = { "Sun", "Earth", "Moon", "Mars", "Venus" };
    BodyListSettings bodySettings = getDefaultBodySettings( bodiesToCreate, "SSB", "ECLIPJ2000" );
    bodySettings.at( "Earth" )->ephemerisSettings = std::make_shared< ConstantEphemerisSettings >(
            Eigen::Vector6d::Zero( ) );

    // Declare occulting bodies for radiation interface
    std::map< std::string, std::vector< std::string > > occultingBodies;
    occultingBodies[ "Sun" ] = { "Earth" };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             ASTERIX SETTINGS            ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create body settings for Asterix
    bodySettings.addSettings( "Asterix" );
    bodySettings.at("Asterix")->constantMass = 400.0;

    // Create and add aerodynamic coefficient interface
    double asterixReferenceArea = 4.0;
    double asterixAerodynamicCoefficient = 1.2;
    bodySettings.at( "Asterix" )->aerodynamicCoefficientSettings =
            constantAerodynamicCoefficientSettings(
                    asterixReferenceArea, asterixAerodynamicCoefficient * Eigen::Vector3d::UnitX( ) );

    // Create and add radiation pressure interface
    double asterixReferenceAreaRadiation = 4.0;
    double asterixRadiationPressureCoefficient = 1.2;
    bodySettings.at( "Asterix" )->radiationPressureTargetModelSettings =
            std::make_shared<CannonballRadiationPressureTargetModelSettings>(
                    asterixReferenceAreaRadiation,
                    asterixRadiationPressureCoefficient,
                    occultingBodies);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             OBELIX SETTINGS             ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create body settings for Obelix
    bodySettings.addSettings( "Obelix" );
    bodySettings.at("Obelix")->constantMass = 800.0;

    // Create and add aerodynamic coefficient interface
    double obelixReferenceArea = 12.0;
    double obelixAerodynamicCoefficient = 1.6;
    bodySettings.at( "Obelix" )->aerodynamicCoefficientSettings =
            constantAerodynamicCoefficientSettings(
                    obelixReferenceArea, obelixAerodynamicCoefficient * Eigen::Vector3d::UnitX( ) );

    // Create and add radiation pressure interface
    double obelixReferenceAreaRadiation = 12.0;
    double obelixRadiationPressureCoefficient = 1.2;
    bodySettings.at( "Obelix" )->radiationPressureTargetModelSettings =
            std::make_shared<CannonballRadiationPressureTargetModelSettings>(
                    obelixReferenceAreaRadiation,
                    obelixRadiationPressureCoefficient,
                    occultingBodies);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE BODIES          /////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    SystemOfBodies bodies = createSystemOfBodies( bodySettings );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            SET UP ACCELERATION MODEL         //////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    SelectedAccelerationMap accelerationSettingsList;
    std::vector< std::string > bodiesToPropagate = { "Asterix", "Obelix" };
    std::vector< std::string > centralBodies = { "Earth", "Earth" };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE ACCELERATIONS ON ASTERIX         ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define propagation settings.
    std::map< std::string, std::vector< std::shared_ptr< AccelerationSettings > > > accelerationsOfAsterix;

    accelerationsOfAsterix[ "Earth" ] = {
            sphericalHarmonicAcceleration( 8, 8 ),
            aerodynamicAcceleration() };

    accelerationsOfAsterix[ "Sun" ] = {
            pointMassGravityAcceleration( ),
            radiationPressureAcceleration( ) };

    accelerationsOfAsterix[ "Mars" ] = {
            pointMassGravityAcceleration( ) };

    accelerationsOfAsterix[ "Venus" ] = {
            pointMassGravityAcceleration( ) };

    accelerationsOfAsterix[ "Moon" ] = {
            pointMassGravityAcceleration( ) };

    // Define propagator settings variables.
    accelerationSettingsList[ "Asterix" ] = accelerationsOfAsterix;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE ACCELERATIONS ON OBELIX          ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define propagation settings.
    std::map< std::string, std::vector< std::shared_ptr< AccelerationSettings > > > accelerationsOfObelix;

    accelerationsOfObelix[ "Earth" ] = {
            sphericalHarmonicAcceleration( 8, 8 ),
            aerodynamicAcceleration() };

    accelerationsOfObelix[ "Sun" ] = {
            pointMassGravityAcceleration( ),
            radiationPressureAcceleration( ) };

    accelerationsOfObelix[ "Mars" ] = {
            pointMassGravityAcceleration( ) };

    accelerationsOfObelix[ "Venus" ] = {
            pointMassGravityAcceleration( ) };

    accelerationsOfObelix[ "Moon" ] = {
            pointMassGravityAcceleration( ) };

    // Define propagator settings variables.
    accelerationSettingsList[ "Obelix" ] = accelerationsOfObelix;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE SYSTEM ACCELERATION MODEL          //////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    basic_astrodynamics::AccelerationMap accelerationModelMap = createAccelerationModelsMap(
            bodies, accelerationSettingsList, bodiesToPropagate, centralBodies );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             CREATE PROPAGATION SETTINGS            ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Set initial conditions for satellites that will be propagated in this simulation.
    // The initial conditions are given in Keplerian elements and later on converted to
    // Cartesian elements.

    // Set Keplerian elements for Asterix.
    Eigen::Vector6d asterixInitialStateInKeplerianElements;
    asterixInitialStateInKeplerianElements( semiMajorAxisIndex ) = 7500.0e3;
    asterixInitialStateInKeplerianElements( eccentricityIndex ) = 0.1;
    asterixInitialStateInKeplerianElements( inclinationIndex ) = convertDegreesToRadians( 85.3 );
    asterixInitialStateInKeplerianElements( argumentOfPeriapsisIndex )
            = convertDegreesToRadians( 235.7 );
    asterixInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex )
            = convertDegreesToRadians( 23.4 );
    asterixInitialStateInKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 139.87 );

    // Set Keplerian elements for Obelix.
    Eigen::Vector6d obelixInitialStateInKeplerianElements( 6 );
    obelixInitialStateInKeplerianElements( semiMajorAxisIndex ) = 12040.6e3;
    obelixInitialStateInKeplerianElements( eccentricityIndex ) = 0.4;
    obelixInitialStateInKeplerianElements( inclinationIndex ) = convertDegreesToRadians( -23.5 );
    obelixInitialStateInKeplerianElements( argumentOfPeriapsisIndex )
            = convertDegreesToRadians( 10.6 );
    obelixInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex )
            = convertDegreesToRadians( 367.9 );
    obelixInitialStateInKeplerianElements( trueAnomalyIndex ) = convertDegreesToRadians( 93.4 );

    // Convert initial states from Keplerian to Cartesian elements.
    double earthGravitationalParameter = bodies.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( );

    // Convert Asterix state from Keplerian elements to Cartesian elements.
    const Eigen::Vector6d asterixInitialState = convertKeplerianToCartesianElements(
                asterixInitialStateInKeplerianElements,
                earthGravitationalParameter );

    // Convert Obelix state from Keplerian elements to Cartesian elements.
    const Eigen::Vector6d obelixInitialState = convertKeplerianToCartesianElements(
                obelixInitialStateInKeplerianElements,
                earthGravitationalParameter );

    // Set initial state
    Eigen::VectorXd systemInitialState = Eigen::VectorXd( 12 );
    systemInitialState.segment( 0, 6 ) = asterixInitialState;
    systemInitialState.segment( 6, 6 ) = obelixInitialState;

    std::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings =
            std::make_shared< TranslationalStatePropagatorSettings< double > >
            ( centralBodies, accelerationModelMap, bodiesToPropagate, systemInitialState, simulationEndEpoch );
    std::shared_ptr< IntegratorSettings< > > integratorSettings =
            std::make_shared< IntegratorSettings< > >
            ( rungeKutta4, simulationStartEpoch, fixedStepSize );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             PROPAGATE ORBIT            ////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "- Propagating..." << std::endl;

    // Create simulation object and propagate dynamics.
    SingleArcDynamicsSimulator< > dynamicsSimulator(
                bodies, integratorSettings, propagatorSettings, true, false, false );
    std::map< double, Eigen::VectorXd > integrationResult = dynamicsSimulator.getEquationsOfMotionNumericalSolution( );

    std::cout << "- Propagation complete" << std::endl;

    // Retrieve numerically integrated states of vehicles.
    std::map< double, Eigen::VectorXd > asterixPropagationHistory;
    std::map< double, Eigen::VectorXd > obelixPropagationHistory;
    for( std::map< double, Eigen::VectorXd >::const_iterator stateIterator = integrationResult.begin( );
         stateIterator != integrationResult.end( ); stateIterator++ )
    {
        asterixPropagationHistory[ stateIterator->first ] = stateIterator->second.segment( 0, 6 );
        obelixPropagationHistory[ stateIterator->first ] = stateIterator->second.segment( 6, 6 );
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////        PROVIDE OUTPUT TO FILE                        //////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "- Saving simulation data..." << std::endl;

    std::string outputSubFolder = "output_AsterixAndObelixExample/";

    // Write Asterix propagation history to file.
    writeDataMapToTextFile( asterixPropagationHistory,
                            "asterixPropagationHistory.csv",
                            tudat_applications::getOutputPath( ) + outputSubFolder,
                            "",
                            std::numeric_limits< double >::digits10,
                            std::numeric_limits< double >::digits10,
                            "," );

    // Write obelix propagation history to file.
    writeDataMapToTextFile( obelixPropagationHistory,
                            "obelixPropagationHistory.csv",
                            tudat_applications::getOutputPath( ) + outputSubFolder,
                            "",
                            std::numeric_limits< double >::digits10,
                            std::numeric_limits< double >::digits10,
                            "," );

    std::cout << "    Simulation data saved to: " << tudat_applications::getOutputPath( ) + outputSubFolder << std::endl;

    // Final statement.
    // The exit code EXIT_SUCCESS indicates that the program was successfully executed.
    return EXIT_SUCCESS;
}
