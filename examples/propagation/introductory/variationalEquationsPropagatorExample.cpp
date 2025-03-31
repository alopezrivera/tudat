/*    Copyright (c) 2010-2019, Delft University of Technology
 *    All rigths reserved
 *
 *    This file is part of the Tudat. Redistribution and use in source and
 *    binary forms, with or without modification, are permitted exclusively
 *    under the terms of the Modified BSD license. You should have received
 *    a copy of the license with this file. If not, please or visit:
 *    http://tudat.tudelft.nl/LICENSE.
 */

#include "tudat/simulation/estimation.h"

#include "tudat/io/applicationOutput.h"

//! Execute propagation of orbit of Asterix around the Earth.
int main( )
{
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            USING STATEMENTS              //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    using namespace tudat;
    using namespace tudat::spice_interface;
    using namespace tudat::simulation_setup;
    using namespace tudat::propagators;
    using namespace tudat::basic_astrodynamics;
    using namespace tudat::numerical_integrators;
    using namespace tudat::orbital_element_conversions;
    using namespace tudat::basic_mathematics;
    using namespace tudat::gravitation;
    using namespace tudat::ephemerides;
    using namespace tudat::orbit_determination;
    using namespace tudat::estimatable_parameters;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////     CREATE ENVIRONMENT AND VEHICLE       //////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Load Spice kernels.
    spice_interface::loadStandardSpiceKernels( );

    // Set simulation time settings.
    const double simulationStartEpoch = 0.0;
    const double simulationEndEpoch = tudat::physical_constants::JULIAN_DAY;

    // Define body settings for simulation.
    std::vector< std::string > bodiesToCreate;
    bodiesToCreate.push_back( "Sun" );
    bodiesToCreate.push_back( "Earth" );
    bodiesToCreate.push_back( "Moon" );
    bodiesToCreate.push_back( "Mars" );
    bodiesToCreate.push_back( "Venus" );

    // Create body objects.
    BodyListSettings bodySettings = getDefaultBodySettings( bodiesToCreate, simulationStartEpoch,
                                    simulationEndEpoch,  "Earth", "J2000" );

    // Declare occulting bodies for radiation interface
    std::map< std::string, std::vector< std::string > > occultingBodies;
    occultingBodies[ "Sun" ] = { "Earth" };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             CREATE VEHICLE            /////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Create spacecraft object.
    bodySettings.addSettings( "Asterix" );
    bodySettings.at("Asterix")->constantMass = 400.0;

    // Create and add radiation pressure interface
    double referenceAreaRadiation = 4.0;
    double radiationPressureCoefficient = 1.2;
    bodySettings.at("Asterix")->radiationPressureTargetModelSettings =
            std::make_shared<CannonballRadiationPressureTargetModelSettings>(
                    referenceAreaRadiation,
                    radiationPressureCoefficient,
                    occultingBodies);

    // Create and add aerodynamic coefficient interface
    double referenceArea = 4.0;
    double aerodynamicCoefficient = 1.2;
    bodySettings.at( "Asterix" )->aerodynamicCoefficientSettings =
            constantAerodynamicCoefficientSettings(
                    referenceArea, aerodynamicCoefficient * Eigen::Vector3d::UnitX( ) );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE BODIES          /////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    SystemOfBodies bodies = createSystemOfBodies( bodySettings );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////            CREATE ACCELERATIONS          //////////////////////////////////////////////////////
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
    SelectedAccelerationMap accelerationSettingsList;
    accelerationSettingsList[ "Asterix" ] = accelerationsOfAsterix;

    std::vector< std::string > bodiesToPropagate = { "Asterix" };
    std::vector< std::string > centralBodies = { "Earth" };

    basic_astrodynamics::AccelerationMap accelerationModelMap = createAccelerationModelsMap(
            bodies, accelerationSettingsList, bodiesToPropagate, centralBodies );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             CREATE PROPAGATION SETTINGS            ////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Set Keplerian elements for Asterix.
    Eigen::Vector6d asterixInitialStateInKeplerianElements;
    asterixInitialStateInKeplerianElements( semiMajorAxisIndex ) = 7500.0E3;
    asterixInitialStateInKeplerianElements( eccentricityIndex ) = 0.1;
    asterixInitialStateInKeplerianElements( inclinationIndex ) = unit_conversions::convertDegreesToRadians( 85.3 );
    asterixInitialStateInKeplerianElements( argumentOfPeriapsisIndex ) = unit_conversions::convertDegreesToRadians( 235.7 );
    asterixInitialStateInKeplerianElements( longitudeOfAscendingNodeIndex ) = unit_conversions::convertDegreesToRadians( 23.4 );
    asterixInitialStateInKeplerianElements( trueAnomalyIndex ) = unit_conversions::convertDegreesToRadians( 139.87 );

    double earthGravitationalParameter = bodies.at( "Earth" )->getGravityFieldModel( )->getGravitationalParameter( );
    const Eigen::Vector6d asterixInitialState = convertKeplerianToCartesianElements(
                asterixInitialStateInKeplerianElements, earthGravitationalParameter );

    std::shared_ptr< TranslationalStatePropagatorSettings< double > > propagatorSettings =
            std::make_shared< TranslationalStatePropagatorSettings< double > >(
                centralBodies, accelerationModelMap, bodiesToPropagate, asterixInitialState, simulationEndEpoch );

    const double fixedStepSize = 10.0;
    std::shared_ptr< IntegratorSettings< > > integratorSettings =
            std::make_shared< IntegratorSettings< > >( rungeKutta4, 0.0, fixedStepSize );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////    DEFINE PARAMETERS FOR WHICH SENSITIVITY IS TO BE COMPUTED   ////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Define parameters.
    std::vector< std::shared_ptr< EstimatableParameterSettings > > parameterNames;
    {
        parameterNames = getInitialStateParameterSettings< double >( propagatorSettings, bodies );
        parameterNames.push_back( std::make_shared< EstimatableParameterSettings >( "Asterix", radiation_pressure_coefficient ) );
        parameterNames.push_back( std::make_shared< EstimatableParameterSettings >( "Asterix", constant_drag_coefficient ) );
        parameterNames.push_back( std::make_shared< EstimatableParameterSettings >( "Moon", gravitational_parameter ) );

        parameterNames.push_back( std::make_shared< SphericalHarmonicEstimatableParameterSettings >(
                3, 0, 3, 3, "Earth", spherical_harmonics_cosine_coefficient_block ) );
        parameterNames.push_back( std::make_shared< SphericalHarmonicEstimatableParameterSettings >(
                3, 1, 3, 3, "Earth", spherical_harmonics_sine_coefficient_block ) );
    }

    // Create parameters
    std::shared_ptr< estimatable_parameters::EstimatableParameterSet< > > parametersToEstimate =
            createParametersToEstimate( parameterNames, bodies );

    // Print identifiers and indices of parameters to terminal.
    printEstimatableParameterEntries( parametersToEstimate );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////             PROPAGATE ORBIT AND VARIATIONAL EQUATIONS         /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "- Propagating..." << std::endl;

    // Create simulation object and propagate dynamics.
    SingleArcVariationalEquationsSolver< > variationalEquationsSimulator =
                SingleArcVariationalEquationsSolver< >(
                    bodies, integratorSettings, propagatorSettings, parametersToEstimate,
                    1, std::shared_ptr< numerical_integrators::IntegratorSettings< double > >( ), 0, 0 );

    std::cout << "- Propagation complete" << std::endl;

    std::map< double, Eigen::MatrixXd > stateTransitionResult =
            variationalEquationsSimulator.getNumericalVariationalEquationsSolution( ).at( 0 );
    std::map< double, Eigen::MatrixXd > sensitivityResult =
            variationalEquationsSimulator.getNumericalVariationalEquationsSolution( ).at( 1 );
    std::map< double, Eigen::VectorXd > integrationResult =
            variationalEquationsSimulator.getDynamicsSimulator( )->getEquationsOfMotionNumericalSolution( );

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////        PROVIDE OUTPUT TO CONSOLE AND FILES           //////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "- Saving simulation data..." << std::endl;

    std::string outputSubFolder = "output_PerturbedSatelliteVariationalExample/";

    // Write perturbed satellite propagation history to file.
    input_output::writeDataMapToTextFile( integrationResult,
                                          "singlePerturbedSatellitePropagationHistory.csv",
                                          tudat_applications::getOutputPath( ) + outputSubFolder,
                                          "",
                                          std::numeric_limits< double >::digits10,
                                          std::numeric_limits< double >::digits10,
                                          "," );

    input_output::writeDataMapToTextFile( stateTransitionResult,
                                          "singlePerturbedSatelliteStateTransitionHistory.csv",
                                          tudat_applications::getOutputPath( ) + outputSubFolder,
                                          "",
                                          std::numeric_limits< double >::digits10,
                                          std::numeric_limits< double >::digits10,
                                          "," );

    input_output::writeDataMapToTextFile( sensitivityResult,
                                          "singlePerturbedSatelliteSensitivityHistory.csv",
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

