Tudat Example Applications  
======

The following is a collection of examples illustrating the use of the Tudat libraries for various use cases. These examples demonstrate how to propagate spacecraft trajectories, estimate states from tracking data, design mission trajectories, and solve optimization problems using the Pagmo library.

---
**Contents**  
1. Propagation  
2. Estimation  
3. Mission design  
4. Optimization with Pagmo  

---

Propagation  
-----------

This section shows how to simulate spacecraft motion under various force models. It is divided into two parts:

- **Introductory:**  
  These examples cover basic satellite propagation and simple perturbation models:  
  - *asterixAndObelixPropagator.cpp*: A simple propagator illustrating basic Tudat setup.  
  - *empiricalAccelerationExample.cpp*: Demonstrates how to include empirical acceleration models.  
  - *fullPropagationSpacecraftCR3BP.cpp*: Simulates spacecraft dynamics in the circular restricted three-body problem.  
  - *galileoConstellationSimulator.cpp*: A basic simulation of a navigation constellation.  
  - *innerLongSolarSystemPropagation.cpp* and *innerSolarSystemPropagation.cpp*: Show long- and short-range solar system propagation.  
  - *lageosRadiationPressureAcceleration.cpp*: Includes radiation pressure effects as seen on LAGEOS-type orbits.  
  - *singlePerturbedSatellitePropagator.cpp* and *singleSatellitePropagator.cpp*: Compare perturbed and unperturbed satellite propagation.  
  - *spacexTeslaTrajectory.cpp*: A playful demonstration of trajectory propagation inspired by a high-profile launch.  
  - *thrustAccelerationFromFileExample.cpp* and *thrustAlongVelocityVectorExample.cpp*: Illustrate the use of file-based thrust data and directional thrust application.  
  - *variationalEquationsPropagatorExample.cpp*: Includes computation of variational equations during propagation for sensitivity analysis.

- **Advanced:**  
  These examples focus on more complex simulations and comparative studies:  
  - *apolloCapsuleEntry.cpp*: Simulates reentry dynamics for an Apollo capsule-like vehicle.  
  - *lifetimeMaximisation.cpp*: Explores strategies to maximize spacecraft lifetime through propagation analysis.  
  - *propagatorTypesComparison.cpp*: Compares various propagation schemes available in Tudat.  
  - *tabulatedAtmosphereUsage.cpp*: Demonstrates the use of tabulated atmospheric data in propagation.

---

Estimation  
----------

This section covers examples on orbit determination and state estimation using tracking data.  
- *RadioAstronODsimulationArcs2.cpp*: Simulates orbit determination for the RadioAstron mission using multiple data arcs.  
- *earthOrbiterBasicStateEstimation.cpp* and *earthOrbiterStateEstimation.cpp*: Provide basic and more refined state estimation setups for an Earth-orbiting spacecraft.  
- *filterExample.cpp*: Illustrates filtering techniques for dynamic state estimation.  
- *MikhailData.txt*: Accompanies the estimation examples with observational or measurement data.

---

Mission design  
--------------

These examples focus on designing and analyzing interplanetary trajectories.  
- *fullPropagationMga.cpp*: Simulates full-propagation multiple gravity assist (MGA) trajectories.  
- *shapeBasedTrajectoryDesign.cpp*: Demonstrates a shape-based method for preliminary trajectory design, useful for rapid mission analysis.

---

Optimization with Pagmo  
-----------------------

This section shows how to leverage the Pagmo optimization library for solving trajectory and mission design problems.  
- *cec2013OptimizerComparison.cpp*: Compares the performance of different optimizers using CEC2013 benchmark problems.  
- *earthMarsTransferExample.cpp*: Optimizes Earth-to-Mars transfer trajectories.  
- *himmelblauOptimization.cpp*: Solves the Himmelblau test function, illustrating non-convex optimization challenges.  
- *hodographicShapingFullOptimisationExample.cpp* and *hodographicShapingTrajectoryExample.cpp*: Use hodographic shaping techniques for trajectory design and full optimization.  
- *mgaTransferExample.cpp*: Applies multiple gravity assist strategies within an optimization framework.  
- *multiObjectiveEarthMarsTransferExample.cpp*: Tackles the Earth-Mars transfer as a multi-objective problem.  
- Within the **problems** subfolder, several files support common optimization tasks, including:  
  - *earthMarsTransfer.cpp/h*: Core routines for Earth-Mars transfer problems.  
  - *multipleGravityAssist.cpp/h*: Functions for modeling multiple gravity assist scenarios.  
  - *propagationTargeting.cpp/h*: Routines to target desired propagation outcomes.  
  - Additional headers such as *applicationOutput.h*, *getAlgorithm.h*, *himmelblau.h*, and *saveOptimizationResults.h* provide supporting functionality.  
- Other examples include:  
  - *propagationTargetingExample.cpp*: Demonstrates targeting techniques within an optimization context.  
  - *satelliteRendezVousExample.cpp*: Optimizes trajectories for satellite rendezvous.  
  - *simsFlanaganTrajectoryExample.cpp*: Implements a trajectory design using the Sims-Flanagan method.  
  - *zdtMultiObjectiveOptimizerComparison.cpp*: Compares optimizers on standard ZDT multi-objective test problems.
