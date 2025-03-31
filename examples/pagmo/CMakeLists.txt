#    Copyright (c) 2010-2019, Delft University of Technology
#    All rigths reserved
#
#    This file is part of the Tudat. Redistribution and use in source and
#    binary forms, with or without modification, are permitted exclusively
#    under the terms of the Modified BSD license. You should have received
#    a copy of the license with this file. If not, please or visit:
#    http://tudat.tudelft.nl/LICENSE.

TUDAT_ADD_EXECUTABLE(application_PagmoHimmelblauOptimization
    "himmelblauOptimization.cpp"
    tudat_pagmo_example_problems tudat_input_output)

TUDAT_ADD_EXECUTABLE(application_PagmoMgaTransferExample
    "mgaTransferExample.cpp"
    tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoSatelliteRendezvousExample
    "satelliteRendezVousExample.cpp"
    tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoHodographicShapingExample
        "hodographicShapingTrajectoryExample.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoHodographicShapingFullOptimizationExample
        "hodographicShapingFullOptimisationExample.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoMultiObjectiveEarthMarsTransferExample
        "multiObjectiveEarthMarsTransferExample.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoPropagationTargetingExample
        "propagationTargetingExample.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoSimsFlanaganTrajectoryExample
        "simsFlanaganTrajectoryExample.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )

TUDAT_ADD_EXECUTABLE(application_PagmoZDTMultiObjectiveOptimizerComparison
        "zdtMultiObjectiveOptimizerComparison.cpp"
        tudat_pagmo_example_problems tudat_mission_segments ${Tudat_PROPAGATION_LIBRARIES} )
