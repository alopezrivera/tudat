#ifndef TUDAT_APPLICATIONOUTPUT_H
#define TUDAT_APPLICATIONOUTPUT_H

#include <iostream>

namespace tudat_applications
{

//! Get path for output directory.
static inline std::string getOutputPath( const std::string& extraDirectory = "SimulationOutput" )
{
    // Declare file path string assigned to filePath.
    // __FILE__ only gives the absolute path of the header file!
    std::string filePath_( __FILE__ );

    // Strip filename from temporary string and return root-path string.
    std::string reducedPath = filePath_.substr( 0, filePath_.length( ) -
                                std::string( "include/tudat/io/applicationOutput.h" ).length( ) );
    std::string outputPath = reducedPath;
    if( extraDirectory != "" )
    {
        outputPath += extraDirectory;
    }

    if( outputPath.at( outputPath.size( ) - 1 ) != '/' )
    {
        outputPath += "/";
    }

    return outputPath;
}
}


#endif // TUDAT_APPLICATIONOUTPUT_H
