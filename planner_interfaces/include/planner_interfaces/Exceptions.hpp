#ifndef PLANNER_INTERFACE_EXCEPTIONS_HPP_
#define PLANNER_INTERFACE_EXCEPTIONS_HPP_

#include <sm/assert_macros.hpp>

namespace planning2d {
  SM_DEFINE_EXCEPTION( RuntimeException, std::runtime_error );          // Something strange happened at runtime
  SM_DEFINE_EXCEPTION( InitializationException, std::runtime_error );   // Class not properly initialized yet
  SM_DEFINE_EXCEPTION( NullPointerException, std::runtime_error );      // Trying to access null pointer
  SM_DEFINE_EXCEPTION( OutOfBoundAccessException, std::runtime_error ); // Trying to access an error beyond the limits
  SM_DEFINE_EXCEPTION( LookupException, std::runtime_error );           // Trying to lookup something that does not exist
  SM_DEFINE_EXCEPTION( NoImplementationException, std::runtime_error ); // Trying to call a method that is not implemented
  SM_DEFINE_EXCEPTION( ConfigurationException, std::runtime_error );    // Cannot load configuration
  SM_DEFINE_EXCEPTION( ParameterException, std::runtime_error );        // Something is wrong with a parameter
  SM_DEFINE_EXCEPTION( FunctionInputException, std::runtime_error );    // Something is wrong with a function input
  SM_DEFINE_EXCEPTION( WrongUsageException, std::runtime_error );    // Wrong usage
}

#endif
