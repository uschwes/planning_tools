#ifndef PLANNER_INTERFACES_RETURN_CODES_HPP_
#define PLANNER_INTERFACES_RETURN_CODES_HPP_

namespace planning2d {

  enum ReturnCode {
      RETCODE_OK=0,
      RETCODE_GENERAL_ERROR,
      RETCODE_INVALID_INPUTS,
      RETCODE_EXCEPTION_OCCURED,
      RETCODE_ROBOT_IN_COLLISION_NOW,
      RETCODE_ROBOT_INEVITABLE_COLLISION_IN_FUTURE
  };
  
  
  inline std::ostream& operator<<(std::ostream &os, const ReturnCode c) {
    
    switch (c) {
      case RETCODE_OK:
        os << "RETCODE_OK";
        break;
      case RETCODE_GENERAL_ERROR:
        os << "RETCODE_GENERAL_ERROR";
        break;
      case RETCODE_INVALID_INPUTS:
        os << "RETCODE_INVALID_INPUTS";
        break;
      case RETCODE_EXCEPTION_OCCURED:
        os << "RETCODE_EXCEPTION_OCCURED";
        break;
      case RETCODE_ROBOT_IN_COLLISION_NOW:
        os << "RETCODE_ROBOT_IN_COLLISION_NOW";
        break;
      case RETCODE_ROBOT_INEVITABLE_COLLISION_IN_FUTURE:
        os << "RETCODE_ROBOT_INEVITABLE_COLLISION_IN_FUTURE";
        break;
    }
    return os;
  }

} /* namespace planning2d */

#endif /* PLANNER_INTERFACES_RETURN_CODES_HPP_ */