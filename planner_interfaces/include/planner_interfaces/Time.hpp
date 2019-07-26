#ifndef PLANNER_INTERFACES_TIME_HPP
#define PLANNER_INTERFACES_TIME_HPP

#include <cmath>
#include <string>
#include <limits>

#include <boost/cstdint.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace planning2d {

class Time;
class Duration;

namespace time {

  typedef boost::int64_t T; ///< type used for representing nanoseconds

  /**
   * @brief Converts nanoseconds to seconds
   * @param _nanosec Time/Duration in nanoseconds
   * @return Seconds
   */
  inline double toSec(const T _nanosec) { return (double)_nanosec*1e-9; };

  /**
   * @brief Converts nanoseconds to milliseconds
   * @param _nanosec Time/Duration in nanoseconds
   * @return Milliseconds
   */
  inline double toMilliSec(const T _nanosec) { return (double)_nanosec*1e-6; };

  /**
   * @brief Converts nanoseconds to microseconds
   * @param _nanosec Time/Duration in nanoseconds
   * @return Microseconds
   */
  inline double toMicroSec(const T _nanosec) { return (double)_nanosec*1e-3; };

  /**
   * @brief Converts seconds to nanoseconds
   * @param _sec Time/Duration in seconds
   * @return Nanoseconds
   */
  inline T fromSec(const double _sec) { return round(_sec*1e9); }

  /**
   * @brief Constructs nanoseconds from boost posix time
   * @param t Boost posix time struct
   * @return Nanoseconds
   */
  inline T fromBoost(const boost::posix_time::ptime& t) { return (T)t.time_of_day().total_nanoseconds(); }

  /**
   * @brief Constructs nanoseconds from boost posix duration
   * @param t Boost posix duration struct
   * @return Nanoseconds
   */
  inline T fromBoost(const boost::posix_time::time_duration& d) { return (T)d.total_nanoseconds(); }

  /**
   * @brief Returns boost posix Time from nanoseconds
   * @param _nanosec Nanoseconds
   * @return Boost posix time struct
   */
  inline boost::posix_time::ptime toBoost(time::T _nanosec);

  /**
   * @brief Returns boost posix duration from nanoseconds
   * @param _nanosec Nanoseconds
   * @return Boost posix duration struct
   */
  inline boost::posix_time::time_duration toBoostDuration(time::T _nanosec);

  enum TimeUnit {SEC=0, MILLISEC=1, MICROSEC=2, NANOSEC=3};

  /**
   * @class Formatter
   * @brief Helper struct to format time formats
   */
  struct Formatter {
    Formatter(TimeUnit unit_ = NANOSEC, int precision_ = -1, const std::string& bracketLeft_ = " [", const std::string& bracketRight_ = "]")
      : unit(unit_), precision(precision_), bl(bracketLeft_), br(bracketRight_) {
    }
    const TimeUnit unit;
    const int precision;
    const std::string bl, br;
    static const constexpr char* const UnitStrings[] = {"s", "ms", "us", "ns"};
  };

  /// \brief puts formatted nanosecond integer to ostream
  inline std::ostream& print(std::ostream& s, const T nanosec, const Formatter& fmt);

  template <typename T>
  class WithFormat {
   public:
    WithFormat(const T& t, const Formatter& fmt) : _t(t), _fmt(fmt) { }
    friend std::ostream & operator<< (std::ostream& s, const WithFormat& wf) {
      return print(s, wf._t.nanosec, wf._fmt);
    }
    std::string toString(bool fixed = false) const {
      std::ostringstream os;
      if (fixed) os << std::fixed;
      os << *this;
      return os.str();
    }
   private:
    const T& _t;
    const Formatter& _fmt;
  };


  namespace details {

    inline void toSecNanosec(const time::T _nanosec, time::T& sec, time::T& nanosec);

  } /* namespace details */

} /* namespace time */


static constexpr time::T TIME_INVALID = std::numeric_limits<::planning2d::time::T>::min();

/**
 * class representing a duration
 */
class Duration {
  
 public:
    
  time::T nanosec; ///< duration in nanoseconds

  //! Default constructor initializes duration to zero nanoseconds
  inline Duration() : nanosec(0) { }

  /**
   * Constructor initializing duration
   * @param[in] _nanosec Duration in nanoseconds
   */
  inline Duration(time::T _nanosec) : nanosec(_nanosec) { }

  /**
   * Constructor initializing duration
   * @param[in] t Duration in seconds
   */
  inline Duration(double _t) : nanosec(time::fromSec(_t)) { }

  /**
  * Constructor initializing duration from boost posix duration
  * @param[in] _duration Duration as boost posix type
  */
  inline Duration(const boost::posix_time::time_duration& _duration) : nanosec(time::fromBoost(_duration)) { }

  /**
   * Returns duration in seconds
   * @return duration in seconds
   */
  inline double toSec() const { return time::toSec(nanosec); }

  //! sleep for the amount of time specified by this Duration.
  inline void sleep() const;

  //! Adds duration \var rhs to time and returns new Duration object
  inline Duration operator+(const Duration &rhs) const;
  //! Subtracts duration \var rhs to time and returns new Duration object
  inline Duration operator-(const Duration &rhs) const;
  //! Multiplies duration by \var rhs
  inline Duration operator*(const double rhs) const;
  //! Divides duration by \var rhs
  inline Duration operator/(const double rhs) const;
  //! Divides duration by \var rhs
  inline double operator/(const Duration& rhs) const;
  //! Adds duration \var rhs to this Duration object
  inline Duration& operator+=(const Duration &rhs);
  //! Subtracts duration \var rhs to this Duration object
  inline Duration& operator-=(const Duration &rhs);
  //! Multiplies duration by \var rhs
  inline Duration& operator*=(const double rhs);
  //! Divides duration by \var rhs
  inline Duration& operator/=(const double rhs);
  //! Equality operator
  inline bool operator==(const Duration &rhs) const;
  //! Inequality operator
  inline bool operator!=(const Duration &rhs) const { return !(*this == rhs); }
  //! Greater than comparison operator
  inline bool operator>(const Duration &rhs) const;
  //! Less than comparison operator
  inline bool operator<(const Duration &rhs) const;
  //! Greater than or equal to than comparison operator
  inline bool operator>=(const Duration &rhs) const;
  //! Less than or equal to than comparison operator
  inline bool operator<=(const Duration &rhs) const;
  //! Cast to double
  explicit inline operator double() const;

  //! Create formatted string
  inline time::WithFormat<Duration> format(const time::Formatter& format) const;

  /**
   * Returns formatted duration string
   * @param format Duration formatter, see http://www.boost.org/doc/libs/1_35_0/doc/html/date_time/date_time_io.html
   * @return string
   */
  inline std::string format(const std::string& format) const;

  //! Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

}; /* class Duration */

//! Stream operator for Duration
inline std::ostream& operator<<(std::ostream &os, const Duration& d);

class Time {

 public:

  time::T nanosec; ///< UTC time in nanoseconds

  //! Default constructor initializes duration to a value considered as invalid
  inline Time() : nanosec(TIME_INVALID) { }

  /**
   * Constructor initializing time
   * @param _nanosec UTC time in nanoseconds
   */
  inline Time(time::T _nanosec) : nanosec(_nanosec) { }

  /**
   * Constructor initializing time
   * @param t UTC time in seconds
   */
  inline Time(double t) : nanosec(time::fromSec(t)) { }

  /**
   * Constructor initializing time from boost posix type
   * @param t Time as boost posix type
   */
  inline Time(const boost::posix_time::ptime& t) : nanosec(time::fromBoost(t)) { }

  //! Adds duration \var rhs to time and returns new Time object
  inline Time operator+(const Duration &rhs) const;
  //! Subtracts duration \var rhs from time and returns new Time object
  inline Time operator-(const Duration &rhs) const;
  //! Subtracts Time \var rhs from time and returns new Duration object
  inline Duration operator-(const Time &rhs) const;
  //! Adds duration \var rhs to this Time object
  inline Time& operator+=(const Duration &rhs);
  //! Subtracts duration \var rhs from this Time object
  inline Time& operator-=(const Duration &rhs);
  //! Equality operator
  inline bool operator==(const Time &rhs) const;
  //! Inequality operator
  inline bool operator!=(const Time &rhs) const { return !(*this == rhs); }
  //! Greater than comparison operator
  inline bool operator>(const Time &rhs) const;
  //! Less than comparison operator
  inline bool operator<(const Time &rhs) const;
  //! Greater than or equal to than comparison operator
  inline bool operator>=(const Time &rhs) const;
  //! Less than or equal to than comparison operator
  inline bool operator<=(const Time &rhs) const;

  /**
   * Returns whether the Time object holds a valid UTC time
   * @return True iff time is valid, i.e. was not constructed with default constructor and never set afterwards
   */
  inline bool isValid() const { return (nanosec != TIME_INVALID); }

  /**
   * Returns whether the Time object holds UTC time zero (Jan 1st, 1970)
   * @return True iff UTC Time hold by this object is zero
   */
  inline bool isZero() const { return nanosec == 0; }

  /**
   * Returns date string for time
   * @return Date string
   */
  inline std::string toDateString() const;

  /**
   * Returns date string for time
   * @param format Time formatter, see http://www.boost.org/doc/libs/1_35_0/doc/html/date_time/date_time_io.html
   * @return Date string
   */
  inline std::string toDateString(const std::string& format) const;

  //! Create formatted string
  inline time::WithFormat<Time> format(const time::Formatter& format) const;

  /**
   * Returns Time in seconds
   * @return Time in seconds
   */
  inline double toSec() const { return time::toSec(nanosec); }

  //! Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

};

//! Stream operator for Time
inline std::ostream& operator<<(std::ostream &os, const Time& t);


/**
 * @class Rate
 * @brief Class to help run loops at a desired frequency
 */
class Rate {
 public:
  //! Constructor, creates a Rate with \p frequency being the desired rate to run at in Hz
  inline Rate(double frequencyHz);

  //! Constructor, creates a Rate with \p cycleTime being the desired cycle time
  inline explicit Rate(const Duration& cycleTime);

  //! Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
  //! @return True if the desired rate was met for the cycle, false otherwise.
  inline bool sleep();

  //! Sets the start time for the rate to now
  inline void reset();

  //!  Get the actual run time of a cycle from start to sleep
  inline Duration cycleTime() const;

  //! Get the expected cycle time -- one over the frequency passed in to the constructor
  inline Duration expectedCycleTime() const { return _expectedCycleTime; }

 private:
  Time _start;
  Duration _expectedCycleTime = Duration((time::T)0);
  Duration _actualCycleTime = Duration((time::T)0);
};

namespace time {

//! Return current system time
inline Time getCurrentTime() {
  static const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
return Time((T)time::fromBoost(boost::posix_time::nanosec_clock::local_time() - epoch));
#else
  return Time((T)time::fromBoost(boost::posix_time::microsec_clock::local_time() - epoch));
#endif
}

//! Return current system time
inline Time now() { return getCurrentTime(); }

} /* namespace time */

} /* namespace planning2d */

#include "planner_interfaces/implementation/TimeImplementation.hpp"

#endif /* PLANNER_INTERFACES_TIME_HPP */
