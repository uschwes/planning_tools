#ifndef PLANNER_INTERFACES_TIME_IMPLEMENTATION_HPP
#define PLANNER_INTERFACES_TIME_IMPLEMENTATION_HPP

#include <cmath> // std::round
#include <boost/thread.hpp> // thread::sleep

#include "../Exceptions.hpp"

namespace planning2d {

namespace time {

namespace details {

void toSecNanosec(const time::T _nanosec, time::T& sec, time::T& nanosec) {
  sec = (time::T)floor(time::toSec(_nanosec));
  nanosec = (time::T)std::round(_nanosec-sec*1000000000);
}

} /* namespace details */

boost::posix_time::ptime toBoost(time::T _nanosec) {
  namespace pt = boost::posix_time;

  time::T sec, nanosec;
  details::toSecNanosec(_nanosec, sec, nanosec);

#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
  return pt::from_time_t(sec) + pt::nanoseconds(nsec);
#else
  return pt::from_time_t(sec) + pt::microseconds(nanosec/1000.0);
#endif

}

boost::posix_time::time_duration toBoostDuration(time::T _nanosec) {
  namespace pt = boost::posix_time;
  time::T sec, nanosec;
  details::toSecNanosec(_nanosec, sec, nanosec);
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
  return pt::time_duration(0, 0, sec, nanosec);
#else
  return pt::time_duration(0, 0, sec, nanosec/1000.0);
#endif
}

std::ostream& print(std::ostream& s, const T nanosec, const Formatter& fmt) {
  using namespace planning2d::time;
  std::streamsize old_precision;
  if(fmt.precision >= 0) old_precision = s.precision(fmt.precision);

  switch (fmt.unit) {
    case NANOSEC: s << static_cast<double>(nanosec); break;
    case MICROSEC: s << time::toMicroSec(nanosec); break;
    case MILLISEC: s << time::toMilliSec(nanosec); break;
    case SEC: s << time::toSec(nanosec); break;
  }
  s << fmt.bl << Formatter::UnitStrings[fmt.unit] << fmt.br;
  if(fmt.precision >= 0) s.precision(old_precision);
  return s;
}

} /* namespace time */

// ******************** //
// * Duration methods * //
// ******************** //

void Duration::sleep() const {
#if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
  boost::this_thread::sleep(boost::posix_time::nanoseconds(nanosec));
#else
  boost::this_thread::sleep(boost::posix_time::microseconds(time::toMicroSec(nanosec)));
#endif
}

Duration Duration::operator+(const Duration &rhs) const {
  return Duration(nanosec + rhs.nanosec);
}

Duration Duration::operator-(const Duration &rhs) const {
  return Duration(nanosec - rhs.nanosec);
}

Duration Duration::operator*(const double rhs) const {
  return Duration(static_cast<time::T>(std::round(nanosec*rhs)));
}

Duration Duration::operator/(const double rhs) const {
  return Duration(static_cast<time::T>(std::round(nanosec/rhs)));
}

double Duration::operator/(const Duration& rhs) const {
  return static_cast<double>(nanosec)/static_cast<double>(rhs.nanosec);
}

Duration& Duration::operator+=(const Duration &rhs) {
  this->nanosec += rhs.nanosec;
  return *this;
}

Duration& Duration::operator-=(const Duration &rhs) {
  this->nanosec -= rhs.nanosec;
  return *this;
}

Duration& Duration::operator*=(const double rhs) {
  this->nanosec = std::round(this->nanosec * rhs);
  return *this;
}

Duration& Duration::operator/=(const double rhs) {
  this->nanosec = std::round(this->nanosec / rhs);
  return *this;
}


bool Duration::operator==(const Duration &rhs) const {
  return nanosec == rhs.nanosec;
}


bool Duration::operator<(const Duration &rhs) const {
  return (nanosec < rhs.nanosec);
}


bool Duration::operator>(const Duration &rhs) const {
  return (nanosec > rhs.nanosec);
}


bool Duration::operator<=(const Duration &rhs) const {
  return (nanosec <= rhs.nanosec);
}

bool Duration::operator>=(const Duration &rhs) const {
  return (nanosec >= rhs.nanosec);
}

Duration::operator double() const {
  return this->toSec();
}

time::WithFormat<Duration> Duration::format(const time::Formatter& format) const {
  return time::WithFormat<Duration>(*this, format);
}

std::string Duration::format(const std::string& format) const {
  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->time_duration_format(format.c_str());
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << time::toBoostDuration(nanosec);
  return stream.str();
}

template<class Archive>
inline void Duration::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & nanosec;
}

// ******************** //
// *** Time methods *** //
// ******************** //

Time Time::operator+(const Duration &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return Time(nanosec + rhs.nanosec);
}

Time Time::operator-(const Duration &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return Time(nanosec - rhs.nanosec);
}

inline Duration Time::operator-(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return Duration(nanosec - rhs.nanosec);
}

Time& Time::operator+=(const Duration &rhs) {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  this->nanosec += rhs.nanosec;
  return *this;
}

Time& Time::operator-=(const Duration &rhs) {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  this->nanosec -= rhs.nanosec;
  return *this;
}

bool Time::operator==(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return nanosec == rhs.nanosec;
}

bool Time::operator<(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return (nanosec < rhs.nanosec);
}


bool Time::operator>(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return (nanosec > rhs.nanosec);
}

bool Time::operator<=(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return (nanosec <= rhs.nanosec);
}

bool Time::operator>=(const Time &rhs) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return (nanosec >= rhs.nanosec);
}

std::string Time::toDateString() const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  return boost::posix_time::to_simple_string(time::toBoost(nanosec));
}

std::string Time::toDateString(const std::string& format) const {
  SM_ASSERT_TRUE_DBG(InitializationException, this->isValid(),
                     "The time object was not initialized properly.");
  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format(format.c_str());
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << time::toBoost(nanosec);
  return stream.str();
}

time::WithFormat<Time> Time::format(const time::Formatter& format) const {
  return time::WithFormat<Time>(*this, format);
}

template<class Archive>
inline void Time::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & nanosec;
}

inline std::ostream& operator<<(std::ostream &os, const Duration& d) {
  os << d.nanosec << " [ns]";
  return os;
}

inline std::ostream& operator<<(std::ostream &os, const Time& t) {
  os << t.nanosec << " [ns]";
  return os;
}

// ******************** //
// *** Rate methods *** //
// ******************** //

Rate::Rate(double frequency)
   : _start(time::now()), _expectedCycleTime(1.0 / frequency) {

}

Rate::Rate(const Duration& d)
    : _start(time::now()), _expectedCycleTime(d) {

}

bool Rate::sleep()
{
  Time expectedEnd = _start + _expectedCycleTime;
  const Time actualEnd = time::now();

  // detect backward jumps in time
  if (actualEnd < _start)
    expectedEnd = actualEnd + _expectedCycleTime;

  //calculate the time we'll sleep for
  const Duration sleepTime = expectedEnd - actualEnd;

  //set the actual amount of time the loop took in case the user wants to know
  _actualCycleTime = actualEnd - _start;

  //make sure to reset our start time
  _start = expectedEnd;

  //if we've taken too much time we won't sleep
  if(sleepTime <= Duration(0.0)) {
    // if we've jumped forward in time, or the loop has taken more than a full extra cycle, reset our cycle
    if (actualEnd > expectedEnd + _expectedCycleTime)
      _start = actualEnd;
    // return false to show that the desired rate was not met
    return false;
  }

  sleepTime.sleep();
  return true;
}

void Rate::reset()
{
  _start = time::now();
}

Duration Rate::cycleTime() const
{
  return _actualCycleTime;
}

} /* namespace planning2d */

#endif /* PLANNER_INTERFACES_TIME_IMPLEMENTATION_HPP */
