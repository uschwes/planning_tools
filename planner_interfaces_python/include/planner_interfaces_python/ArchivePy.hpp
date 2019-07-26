/*
 * SerializationSupportPy.hpp
 *
 *  Created on: 04.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_PYTHON_SERIALIZATIONSUPPORTPY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_PYTHON_SERIALIZATIONSUPPORTPY_HPP_

#include <fstream>
#include <memory>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <planner_interfaces/Exceptions.hpp>

struct OArchive {
  OArchive(const std::string& path)
      : ofs(path), ar(new boost::archive::binary_oarchive(ofs))
  {

  }
  void close() { ar.reset(); ofs.close(); }
  std::string path;
  std::ofstream ofs;
  std::unique_ptr<boost::archive::binary_oarchive> ar;
};

struct IArchive {
  IArchive(const std::string& path)
      : ifs(path), ar(new boost::archive::binary_iarchive(ifs))
  {

  }
  void close() { ar.reset(); ifs.close(); }
  std::ifstream ifs;
  std::unique_ptr<boost::archive::binary_iarchive> ar;
};

template <typename Archive, typename Type>
void registerType(Archive& archive) {
  SM_ASSERT_TRUE(planning2d::WrongUsageException, archive.ar != nullptr, "Archive already closed");
  archive.ar->template register_type<Type>();
}

#endif /* INCLUDE_PROBABILISTIC_PLANNER_PYTHON_SERIALIZATIONSUPPORTPY_HPP_ */
