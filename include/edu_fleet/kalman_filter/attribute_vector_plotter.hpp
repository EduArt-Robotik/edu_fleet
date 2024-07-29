/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_fleet/kalman_filter/attribute.hpp"
#include "edu_fleet/kalman_filter/attribute_vector.hpp"

#include <fstream>
#include <ios>
#include <stdexcept>

namespace eduart {
namespace fleet {
namespace kalman_filter {

template <class>
class AttributeVectorPlotter;

template <Attribute... Attributes>
class AttributeVectorPlotter<AttributeVector<Attributes...>>
{
public:
  AttributeVectorPlotter(const std::string& file_name)
    : _file(file_name)
  {
    if (_file.is_open() == false) {
      throw std::runtime_error("AttributeVectorPlotter: can't create file");
    }

    printHeader();
  }
  ~AttributeVectorPlotter()
  {
    _file.close();
  }

  void printState(const AttributeVector<Attributes...>& state, const Data stamp) {
    const auto& vector = state.get();

    _file << stamp << ";";
    ((_file << vector[state.template index<Attributes>()] << ";"), ...);
    _file << std::endl;
  }

private:
  void printHeader() {
    _file << "stamp (s);";
    ((_file << attribute_name<Attributes>() << ";"), ...);
    _file << std::endl;
  }

  std::ofstream _file;
};

} // end namespace kalman_filter
} // end namespace fleet
} // end namespace eduart
