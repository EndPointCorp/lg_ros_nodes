#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <stdexcept>
#include <new>
#include <vector>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "viewport_mapper.h"
#include "util.h"
#include "lg_common/WindowGeometry.h"

using lg_common::WindowGeometry;
using lg_common::WindowGeometryPtr;

using namespace ViewportMapperTypes;

typedef WindowGeometry::_width_type WGWT;
typedef WindowGeometry::_height_type WGHT;
typedef WindowGeometry::_x_type WGXT;
typedef WindowGeometry::_y_type WGYT;

ViewportMapperStringError::ViewportMapperStringError(const char* msg): msg_(msg) {}
ViewportMapperStringError::ViewportMapperStringError(const std::string& msg): msg_(msg) {}
ViewportMapperStringError::~ViewportMapperStringError() throw() {}
const char* ViewportMapperStringError::what() const throw() {
  return msg_.c_str();
}

ViewportMapperExecError::ViewportMapperExecError(const char* msg): msg_(msg) {}
ViewportMapperExecError::ViewportMapperExecError(const std::string& msg): msg_(msg) {}
ViewportMapperExecError::~ViewportMapperExecError() throw() {}
const char* ViewportMapperExecError::what() const throw() {
  return msg_.c_str();
}

/**
 * \brief Constructor
 * \param device_name Name of the xinput device.
 * \param viewport_geometry Xorg geometry string for target viewport.
 */
ViewportMapper::ViewportMapper(const std::string& device_name, const std::string& viewport_geometry, const bool should_flip_axis, const int x_flip, const in y_flip):
  device_name_(device_name), should_flip_axis(should_flip_axis), x_flip(x_flip), y_flip(y_flip)
{
  viewport_geometry_ = GeometryFromString(viewport_geometry);
}

/**
 * \brief Map xinput events for this device.
 *
 * May throw ViewportMapperStringError, ViewportMapperExecError.
 */
void ViewportMapper::Map() const {
  WindowGeometryPtr root_geometry = GetRootGeometry();

  std::ostringstream cmd;
  std::ostringstream cmd2;
  cmd << "/usr/bin/xinput set-prop '" << device_name_ << "' 'Coordinate Transformation Matrix'";
  cmd << "/usr/bin/xinput set-prop '" << device_name << "' 'Evdev Axis Inversion' " << x_flip << " " << y_flip;

  TransformMatrixPtr m_ptr = TransformGeometry(viewport_geometry_, root_geometry);
  TransformMatrix& m = *m_ptr;

  for (std::size_t i = 0; i < m.size1(); ++i)
    for (std::size_t j = 0; j < m.size2(); ++j)
      cmd << " " << (float)m(i, j);

  int stat = system(cmd.str().c_str());
  int stat2 = system(cmd2.str().c_str());
  if (stat != 0) {
    throw ViewportMapperExecError("xinput command to transform matrix returned non-zero");
  }
  if (stat2 != 0) {
    thorw ViewportMapperExecError("xinput command to invert axis returned non-zero");
  }
}

/**
 * \brief Calculates xinput transform matrix from one window onto another.
 *
 * This is used to transform touch events onto a viewport,
 *
 * See https://wiki.archlinux.org/index.php/Calibrating_Touchscreen#Calculate_the_Coordinate_Transformation_Matrix
 *
 * \param a First window geometry.
 * \param b Second window geometry (typically the root).
 * \return Transform matrix for xinput.
 */
TransformMatrixPtr ViewportMapper::TransformGeometry(WindowGeometryPtr a, WindowGeometryPtr b) {
  TransformMatrixPtr m_ptr(new TransformMatrix(3, 3));
  TransformMatrix& m = *m_ptr;

  for (std::size_t i = 0; i < m.size1(); ++i)
    for (std::size_t j = 0; j < m.size2(); ++j)
      m(i, j) = 0.0;

  m(0, 0) = (float)a->width / (float)b->width;
  m(0, 2) = (float)a->x / (float)b->width;
  m(1, 1) = (float)a->height / (float)b->height;
  m(1, 2) = (float)a->y / (float)b->height;
  m(2, 2) = 1.0;

  return m_ptr;
}

/**
 * \brief Convert a geometry string into numeric window geometry.
 *
 * Throws ViewportMapperStringError if the geometry string is invalid.
 *
 * \param s Xorg window geometry string.
 * \return Window geometry.
 */
WindowGeometryPtr ViewportMapper::GeometryFromString(const std::string& source) {
  const std::string s = boost::trim_copy(source);
  std::size_t hwsep = s.find("x", 0);
  if (hwsep == std::string::npos) {
    throw ViewportMapperStringError("Geometry string is missing x");
  }

  std::size_t xi = s.find_first_of("+-", hwsep);
  if (xi == std::string::npos) {
    throw ViewportMapperStringError("Geometry string is missing first +-");
  }

  std::size_t yi = s.find_first_of("+-", xi+1);
  if (yi == std::string::npos) {
    throw ViewportMapperStringError("Geometry string is missing second +-");
  }

  WindowGeometryPtr geometry(new WindowGeometry);

  try {
    geometry->width = boost::lexical_cast<WGWT>(
      s.substr(0, hwsep)
    );
    geometry->height = boost::lexical_cast<WGHT>(
      s.substr(hwsep+1, xi-hwsep-1)
    );
    geometry->x = boost::lexical_cast<WGXT>(
      s.substr(xi, yi-xi)
    );
    geometry->y = boost::lexical_cast<WGYT>(
      s.substr(yi, s.length()-yi)
    );

  } catch(boost::bad_lexical_cast& e) {
    throw ViewportMapperStringError("Invalid numeric in geometry string");
  } catch(std::out_of_range& e) {
    throw ViewportMapperStringError("Substring index out of range");
  } catch(std::bad_alloc& e) {
    throw ViewportMapperStringError("Failed to allocate substring");
  }

  return geometry;
}

/**
 * \brief Find the geometry of the root Xorg window.
 * \return Root window geometry.
 */
WindowGeometryPtr ViewportMapper::GetRootGeometry() {
  const char* CMD = "/usr/bin/xwininfo -root | /usr/bin/awk '/-geometry/ { print $2 }'";

  std::string s = util::exec(CMD);

  WindowGeometryPtr geometry = GeometryFromString(s);
  return geometry;
}
