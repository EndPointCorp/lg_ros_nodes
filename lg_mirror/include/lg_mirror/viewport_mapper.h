#ifndef _VIEWPORT_MAPPER_H_
#define _VIEWPORT_MAPPER_H_

#include "ros/ros.h"
#include <exception>
#include <string>
#include "lg_common/WindowGeometry.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/shared_ptr.hpp>

namespace ViewportMapperTypes {
  typedef boost::numeric::ublas::matrix<float> TransformMatrix;
  typedef boost::shared_ptr<TransformMatrix> TransformMatrixPtr;
}

class ViewportMapperStringError : public std::exception {
  public:
    explicit ViewportMapperStringError(const char* msg);
    explicit ViewportMapperStringError(const std::string& msg);
    virtual ~ViewportMapperStringError() throw();
    virtual const char* what() const throw();
  protected:
    std::string msg_;
};

class ViewportMapperExecError : public std::exception {
  public:
    explicit ViewportMapperExecError(const char* msg);
    explicit ViewportMapperExecError(const std::string& msg);
    virtual ~ViewportMapperExecError() throw();
    virtual const char* what() const throw();
  protected:
    std::string msg_;
};

class ViewportMapper {
  public:
    ViewportMapper(const std::string& device_name, const std::string& viewport_geometry);
    void Map() const;
    static lg_common::WindowGeometryPtr GeometryFromString(const std::string& source);
    static lg_common::WindowGeometryPtr GetRootGeometry();
    static ViewportMapperTypes::TransformMatrixPtr TransformGeometry(
      lg_common::WindowGeometryPtr a,
      lg_common::WindowGeometryPtr b
    );

  private:
    std::string device_name_;
    lg_common::WindowGeometryPtr viewport_geometry_;
};

#endif // _VIEWPORT_MAPPER_H_
