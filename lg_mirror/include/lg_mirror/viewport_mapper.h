#ifndef _VIEWPORT_MAPPER_H_
#define _VIEWPORT_MAPPER_H_

#include "ros/ros.h"
#include <exception>
#include <string>
#include <lg_msg_defs/WindowGeometry.h>
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
    ViewportMapper(const std::string& device_name, const std::string& viewport_geometry,
        bool should_flip_axis, int x_flip, int y_flip);
    void Map() const;
    static lg_msg_defs::WindowGeometryPtr GeometryFromString(const std::string& source);
    static lg_msg_defs::WindowGeometryPtr GetRootGeometry();
    static ViewportMapperTypes::TransformMatrixPtr TransformGeometry(
      lg_msg_defs::WindowGeometryPtr a,
      lg_msg_defs::WindowGeometryPtr b
    );

  private:
    std::string device_name_;
    bool should_flip_axis_;
    int x_flip_;
    int y_flip_;
    lg_msg_defs::WindowGeometryPtr viewport_geometry_;
};

#endif // _VIEWPORT_MAPPER_H_
