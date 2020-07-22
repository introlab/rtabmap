/**
 * Python interface for python descriptors like:
 *  - NetVLAD: https://github.com/uzh-rpg/netvlad_tf_open
 */

#ifndef PYDESCRIPTOR_H
#define PYDESCRIPTOR_H

#include <opencv2/core/mat.hpp>
#include <rtabmap/core/GlobalDescriptorExtractor.h>
#include <vector>

#include <Python.h>

namespace rtabmap
{

class PyDescriptor : public GlobalDescriptorExtractor
{
public:
  PyDescriptor(const std::string & pythonDescriptorPath, int dim = 4096);
  PyDescriptor(const ParametersMap & parameters);
  virtual ~PyDescriptor();

  const std::string & path() const {return path_;}
  float dim() const {return dim_;}

  virtual GlobalDescriptor extract(const SensorData & data) const;
  virtual GlobalDescriptorExtractor::Type getType() const {return kPyDescriptor;}

private:
  PyObject * pModule_;
  PyObject * pFunc_;
  std::string path_;
  int dim_;
};

}

#endif
