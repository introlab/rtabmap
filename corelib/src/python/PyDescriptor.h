/**
 * Python interface for python descriptors like:
 *  - NetVLAD: https://github.com/uzh-rpg/netvlad_tf_open
 */

#ifndef PYDESCRIPTOR_H
#define PYDESCRIPTOR_H

#include <rtabmap/core/GlobalDescriptorExtractor.h>
#include "rtabmap/core/PythonInterface.h"
#include <Python.h>

namespace rtabmap
{

class PyDescriptor : public GlobalDescriptorExtractor
{
public:
  PyDescriptor(const ParametersMap & parameters = ParametersMap());
  virtual ~PyDescriptor();

  const std::string & path() const {return path_;}
  float dim() const {return dim_;}

  virtual void parseParameters(const ParametersMap & parameters);
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
