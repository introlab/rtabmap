#include "g2o/types/types_six_dof_expmap.h"

/**
 * \brief 3D edge between two SBAcam
 */
 class EdgeSE3Expmap : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3Expmap():  BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>(){}
    bool read(std::istream& is)
      {
        return false;
      }

      bool write(std::ostream& os) const
      {
        return false;
      }

    void computeError()
    {
      const g2o::VertexSE3Expmap* v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
      const g2o::VertexSE3Expmap* v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
      g2o::SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error[0]=delta.translation().x();
      _error[1]=delta.translation().y();
      _error[2]=delta.translation().z();
      _error[3]=delta.rotation().x();
      _error[4]=delta.rotation().y();
      _error[5]=delta.rotation().z();
    }

    virtual void setMeasurement(const g2o::SE3Quat& meas){
      _measurement=meas;
      _inverseMeasurement=meas.inverse();
    }

    virtual double initialEstimatePossible(const g2o::OptimizableGraph::VertexSet& , g2o::OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet& from_, g2o::OptimizableGraph::Vertex* ){
    	g2o::VertexSE3Expmap* from = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    	g2o::VertexSE3Expmap* to = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
		if (from_.count(from) > 0)
		  to->setEstimate((g2o::SE3Quat) from->estimate() * _measurement);
		else
		  from->setEstimate((g2o::SE3Quat) to->estimate() * _inverseMeasurement);
    }

    virtual bool setMeasurementData(const double* d){
      Eigen::Map<const g2o::Vector7d> v(d);
      _measurement.fromVector(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Eigen::Map<g2o::Vector7d> v(d);
      v = _measurement.toVector();
      return true;
    }

    virtual int measurementDimension() const {return 7;}

    virtual bool setMeasurementFromState() {
    	const g2o::VertexSE3Expmap* v1 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
		const g2o::VertexSE3Expmap* v2 = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
		_measurement = (v1->estimate().inverse()*v2->estimate());
		_inverseMeasurement = _measurement.inverse();
		return true;
    }

  protected:
    g2o::SE3Quat _inverseMeasurement;
};