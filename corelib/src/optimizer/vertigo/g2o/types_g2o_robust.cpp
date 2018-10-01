#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include "edge_switchPrior.h"
#include "edge_se2Switchable.h"
//#include "edge_se2MaxMixture.h"
#include "edge_se3Switchable.h"
#include "vertex_switchLinear.h"


G2O_REGISTER_TYPE(EDGE_SWITCH_PRIOR, EdgeSwitchPrior);
G2O_REGISTER_TYPE(EDGE_SE2_SWITCHABLE, EdgeSE2Switchable);
//G2O_REGISTER_TYPE(EDGE_SE2_MAXMIX, EdgeSE2MaxMixture);
G2O_REGISTER_TYPE(EDGE_SE3_SWITCHABLE, EdgeSE3Switchable);
G2O_REGISTER_TYPE(VERTEX_SWITCH, VertexSwitchLinear);

/*
#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(EdgeSE2SwitchableDrawAction);
  //G2O_REGISTER_ACTION(EdgeSE2MaxMixtureDrawAction);
  G2O_REGISTER_ACTION(EdgeSE3SwitchableDrawAction);
#endif
*/
