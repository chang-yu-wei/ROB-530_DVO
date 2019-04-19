#ifndef DVO_FACTOR
#define DVO_FACTOR

#include <gtsam/nonlinear/BetweenFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <DVO_backend.h>

class DVO_factor: public gtsam::BetweenFactor<gtsam::Pose3> {

private:
  // measurement information
  DVO_backend * DVO_handler;
  

public:

  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model image
   * @param m          Point2 measurement
   */
    DVO_factor(gtsam::Key poseKey1, gtsam::Key poseKey2, const gtsam::Pose3 p, gtsam::SharedNoiseModel model, DVO_backend* handler_ptr) :
      gtsam::BetweenFactor<gtsam::Pose3>(poseKey1, poseKey2, p, model) {
          DVO_handler = handler_ptr;
      }

    ~DVO_factor()
    {
        delete DVO_handler;
        DVO_handler = NULL;
    }

    // error function
    // @param p    the pose in Pose3
    // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
    gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2, boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    
        // note that use boost optional like a pointer
        // only calculate jacobian matrix when non-null pointer exists
        if (H1) *H1 = (gtsam::Matrix23() << 1.0, 0.0, 0.0, 
                                        0.0, 1.0, 0.0).finished();
        if (H2) *H2 = (gtsam::Matrix23() << 1.0, 0.0, 0.0, 
                                        0.0, 1.0, 0.0).finished();
        
        // return error vector
        return (gtsam::Vector2() << p.x() - mx_, p.y() - my_).finished();
    }

};

#endif