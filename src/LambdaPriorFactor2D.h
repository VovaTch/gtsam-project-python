// Including Python stuff
#include <Python.h>

// Including gtsam stuff
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>

// Including boost python to interface with python
#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/python/stl_iterator.hpp>

namespace gtsam
{

class LambdaPriorFactor: public NonlinearFactor
{
    private:

        typedef LambdaPriorFactor This;
        typedef NonlinearFactor Base;
        Vector prior_lambda_vec;
        SharedNoiseModel noise_model;

    public:

	LambdaPriorFactor(){}
        LambdaPriorFactor(const Key& plambda_key, const Vector& plambda_exp, const SharedNoiseModel& plambda_cov);
        virtual size_t dim() const;
        Vector unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const;
        Vector whitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const;
        virtual double error(const Values& x) const;
        virtual boost::shared_ptr<GaussianFactor> linearize(const Values& x) const;
        virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
        virtual NonlinearFactor::shared_ptr clone() const;

};

} // Namespace gtsam

