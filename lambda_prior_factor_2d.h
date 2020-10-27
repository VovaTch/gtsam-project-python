/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file     gtsam_example.h
 * @brief    Example wrapper interface file for Python
 * @author   Varun Agrawal
 */

// This is an interface file for automatic Python wrapper generation.
// See gtsam.h for full documentation and more examples.

virtual class gtsam::NonlinearFactor;

// The namespace should be the same as in the c++ source code.
namespace gtsam {

#include <src/LambdaPriorFactor2D.h>
class LambdaPriorFactor: gtsam::NonlinearFactor {
	LambdaPriorFactor();
	LambdaPriorFactor(const gtsam::Key& plambda_key, const gtsam::Vector& plambda_exp, const gtsam::SharedNoiseModel& plambda_cov);
	size_t dim() const;
	//Vector unwhitenedError(const gtsam::Values& x, boost::optional<std::vector<gtsam::Matrix>&> H = boost::none) const;
	//Vector whitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H = boost::none) const;
	//double error(const Values& x) const;
	//boost::shared_ptr<GaussianFactor> linearize(const Values& x) const;
	//void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
	NonlinearFactor::shared_ptr clone() const;
};

}  // namespace gtsam
