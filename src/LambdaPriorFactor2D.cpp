//#pragma once

// Including basic libraries
#include <iostream>
#include <ostream>
#include <functional>
#include <math.h>
#include <cmath>
#include <vector>
#include <memory>
#include <Python.h>
#include <stdexcept>
#include <string>
#include <Python.h>
#include <numeric>
#include <Eigen/Dense>

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

// Include Header
#include "LambdaPriorFactor2D.h"


// Function to convert a std::vector vector to gtsam::Vector
gtsam::Vector gtsam_vector_from_vector(std::vector<double> vec_vec)
{
    gtsam::Vector vec = gtsam::Vector::Map(vec_vec.data(), vec_vec.size());
    return vec;
}

// Point 2 from a vector; necessary for the entire thing to work
gtsam::Point2 gtsam_point2_from_vector(std::vector<double> vec_vec)
{
    gtsam::Point2 point_lambda = gtsam::Point2(vec_vec[0], vec_vec[1]);
    return point_lambda;
}


// Convert py objects to vectors
std::vector<double> listTupleToVector_Double(PyObject* incoming) {
	std::vector<double> data;
	if (PyTuple_Check(incoming)) {
		for(Py_ssize_t i = 0; i < PyTuple_Size(incoming); i++) {
			PyObject *value = PyTuple_GetItem(incoming, i);
			data.push_back( PyFloat_AsDouble(value) );
		}
	} else {
		if (PyList_Check(incoming)) {
			for(Py_ssize_t i = 0; i < PyList_Size(incoming); i++) {
				PyObject *value = PyList_GetItem(incoming, i);
				data.push_back( PyFloat_AsDouble(value) );
			}
		} else {
			throw std::logic_error("Passed PyObject pointer was not a list or tuple!");
		}
	}
	return data;
}

// Convert C++ vectors to lists
PyObject* vectorToList_Double(const std::vector<double> &data) {
  PyObject* listObj = PyList_New( data.size() );
	if (!listObj) throw std::logic_error("Unable to allocate memory for Python list");
	for (unsigned int i = 0; i < data.size(); i++) {
		PyObject *num = PyFloat_FromDouble(data[i]);
		if (!num) {
			Py_DECREF(listObj);
			throw std::logic_error("Unable to allocate memory for Python list");
		}
		PyList_SET_ITEM(listObj, i, num);
	}
	return listObj;
}

// Dot multiplication function
double dot_multiplication(std::vector<double> v_1, std::vector<double> v_2)
{
    double product = 0.0;
    int size_v_1 = v_1.size();
    if (v_1.size() != v_2.size())
        throw std::logic_error("Vectors are not at the same size");
    for (int i = 0; i != size_v_1; i++)
    {
        product = product + v_1[i] * v_2[i];
    }
    return product;
}

// Matrix/vector multiplication
std::vector<double> mat_vec_multiplication(std::vector<std::vector<double> > mat, std::vector<double> vec)
{
    int vec_length = vec.size();
    int mat_x_size = mat.size();
    int mat_y_size = mat[0].size();
    if (vec_length != mat_y_size)
        throw std::logic_error("Vector and matrix dimensions do not agree");

    //
    std::vector<double> mult_vector;
    double temp = 0.0;
    for (int i = 0; i != mat_x_size; i++)
    {
        temp = 0.0;
        for (int j = 0; j != mat_y_size; j++)
        {
            temp += mat[i][j] * vec[j];
        }
        mult_vector.push_back(temp);
    }

    return mult_vector;
}

// Fix angles to be between -pi and pi
double angle_fix(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

// FACTOR CLASS ---------------------------------------------------------------------

// Functions to convert Pose to std::vector. Necessary to talk with Libtorch because it refuses directly to talk with gtsam
std::vector<double> Pose_to_vector(gtsam::Pose2 pose)
{
    return {pose.x(), pose.y(), pose.theta()};
}

std::vector<double> Pose_to_vector(gtsam::Pose3 pose)
{
    return {pose.x(), pose.y(), pose.z(), pose.rotation().yaw(), pose.rotation().pitch(), pose.rotation().roll()};
}

// Function to convert a std::vector<vector> matrix to gtsam::Matrix
gtsam::Matrix gtsam_matrix_from_vector(std::vector<std::vector<double> > vec_mat)
{
    int rows = vec_mat.size();
    int cols = vec_mat[0].size();
    gtsam::Matrix mat; mat.setZero(rows, cols);

    for (int i = 0; i != rows; i++)
    {
        for (int j = 0; j != cols; j++)
        {
            mat(i,j) = vec_mat[i][j];
        }
    }
    return mat;
}

std::vector<double> Reverse_pose2(std::vector<double> pose2)
{
    return {-pose2[0], -pose2[1], pose2[2] + 3.1415};
}

std::vector<double> Reverse_pose3(std::vector<double> pose3)
{
    return {-pose3[0], -pose3[1], -pose3[2], pose3[3] + 3.1415, pose3[4] + 3.1415, pose3[5] + 3.1415};
}

// FACTOR CLASS ---------------------------------------------------------------------

using boost::assign::cref_list_of;
//namespace LambdaPriorFactor

namespace gtsam
{

    //Constructor
LambdaPriorFactor::LambdaPriorFactor(const Key& plambda_key, const Vector& plambda_exp, const SharedNoiseModel& plambda_cov):
    Base(cref_list_of<1>(plambda_key)),
    prior_lambda_vec(plambda_exp),
    noise_model(plambda_cov) {}

//Unwhitened error function;
Vector LambdaPriorFactor::unwhitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H) const
{
    Vector plambda = x.at<Vector>(keys_[0]);

    std::vector<double> err;
//        for (int i=0; i != prior_lambda_vec.size() ; i++ )
//        {
//            err.push_back( -prior_lambda_vec[i] + plambda(i));
//        }
    err.push_back( -prior_lambda_vec[0] + plambda(0));
    err.push_back( -prior_lambda_vec[1] + plambda(1));
    //double R_dim = prior_lambda_vec.size();
    double R_dim = 2;
    Vector err_gtsam = gtsam_vector_from_vector(err);

    Matrix H_1_t;
    H_1_t.setZero(R_dim, R_dim);

    for (int i=0 ; i != R_dim ; i++ )
    {
        H_1_t(i, i) = 1;
    }

    if (H)
    {
        (*H)[0].resize(R_dim, R_dim);
        (*H)[0] << H_1_t;
    }

    return err_gtsam;
}

//Whitened error function;
Vector LambdaPriorFactor::whitenedError(const Values& x, boost::optional<std::vector<Matrix>&> H) const
{
    Vector plambda = x.at<Vector>(keys_[0]);

    //double R_dim = prior_lambda_vec.size();
    //double R_dim = 2;

    Vector error = unwhitenedError(x, H);

    if (H)
    {
        noise_model->WhitenSystem(*H, error);
    }
    else
    {
        noise_model->whitenInPlace(error);
    }

    return error;
}

//Necessary for some reason
size_t LambdaPriorFactor::dim() const { return 1; }

//Total error
double LambdaPriorFactor::error(const Values& x) const {
    return whitenedError(x).squaredNorm();
}

// Linearization function; (do we need it?)
boost::shared_ptr<GaussianFactor> LambdaPriorFactor::linearize(const Values& x) const
{
// Only linearize if the factor is active
    if (!this->active(x))
        return boost::shared_ptr<gtsam::JacobianFactor>();

    Matrix A1;
    std::vector<Matrix> A(this->size());
    Vector b = -whitenedError(x, A);
    A1 = A[0];

    bool constrained = false;
            if(constrained)
                return GaussianFactor::shared_ptr(
                        new JacobianFactor(keys_[0], A1, b, noiseModel::Constrained::All(1)));
            else
                return GaussianFactor::shared_ptr(
                        new JacobianFactor(keys_[0], A1, b, noiseModel::Unit::Create(b.size())));

}


//Print function
void LambdaPriorFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const
{
    Base::print(s + "LambdaPriorFactor", keyFormatter);
    std::cout << "Prior Lambda: ( ";
    for( unsigned int i = 0 ; i != prior_lambda_vec.size() ; i++ )
        std::cout << prior_lambda_vec[i] << " ";
    std::cout << ")\n";
}

// @return a deep copy of this factor
NonlinearFactor::shared_ptr LambdaPriorFactor::clone() const
{
  return boost::static_pointer_cast<NonlinearFactor>(
      NonlinearFactor::shared_ptr(new This(*this)));
}

}


