#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include "navframe.hpp"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "isaeslam/slamCore.h"
#include <unordered_map>
#include <utilities/geometry.h>
#include <vector>

struct RelativePoseFactor
{
  Eigen::Affine3d T_a_b;
  Eigen::MatrixXd inf;
  std::shared_ptr<NavFrame> nf_a;
  std::shared_ptr<NavFrame> nf_b;
};

struct AbsolutePositionFactor
{
  Eigen::Vector3d t;
  Eigen::Matrix3d inf;
  std::shared_ptr<NavFrame> nf;
};

struct AbsolutePoseFactor
{
  Eigen::Affine3d T;
  Eigen::MatrixXd inf;
  std::shared_ptr<NavFrame> nf;
};

class PoseGraph
{
public:
  PoseGraph() {};

  void solveGraph();
  void marginalize(std::shared_ptr<NavFrame> nf);

  std::unordered_map<std::shared_ptr<NavFrame>, AbsolutePoseFactor> _nf_abspose_map;
  std::unordered_map<std::shared_ptr<NavFrame>, AbsolutePositionFactor> _nf_absfact_map;
  std::unordered_map<std::shared_ptr<NavFrame>, RelativePoseFactor> _nf_relfact_map;
};

// Residuals needed for pose graph optim
class PosePriordx : public ceres::SizedCostFunction<6, 6>
{
public:
  PosePriordx(const Eigen::Affine3d T, const Eigen::Affine3d T_prior,
              const Eigen::MatrixXd sqrt_inf)
      : _T(T), _T_prior(T_prior), _sqrt_inf(sqrt_inf) {}
  PosePriordx() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
  {
    Eigen::Map<isae::Vector6d> err(residuals);
    Eigen::Affine3d T = _T * isae::geometry::se3_doubleVec6dtoRT(parameters[0]);
    err = _sqrt_inf * isae::geometry::se3_RTtoVec6d(T * _T_prior.inverse());

    if (jacobians != NULL)
    {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setIdentity();
      Eigen::Vector3d dw =
          Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);

      Eigen::Vector3d w = isae::geometry::log_so3(
          T.rotation() * _T_prior.rotation().transpose());
      J.block(0, 0, 3, 3) = isae::geometry::so3_rightJacobian(w).inverse() *
                            _T_prior.rotation() *
                            isae::geometry::so3_rightJacobian(dw);
      J.block(3, 0, 3, 3) =
          T.rotation() *
          isae::geometry::skewMatrix(_T_prior.rotation().transpose() *
                                     _T_prior.translation()) *
          isae::geometry::so3_rightJacobian(dw);
      J.block(3, 3, 3, 3) = _T.rotation();
      J = _sqrt_inf * J;
    }

    return true;
  }

  Eigen::Affine3d _T, _T_prior;
  Eigen::MatrixXd _sqrt_inf;
};

class PositionPrior : public ceres::SizedCostFunction<3, 6>
{
public:
  PositionPrior(const Eigen::Affine3d T, const Eigen::Vector3d t_prior,
              const Eigen::Matrix3d sqrt_inf)
      : _T(T), _t_prior(t_prior), _sqrt_inf(sqrt_inf) {}
  PositionPrior() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
  {
    Eigen::Map<Eigen::Vector3d> err(residuals);
    Eigen::Affine3d T = _T * isae::geometry::se3_doubleVec6dtoRT(parameters[0]);
    err = _sqrt_inf * (T.translation() - _t_prior);

    if (jacobians != NULL)
    {
      Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J(jacobians[0]);
      J.setZero();
      J.block(0, 3, 3, 3) = _T.rotation();

      J = _sqrt_inf * J;
    }

    return true;
  }

  Eigen::Affine3d _T;
  Eigen::Vector3d _t_prior;
  Eigen::Matrix3d _sqrt_inf;
};

// Residuals for orientation calibration
class OrientationCalib2D : public ceres::SizedCostFunction<2, 1>
{
public:
  OrientationCalib2D(const Eigen::Vector3d t_w_f, const Eigen::Vector3d t_e_f,
                   const Eigen::Matrix2d sqrt_inf)
      : _t_w_f(t_w_f), _t_e_f(t_e_f), _sqrt_inf(sqrt_inf) {}
  OrientationCalib2D() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
  {

    Eigen::Map<Eigen::Vector2d> err(residuals);
    double theta = parameters[0][0];
    Eigen::Matrix2d R, Rd;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
    Rd << -std::sin(theta), -std::cos(theta), std::cos(theta), -std::sin(theta);
    Eigen::Vector2d t_e_f_est = R * _t_w_f.segment(0,2);
    err = _sqrt_inf * (t_e_f_est - _t_e_f.segment(0,2));

    if (jacobians != NULL)
    {
      Eigen::Map<Eigen::Vector2d> J(jacobians[0]);
      J =  _sqrt_inf * Rd * _t_w_f.segment(0,2); 
    }

    return true;
  }

  Eigen::Vector3d _t_w_f, _t_e_f;
  Eigen::Matrix2d _sqrt_inf;
};

class OrientationCalib : public ceres::SizedCostFunction<3, 3>
{
public:
  OrientationCalib(const Eigen::Vector3d t_w_f, const Eigen::Vector3d t_e_f,
                   const Eigen::Matrix3d sqrt_inf)
      : _t_w_f(t_w_f), _t_e_f(t_e_f) {}
  OrientationCalib() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
  {

    Eigen::Map<Eigen::Vector3d> err(residuals);
    Eigen::Vector3d w = Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Matrix3d R_e_w = isae::geometry::exp_so3(Eigen::Vector3d(parameters[0][0], parameters[0][1], parameters[0][2]));
    Eigen::Vector3d t_e_f_est = R_e_w * _t_w_f;
    err = _sqrt_inf * (t_e_f_est - _t_e_f);

    if (jacobians != NULL)
    {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J(jacobians[0]);
      J = - R_e_w * isae::geometry::skewMatrix(_t_w_f) * isae::geometry::so3_rightJacobian(w); 
    }

    return true;
  }

  Eigen::Vector3d _t_w_f, _t_e_f;
  Eigen::Matrix3d _sqrt_inf;
};

// Residuals needed for pose graph optim
class Relative6DPose : public ceres::SizedCostFunction<6, 6, 6>
{
public:
  Relative6DPose(const Eigen::Affine3d T_w_a, const Eigen::Affine3d T_w_b,
                 const Eigen::Affine3d T_a_b_prior,
                 const Eigen::MatrixXd sqrt_inf)
      : _T_w_a(T_w_a), _T_w_b(T_w_b), _T_a_b_prior(T_a_b_prior),
        _sqrt_inf(sqrt_inf) {}
  Relative6DPose() {}

  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const
  {
    Eigen::Map<isae::Vector6d> err(residuals);
    Eigen::Affine3d T_w_a_up =
        _T_w_a * isae::geometry::se3_doubleVec6dtoRT(parameters[0]);
    Eigen::Affine3d T_w_b_up =
        _T_w_b * isae::geometry::se3_doubleVec6dtoRT(parameters[1]);
    Eigen::Affine3d T_b_a_prior = _T_a_b_prior.inverse();
    Eigen::Affine3d T = T_b_a_prior * T_w_a_up.inverse() * T_w_b_up;
    err = _sqrt_inf * isae::geometry::se3_RTtoVec6d(T);

    if (jacobians != NULL)
    {

      Eigen::Vector3d tb = T_w_b_up.translation();
      Eigen::Vector3d ta = T_w_a_up.translation();
      Eigen::Vector3d w = isae::geometry::log_so3(T.rotation());

      if (jacobians[0] != NULL)
      {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(
            jacobians[0]);
        J.setIdentity();
        Eigen::Vector3d dw = Eigen::Vector3d(parameters[0][0], parameters[0][1],
                                             parameters[0][2]);

        // d(log(dr)) / d(taua)
        J.block(0, 0, 3, 3) = -isae::geometry::so3_rightJacobian(w).inverse() *
                              T_w_b_up.rotation().transpose() *
                              T_w_a_up.rotation() *
                              isae::geometry::so3_rightJacobian(dw);

        // d(dt) / d(taua)
        J.block(3, 0, 3, 3) =
            T_b_a_prior.rotation() * T_w_a_up.rotation().transpose() *
            isae::geometry::skewMatrix(tb - ta) * T_w_a_up.rotation() *
            isae::geometry::so3_rightJacobian(dw);

        // d(dt) / d(ta)
        J.block(3, 3, 3, 3) = -T_b_a_prior.rotation();
        J = _sqrt_inf * J;
      }

      if (jacobians[1] != NULL)
      {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J(
            jacobians[1]);
        J.setIdentity();
        Eigen::Vector3d dw = Eigen::Vector3d(parameters[1][0], parameters[1][1],
                                             parameters[1][2]);

        // d(log(dr)) / d(taub)
        J.block(0, 0, 3, 3) = isae::geometry::so3_rightJacobian(w).inverse() *
                              isae::geometry::so3_rightJacobian(dw);

        // d(dt) / d(tb)
        J.block(3, 3, 3, 3) = T_b_a_prior.rotation() *
                              T_w_a_up.rotation().transpose() *
                              T_w_b_up.rotation();
        J = _sqrt_inf * J;
      }
    }

    return true;
  }

  Eigen::Affine3d _T_w_a, _T_w_b, _T_a_b_prior;
  Eigen::MatrixXd _sqrt_inf;
};

#endif // POSE_GRAPH_H
