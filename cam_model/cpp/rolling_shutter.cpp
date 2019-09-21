#ifndef __POSE_STATE_H__
#define __POSE_STATE_H__

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <vector>
#include <map>

#if 0
namespace Eigen
{

AngleAxisd FromRotVec(const Vector3dR& rotvec)
{
  if(rotvec.isZero()) 
    return AngleAxisd(0, Vector3d::UnitX());
  return AngleAxisd(rotvec.norm(), rotvec.normalized());
}

Vector3d AsRotVec(const Quaterniond& q)
{
  AngleAxisd a = AngleAxisd(q);
  return a.axis() * a.angle();
}

}// namespace Eigen
#endif

using MatrixX3dR = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Vector3dR = Eigen::Matrix<double, 1, 3, Eigen::RowMajor>;

class ScipyRotation
{
 public:
  MatrixX3dR as_rotvec()
  {
    MatrixX3dR rotvecs(quaternions_.size(), 3);
    for(int i = 0; i < quaternions_.size(); i++)
    {
      Eigen::AngleAxisd a = Eigen::AngleAxisd(quaternions_.at(i));
      rotvecs.row(i) = a.axis() * a.angle();
    }
    return rotvecs;
  }

  ScipyRotation inv() const
  {
    std::vector<Eigen::Quaterniond> quaternions; 
    for(const auto& q : quaternions_)
      quaternions.emplace_back(q.inverse());
    return ScipyRotation(quaternions);
  }

  static ScipyRotation from_rotvec(const MatrixX3dR& rotvecs)
  {
    std::vector<Eigen::Quaterniond> quaternions;
    for(int i = 0; i < rotvecs.rows(); i++)
    {
      if(rotvecs.row(i).isZero())
        quaternions.push_back(
          Eigen::Quaterniond(
            Eigen::AngleAxisd(0, Vector3dR::UnitX())));
      else
        quaternions.push_back(
          Eigen::Quaterniond(
            Eigen::AngleAxisd(rotvecs.row(i).norm(), rotvecs.row(i).normalized())));
    }
    return ScipyRotation(quaternions);
  }

  ScipyRotation operator * (const ScipyRotation& other) const
  {
    assert(quaternions_.size() == other.quaternions_.size());
    auto result = ScipyRotation();
    for(int i = 0; i < quaternions_.size(); i++)
    {
      result.quaternions_.push_back(quaternions_.at(i) * other.quaternions_.at(i));
    }
    return result;
  }

  MatrixX3dR apply(const MatrixX3dR& rotvecs) const
  {
    MatrixX3dR result(rotvecs.rows(), 3);
    for(int i = 0; i < rotvecs.rows(); i++)
    {
      result.row(i) = (quaternions_.at(i).toRotationMatrix() * rotvecs.row(i).transpose()).transpose();
    }
    return result;
  }

  private:
  ScipyRotation(const std::vector<Eigen::Quaterniond>& qs=std::vector<Eigen::Quaterniond>()):
  quaternions_(qs)
  {}

  std::vector<Eigen::Quaterniond> quaternions_; 
};
 
using R = ScipyRotation;

#if 0
void GetPointInROI(
  const Eigen::Matrix<-1, 2>& points, 
  const std::vector<double>& width_range, 
  const std::vector<double>& height_range, 
  std::map<int, Eigen::Vector2d>& idx_pt_map,
  double point_size=0.4)
{
  assert(width_range.size() == 2);
  assert(height_range.size() == 2);

  for(int i = 0; i < points.rows(); i++)
  {
    if (width_range.at(0) - point_size < points(i, 0) 
    && width_range.at(1) + point_size > points(i, 0) 
    && height_range.at(0) - point_size < points(i, 1) 
    && height_range.at(1) + point_size > points(i, 1))
    {
      idx_pt_map[i].push_back(points.row(i));
    }
  }
}
#endif

class PoseState
{
 public:
  PoseState(
    const Vector3dR& lin_p,
    const Vector3dR& lin_v,
    const Vector3dR& lin_a,
    const Vector3dR& ang_p,
    const Vector3dR& ang_v,
    const Vector3dR& ang_a):
    lin_p_(lin_p),
    lin_v_(lin_v),
    lin_a_(lin_a),
    ang_p_(ang_p),
    ang_v_(ang_v),
    ang_a_(ang_a){}

  Vector3dR& getLinP() {return lin_p_;}
  Vector3dR& getLinV() {return lin_v_;}
  Vector3dR& getLinA() {return lin_a_;}
  Vector3dR& getAngP() {return ang_p_;}
  Vector3dR& getAngV() {return ang_v_;}
  Vector3dR& getAngA() {return ang_a_;}

  const Vector3dR& getLinP() const {return lin_p_;}
  const Vector3dR& getLinV() const {return lin_v_;}
  const Vector3dR& getLinA() const {return lin_a_;}
  const Vector3dR& getAngP() const {return ang_p_;}
  const Vector3dR& getAngV() const {return ang_v_;}
  const Vector3dR& getAngA() const {return ang_a_;}

  void setLinP(const Vector3dR& in) { lin_p_ = in;}
  void setLinV(const Vector3dR& in) { lin_v_ = in;}
  void setLinA(const Vector3dR& in) { lin_a_ = in;}
  void setAngP(const Vector3dR& in) { ang_p_ = in;}
  void setAngV(const Vector3dR& in) { ang_v_ = in;}
  void setAngA(const Vector3dR& in) { ang_a_ = in;}

#if 0
  PoseState operator * (const PoseState& other) const
  {
    PoseState result;
    result.setAngP((R::from_rotvec(getAngP()) * R::from_rotvec(other.getAngP())).as_rotvec());

    result.setLinP(R::from_rotvec(getAngP()).apply(other.getLinP()) + getLinP());
    result.setLinV(R::from_rotvec(getAngP()).apply(other.getLinV()) + getLinV());

    return result;
  }
#endif

  PoseState Propagate(double dt)
  {
    Vector3dR lin_p = lin_p_ + lin_v_ * dt + 0.5 * lin_a_ * dt * dt;
    Vector3dR lin_v = lin_v_ + lin_a_ * dt;

    Vector3dR ang_p = (R::from_rotvec(0.5 * ang_a_ * dt * dt) * R::from_rotvec(ang_v_ * dt) * R::from_rotvec(ang_p_)).as_rotvec();
    Vector3dR ang_v = ang_v_ + ang_a_ * dt;
    return PoseState(lin_p, lin_v, lin_a_, ang_p, ang_v, ang_a_);
  }

  std::vector<PoseState> PropagateN(double dt, int len, bool half_step_start=true)
  {
    std::vector<PoseState> states;
    auto current_state = *this;
    if(half_step_start)
      current_state = Propagate(-.5 * dt);

    for(int i = 0; i < len; i++)
    {
      current_state = current_state.Propagate(dt);
      states.push_back(current_state);
    }
    return states;
  }

  std::vector<PoseState> PredictNeighbors(double dt, int n_radius)
  {

    auto states_up = PropagateN(dt, n_radius);
    auto states_down = PropagateN(-dt, n_radius);

    std::vector<PoseState> states;
    states.insert(states.end(), states_down.rbegin(), states_down.rend());
    states.insert(states.end(), states_up.begin(), states_up.end());
    return states;
  }

  std::string String()
  {
    Eigen::MatrixXd output(3,6);

    output.block<1,3>(0,0) = ang_p_.transpose();
    output.block<1,3>(1,0) = ang_v_.transpose();
    output.block<1,3>(2,0) = ang_a_.transpose();

    output.block<1,3>(0,3) = lin_p_.transpose();
    output.block<1,3>(1,3) = lin_v_.transpose();
    output.block<1,3>(2,3) = lin_a_.transpose();

    std::stringstream ss;
    ss << std::endl << output << std::endl;
    return ss.str();
  }

 private:
  Vector3dR lin_p_;
  Vector3dR lin_v_;
  Vector3dR lin_a_;
  Vector3dR ang_p_;
  Vector3dR ang_v_;
  Vector3dR ang_a_;

};

// ----------------
// Python interface
// ----------------

namespace py = pybind11;
using GetterType = Vector3dR& (PoseState::*)();

PYBIND11_MODULE(rolling_shutter,m)
{
  m.doc() = "pybind11 rolling_shutter plugin";

  py::class_<PoseState>(m, "PoseState")
  .def(
    py::init<
      const Vector3dR&,
      const Vector3dR&,
      const Vector3dR&,
      const Vector3dR&,
      const Vector3dR&,
      const Vector3dR&>(),
      pybind11::arg("lin_p")=Eigen::VectorXd::Zero(3),
      pybind11::arg("lin_v")=Eigen::VectorXd::Zero(3),
      pybind11::arg("lin_a")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_p")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_v")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_a")=Eigen::VectorXd::Zero(3))

  .def_property("lin_p", static_cast<GetterType>(&PoseState::getLinP), &PoseState::setLinP)
  .def_property("lin_v", static_cast<GetterType>(&PoseState::getLinV), &PoseState::setLinV)
  .def_property("lin_a", static_cast<GetterType>(&PoseState::getLinA), &PoseState::setLinA)
  .def_property("ang_p", static_cast<GetterType>(&PoseState::getAngP), &PoseState::setAngP)
  .def_property("ang_v", static_cast<GetterType>(&PoseState::getAngV), &PoseState::setAngV)
  .def_property("ang_a", static_cast<GetterType>(&PoseState::getAngA), &PoseState::setAngA)

  // .def("Propagate", &PoseState::Propagate)
  .def("PredictNeighbors", &PoseState::PredictNeighbors)
  .def("__str__", &PoseState::String)
  .def("__repr__", &PoseState::String);

  // .def("__repr__",
  //   [](const PoseState &a) {
  //     return "<pose_state.PoseState>";
  //   }
  // );
}
#endif //__POSE_STATE_H__