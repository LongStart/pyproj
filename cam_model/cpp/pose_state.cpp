#ifndef __POSE_STATE_H__
#define __POSE_STATE_H__

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <vector>

namespace Eigen
{

AngleAxisd FromRotVec(const Eigen::Vector3d& rotvec)
{
  return Eigen::AngleAxisd(rotvec.norm(), rotvec.normalized());
}

Vector3d AsRotVec(const Quaterniond& q)
{
  AngleAxisd a = AngleAxisd(q);
  return a.axis() * a.angle();
}

}

class PoseState
{
 public:
  PoseState(
    const Eigen::Vector3d& lin_p,
    const Eigen::Vector3d& lin_v,
    const Eigen::Vector3d& lin_a,
    const Eigen::Vector3d& ang_p,
    const Eigen::Vector3d& ang_v,
    const Eigen::Vector3d& ang_a):
    lin_p_(lin_p),
    lin_v_(lin_v),
    lin_a_(lin_a),
    ang_p_(ang_p),
    ang_v_(ang_v),
    ang_a_(ang_a){}

  Eigen::Vector3d& getLinP() {return lin_p_;}
  Eigen::Vector3d& getLinV() {return lin_v_;}
  Eigen::Vector3d& getLinA() {return lin_a_;}
  Eigen::Vector3d& getAngP() {return ang_p_;}
  Eigen::Vector3d& getAngV() {return ang_v_;}
  Eigen::Vector3d& getAngA() {return ang_a_;}

  void setLinP(const Eigen::Vector3d& in) { lin_p_ = in;}
  void setLinV(const Eigen::Vector3d& in) { lin_v_ = in;}
  void setLinA(const Eigen::Vector3d& in) { lin_a_ = in;}
  void setAngP(const Eigen::Vector3d& in) { ang_p_ = in;}
  void setAngV(const Eigen::Vector3d& in) { ang_v_ = in;}
  void setAngA(const Eigen::Vector3d& in) { ang_a_ = in;}

  PoseState Propagate(double dt)
  {
    Eigen::Vector3d lin_p = lin_p_ + lin_v_ * dt + 0.5 * lin_a_ * dt * dt;
    Eigen::Vector3d lin_v = lin_v_ + lin_a_ * dt;

    Eigen::Vector3d ang_p = Eigen::AsRotVec(Eigen::FromRotVec(0.5 * ang_a_ * dt * dt) * Eigen::FromRotVec(ang_v_ * dt) * Eigen::FromRotVec(ang_p_));
    Eigen::Vector3d ang_v = ang_v_ + ang_a_ * dt;
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
    states.insert(states.end(), states_up.begin(), states_up.end());
    states.insert(states.end(), states_down.rbegin(), states_down.rend());

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
  Eigen::Vector3d lin_p_;
  Eigen::Vector3d lin_v_;
  Eigen::Vector3d lin_a_;
  Eigen::Vector3d ang_p_;
  Eigen::Vector3d ang_v_;
  Eigen::Vector3d ang_a_;

};

// ----------------
// Python interface
// ----------------

namespace py = pybind11;

PYBIND11_MODULE(pose_state,m)
{
  m.doc() = "pybind11 example plugin";

  py::class_<PoseState>(m, "PoseState")
  .def(
    py::init<
      const Eigen::Vector3d&,
      const Eigen::Vector3d&,
      const Eigen::Vector3d&,
      const Eigen::Vector3d&,
      const Eigen::Vector3d&,
      const Eigen::Vector3d&>(),
      pybind11::arg("lin_p")=Eigen::VectorXd::Zero(3),
      pybind11::arg("lin_v")=Eigen::VectorXd::Zero(3),
      pybind11::arg("lin_a")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_p")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_v")=Eigen::VectorXd::Zero(3),
      pybind11::arg("ang_a")=Eigen::VectorXd::Zero(3))

  .def_property("lin_p", &PoseState::getLinP, &PoseState::setLinP)
  .def_property("lin_v", &PoseState::getLinV, &PoseState::setLinV)
  .def_property("lin_a", &PoseState::getLinA, &PoseState::setLinA)
  .def_property("ang_p", &PoseState::getAngP, &PoseState::setAngP)
  .def_property("ang_v", &PoseState::getAngV, &PoseState::setAngV)
  .def_property("ang_a", &PoseState::getAngA, &PoseState::setAngA)

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