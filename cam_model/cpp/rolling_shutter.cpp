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

namespace Eigen
{

AngleAxisd FromRotVec(const Eigen::Vector3d& rotvec)
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

class ScipyRotation
{
  public:
  ScipyRotation(){}
  private:
};

// def GetPointInROI(points, width, height, pt_size=0.4):
//     result = {}
//     for i in range(len(points)):
//         if width[0] - pt_size< points[i][0] <= width[1] + pt_size and height[0] - pt_size < points[i][1] <= height[1] + pt_size:
//             result[i] = points[i]
//     return result

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

  const Eigen::Vector3d& getLinP() const {return lin_p_;}
  const Eigen::Vector3d& getLinV() const {return lin_v_;}
  const Eigen::Vector3d& getLinA() const {return lin_a_;}
  const Eigen::Vector3d& getAngP() const {return ang_p_;}
  const Eigen::Vector3d& getAngV() const {return ang_v_;}
  const Eigen::Vector3d& getAngA() const {return ang_a_;}

  void setLinP(const Eigen::Vector3d& in) { lin_p_ = in;}
  void setLinV(const Eigen::Vector3d& in) { lin_v_ = in;}
  void setLinA(const Eigen::Vector3d& in) { lin_a_ = in;}
  void setAngP(const Eigen::Vector3d& in) { ang_p_ = in;}
  void setAngV(const Eigen::Vector3d& in) { ang_v_ = in;}
  void setAngA(const Eigen::Vector3d& in) { ang_a_ = in;}

#if 0
  PoseState operator * (const PoseState& other) const
  {
    PoseState result;
    result.setAngP(Eigen::AsRotVec(Eigen::FromRotVec(getAngP()) * Eigen::FromRotVec(other.getAngP())));
    result.setLinP(Eigen::FromRotVec(getAngP()).toRotationMatrix() * other.getLinP() + getLinP());
    return result;
  }
#endif

  PoseState Propagate(double dt)
  {
    Eigen::Vector3d lin_p = lin_p_ + lin_v_ * dt + 0.5 * lin_a_ * dt * dt;
    Eigen::Vector3d lin_v = lin_v_ + lin_a_ * dt;

    Eigen::Vector3d ang_p = Eigen::AsRotVec(Eigen::FromRotVec(0.5 * ang_a_ * dt * dt) * Eigen::FromRotVec(ang_v_ * dt) * Eigen::FromRotVec(ang_p_));
    // std::cout << "ang_p: " << ang_p.transpose() << std::endl;
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

PYBIND11_MODULE(rolling_shutter,m)
{
  m.doc() = "pybind11 rolling_shutter plugin";

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

  .def_property("lin_p", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getLinP), &PoseState::setLinP)
  .def_property("lin_v", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getLinV), &PoseState::setLinV)
  .def_property("lin_a", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getLinA), &PoseState::setLinA)
  .def_property("ang_p", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getAngP), &PoseState::setAngP)
  .def_property("ang_v", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getAngV), &PoseState::setAngV)
  .def_property("ang_a", static_cast<Eigen::Vector3d& (PoseState::*)()>(&PoseState::getAngA), &PoseState::setAngA)

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