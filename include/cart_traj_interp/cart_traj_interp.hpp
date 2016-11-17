#ifndef __CART_TRAJ_INTERP_HPP__
#define __CART_TRAJ_INTERP_HPP__

#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <memory>
#include <iostream>

class CartTrajInterp : public RTT::TaskContext
{
public:
  CartTrajInterp(const std::string& name);
  virtual ~CartTrajInterp(){}

  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
protected:
  // Input ports
  RTT::InputPort<moveit_msgs::MoveGroupActionResult> port_traj_in_;
  RTT::InputPort<Eigen::VectorXd> port_joint_position_in_;
  RTT::InputPort<Eigen::VectorXd> port_joint_velocity_in_;
  
  // Current joint position and velocity
  Eigen::VectorXd joint_position_in_, joint_velocity_in_;
  
  // Output ports
  RTT::OutputPort<KDL::FrameAcc> port_traj_pt_out_;
  RTT::OutputPort<KDL::JntArrayAcc> port_traj_joint_out_;
  
  // Chain utils
  rtt_ros_kdl_tools::ChainUtils arm_;
  
  // Trajectory in
  moveit_msgs::MoveGroupActionResult traj_in_;
  KDL::JntArrayAcc pt_in_;
  
  // Cartesian trajectory point out
  KDL::FrameAcc traj_pt_out_;
  
  // FK solver
  boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_vel_;
  
  // Jdot solver
  boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jntToJacDotSolver_;
  
  // Number of joints
  int dof_;
  
  // Specify which command point needs to be sent
  int traj_pt_nb_;
};

ORO_CREATE_COMPONENT(CartTrajInterp)
#endif