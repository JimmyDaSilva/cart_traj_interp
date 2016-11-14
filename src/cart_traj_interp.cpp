#include <cart_traj_interp/cart_traj_interp.hpp>

CartTrajInterp::CartTrajInterp(std::string const& name) : TaskContext(name){
  this->addPort("TrajectoryIn",this->port_traj_in_);
  this->addPort("JointPosition",this->port_joint_position_in_);
  this->addPort("JointVelocity",this->port_joint_velocity_in_);
  this->addPort("TrajectoryPointOut",this->port_traj_pt_out_);
}

bool CartTrajInterp::configureHook(){
  // Initialise the model, the internal solvers etc
  if( !this->arm_.init() )
  {
    RTT::log(RTT::Error) << "Could not init chain utils !" << RTT::endlog();
    return false;
  }
  
  // The number of joints
  dof_ = this->arm_.getNrOfJoints();
  
  // Resize data
  pt_in_.q.data.resize(dof_);
  pt_in_.qdot.data.resize(dof_);
  pt_in_.qdotdot.data.resize(dof_);
  
  // Instantiate solvers
  this->fk_solver_vel_.reset(new KDL::ChainFkSolverVel_recursive(this->arm_.Chain()));
  this->jntToJacDotSolver_.reset(new KDL::ChainJntToJacDotSolver(this->arm_.Chain()));
  
  this->traj_pt_nb_ = 0;
  
  return true;
}

void CartTrajInterp::updateHook(){
  
  // Read the ports
  RTT::FlowStatus ftraj = this->port_traj_in_.read(this->traj_in_);
  RTT::FlowStatus fp = this->port_joint_position_in_.read(this->joint_position_in_);
  RTT::FlowStatus fv = this->port_joint_velocity_in_.read(this->joint_velocity_in_);
  
  // If we get a new trajectory update it and start with point 0
  if(ftraj == RTT::NewData)
  {
    RTT::log(RTT::Warning) << "\n\n\n New trajectory with "<<traj_in_.result.planned_trajectory.joint_trajectory.points.size() <<" points \n\n\n" << RTT::endlog();
    this->traj_pt_nb_ = 0;
  }

  // Return if not giving any robot update (might happend during startup)
  if(fp == RTT::NoData || fv == RTT::NoData)
    return
  
  // Update the chain model
  this->arm_.setState(this->joint_position_in_,this->joint_velocity_in_);
  this->arm_.updateModel();
  
  // Check trajectory is not empty
  if (this->traj_in_.result.planned_trajectory.joint_trajectory.points.size() < 1){
//     RTT::log(RTT::Warning) << "Empty trajectory" << RTT::endlog();
    return;
  }
  
  // Check trajectory is not finished
  if (this->traj_pt_nb_ > this->traj_in_.result.planned_trajectory.joint_trajectory.points.size() - 1){
//     RTT::log(RTT::Warning) << "All points in the current trajectory have been sent" << RTT::endlog();
    return;
  }
  
  // Pick current point in trajectory     
  for(unsigned int i = 0; i < this->dof_; ++i){
    this->pt_in_.q(i) = this->traj_in_.result.planned_trajectory.joint_trajectory.points[this->traj_pt_nb_].positions[i];
    this->pt_in_.qdot(i) = this->traj_in_.result.planned_trajectory.joint_trajectory.points[this->traj_pt_nb_].velocities[i] ;
    this->pt_in_.qdotdot(i) = this->traj_in_.result.planned_trajectory.joint_trajectory.points[this->traj_pt_nb_].accelerations[i] ;
  }
  
  // Compute FK
  KDL::FrameVel traj_pt_vel;
  KDL::JntArrayVel pt_in_vel;
  pt_in_vel.q = this->pt_in_.q;
  pt_in_vel.qdot = this->pt_in_.qdot;
  KDL::Twist JdotQdot;
  
  // If FK or JdotQdot solver fails start over 
  if ((this->fk_solver_vel_->JntToCart(pt_in_vel, traj_pt_vel) == 0) &&  (this->jntToJacDotSolver_->JntToJacDot(pt_in_vel, JdotQdot) == 0) ){
        
    KDL::Jacobian J = this->arm_.getJacobian();
    KDL::Twist xdotdot;
    
    // Xdd = Jd * qd + J * qdd
    for(unsigned int i = 0; i < 6; ++i )
      xdotdot(i) = JdotQdot(i) + J.data(i) * this->pt_in_.qdotdot(i);
    
    this->traj_pt_out_ = KDL::FrameAcc(traj_pt_vel.GetFrame(), traj_pt_vel.GetTwist(), xdotdot);
    
    // Send cartesian trajectory point to the controller
    this->port_traj_pt_out_.write(this->traj_pt_out_);

    // Increment counter
    this->traj_pt_nb_++;
    
    RTT::log(RTT::Warning) << "Sending point "<< this->traj_pt_nb_ << RTT::endlog();
  }
  
}

bool CartTrajInterp::startHook(){
  return true;
}

void CartTrajInterp::stopHook() {
}