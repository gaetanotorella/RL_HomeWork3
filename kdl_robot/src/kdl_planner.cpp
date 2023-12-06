#include "kdl_ros_control/kdl_planner.h"

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}



//trapeziodail vel pt1.a
void KDLPlanner::trapezoidal_vel(double time, double timeC,double &s, double &dot_s, double &ddot_s)
{

  double s_i = 0;
  double s_f = 1;
  double ddot_sc = -1.0/(std::pow(timeC,2) - trajDuration_*timeC) * (s_f-s_i);

  if(time <= timeC)
  {
      s = 0.5*ddot_sc*std::pow(time,2);
      dot_s = ddot_sc*time;
      ddot_s = ddot_sc;
  }

  else if(time <= trajDuration_-timeC)
  {
      s = ddot_sc*timeC*(time-timeC/2);
      dot_s = ddot_sc*timeC;
      ddot_s= 0;
  }

  else
  {
      s = s_f - 0.5*ddot_sc*std::pow(trajDuration_-time,2);
      dot_s = ddot_sc*(trajDuration_-time);
      ddot_s = - ddot_sc;
  }

}


//cubic_polinomial 1.b
void KDLPlanner::cubic_polinomial(double time, double &s, double &dot_s, double &ddot_s)
{

  double a0 = 0;      // s0 = 0
  double a1 = 0;      // dot_so = 0
  double a2 = 3/(std::pow(trajDuration_,2));       
  double a3 = -2/(std::pow(trajDuration_,3)); 

  s = a3*std::pow(time,3) + a2*std::pow(time,2) + a1*time + a0;
  dot_s = 3*a3*std::pow(time,2) + 2*a2*time + a1;
  ddot_s = 6*a3*time + 2*a2;


}

//Point - compute circular trajectory with trapeziodal vel 2.b
trajectory_point KDLPlanner::compute_circolar_trajectory(double time, double timeC, double &s,double &dot_s,double &ddot_s)
{

  //trapezoidal_vel(time, timeC, s, dot_s, ddot_s);
  cubic_polinomial(time, s, dot_s, ddot_s);

  trajectory_point traj;

  traj.pos(0) = trajInit_(0);
  traj.pos(1)= trajInit_(1) - trajRadius_ * cos(2*M_PI*s) ;
  traj.pos(2)= trajInit_(2) - trajRadius_ * sin(2*M_PI*s);

  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*trajRadius_ * sin(2*M_PI*s) * dot_s ;
  traj.vel(2) = - 2*M_PI * trajRadius_ * cos(2*M_PI*s) * dot_s;

  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI * trajRadius_ * ((cos(2*M_PI*s) * std::pow(dot_s,2) * 2*M_PI) +(sin(2*M_PI*s) * ddot_s));
  traj.acc(2) = - 2*M_PI * trajRadius_ * ((-sin(2*M_PI*s)* std::pow(dot_s,2)* 2*M_PI) + (cos(2*M_PI*s) * ddot_s));
  
  return traj;

}

//Point - compute linear trajectory with trapeziodal vel 2.c 
trajectory_point KDLPlanner::compute_linear_trajectory(double time, double timeC, double &s,double &dot_s,double &ddot_s)
{

  trajectory_point traj;

  //trapezoidal_vel(time, timeC, s, dot_s, ddot_s);
  cubic_polinomial(time, s, dot_s, ddot_s);
  traj.pos = (1-s) * trajInit_ + s * trajEnd_;
  traj.vel = (-trajInit_ + trajEnd_)*dot_s;
  traj.acc = Eigen::Vector3d::Zero();
  
  return traj;

}

trajectory_point KDLPlanner::sToTrajector(double s, double dot_s, double ddot_s)
{
  trajectory_point traj;

  traj.pos = (1-s) * trajInit_ + s * trajEnd_;
  traj.vel = -trajInit_ + trajEnd_;
  traj.acc = Eigen::Vector3d::Zero();

  return traj;
}