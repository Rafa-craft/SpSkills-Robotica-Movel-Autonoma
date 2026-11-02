/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.studica.frc.Lidar;
import com.studica.frc.LidarMini;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Add your docs here.
 */
    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.kinematicConstants;

public class Kinematics{

  public double spBack; 
  public double spLeft; 
  public double spRight;

  public LidarMini lidar;
  
  public void kinematics(double Vx,double Vy,double Vphi)
  {
    
    
    Vx = MathUtil.clamp(Vx, -100 , 100);
    Vy = MathUtil.clamp(Vy, -100 , 100);
    Vphi = MathUtil.clamp(Vphi, -180, 180);

    Vx = Vx/100;
    Vy = Vy/100;
    Vphi = Vphi/180;

    double spBack; 
    double spLeft; 
    double spRight;

    double wheelRps = (Vphi/360)/kinematicConstants.robotAndWhellRatio;
    double motorRps = wheelRps * 61;
    double motorRpm = motorRps * 60;

    spLeft = (Vx * 0.866) - (Vy * 0.5) 
    - (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);
    spBack = Vy
    - (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);;
    spRight = -(Vx * 0.866) - (Vy * 0.5)
    - (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);;

    double max = Math.abs(spRight);
    if(Math.abs(spLeft) > max) max = Math.abs(spLeft);
    if(Math.abs(spRight) > max) max = Math.abs(spRight);

    if(max > 1)
    {
      spRight = spRight/ max;
      spLeft = spLeft/ max;
      spBack = spBack/ max;

    }
    //double rpsLeft = spLeft/ (2*wheelRps*2* Math.PI * kinematicConstants.wheelRadius);
    //double rpsback = spBack/ (2*wheelRps*2* Math.PI * kinematicConstants.wheelRadius);
    //double rpsRight = spRight/ (2*wheelRps*2* Math.PI * kinematicConstants.wheelRadius);
  
    //double ratio = 1;
    //if(spLeft > 100) ratio = 100/spLeft;
    //if(spRight > 100 && spRight > spLeft) ratio = 100/spRight;
    //if(spBack > 100 && spBack > spRight && spBack > spLeft) ratio = 100/spBack;

    //spLeft -= spLeft * ratio;
    //spBack -= spBack * ratio;
    //spRight -= spRight * ratio;

    this.spBack = spBack;
    this.spLeft = spLeft;
    this.spRight = spRight;

    
  }


  public void cinematica()
  {
    SmartDashboard.putNumber("MotorBack/Speed", spBack );
    SmartDashboard.putNumber("MotorLeft/Speed", spRight );
    SmartDashboard.putNumber("MotorRight/Speed", spLeft );
  }
}

