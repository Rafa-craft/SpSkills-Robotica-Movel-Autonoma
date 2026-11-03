/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDistanceCmd extends CommandBase {
  
  public static final DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

  private double setpoitDistance;

  
  //PID Constants
  private double kP = 0.035;
  private double kI = 0.0;
  private double kD = 0.0;

  //Trapezoidal constants
  private static double kDt = 0.02; //The period between controller updates in seconds. 
  private static double kMaxVelocity = 544.52;
  private static double kMaxAcceleration = 272.27;

  private final TrapezoidProfile.Constraints constraints;

  private final ProfiledPIDController controller;
  /**
   * Creates a new DriveToDistanceCmd.
   */
  public DriveToDistanceCmd(double distMM ) 
  {
    
    // Use addRequirements() here to declare subsystem dependencies.
    constraints = new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
    controller = new ProfiledPIDController(kP, kI, kD, constraints , kDt);
    controller.setTolerance(5, 10);

    setpoitDistance = distMM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    controller.reset(0, 0);
    driveSubsystem.resetPositions();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(driveSubsystem.getAverageEncoderForwardDist(), setpoitDistance);

    driveSubsystem.setOmniDrive(- speed, 0.0, 0.0);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    driveSubsystem.stopMotorsDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
