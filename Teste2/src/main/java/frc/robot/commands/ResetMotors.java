/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GarraSubsystem;

public class ResetMotors extends CommandBase {
  public final DriveTrainSubsystem driveTrainSubsystem;
  public final GarraSubsystem garraSubsystem;
  public final ElevatorSubsystem elevatorSubsystem;

  public boolean acabou = false;
  
  /**
   * Creates a new ResetMotors.
   */
  public ResetMotors(DriveTrainSubsystem driveTrainSubsystem, GarraSubsystem garraSubsystem, ElevatorSubsystem elevatorSubsystem) 
  {
    this.driveTrainSubsystem = driveTrainSubsystem; // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem; // Use addRequirements() here to declare subsystem dependencies.
    this.garraSubsystem = garraSubsystem ; // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSubsystem.stopMotorDrive();
    elevatorSubsystem.stopMotorElevator();
    garraSubsystem.stopMotorGarra();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.resetEncoderDrive();
    elevatorSubsystem.resetPositionElevator();
    garraSubsystem.resetEncoderGarra();
    new WaitCommand(5);
    acabou = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return acabou;
  }
}
