/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.kinematicConstants;
import frc.robot.subsystems.DriveSubsystem;

public class OmniDriveCmd extends CommandBase {
  
  private final DriveSubsystem driveSubsystem;
  private final XboxController joystick;
  /**
   * Creates a new OmniDriveCmd.
   */
  public OmniDriveCmd(DriveSubsystem driveSubsystem, XboxController joystick) 
  {
    this.driveSubsystem = driveSubsystem;
    this.joystick = joystick;

    JoystickButton ligaMotorDrive = new JoystickButton(joystick, XboxController.Button.kB.value);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetPositions();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMotorsDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
