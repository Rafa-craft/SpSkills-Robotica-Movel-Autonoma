/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.OmniDriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Kinematics;
import frc.robot.subsystems.LidarSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Titan titan = new Titan(Constants.TITAN_ID);
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem; 
  private final Kinematics kinematics;
  private final LidarSubsystem lidarSubsystem;

  private final OmniDriveCmd omniDriveCmd;

  private final XboxController joystick = new  XboxController(0);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    driveSubsystem = new DriveSubsystem(titan);
    kinematics = new Kinematics();
    lidarSubsystem = new LidarSubsystem();

    omniDriveCmd = new OmniDriveCmd(driveSubsystem, joystick);

    driveSubsystem.setDefaultCommand(new RunCommand(()-> driveSubsystem.stopMotorsDrive(), driveSubsystem  ));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton ligaMotorDriveDir = new JoystickButton(joystick, XboxController.Button.kA.value);
    JoystickButton ligaMotorDriveEsq = new JoystickButton(joystick, XboxController.Button.kY.value);
    JoystickButton desligaMotor = new JoystickButton(joystick, XboxController.Button.kB.value);
    //JoystickButton PutLidarData = new JoystickButton(joystick, XboxController.Button.kY.value);

    //PutLidarData.whenActive(new RunCommand(() -> lidarSubsystem.PutDataLidar(), lidarSubsystem));
    
    ligaMotorDriveEsq.whenPressed(new RunCommand (()-> driveSubsystem.setOmniDrive(0.0,100.0,0.0), driveSubsystem));
    ligaMotorDriveDir.whenPressed(new RunCommand (()-> driveSubsystem.setOmniDrive(0.0,-100.0,0.0), driveSubsystem));
    desligaMotor.whenPressed(new RunCommand (() -> driveSubsystem.stopMotorsDrive(), driveSubsystem));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
