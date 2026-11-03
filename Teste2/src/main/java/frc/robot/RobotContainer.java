/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.studica.frc.Titan;

//java imports


//WPI imports
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//Command imports
import frc.robot.commands.SimpleDrive;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveMotor;
import frc.robot.commands.ElevadorNaPosicaoCmd;
import frc.robot.commands.PIDControlerCmd;
import frc.robot.commands.ResetMotors;
//Subsystem imports
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GarraSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  private final Titan titan = new Titan(Constants.TITAN_ID);
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveSubsystem;
  private final GarraSubsystem garraSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  
  private final SimpleDrive simpleDrive; 
  private final AutoCommand autoCommand;
  private final ElevadorNaPosicaoCmd elevadorNaPosCmd;
  private final PIDControlerCmd pidControlerCmd;
  private final ResetMotors resetMotors;

  private final XboxController joystick = new  XboxController(0);


  //Define the auto selector
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  //public static Map<String, AutoCommand> autoMode = new HashMap<>();

  
  /**
   * Constructor
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() 
  {


    /**
     * Create an instance of the subsystems
     */
    driveSubsystem = new DriveTrainSubsystem(titan);
    garraSubsystem = new GarraSubsystem(titan);
    elevatorSubsystem = new ElevatorSubsystem(titan);


     /**
     * Create an instance of the commands
     */
    simpleDrive = new SimpleDrive (driveSubsystem);
    autoCommand = new AutoCommand( driveSubsystem);
    elevadorNaPosCmd = new ElevadorNaPosicaoCmd(elevatorSubsystem, 500);
    pidControlerCmd = new PIDControlerCmd(elevatorSubsystem);
    resetMotors = new ResetMotors(driveSubsystem, garraSubsystem, elevatorSubsystem);

  
    //set default command to autocommand
    garraSubsystem.setDefaultCommand(new RunCommand(() -> garraSubsystem.stopMotorGarra(), garraSubsystem));
    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.stopMotorDrive(), driveSubsystem));
    elevatorSubsystem.setDefaultCommand(new RunCommand(() -> elevatorSubsystem.stopMotorElevator(), elevatorSubsystem));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {

    // Setup SmartDashboard options

    autoChooser.setDefaultOption("nenhum", new InstantCommand());
    autoChooser.addOption("Auto Routine", new AutoCommand(driveSubsystem));
    autoChooser.addOption("Auto Time", new DriveMotor(driveSubsystem));
    SmartDashboard.putData(autoChooser);
    
    JoystickButton ligaMotorDrive = new JoystickButton(joystick, XboxController.Button.kA.value);
    JoystickButton resetMotorsButton = new JoystickButton(joystick, XboxController.Button.kB.value);

    ligaMotorDrive.whenPressed(pidControlerCmd);
    resetMotorsButton.whenPressed(resetMotors);

    
    
    //new Trigger(()->driveSubsystem.isAtSpeedDrive()).whenActive(elevatorSubsystem.ligaElevador());
    //new Trigger(()->elevatorSubsystem.isAtDistElevator()).whenActive(garraSubsystem.ligaGarra());

    //ligaMotorDrive.whileHeld(new RunCommand(() -> driveSubsystem.setMotorSpeed(), driveSubsystem).withTimeout(1));
                 // .whenReleased(new ParallelCommandGroup(new RunCommand(() -> elevatorSubsystem.stopMotorElevator(), elevatorSubsystem)
                 // ,new RunCommand(() -> garraSubsystem.stopMotorGarra(), garraSubsystem)
                 // ,new RunCommand(() -> garraSubsystem.resetEncoderGarra(), garraSubsystem)
                 // ,new RunCommand(() -> driveSubsystem.stopMotorDrive(), driveSubsystem)
                 // ,new RunCommand(() -> elevatorSubsystem.resetEncoderElevator(), elevatorSubsystem)));

                  //.whenReleased(new RunCommand(() -> elevatorSubsystem.stopMotorElevator(), elevatorSubsystem))
                  //.whenReleased(new RunCommand(() -> garraSubsystem.stopMotorGarra(), garraSubsystem));
                

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() 
    {
      //the call to return the autonomous routine that we want the scheduler to run.
      
      return autoChooser.getSelected();
    }
}
