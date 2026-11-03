/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//Vendor imports
import com.studica.frc.Titan;
import frc.robot.Constants;

/**
 * DriveTrain class
 * <p>
 * This class creates the instance of the Titan and enables and sets the speed of the defined motor.
 */

public class DriveTrainSubsystem extends SubsystemBase 
{
  /**
     * Motors
     */
    //private Titan titan;
  private Titan.Motor motor;
  private Titan.Encoder encoder;

    /**
     * Constructor
     */
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrainSubsystem(Titan titan) 
  {
    //Motors
    titan = new Titan(Constants.TITAN_ID);
    motor = titan.getMotor(Constants.M3);
    encoder = titan.getEncoder(Constants.kencoder3Channel, Constants.wheelDistPerTick);
  }

  @Override
  public void periodic() 
  {
    telemetria();
  }

  /**
     * Sets the speed of the motor
     * <p>
     * @param speed range -1 to 1 (0 stop)
     */
    public void setMotorSpeed( )
    {
        motor.set(0.5);
    }

    public Command ligaMotor()
    {

      return new SequentialCommandGroup(new InstantCommand(() -> this.setMotorSpeed(), this),
             new WaitCommand(5),
             new RunCommand(() -> this.stopMotorDrive(), this));
    }

    /**
     * Stops the motor drive
     */
    public void stopMotorDrive()
    {
      motor.set(0.0);
    }


    public Command DesligaMotor()
    {
      return new RunCommand(() -> this.stopMotorDrive(), this);
    }

    public double getSpeedEncoderDrive()
    {
      return encoder.getSpeed();
    }

    public double getDistEncoderDrive()
    {
      return encoder.getDistance();
    }

    public boolean isAtSpeedDrive()
    {
      return getDistEncoderDrive() > 10; 
    }

    public void resetEncoderDrive()
    {
      encoder.reset();
    }
    public void telemetria(){

      SmartDashboard.putBoolean("driveSubsystem/Atingiu Velocidade", this.isAtSpeedDrive());
      SmartDashboard.putNumber("driveSubsystem/Velociade RPM", this.getSpeedEncoderDrive());
      SmartDashboard.putNumber("driveSubsystem/Distancia encoder", this.getDistEncoderDrive());


  }
}
