/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AtiradorConstants;
import frc.robot.Constants.ElevatorConstants;


/**
 * Add your docs here.
 */
public class ElevatorSubsystem extends SubsystemBase
 {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public final Titan.Motor motorElevator;
  public final Titan.Encoder encoderElevator;

  public boolean flagPosition = false;

  public PIDController pidController = new PIDController(AtiradorConstants.kPositionP, AtiradorConstants.kPositionI, AtiradorConstants.kPositionD);

  public ElevatorSubsystem(Titan titan)
  {
    motorElevator = titan.getMotor(ElevatorConstants.M0);
    encoderElevator = titan.getEncoder(ElevatorConstants.kencoder0Channel,
                                      ElevatorConstants.wheelDistPerTick);
    
    pidController.setTolerance(AtiradorConstants.kPositionToleranceMM);

  }

  @Override
    public void periodic()
    {
        telemetria();
    }

    public void setMotorSpeedElevator(double speed)
    {
      motorElevator.set(speed);
    }

    public Command ligaElevador()
    {
      return new RunCommand(() -> this.setMotorSpeedElevator(0.5), this);
    }

    public void stopMotorElevator()
    {
      motorElevator.set(0.0);
    }

    public double getEncoderDistElevator()
    {
      return encoderElevator.getDistance();
    }

    public double getSpeedEncoderElevator()
    {
      return encoderElevator.getSpeed();
    }

    public boolean isAtDistElevator()
    { 
      return pidController.atSetpoint() || getEncoderDistElevator() > 500;
    }

    public void resetPositionElevator()
    {
      encoderElevator.reset();
      pidController.reset();
      flagPosition = false;
    }

    public void runToPositionPID(double setpoint)
    {
      double output = pidController.calculate(this.getEncoderDistElevator(),
                                              setpoint
      );

      motorElevator.setVoltage(output);
    }

    public void runToPosition(double setpoint)
    {
      
      if(this.getEncoderDistElevator() < setpoint)
      {
        this.setMotorSpeedElevator(0.5);
        flagPosition = false;
      }else{
        this.stopMotorElevator();
        flagPosition = true;
      }
    }

    public void telemetria()
    {
      SmartDashboard.putNumber("ElevatorSubsystem/Velocidade RPM", this.getSpeedEncoderElevator());
      SmartDashboard.putNumber("ElevatorSubsystem/Distancia encoder", this.getEncoderDistElevator());    }


}
