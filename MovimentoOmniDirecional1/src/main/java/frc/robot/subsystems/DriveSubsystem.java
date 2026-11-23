/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  public final Kinematics kinematics;

  public final Titan.Motor MotorBack;
  public final Titan.Motor MotorRight;
  public final Titan.Motor MotorLeft;

  public final Titan.Encoder encoderBack;
  public final Titan.Encoder encoderRight;
  public final Titan.Encoder encoderLeft;

  private final AHRS ahrs;
  
  public double spBack;
  public double spLeft;
  public double spRight;
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem(Titan titan) 
  {
    MotorBack = titan.getMotor(Constants.DriveConstants.M3);
    encoderBack = titan.getEncoder(Constants.DriveConstants.kencoder3Channel, Constants.DriveConstants.wheelDistPerTick);
    
    MotorRight = titan.getMotor(Constants.DriveConstants.M0);
    encoderRight = titan.getEncoder(Constants.DriveConstants.kencoder0Channel, Constants.DriveConstants.wheelDistPerTick);
    
    MotorLeft = titan.getMotor(Constants.DriveConstants.M2);
    encoderLeft = titan.getEncoder(Constants.DriveConstants.kencoder2Channel, Constants.DriveConstants.wheelDistPerTick);
    
    ahrs = new AHRS(SPI.Port.kMXP);
    
    kinematics = new Kinematics();
  }

  @Override
  public void periodic() 
  {
    telemetria();
  }
  
  public void telemetria()
    {
      SmartDashboard.putNumber("EncoderBack/ dist", getEncoderBackDist());
      SmartDashboard.putNumber("EncoderBack/ speed", getSpeedEncoderBack());
      SmartDashboard.putNumber("EncoderRight/ dist", getEncoderRightDist());
      SmartDashboard.putNumber("EncoderRight/ speed", getSpeedEncoderRight());
      SmartDashboard.putNumber("EncoderLeft/ dist", getEncoderLeftDist());
      SmartDashboard.putNumber("EncoderLeft/ speed", getSpeedEncoderLeft());

      SmartDashboard.putNumber("Encoders Average/ forward dist", getAverageEncoderForwardDist());

      kinematics.cinematica();
    }
  /**
   * Define a velocidade do motor de tras
   * @param speed de -1 a 1
   */
  public void setMotorBackSpeed(double speed)
  {
    MotorBack.set(speed);
  }
  /**
   * Define a velocidade do motor da esquerda
   * @param speed de -1 a 1
   */
  public void setMotorLeftSpeed(double speed)
  {
    MotorLeft.set(speed);
  }
  /**
   * Define a velocidade do motor da direita
   * @param speed de -1 a 1
   */
  public void setMotorRightSpeed(double speed)
  {
    
    MotorRight.set(speed);
  }
  /**
   * Define a velocidade de cada motor da base
   * @param speedBack -1 a 1
   * @param speedRight -1 a 1
   * @param speedLeft -1 a 1
   */
  public void setMotorsDriveSpeed(double speedBack, double speedRight, double speedLeft)
  {
    setMotorBackSpeed(speedBack);
    setMotorLeftSpeed(speedLeft);
    setMotorRightSpeed(speedRight);
  }
  /**
   * Para os motores da base
   */
  public void stopMotorsDrive()
  {
    MotorBack.stopMotor();
    MotorLeft.stopMotor();
    MotorRight.stopMotor();
  }
  /**
   * Obtem a distancia percorrida pelo encoder de tras
   * @return
   */
  public double getEncoderBackDist()
    {
      return encoderBack.getDistance();
    }
  /**
   * Obtem a Velocidade do encoder de tras
   * @return
   */
  public double getSpeedEncoderBack()
    {
      return encoderBack.getSpeed();
    }
  /**
   * Obtem a distancia percorrida pelo encoder de tras
   * @return
   */
  public double getEncoderLeftDist()
    {
      return encoderLeft.getDistance();
    }
  /**
   * Obtem a Velocidade do encoder de tras
   * @return
   */
  public double getSpeedEncoderLeft()
    {
      return encoderLeft.getSpeed();
    }
  /**
   * Obtem a distancia percorrida pelo encoder de tras
   * @return
   */
  public double getEncoderRightDist()
    {
      return encoderRight.getDistance();
    }
  /**
   * Obtem a Velocidade do encoder de tras
   * @return
   */
  public double getSpeedEncoderRight()
    {
      return encoderRight.getSpeed();
    }

  public double getAverageEncoderForwardDist()
  {
    return (getEncoderRightDist() + -getEncoderLeftDist())/2;
  }
  /**
   * Zera o valor dos encoders
   */
  public void resetPositions()
  {
    encoderBack.reset();
    encoderRight.reset();
    encoderLeft.reset();
  }

  public void setOmniDrive(double Vx, double Vy, double Vphi)
  {
    kinematics.kinematics(Vx, Vy, Vphi);

    this.spBack = kinematics.spBack;
    this.spRight = kinematics.spRight;
    this.spLeft = kinematics.spLeft;

    setMotorsDriveSpeed(spBack, spRight, spLeft);
  }

}
