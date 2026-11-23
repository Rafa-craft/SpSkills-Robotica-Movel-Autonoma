/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


  public final Titan.Motor MotorBack;
  public final Titan.Motor MotorRight;
  public final Titan.Motor MotorLeft;

  public final Titan.Encoder encoderBack;
  public final Titan.Encoder encoderRight;
  public final Titan.Encoder encoderLeft;

  public final PIDController pid1 = new PIDController(DriveConstants.kP,DriveConstants.kI, DriveConstants.kD);
  public final PIDController pid2 = new PIDController(DriveConstants.kP,DriveConstants.kI, DriveConstants.kD);
  public final PIDController pid3 = new PIDController(DriveConstants.kP,DriveConstants.kI, DriveConstants.kD);

  private final AHRS ahrs;
  
 
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

      SmartDashboard.putNumber("MotorLeft/ RPM", MotorLeft.getRPM());
      SmartDashboard.putNumber("MotorRight/ RPM", MotorRight.getRPM());
      SmartDashboard.putNumber("MotorBack/ RPM", MotorBack.getRPM());

      SmartDashboard.putNumber("Encoders Average/ forward dist", getAverageEncoderForwardDist());

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
    return (-getEncoderRightDist() + getEncoderLeftDist())/2;
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
    Vx = Math.abs(Vx) > 0.1 ? Vx : 0.0; //se abs de x for maior que deadband manda x senao manda 0
    Vy = Math.abs(Vy) > 0.1 ? Vy : 0.0;
    Vphi = Math.abs(Vphi) > 0.1 ? Vphi : 0.0;
    

    double targetPercentBack; 
    double targetPercentLeft; 
    double targetPercentRight;

    //double wheelRps = (Vphi/360)/kinematicConstants.robotAndWhellRatio;
    //double motorRps = wheelRps * 61;
    //double motorRpm = motorRps * 60;

    targetPercentLeft = (Vx * 0.866) - (Vy * 0.5) + Vphi;
    //- (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);
    targetPercentBack = Vy + Vphi;
    //- (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);;
    targetPercentRight = -(Vx * 0.866) - (Vy * 0.5) + Vphi;
    //- (wheelRps*2* Math.PI * kinematicConstants.wheelRadius);;

    double max = Math.abs(targetPercentBack);
    if(Math.abs(targetPercentLeft) > max) max = Math.abs(targetPercentLeft);
    if(Math.abs(targetPercentRight) > max) max = Math.abs(targetPercentRight);

    if(max > 1)
    {
      targetPercentRight /= max;
      targetPercentLeft /=  max;
      targetPercentBack /=  max;

    }


    // Converte em RPM Alvo
    double targetRPMBack = targetPercentBack * DriveConstants.kMaxRPM;
    double targetRPMLeft = targetPercentLeft * DriveConstants.kMaxRPM;
    double targetRPMRight = targetPercentRight * DriveConstants.kMaxRPM;

    // Leitura real dos encoders
    double currentRPMBack = MotorBack.getRPM();
    double currentRPMLeft = MotorLeft.getRPM();
    double currentRPMRight = MotorRight.getRPM();

    // Correção PID
    double outputBack = pid1.calculate(currentRPMBack, targetRPMBack);
    double outputLeft = pid2.calculate(currentRPMLeft, targetRPMLeft);
    double outputRight = pid3.calculate(currentRPMRight, targetRPMRight);
    

    setMotorsDriveSpeed(outputBack, outputRight, outputLeft);
  }

}
