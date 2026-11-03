package frc.robot.subsystems;

import com.studica.frc.Titan;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GarraConstants;

public class GarraSubsystem extends SubsystemBase
{
  // private Titan titan = new Titan(Constants.TITAN_ID);
    private Titan.Motor  motorGarra; 
    private Titan.Encoder ecoderGarra; 
    public GarraSubsystem(Titan titan){
        motorGarra = titan.getMotor(GarraConstants.M2);
        ecoderGarra = titan.getEncoder(GarraConstants.kencoder2Channel,GarraConstants.wheelDistPerTick );

    }

    @Override
    public void periodic()
    {
        telemetria();
    }

    public Command ligaGarra()
    {
      return new RunCommand(() -> this.setMotorGarra(), this);
    }

    public void setMotorGarra()
    {
        motorGarra.set(0.5);
    }

    public void stopMotorGarra()
    {
        motorGarra.stopMotor();
    }

    public void resetEncoderGarra()
    {
        ecoderGarra.reset();
    }

    /**
     * Obtem a velocidade do motor em RPM
     * @return double RPM
     */
    public double getSpeedGarra()
    {
        return ecoderGarra.getSpeed();
    }

    /**
     * Obtem a velocidade do motor em RPS
     * @return
     */
    public double getSpeedGarraRPS()
    {
        return ecoderGarra.getSpeed() / 60;
    }

    /**
     * Verdadeiro se a velocidade maior que 50 rpm
     * @return Boolean se atingiu a velocidade
     */
    public boolean isAtSpeed()
    {
        return this.getSpeedGarra() > 50;
    }
    public void telemetria(){

        SmartDashboard.putBoolean("GarraSubsystem/Atingiu Velocidade", this.isAtSpeed());
        SmartDashboard.putNumber("GarraSubsystem/Velociade RPS", this.getSpeedGarraRPS());


    }
}