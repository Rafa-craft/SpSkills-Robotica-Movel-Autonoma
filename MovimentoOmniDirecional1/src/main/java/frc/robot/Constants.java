/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    /**
     * CAN ID's
     */
    public static final int TITAN_ID        = 42;
    
    
    /**
     * Motors
     */
    public static final int M3                      = 3; //back Motor
    public static final int M2                      = 2; //Left Motor
    public static final int M0                      = 0; //Right Motor
    
    
    /**
     * Encoders
     */
    public static final int kencoder3Channel = M3;
    public static final int kencoder2Channel = M2;
    public static final int kencoder0Channel = M0;

     // Omni wheel radius
    public static final double wheelRadius          = 55; //mm  Diametro 105 mm

    // Encoder pulse per revolution
    public static final double pulsePerRevolution   = 1440;

    // Gear Ratio between encoder and wheel 
    public static final double gearRatio            = 1.0/61.0; // 1/61 em int dava 0

    // Pulse per revolution of wheel
    public static final double wheelPulseRatio      = pulsePerRevolution * gearRatio;

    // Distance per tick
    public static final double wheelDistPerTick  = (Math.PI * 2 * wheelRadius) / pulsePerRevolution; //mm wheelPulseRatio;

    public class kinematicConstants
    {
        public static final double wheelRadius = 50.8;

        public static final double RobotCircuferenceRadius = 150;

        public static final double robotAndWhellRatio = wheelRadius / RobotCircuferenceRadius;
        
    }
    
        
        
        
      
       
    

       

    public class PIDConstants
    {
        public static final double kPositionP = 0.2;
        
        public static final double kPositionI = 0.0;
        
        public static final double kPositionD = 0.0;

		public static final double kPositionToleranceMM = 5.0;
        
        public static final double kV = 0.0;
        
        public static final double kS = 0.0;
        
        public static final double kA = 0.0;

    }
}
    
