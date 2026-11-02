/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.studica.frc.LidarMini;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarSubsystem extends SubsystemBase {
  
  private LidarMini lidarMini;

  public LidarMini.ScanData scanData;


  public boolean scanning = false;
  /**
   * Creates a new LidarSubsystem.
   */
  public LidarSubsystem() 
  {
    lidarMini = new LidarMini(LidarMini.Port.kUSB1);
    
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startScan(){
    lidarMini.start();
    scanning = true;
  }

  public void stopScan(){
    lidarMini.stop();
    scanning = false;
  }

  public void PutDataLidar(){
      scanData = lidarMini.getData();
      var d = scanData.getDistanceAt(90, 2); //esse comando trava o robo;
      //System.out.print(d);

      SmartDashboard.putNumber("LidarSubsystem/ Distance ", d );
      //SmartDashboard.putNumber("LidarSubsystem/ Degree ", scanData.angle[0] );
  }
     // public void LidarGetData()
  
}
