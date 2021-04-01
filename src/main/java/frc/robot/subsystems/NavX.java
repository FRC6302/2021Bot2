/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase{
  private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
  
  /**
   * Creates a new NavX.
   */
  public NavX() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gyroYaw", getGyroYaw());
    SmartDashboard.putBoolean("gyroIsCalibrating", gyroIsCalibrating());
  }

  public static double getGyroYaw() { //yaw is rotation left or right
    return -gyro.getYaw(); 
  }

  //navx didnt have a built in getRotation2d method (in this WPILIB version) so i had to get it like this
  public static Rotation2d getGyroRotation2d() {
    //i made yaw negative because it needs to increase as it turns left to work for DiffDrive
    return Rotation2d.fromDegrees(getGyroYaw());
  }

  public static void zeroGyroYaw() {
    gyro.zeroYaw();
  }

  public static boolean gyroIsCalibrating() {
    return gyro.isCalibrating();
  }

}
