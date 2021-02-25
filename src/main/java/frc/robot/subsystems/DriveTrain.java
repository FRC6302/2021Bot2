/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class DriveTrain extends SubsystemBase {
  WPI_TalonSRX motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
  WPI_TalonSRX motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
  WPI_TalonSRX motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
  WPI_TalonSRX motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(motorL1, motorL2);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(motorR1, motorR2);

  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  //TO DO: put encoders here:

  //TO DO: AHRS doesnt implement Gyro interface? or does it

  //Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  //TODO: put on github
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(NavX.getGyroYaw()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
