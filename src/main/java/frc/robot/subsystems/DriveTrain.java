/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//MAKING THIS CLASS (AND PROJECT) WITH THE WPILIB TRAJECTORY TUTORIAL
public class DriveTrain extends SubsystemBase {
  WPI_TalonSRX motorL1 = new WPI_TalonSRX(Constants.motorL1Value);
  WPI_TalonSRX motorL2 = new WPI_TalonSRX(Constants.motorL2Value);
  WPI_TalonSRX motorR1 = new WPI_TalonSRX(Constants.motorR1Value);
  WPI_TalonSRX motorR2 = new WPI_TalonSRX(Constants.motorR2Value);

  private final NeutralMode motorMode = NeutralMode.Brake;

  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;

  //private final DifferentialDrive diffDrive;
  //TO DO: SuppressWarnings class ??
  //TO DO: put tiny blacks screws and plastic cover back on talon. Uses 3/32 allen

  //encoder cycles per rev = pulse per rev
  //I am using type 1x here because the data is too noisy on the others
  private final Encoder leftDriveEnc = new Encoder(Constants.leftDriveEncChannelA, 
    Constants.leftDriveEncChannelB, false, CounterBase.EncodingType.k1X);
  private final Encoder rightDriveEnc = new Encoder(Constants.rightDriveEncChannelA, 
    Constants.rightDriveEncChannelB, true, CounterBase.EncodingType.k1X);
  //private final Encoder testEnc = new Encoder(Constants.testEncChannelA, 
    //Constants.testEncChannelB, false);

  //private final AnalogEncoder leftDriveEnc2 = new AnalogEncoder(new AnalogInput(0));
  //private final AnalogEncoder rightDriveEnc2 = new AnalogEncoder(new AnalogInput(3));

  /*
  Distance Per Pulse (dpp) calculation explanation:
  distance per pulse is pi * (wheel diameter / counts per revolution) according to Andymark enc example code
  using meters for wheel diam because tutorial said to
  counts per rev is 8192 for Rev Through Bore Encoder
  */
  private final double driveEncDPP = Math.PI * 0.1524 / 2048; //0.152400 meters is 6 inches
  //private final double driveEncDPP = 1 / 2048; //in this case the distance unit is one rotation i think

  //odometry class for tracking robot pose
  //pose is the robot's position and orientation
  //odometry is "the use of data from motion sensors to estimate change in position over time" -wikipedia
  private final DifferentialDriveOdometry odometry;

  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    //leftDriveEnc2.reset();
    //rightDriveEnc2.reset();

    leftDriveEnc.setDistancePerPulse(driveEncDPP);
    rightDriveEnc.setDistancePerPulse(driveEncDPP);
    //the samples to average thing makes the data less noisy
    //dont change number without testing the new number with the frc drive charaterization data plots
    leftDriveEnc.setSamplesToAverage(30); //30
    rightDriveEnc.setSamplesToAverage(30);

    //leftDriveEnc2.setDistancePerRotation(driveEncDPP);
    //rightDriveEnc2.setDistancePerRotation(driveEncDPP);

    motorL1.setNeutralMode(motorMode);
    motorL2.setNeutralMode(motorMode);
    motorR1.setNeutralMode(motorMode);
    motorR2.setNeutralMode(motorMode);

    motorL1.setSafetyEnabled(true);
    motorL2.setSafetyEnabled(true);
    motorR1.setSafetyEnabled(true);
    motorR2.setSafetyEnabled(true);

    //motorL1.setExpiration(30);
    //motorL2.setExpiration(30);
    //motorR1.setExpiration(10);
    //motorR2.setExpiration(10);

    leftMotors = new SpeedControllerGroup(motorL1, motorL2);
    rightMotors = new SpeedControllerGroup(motorR1, motorR2);
    //diffDrive = new DifferentialDrive(leftMotors, rightMotors);

    //encoders have to be set to zero before constructing odometry class
    resetEncoders();
    NavX.zeroGyroYaw();
    
    odometry = new DifferentialDriveOdometry(NavX.getGyroRotation2d());
    
  }
  //TODO: follow along troubleshooting trajectory wpilib article
  //TODO: make a test path with PathWeaver and then put it into trajectory. If that doesnt work then post to chief delphi
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(NavX.getGyroRotation2d(), leftDriveEnc.getDistance(), rightDriveEnc.getDistance());
    var translation = odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("translation x", translation.getX());
    SmartDashboard.putNumber("translation y", translation.getY());
    SmartDashboard.putNumber("rotation2d", odometry.getPoseMeters().getRotation().getDegrees());

    SmartDashboard.putNumber("leftDriveEncDistance", leftDriveEnc.getDistance());
    SmartDashboard.putNumber("leftDriveEncRate", leftDriveEnc.getRate());
    SmartDashboard.putNumber("leftDriveEncCount", leftDriveEnc.get());
    SmartDashboard.putNumber("rightDriveEncDistance", rightDriveEnc.getDistance());
    SmartDashboard.putNumber("rightDriveEncRate", rightDriveEnc.getRate());
    SmartDashboard.putNumber("rightDriveEncCount", rightDriveEnc.get());
    SmartDashboard.putNumber("motorL1 output percent", motorL1.getMotorOutputPercent());
    //SmartDashboard.putBoolean("leftEncStopped", leftDriveEnc.getStopped());
    //SmartDashboard.putBoolean("rightEncStopped", rightDriveEnc.getStopped());

    motorL1.feed();
    motorL2.feed();
    motorR1.feed();
    motorR2.feed();

  }

  /*public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    diffDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("zRotation", zRotation);
  }*/

  public void setLeftMotors(double speed){
    motorL1.set(ControlMode.PercentOutput, speed);
    motorL2.set(ControlMode.PercentOutput, speed);
  }

  //right motors have inverted speed bc of how the motors are oriented on robot
  public void setRightMotors(double speed){
    motorR1.set(ControlMode.PercentOutput, -speed);
    motorR2.set(ControlMode.PercentOutput, -speed);
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);

    //idrk what these do but the project had it so
    motorL1.feed();
    motorL2.feed();
    motorR1.feed();
    motorR2.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveEnc.getRate(), rightDriveEnc.getRate());
  }

  public void setMaxOutput(double maxOutput) {
    //diffDrive.setMaxOutput(maxOutput);
  }

  public void stopDrive() {
    //motorL1.setNeutralMode(NeutralMode.Brake);
    //motorL2.setNeutralMode(NeutralMode.Brake);
    //motorR1.setNeutralMode(NeutralMode.Brake);
    //motorR2.setNeutralMode(NeutralMode.Brake);    
    
    motorL1.set(ControlMode.PercentOutput, 0);
    motorL2.set(ControlMode.PercentOutput, 0);
    motorR1.set(ControlMode.PercentOutput, 0);
    motorR2.set(ControlMode.PercentOutput, 0);
  }

  public void resetEncoders() {
    leftDriveEnc.reset();
    rightDriveEnc.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, NavX.getGyroRotation2d());
  }
  
}
