/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	//motors
	public static final int motorL1Value = 0;
	public static final int motorL2Value = 1;
	public static final int motorR1Value = 2;
	public static final int motorR2Value = 3;

	//controller ports
	public static final int driverControllerPort = 0;
	//public static final int operatorControllerPort = 1;

	//controller values
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int leftStickX = 0;
    public static final int leftStickY = 1;
    public static final int rightStickX = 4;
    public static final int rightStickY = 5;
    public static final int aButton = 1;
    public static final int bButton = 2;
    public static final int xButton = 3;
    public static final int yButton = 4;
    public static final int leftBumper = 5;
    public static final int rightBumper = 6;

    //values obtained from frc drive characterization routine
    public static final double ksVolts = 0.978; //orig 1.08
    public static final double kvVoltSecondsPerMeter = 3.02; //orig 2.99
    public static final double kaVoltSecondsSquaredPerMeter = 0;//0.0561; //orig 0.0171
    public static final double kPDriveVel = 0.0133 / 10; //orig 0.065
    public static final double kTrackwidthMeters = 0.578180; //orig 0.5778765
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    //values i stole from wpilib for testing
    //public static final double ksVolts = 0.22;
    //public static final double kvVoltSecondsPerMeter = 1.98;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    //public static final double kPDriveVel = 8.5;

    
    //encoders
    public static final int leftDriveEncChannelA = 7; //roborio DIO pins that the cables are plugged into
    public static final int leftDriveEncChannelB = 8;
    public static final int rightDriveEncChannelA = 1;
    public static final int rightDriveEncChannelB = 2;
    public static final int testEncChannelA = 7;
    public static final int testEncChannelB = 8;
    
    //move command
    public static final double MoveTime = 0.5;
	public static final double leftMotorsMoveSpeed = 0.5;
	public static final double rightMotorsMoveSpeed = 0.5;
    
	//misc
	public static final double axisDeadzone = 0.4;
    public static final double turningRate = 0.7;
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    //"Reasonable baseline values for a RAMSETE follower in units of meters and seconds" -wpilib website
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
	
	
    
}
