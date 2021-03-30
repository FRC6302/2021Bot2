/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.DriveGTA;
import frc.robot.commands.Move;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  XboxController driverController;
  //XboxController operatorController;

  private final DriveTrain driveTrain;
  private final DriveGTA driveGTA;

  private final NavX gyro;

  private final Move move;

  SendableChooser<Command> chooser = new SendableChooser<>();
  //Smart Dashboard cannot be set to "Editable" if you want to select an option for auton
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverController = new XboxController(Constants.driverControllerPort);
    //operatorController = new XboxController(Constants.operatorControllerPort);

    driveTrain = new DriveTrain();
    driveGTA = new DriveGTA(driveTrain);
    driveGTA.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveGTA);

    gyro = new NavX();

    move = new Move(driveTrain);

    chooser.addOption("move", move);
    chooser.setDefaultOption("Move (default)", move);
    SmartDashboard.putData("Auton Chooser", chooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  public double getDriverRawAxis(final int axis){
    try {
      return driverController.getRawAxis(axis);
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis because: " + exception.getMessage(), true);
    }
    //this error might have something to do with the squared values in DriveGTA
    return 0;
  }

  public double getDriverDeadzoneAxis(final int axis){
    try {
    final double rawValue = driverController.getRawAxis(axis);
    return (Math.abs(rawValue) <= Constants.axisDeadzone) ? 0.0 : rawValue;
    }
    catch(final RuntimeException exception) {
      DriverStation.reportError("Error getting raw axis or returning deadzone axis because: " + exception.getMessage(), true);
    }
    return 0;
  }
  /*
  public double getOperatorDeadzoneAxis(int axis){
    double rawValue = operatorController.getRawAxis(axis);
    return Math.abs(rawValue) < Constants.deadzone ? 0.0 : rawValue;
  }
  */
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  
  // Create a voltage constraint to ensure we don't accelerate too fast
  var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
    Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics, 10); 
    //max voltage is 10, so voltage is the same regardless of current battery voltage because the bat voltage is always >10

  // Create config for trajectory
  TrajectoryConfig config =
  new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    List.of(
      //Pass through these two interior waypoints, making an 's' curve path
      //input as (y, x) if you're thinking of the field as a normal Cartesian plane
      //new Translation2d(1, 1)
      //new Translation2d(2, -1)
      new Translation2d(0.5, 0.75),
      new Translation2d(1.0, 1.5),
      new Translation2d(1.5, 0.75)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(2, 0, new Rotation2d(0)),
    // Pass config
    config
  );

  RamseteController disabledRamsete = new RamseteController() {
    @Override
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
      double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
  };

  PIDController leftController = new PIDController(Constants.kPDriveVel, 0, 0);
  PIDController rightController = new PIDController(Constants.kPDriveVel, 0, 0);
  //PIDController leftController = new PIDController(0, 0, 0);
  //PIDController rightController = new PIDController(0, 0, 0);

  RamseteCommand ramseteCommand = new RamseteCommand(
    exampleTrajectory,
    driveTrain::getPose,
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    //disabledRamsete,
    new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics,
    driveTrain::getWheelSpeeds,
    leftController,
    rightController,
    // RamseteCommand passes volts to the callback
    //driveTrain::tankDriveVolts,
    (leftVolts, rightVolts) -> {
      driveTrain.tankDriveVolts(leftVolts, rightVolts);

      SmartDashboard.putNumber("left measurement", driveTrain.getWheelSpeeds().leftMetersPerSecond);
      SmartDashboard.putNumber("left reference", leftController.getSetpoint());

      SmartDashboard.putNumber("right measurement", driveTrain.getWheelSpeeds().rightMetersPerSecond);
      SmartDashboard.putNumber("right reference", rightController.getSetpoint());
    },
    driveTrain
  );

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0)); 
    //return chooser.getSelected();
  } 
}
