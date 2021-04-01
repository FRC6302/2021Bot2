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

    chooser.addOption("BouncePath", getAutonBouncePathCommand());
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

  public Command getAutonBouncePathCommand() {
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
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        //Pass through these two interior waypoints, making an 's' curve path
        //input as (y, -x) if you're thinking of the field as a normal Cartesian plane
        new Translation2d(1, 0),
        new Translation2d(2, -1)
        //new Translation2d(0.5, 0.75),
        //new Translation2d(1.0, 1.5),
        //new Translation2d(1.5, 0.75)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
    );

    
    //String trajectoryJSON = "paths/Unnamed.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/Paths/Test.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/output/Test.wpilib.json";
    //Path testPath = Filesystem.getDeployDirectory().toPath().resolve(Robot.trajectoryJSON);
    //Trajectory testTrajectory = exampleTrajectory; //new Trajectory(Trajectory.State(1., 1., 1., new Pose2d(0, 0, new Rotation2d(0)), 1.));

    /*RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
    };*/

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
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

    /*Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
      new Pose2d(1.3, -2.2, new Rotation2d(0)),
      List.of(
        //Pass through these two interior waypoints, making an 's' curve path
        //input as (y, -x) if you're thinking of the field as a normal Cartesian plane
        //new Translation2d(1, 1),
        //new Translation2d(2, -1)
        //new Translation2d(0.5, 0.75),
        //new Translation2d(1.0, 1.5),
        //new Translation2d(1.5, 0.75)
        new Translation2d(2.4, -2.1),
        //new Translation2d(2.9, -2.1),
        //new Translation2d(3.4, -2.2),
        new Translation2d(3.76, -2.27),
        //new Translation2d(4.05, -2.49),
        //new Translation2d(4.18, -2.68),
        //new Translation2d(4.29, -2.89),
        //new Translation2d(4.35, -3.15),
        //new Translation2d(4.31, -3.40),
        //new Translation2d(4.13, -3.48),
        new Translation2d(3.83, -3.53),
        //new Translation2d(3.52, -3.49),
        //new Translation2d(3.34, -3.26),
        //new Translation2d(3.38, -2.99),
        new Translation2d(3.51, -2.77),
        //new Translation2d(3.91, -2.50),
        new Translation2d(4.26, -2.42),
        new Translation2d(4.93, -2.37),
        new Translation2d(5.59, -2.28)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(5.82, -2.21, new Rotation2d(-0.3)),
      // Pass config
      config
    );*/

    
    //String trajectoryJSON = "paths/Unnamed.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/Paths/Test.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/output/Test.wpilib.json";
    //Path testPath = Filesystem.getDeployDirectory().toPath().resolve(Robot.trajectoryJSON);
    //Trajectory testTrajectory = exampleTrajectory; //new Trajectory(Trajectory.State(1., 1., 1., new Pose2d(0, 0, new Rotation2d(0)), 1.));

    /*RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
    };*/
    Trajectory exampleTrajectory = Robot.testTrajectory;

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
