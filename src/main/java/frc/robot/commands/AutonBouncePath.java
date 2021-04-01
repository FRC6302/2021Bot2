/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class AutonBouncePath extends CommandBase {
  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  private TrajectoryConfig config;
  private Trajectory exampleTrajectory;
  private PIDController leftController;
  private PIDController rightController;


  /**
   * Creates a new AutonBouncePath.
   */
  public AutonBouncePath() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
    Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics, 10); 
    //max voltage is 10, so voltage is the same regardless of current battery voltage because the bat voltage is always >10

    // Create config for trajectory
    config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        //Pass through these two interior waypoints, making an 's' curve path
        //input as (y, -x) if you're thinking of the field as a normal Cartesian plane
        //new Translation2d(1, 0),
        //new Translation2d(2, -1)
        //new Translation2d(0.5, 0.75),
        //new Translation2d(1.0, 1.5),
        //new Translation2d(1.5, 0.75)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(1, 0, new Rotation2d(0)),
      // Pass config
      config
    );


    //String trajectoryJSON = "paths/Unnamed.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/Paths/Test.wpilib.json";
    //String trajectoryJSON = "C:/Users/admin/Documents/2021Bot2/PathWeaver/output/Test.wpilib.json";
    //Path testPath = Filesystem.getDeployDirectory().toPath().resolve(Robot.trajectoryJSON);
    //Trajectory testTrajectory = exampleTrajectory;

    leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    //PIDController leftController = new PIDController(0, 0, 0);
    //PIDController rightController = new PIDController(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
