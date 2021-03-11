/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveGTA extends CommandBase {
  private final DriveTrain driveTrain;

  /**
   * Creates a new DriveGTA.
   */
  public DriveGTA(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //setSafetyE
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftStickInput = Robot.robotContainer.getDriverDeadzoneAxis(Constants.leftStickX) 
      * Constants.turningRate;
    double triggerInput = Robot.robotContainer.getDriverRawAxis(Constants.rightTrigger) 
      - Robot.robotContainer.getDriverRawAxis(Constants.leftTrigger);

    //change boolean to true if you want inputs squared to make them less sensitive
    //driveTrain.arcadeDrive(triggerInput, leftStickInput, true);

    driveTrain.setLeftMotors(triggerInput + leftStickInput);
    driveTrain.setRightMotors(triggerInput - leftStickInput);

    //SmartDashboard.putNumber("left motors input", triggerInput + leftStickInput);
    //SmartDashboard.putNumber("right motors input", triggerInput - leftStickInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
