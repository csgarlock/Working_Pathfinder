/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Pathfinder;
import frc.robot.subsystems.DriveBase;

public class RamseteTest extends CommandBase {
  private final DriveBase driveBase;

  private Trajectory trajectory;

  public RamseteCommand ramseteCommand;
  /**
   * Creates a new RamseteTest.
   */
  public RamseteTest(DriveBase subsystem) {
    driveBase = subsystem;
    addRequirements(driveBase);
    // Pathfinder path = new Pathfinder(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(2, 0)),
    //     new Pose2d(4, 0, new Rotation2d(0)));
    Translation2d[] t2dEmpty = {};
    Pathfinder path = new Pathfinder(
      new Pose2d(0, 0, new Rotation2d(0)),  // initial pose (or what it should be)
      Arrays.asList(t2dEmpty),  // list of translations for 'local waypoints', probably breaks?
      new Pose2d(3, 1, new Rotation2d(Math.toRadians(0)))  // end pose; should be forward 0.5m
    );

    trajectory = path.generateTrajectory();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveBase.resetPose();
    System.out.println("TESTING TESTING 123");
    this.ramseteCommand = new RamseteCommand(
        trajectory,
        driveBase::getPose,
        new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
        new SimpleMotorFeedforward(
          Constants.sVolts,
          Constants.vVoltsSecondsPerMeter,
          Constants.aVoltsSecondsSquaredPerMeter),
        Constants.driveKinematics,
        driveBase::getWheelSpeeds,
        new PIDController(Constants.pDriveVel, 0, 0),
        new PIDController(Constants.pDriveVel, 0, 0),
        driveBase::TankDriveVolts,
        driveBase
    );

    this.ramseteCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.differDrive.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
