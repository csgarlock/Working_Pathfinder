package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class Pathfinder {

    public TrajectoryConfig config;

    public DifferentialDriveVoltageConstraint constraint;

    public Pose2d start, end;
    public List<Translation2d> internalWaypoints;

    public Pathfinder(Pose2d start, List<Translation2d> internalWay, Pose2d end) {
        this.start = start;
        this.end = end;
        this.internalWaypoints = internalWay;

        this.constraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.sVolts,
                Constants.vVoltsSecondsPerMeter,
                Constants.aVoltsSecondsSquaredPerMeter
            ), 
            Constants.driveKinematics,
            10);


        this.config = new TrajectoryConfig(
            Constants.maxSpeedMetersPerSecond,
            Constants.maxAccelerationMetersPerSecondSquared);

        this.config.setKinematics(Constants.driveKinematics);
        this.config.addConstraint(this.constraint);
    }

    public Trajectory generateTrajectory() {
        return TrajectoryGenerator.generateTrajectory(this.start, this.internalWaypoints, this.end, this.config);
    }

}
 