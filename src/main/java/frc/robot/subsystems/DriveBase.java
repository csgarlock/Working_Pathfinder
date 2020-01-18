/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  /**
   * Creates a new DriveBase.
   */
  private final double Conversion = 12.0 / (Math.PI * 4);

  public DifferentialDrive differDrive;

  public XboxController joystick;

  public WPI_TalonSRX drivesRightMaster, drivesRightFollower;
  public WPI_TalonSRX drivesLeftMaster, drivesLeftFollower;

  public DifferentialDriveOdometry odometry;
  public DifferentialDriveKinematics kinematics;

  public Gyro gyro;

  public Pose2d pose;

  public DriveBase() {

    gyro = new ADXRS450_Gyro();
 
    drivesRightMaster = new WPI_TalonSRX(4);
    drivesRightFollower = new WPI_TalonSRX(5);
    drivesRightFollower.follow(drivesRightMaster);

    drivesLeftMaster = new WPI_TalonSRX(1);
    drivesLeftFollower = new WPI_TalonSRX(2);
    drivesLeftFollower.follow(drivesLeftMaster);

    drivesRightMaster.setSelectedSensorPosition(0);
    drivesLeftMaster.setSelectedSensorPosition(0);

    differDrive = new DifferentialDrive(drivesLeftMaster, drivesRightMaster);

    joystick = new XboxController(0);

    pose = new Pose2d(0, 0, getGyroHeading());

    odometry = new DifferentialDriveOdometry(getGyroHeading(), pose);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(18.665));


  }

  @Override
  public void periodic() {

    differDrive.tankDrive(joystick.getRawAxis(1), joystick.getRawAxis(5));
    pose = getPose();

  }

  public void TankDriveVolts(double leftVolts, double rightVolts) {
    double left = (leftVolts / 22) + 6;
    double right = (rightVolts / 22) + 6;
    //differDrive.tankDrive(left, right, false);
    drivesRightMaster.setVoltage(rightVolts);  // sure
    //drivesRightFollower.setVoltage(rightVolts);
    drivesLeftMaster.setVoltage(leftVolts);
    //drivesLeftFollower.setVoltage(leftVolts);
  }

  public Pose2d getPose() {
    return odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
        toMeters(-drivesLeftMaster.getSelectedSensorPosition()),
        toMeters(drivesRightMaster.getSelectedSensorPosition()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-toMetersPerSec(drivesLeftMaster), toMetersPerSec(drivesRightMaster));
  }

  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double toMeters(double ticks) {
    return Units.feetToMeters((ticks / 4068.0) * Conversion);
  }

  public double toMetersPerSec(WPI_TalonSRX talon) {
    return Units.feetToMeters(((talon.getSelectedSensorVelocity() * 10) / 4068.0) * Conversion);
  }
}
