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
public class Constants
{
    public static final double sVolts = 0.459;
    public static final double vVoltsSecondsPerMeter = 1.09;
    public static final double aVoltsSecondsSquaredPerMeter = 0.0734;

    public static final double pDriveVel = 3.11;
    public static final double trackWidthMeters = 0.474091;

    public static final DifferentialDriveKinematics driveKinematics =
        new DifferentialDriveKinematics(trackWidthMeters);

    //public static final double maxSpeedMetersPerSecond = 2.00;
    public static final double maxSpeedMetersPerSecond = 0.5;  // DEBUG
    //public static final double maxAccelerationMetersPerSecondSquared = 1.00;
    public static final double maxAccelerationMetersPerSecondSquared = 0.25;  // DEBUG

    public static final double ramseteB = 2.00;
    public static final double ramseteZeta = 0.70;

    public static final boolean gyroReversed = false;
}