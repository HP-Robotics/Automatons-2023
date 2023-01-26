// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants { // For operator button bindings, and other stuff directly related to the
                                          // operator
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI * 4; // 1/2 rotation per second

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 2048;

    public static final double driveGearRatio = 6.75;
    public static final double rotationGearRatio = 15.429;

    public final static Translation2d kFrontLeftLocation = new Translation2d(0.244, 0.244);
    public final static Translation2d kFrontRightLocation = new Translation2d(0.244, -0.244);
    public final static Translation2d kBackLeftLocation = new Translation2d(-0.244, 0.244);
    public final static Translation2d kBackRightLocation = new Translation2d(-0.244, -0.244);

    public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

    public static final double drivekP = 0.07;
    public static final double drivekI = 0.000175;
    public static final double drivekD = 0.0;

    public static final double turningkP = .1;
    public static final double turningkI = .0002;
    public static final double turningkD = 1; //TODO: please tune these later
    public static final double turningkAllowableError = 10;

    public static final double encoderTolerance = 0.01;
  }

  public static class RobotConstants { // For robot values that remain the same, such as max speed
    public static final double swerveOffsetFL = 0.617738640443466;
    public static final double swerveOffsetFR = 0.59958238998956;
    public static final double swerveOffsetBL = 0.890661022266526;
    public static final double swerveOffsetBR = 0.125644328141108; // Fill in later
  }

  public static class ArmConstants {
    public static final int shoulderID = 22;
    public static final double shoulderkP = .1;
    public static final double shoulderkI = .0001;
    public static final double shoulderkD = 0;
    public static final boolean useArm = false;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PneumaticsConstants {

    public static final boolean usePneumatics = true;
    public static final int hubID = 49;
    public static final int minPressure = 110;
    public static final int maxPressure = 118;

  }
}
