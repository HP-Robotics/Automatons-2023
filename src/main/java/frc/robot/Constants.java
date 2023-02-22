// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.I2C;

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

  public static class VisionConstants {
    public static String kcameraName = "Arducam_Global_Shutter";
    public static boolean kHasVision = true;
    public static Transform2d leftTrans = new Transform2d(new Translation2d(0.5, 0.6096), new Rotation2d(0));
    public static Transform2d rightTrans = new Transform2d(new Translation2d(0.5, -0.6096), new Rotation2d(0));
    public static Transform2d centerTrans = new Transform2d(new Translation2d(0.5, 0), new Rotation2d(0));
    public static Transform2d cameraToRobot = new Transform2d(new Translation2d(0, 0), new Rotation2d(Math.PI));
  }

  public static String autonomousMode = "NotVision";

  public static class OperatorConstants { // For operator button bindings, and other stuff directly related to the
                                          // operator
    public static final int kDriverControllerPort = 0;
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = false; // drive disabled, reenable later
    public static final boolean useArm = false;
    public static final boolean usePneumatics = false;
    public static final boolean useTurnTables = false;
    public static final boolean useIntake = false;
    public static final boolean useLimelight = false;
    public static final boolean useVision = false;
    public static final boolean useDataManger = false;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 4.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 2048;

    public static final double driveGearRatio = 6.75;
    public static final double rotationGearRatio = 15.429;

    public final static Translation2d kFrontLeftLocation = new Translation2d(0.244, 0.244);
    public final static Translation2d kFrontRightLocation = new Translation2d(0.244, -0.244);
    public final static Translation2d kBackLeftLocation = new Translation2d(-0.244, 0.244);
    public final static Translation2d kBackRightLocation = new Translation2d(-0.244, -0.244);

    public final static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackRightLocation, kBackLeftLocation);

    public static final double drivekP = 0.0015; //old value: 0.0005 OR 0.07
    public static final double drivekI = 0.0001; //old value: 0.000175 new old values: 0.0
    public static final double drivekD = 0.5; //old value: 0.0
    public static final double drivekF = 0.048;
    public static final double drivekAllowableError = 50;
    public static final double drivekMaxIntegralAccumulation = 20000; // TODO: a guess, finetune later
    public static final double drivekIntegralZone = 300; //TODO: a guess, finetune later

    public static final double turningkP = .5;
    public static final double turningkI = .000;
    public static final double turningkD = 4; //TODO: please tune for final robot eventually
    public static final double turningkAllowableError = 50;

    public static final double balancekP = 0.05;
    public static final double balancekI = 0.0001;
    public static final double balancekD = 0.0075;
    public static final int balanceThreshold = 2;

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
    public static final double shoulderkP = .075; //.05
    public static final double shoulderkI = .0000;
    public static final double shoulderkD = 0;
    public static final double shoulderGearRatio = 384; // just a guess
    public static final int shoulderAcceleration = 10000;
    public static final int shoulderMaxVelocity = 10000;
    public static final int shoulderSCurve = 0;
    public static final double shoulderStarting = 0.82;

    public static final int elbowID = 23;
    public static final double elbowkP = .075; //.05
    public static final double elbowkI = .0000;
    public static final double elbowkD = 0;
    public static final double elbowGearRatio = 225;
    public static final int elbowAcceleration = 10000;
    public static final int elbowMaxVelocity = 10000;
    public static final int elbowSCurve = 0;
    public static final double elbowStarting = 0.28;
    public static final double[] shoulderPositions = { 0.0, 69, 5308.0, -1199.0, 54674.0 };
    public static final double[] elbowPositions = { 0.0, -6969, -69726.0, -115990.0, -171442.0 };

    public static final int intakeState = 0;
    public static final int stowState = 1;
    public static final int lowState = 2;
    public static final int midState = 3;
    public static final int highState = 4;

    public static final double errorThreshold = 1000.0;
    public static final boolean useAbsoluteEncoders = false;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;//3
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;//0.5 //TODO measure this
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxChargeStationVelocity = 2;
    public static final double kMaxChargeStationAcceleration = 0.5;
    public static final double kMaxAutoVelocity = 3;
    public static final double kMaxAutoAcceleration = 3;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kDXController = 0.0;
    public static final double kPYController = 1;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double fieldLength = 16.54;
  }

  public static final class PneumaticsConstants {
    public static final int hubID = 49;
    public static final int minPressure = 110;
    public static final int maxPressure = 118;

  }

  public static final class TurntableConstants {
    public static final int motorID = 31;
    public static final double motorkP = .1;
    public static final double motorkI = .0001;
    public static final double motorkD = 0;
    public static final double clockwiseSpeed = .15;
    public static final double counterClockwiseSpeed = -.15;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

    public static final double kConeGThreshold = 3.0;
    public static final double kCubeBThreshold = 2.7;
    public static final double kDistanceThreshold = 130;
    //TODO: Add a current limit
  }

  public static final class IntakeConstants {
    public static final int motorID = 40;
    //TODO: Find motor ID for intake
    public static final double motorkP = .1;
    public static final double motorkI = .0001;
    public static final double motorkD = 0;
    public static final double intakeSpeed = .15;
    public static final double outakeSpeed = -.15;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  }

  public static final class LimelightConstants {
    public static final double turnValue = -0.08; //TODO: tune this number
  }

}
