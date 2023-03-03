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
    public static Transform2d leftTrans = new Transform2d(new Translation2d(0.35 + 0.4 + .12, 0.6096),
        new Rotation2d(0));
    public static Transform2d rightTrans = new Transform2d(new Translation2d(0.35 + 0.4 + 0.12, -0.6096),
        new Rotation2d(0));
    public static Transform2d centerTrans = new Transform2d(new Translation2d(0.35 + 0.4 + .12, 0), new Rotation2d(0));
    public static Transform2d cameraToRobot = new Transform2d(new Translation2d(-0.3302, 0.0889),
        new Rotation2d(Math.PI));
    //0.0.05 is distance of outside of bumper, 2 inches, -0.3302 camera to robot
    //0.41 is the beilived distance from April Tags to the outside
  }

  public static String autonomousMode = "NotVision";

  public static class OperatorConstants { // For operator button bindings, and other stuff directly related to the
                                          // operator
    public static final int kDriverControllerPort = 0;
  }

  public static class SubsystemConstants {
    public static final boolean useDrive = true; // drive disabled, reenable later
    public static final boolean useArm = true;
    public static final boolean usePneumatics = true;
    public static final boolean useTurnTables = true;
    public static final boolean useIntake = true;
    public static final boolean useLimelight = true;
    public static final boolean useVision = true;
    public static final boolean useDataManger = true;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 5.0; // meters per second 
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second  // TODO MENTOR: is this a good turn speed?
    public static final double kSlowSpeed = 2.0;
    public static final double kSlowAngularspeed = Math.PI / 2; // 1/4 rotation per second  // TODO MENTOR: is this a good turn speed?

    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 2048;

    public static final double driveGearRatio = 6.75;
    public static final double rotationGearRatio = 15.429;

    public final static Translation2d kFrontLeftLocation = new Translation2d(0.270, 0.270);
    public final static Translation2d kFrontRightLocation = new Translation2d(0.270, -0.270);
    public final static Translation2d kBackLeftLocation = new Translation2d(-0.270, 0.270);
    public final static Translation2d kBackRightLocation = new Translation2d(-0.270, -0.270);

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
    public static final double balancekI = 0.0;
    public static final double balancekD = 0.01;
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
    public static final int shoulderID = 17;
    public static final double shoulderkP = .07; //.05
    public static final double shoulderkI = .005;
    public static final double shoulderkD = 0;
    public static final double shoulderMaxAllowableError = 200;
    public static final double shoulderIZone = 500;
    public static final double shoulderGearRatio = 384; // just a guess
    public static final int shoulderAcceleration = 25000;//20000 works
    public static final int shoulderMaxVelocity = 50000;
    public static final int shoulderSCurve = 0;
    public static final double shoulderStarting = 0.82;

    public static final int elbowID = 19;
    public static final double elbowkP = .075; //.05
    public static final double elbowkI = .0000;
    public static final double elbowkD = 0;
    public static final double elbowGearRatio = 225;
    public static final int elbowAcceleration = 30000;
    public static final int elbowMaxVelocity = 60000;
    public static final int elbowSCurve = 0;
    public static final double elbowStarting = 0.28;

    public static final int frameCounterThreshold = 25;

    public static final int intakeState = 0;
    public static final int stowState = 1;
    public static final int lowState = 2;
    public static final int midState = 3;
    public static final int highState = 4;
    public static final int scoreState = 5;

    public static final double errorThreshold = 1000.0;
    public static final boolean useAbsoluteEncoders = false;

    public static final double shoulderScore = 53143;
    public static final double elbowScore = 196500;

    public static final double shoulderHigh = 59650;
    public static final double elbowHigh = 217205;

    public static final double shoulderMid = 2688;
    public static final double elbowMid = 129338;

    public static final double shoulderLow = -26100;
    public static final double elbowLow = 80000;

    public static final double shoulderStow = 0;
    public static final double elbowStow = 0;

    public static final double shoulderIntake = -69061;
    public static final double elbowIntake = 2279;

    public static double[] shoulderPositions = { shoulderIntake, shoulderStow, shoulderLow, shoulderMid,
        shoulderHigh, shoulderScore };
    public static double[] elbowPositions = { elbowIntake, elbowStow, elbowLow, elbowMid, elbowHigh, elbowScore };
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5; // TODO MENTOR: And note that we don't use them...
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxChargeStationVelocity = 2.5;
    public static final double kMaxChargeStationAcceleration = 1.25;
    public static final double kFastAutoVelocity = 4.5;
    public static final double kfastAutoAcceleration = 3.0;
    public static final double kMaxAutoVelocity = 3;
    public static final double kMaxAutoAcceleration = 3;

    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kIXController = 0.03;
    public static final double kDXController = 0.0;
    public static final double kIYController = 0.03;
    public static final double kPYController = 5; //TODO why is Y different?
    public static final double kPThetaController = 3; // TODO MENTOR: tune the theta PID
    public static final double kIThetaController = 0.05;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double fieldLength = 16.54;
  }

  public static final class PneumaticsConstants {
    public static final int hubID = 49;
    public static final int minPressure = 100;
    public static final int maxPressure = 118;

  }

  public static final class TurntableConstants {
    public static final int motorID = 10;
    public static final double motorkP = .5;
    public static final double motorkI = .0001;
    public static final double motorkD = 0;
    public static final double clockwiseSpeed = .25;
    public static final double counterClockwiseSpeed = -.5;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

    public static final double kConeGThreshold = 3.0;
    public static final double kCubeBThreshold = 2.7;
    public static final double kDistanceThreshold = 130;

    public static final double coneCorrectionTicks = 3000;
    public static final double cubeCorrectionTicks = 250;

    public static final double ticksThresholdMin = 1000;
    public static final double ticksThresholdMax = 8192;

    public static final double waitTime = 2;

    //TODO: Add a current limit
  }

  public static final class IntakeConstants {
    public static final int motorID = 18;
    public static final double motorkP = .1;
    public static final double motorkI = .0001;
    public static final double motorkD = 0;
    public static final double intakeSpeed = .6;
    public static final double outakeSpeed = -.6;
    public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  }

  public static final class LimelightConstants {
    public static final double turnValue = -0.08; //TODO: tune this number
  }

}
