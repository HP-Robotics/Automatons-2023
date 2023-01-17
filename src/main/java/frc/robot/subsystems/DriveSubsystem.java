/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import frc.robot.SwerveModule;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 4.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI * 4; // 1/2 rotation per second

  private final SwerveModule m_frontLeft = new SwerveModule(13, 12); // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(2, 3); // BIG BONGO 1
  private final SwerveModule m_backLeft = new SwerveModule(14, 15); // BIG BONGO 3
  private final SwerveModule m_backRight = new SwerveModule(50, 1); // BIG BONGO 4

  /// private final DutyCycleEncoder m_testEncoder = new DutyCycleEncoder(id);
  private final DutyCycleEncoder m_frontLeftEncoder = new DutyCycleEncoder(11);
  private final DutyCycleEncoder m_frontRighttEncoder = new DutyCycleEncoder(12);
  private final DutyCycleEncoder m_backLeftEncoder = new DutyCycleEncoder(13);
  private final DutyCycleEncoder m_backRightEncoder = new DutyCycleEncoder(14);

  private final Field2d m_field = new Field2d();
  // Duty Encoders may have the wrong values

  private final PigeonIMU m_pGyro = new PigeonIMU(57);

  SwerveDriveOdometry m_odometry;

  public DriveSubsystem() {
    m_pGyro.setYaw(0);
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    m_odometry = new SwerveDriveOdometry(
        DriverConstants.kDriveKinematics,
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  @Override
  public void periodic() {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    // Update the odometry in the periodic block
    m_odometry.update(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    m_field.setRobotPose(getPose());
    System.out.println(getPose().getX());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the ro bot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    var swerveModuleStates = DriverConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoderPosition(RobotConstants.swerveOffset);
    m_frontRight.resetEncoderPosition(RobotConstants.swerveOffset);
    m_backLeft.resetEncoderPosition(RobotConstants.swerveOffset);
    m_backRight.resetEncoderPosition(RobotConstants.swerveOffset);

    try {
      Thread.sleep(1);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    m_frontLeft.resetEncoderValue();
    m_frontRight.resetEncoderValue();
    m_backLeft.resetEncoderValue();
    m_backRight.resetEncoderValue();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void resetOdometry(Pose2d pose) {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    m_odometry.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }
}
