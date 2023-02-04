/// Copyright (c) FIRST and other WPILib contributors.
//\ Open Source Software; you can modify and/or share it under the terms of
//     the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final DutyCycleEncoder m_frontLeftEncoder = new DutyCycleEncoder(12);
  public final DutyCycleEncoder m_frontRightEncoder = new DutyCycleEncoder(11);
  private final DutyCycleEncoder m_backLeftEncoder = new DutyCycleEncoder(13);
  private final DutyCycleEncoder m_backRightEncoder = new DutyCycleEncoder(14);
  // getAbsolutePosition

  public boolean m_fieldRelative = true;

  private final Field2d m_field = new Field2d();
  // Duty Encoders may have the wrong values

  private final PigeonIMU m_pGyro = new PigeonIMU(57);

  SwerveDriveOdometry m_odometry;

  public DriveSubsystem() {
    m_pGyro.setYaw(0);
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        });

    SmartDashboard.putData("Field", m_field);
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
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        });
    //System.out.println("Distance " + (m_frontLeft.getDistance() + m_frontRight.getDistance() + m_backLeft.getDistance()
    //    + m_backRight.getDistance()) / 4);
    //m_field.setRobotPose(getPose());
    // System.out.println(getPose().getX());
    SmartDashboard.putNumber("Robot x", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Front Left Get Absolute Position", m_frontLeftEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Front Left Get ", m_frontLeftEncoder.get());
    SmartDashboard.putNumber("Front Right", m_frontRightEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Back Left", m_backLeftEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Back Right", m_backRightEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Ben's Front Left", m_frontLeft.getDistance());

    SmartDashboard.putNumber("Front Left Drive Output", m_frontLeft.drivePower());
    SmartDashboard.putNumber("Front Right Drive Output", m_frontRight.drivePower());
    SmartDashboard.putNumber("Back Left Drive Output", m_backLeft.drivePower());
    SmartDashboard.putNumber("Back Right Drive Output", m_backRight.drivePower());
    SmartDashboard.putNumber("Front Left Turn Output", m_frontLeft.turnPower());
    SmartDashboard.putNumber("Front Right Turn Output", m_frontRight.turnPower());
    SmartDashboard.putNumber("Back Left Turn Output", m_backLeft.turnPower());
    SmartDashboard.putNumber("Back Right Turn Output", m_backRight.turnPower());

    SmartDashboard.putNumber("Pigeon Pitch", m_pGyro.getPitch());
    SmartDashboard.putNumber("Pigeon Yaw", m_pGyro.getYaw());
    SmartDashboard.putNumber("Pigeon Roll", m_pGyro.getRoll());

    // System.out.println(m_frontLeftEncoder.get() -
    // m_frontLeftEncoder.getAbsolutePosition());
    m_field.setRobotPose(m_odometry.getPoseMeters());
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
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeonYaw)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public void resetEncodersBegin() {
    m_frontLeft.resetEncoderPosition(RobotConstants.swerveOffsetFL, m_frontLeftEncoder.getAbsolutePosition());
    m_frontRight.resetEncoderPosition(RobotConstants.swerveOffsetFR, m_frontRightEncoder.getAbsolutePosition());
    //System.out.println("Output " + (RobotConstants.swerveOffsetFR - m_frontRightEncoder.getAbsolutePosition())
    //    + "  Current " + m_frontRightEncoder.getAbsolutePosition() + "  Goal " + RobotConstants.swerveOffsetFR);
    m_backLeft.resetEncoderPosition(RobotConstants.swerveOffsetBL, m_backLeftEncoder.getAbsolutePosition());
    m_backRight.resetEncoderPosition(RobotConstants.swerveOffsetBR, m_backRightEncoder.getAbsolutePosition());
  }

  public boolean resetEncoderCheck(double encoderValue) {
    return (Math.abs(encoderValue) < DriveConstants.encoderTolerance);
  }

  public boolean resetEncoderIsFinished() {
    if (resetEncoderCheck(m_frontLeftEncoder.getAbsolutePosition() - RobotConstants.swerveOffsetFL) &
        resetEncoderCheck(m_frontRightEncoder.getAbsolutePosition() - RobotConstants.swerveOffsetFR) &
        resetEncoderCheck(m_backLeftEncoder.getAbsolutePosition() - RobotConstants.swerveOffsetBL) &
        resetEncoderCheck(m_backRightEncoder.getAbsolutePosition() - RobotConstants.swerveOffsetBR)) {

      System.out.println("Success " + (m_frontRightEncoder.getAbsolutePosition() - RobotConstants.swerveOffsetFR));
      return true;
    } else {
      //System.out.println("fail " + m_frontRightEncoder.getAbsolutePosition());
      return false;
    }
  }

  public void resetEncoderEnd() {
    m_frontLeft.resetTurningMotor();
    m_frontRight.resetTurningMotor();
    m_backLeft.resetTurningMotor();
    m_backRight.resetTurningMotor();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getPoseX() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getPoseY() {
    return getPose().getY();
  }

  public Rotation2d getPoseRot() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    SmartDashboard.putNumber("Speed (0)", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Ticks (0)", m_frontLeft.metersToTicks(swerveModuleStates[0].speedMetersPerSecond));
  }

  public void forceRobotRelative() {
    m_fieldRelative = false;
  }

  public void forceFieldRelative() {
    m_fieldRelative = true;
  }

  public void resetOdometry(Pose2d pose) {
    var pigeonYaw = new Rotation2d(Math.toRadians(m_pGyro.getYaw()));
    m_odometry.resetPosition(
        pigeonYaw,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
        },
        pose);
  }

  public double getCombinedRoll() {
    double yaw = m_pGyro.getYaw();
    double pitch = m_pGyro.getPitch();
    double roll = m_pGyro.getRoll();
    double sin = Math.sin(Math.toRadians(yaw));
    double cos = Math.cos(Math.toRadians(yaw));
    return (sin * -1 * pitch) + (roll * cos);
  } // TODO: need to know whether we're on blue or red team, to mulitply by -1 or not.

  public void resetYaw() {
    m_pGyro.setYaw(0);
  }
}
