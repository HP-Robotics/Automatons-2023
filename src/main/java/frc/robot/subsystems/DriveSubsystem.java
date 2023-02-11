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

  private final SwerveModule m_frontLeft = new SwerveModule(13, 12, 12, RobotConstants.swerveOffsetFL, "FL"); // BIG BONGO 2
  private final SwerveModule m_frontRight = new SwerveModule(2, 3, 11, RobotConstants.swerveOffsetFR, "FR"); // BIG BONGO 1
  private final SwerveModule m_backLeft = new SwerveModule(14, 15, 13, RobotConstants.swerveOffsetBL, "BL"); // BIG BONGO 3
  private final SwerveModule m_backRight = new SwerveModule(50, 1, 14, RobotConstants.swerveOffsetBR, "BR"); // BIG BONGO 4

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
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_backLeft.setDesiredState(swerveModuleStates[3]);
    ;
    SmartDashboard.putNumber("Speed (0)", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Ticks (0)", m_frontLeft.metersToTicks(swerveModuleStates[0].speedMetersPerSecond));
  }

  public void forceRobotRelative() {
    m_fieldRelative = false;
  }

  public void forceFieldRelative() {
    m_fieldRelative = true;
  }

  public void resetOffsets() {
    m_frontLeft.resetOffset();
    m_frontRight.resetOffset();
    m_backRight.resetOffset();
    m_backLeft.resetOffset();
  }

  public void resetOdometry(Pose2d pose) {
    if (pose == null) {
      return;
    }
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
