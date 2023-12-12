// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {

  public final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final double m_desired;
  private Double m_turningOffset;
  public final DutyCycleEncoder m_absEncoder;
  String m_name;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel   CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor.
   * 
   * 
   * 
   */

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel, int absEncoder, double desired, String name) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.config_kP(0, DriveConstants.drivekP);
    m_driveMotor.config_kI(0, DriveConstants.drivekI);
    m_driveMotor.config_kD(0, DriveConstants.drivekD);
    m_driveMotor.config_kF(0, DriveConstants.drivekF);
    m_driveMotor.configAllowableClosedloopError(0, DriveConstants.drivekAllowableError);
    m_driveMotor.configMaxIntegralAccumulator(0, DriveConstants.drivekMaxIntegralAccumulation);
    m_driveMotor.config_IntegralZone(0, DriveConstants.drivekIntegralZone);
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningMotor.configFactoryDefault();
    m_turningMotor.config_kP(0, DriveConstants.turningkP);
    m_turningMotor.config_kI(0, DriveConstants.turningkI);
    m_turningMotor.config_kD(0, DriveConstants.turningkD);
    m_turningMotor.configAllowableClosedloopError(0, DriveConstants.turningkAllowableError);
    m_turningMotor.setInverted(true);

    m_absEncoder = new DutyCycleEncoder(absEncoder);
    m_turningOffset = 0.0;
    m_desired = desired;
    m_name = name;

  }

  public void resetOffset() {

    if (m_absEncoder.getAbsolutePosition() != 0) {
      m_turningMotor.setSelectedSensorPosition(0);
      double offset = m_desired - m_absEncoder.getAbsolutePosition();
      m_turningOffset = offset * (DriveConstants.kEncoderResolution * DriveConstants.rotationGearRatio);
    }

  }

  public double radiansToTicks(double radians) {
    // drive ratio: 6.75:1
    // rotation ratio: 15.429:1
    return radians * ((DriveConstants.kEncoderResolution * DriveConstants.rotationGearRatio) / (2 * Math.PI));
  }

  public double ticksToRadians(double ticks) {
    return ticks * ((2 * Math.PI) / (DriveConstants.kEncoderResolution * DriveConstants.rotationGearRatio));
  }

  public double ticksToMeters(double ticks) {
    return ((ticks / DriveConstants.kEncoderResolution) * (2 * Math.PI * DriveConstants.kWheelRadius))
        / DriveConstants.driveGearRatio;
  }

  public double metersToTicks(double meters) {
    return (meters / (2 * Math.PI * DriveConstants.kWheelRadius)) * DriveConstants.kEncoderResolution
        * DriveConstants.driveGearRatio;

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentAngle = new Rotation2d(
        ticksToRadians(applyOffset(m_turningMotor.getSelectedSensorPosition())) % (Math.PI * 2));
    //  System.out.println("delta: " + (desiredState.angle.minus(currentAngle).getDegrees()));
    if (Math.abs(desiredState.angle.minus(currentAngle).getDegrees()) > 90.0) {
      //System.out.println("optimize triggered");
      //System.out.println("desired: " + (desiredState.angle.getDegrees()) + "   current: " + currentAngle.getDegrees());
    }
    if (Math.abs(desiredState.angle.minus(currentAngle).getDegrees()) > 270) {
      System.out.println("angle:" + currentAngle + " " + desiredState);
      if (Math.abs(desiredState.angle.getDegrees()) > Math.abs(currentAngle.getDegrees())) {
        desiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.minus(Rotation2d.fromDegrees(360)));
        System.out.println("change desired state");
      } else {
        currentAngle = currentAngle.minus(Rotation2d.fromDegrees(360));
        System.out.println("change current angle");
      }
      currentAngle.minus(Rotation2d.fromDegrees(360));
      //System.out.println(desiredState.angle.minus(currentAngle).getDegrees());//Curently does not work
    }

    Rotation2d angle_offset = new Rotation2d(
        ((int) (ticksToRadians(applyOffset(m_turningMotor.getSelectedSensorPosition())) / (Math.PI * 2))) * Math.PI
            * 2);
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle.plus(angle_offset));
    SmartDashboard.putNumber(m_name + "turningAngle", state.angle.getDegrees());
    SmartDashboard.putNumber(m_name + "turningOffset", m_turningOffset);
    SmartDashboard.putNumber(m_name + "Absolute Encoder", m_absEncoder.get());
    m_turningMotor.set(ControlMode.Position,
        radiansToTicks(state.angle.getRadians()) + m_turningOffset);
    //Have the right angle, but must make it so that when sending input is in the right zone. (8000 rotations + our amount)
    m_driveMotor.set(ControlMode.Velocity, metersToTicks(state.speedMetersPerSecond) / 10); // the 10 is real, it turns ticks per second into ticks per 100ms

  }

  public void resetEncoderPosition(double desired, double current) {

    m_turningMotor.set(ControlMode.Position, applyOffset(m_turningMotor.getSelectedSensorPosition())
        + DriveConstants.kEncoderResolution * DriveConstants.rotationGearRatio * (desired - current));

  }

  public double getTurningPosition() {
    return m_turningMotor.getSensorCollection().getIntegratedSensorAbsolutePosition();
  }

  public double applyOffset(double position) {
    return position - m_turningOffset;
  }

  public void resetTurningMotor() {
    m_turningMotor.set(ControlMode.PercentOutput, 0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        ticksToMeters(m_driveMotor.getSelectedSensorPosition()),
        new Rotation2d(ticksToRadians(applyOffset(m_turningMotor.getSelectedSensorPosition()))));
  }

  public double getDistance() {
    return m_driveMotor.getSelectedSensorPosition();
  }

  public double turnPower() {
    return m_turningMotor.getMotorOutputPercent();
  }

  public double drivePower() {
    return m_driveMotor.getMotorOutputPercent();
  }

  public void getDrivePower(String key) {
    SmartDashboard.putNumber(key + " Setpoint",
        m_driveMotor.getSelectedSensorVelocity() - m_driveMotor.getClosedLoopError());
    SmartDashboard.putNumber(key + " Velocity", m_driveMotor.getSelectedSensorVelocity());
  }

}
