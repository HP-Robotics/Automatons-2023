// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurntableConstants;

public class TurntableSubsystem extends SubsystemBase {
  public final TalonFX m_turntableMotor;
  private ColorSensorV3 m_colorSensor;

  /** Creates a new TurntablesSubsystem. */
  public TurntableSubsystem() {
    m_turntableMotor = new TalonFX(TurntableConstants.motorID);
    m_turntableMotor.configFactoryDefault();
    m_turntableMotor.setNeutralMode(NeutralMode.Brake);
    m_colorSensor = new ColorSensorV3(TurntableConstants.i2cPort);
    m_turntableMotor.config_kP(0, TurntableConstants.motorkP);
    m_turntableMotor.config_kI(0, TurntableConstants.motorkI);
    m_turntableMotor.config_kD(0, TurntableConstants.motorkD);
    // TODO: Current limit + tuning
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Color Distance", m_colorSensor.getProximity());
    SmartDashboard.putNumber("Red", m_colorSensor.getRed());
    SmartDashboard.putNumber("Blue", m_colorSensor.getBlue());
    SmartDashboard.putNumber("Green", m_colorSensor.getGreen());
    SmartDashboard.putBoolean("FindCube", isCube());
    SmartDashboard.putNumber("RedNormal", (double) m_colorSensor.getRed() / m_colorSensor.getProximity());
    SmartDashboard.putNumber("BlueNormal", (double) m_colorSensor.getBlue() / m_colorSensor.getProximity());
    SmartDashboard.putNumber("GreenNormal", (double) m_colorSensor.getGreen() / m_colorSensor.getProximity());

  }

  public void spinClockwise() {
    // TODO: Use velocity control
    m_turntableMotor.set(ControlMode.PercentOutput, TurntableConstants.clockwiseSpeed);
  }

  public void spinCounterClockwise() {
    // TODO: Use velocity control
    m_turntableMotor.set(ControlMode.PercentOutput, TurntableConstants.counterClockwiseSpeed);
  }

  public void stopSpinning() {
    m_turntableMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isCube() {
    double blueNormal = (double) m_colorSensor.getBlue() / m_colorSensor.getProximity();
    return blueNormal > 2.7 && m_colorSensor.getProximity() > 150;
  }

  public boolean isCone() {
    double blueNormal = (double) m_colorSensor.getBlue() / m_colorSensor.getProximity();
    return blueNormal > 2.7 && m_colorSensor.getProximity() > 150;
  }
}
