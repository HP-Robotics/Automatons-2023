// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurntableConstants;

public class TurntableSubsystem extends SubsystemBase {
  public final TalonFX m_turntableMotor;
  private ColorSensorV3 m_colorSensor;
  private NetworkTableEntry n_entry;
  private int coneCounter = 0;
  private double pastEncoderValue;
  private double presentEncoderValue;
  public boolean m_intakeProcessRunning;
  public boolean m_gamePieceDetected;

  /** Creates a new TurntablesSubsystem. */
  public TurntableSubsystem() {
    m_turntableMotor = new TalonFX(TurntableConstants.motorID);
    m_turntableMotor.configFactoryDefault();
    m_turntableMotor.setNeutralMode(NeutralMode.Brake);
    m_colorSensor = new ColorSensorV3(TurntableConstants.i2cPort);
    m_turntableMotor.config_kP(0, TurntableConstants.motorkP);
    m_turntableMotor.config_kI(0, TurntableConstants.motorkI);
    m_turntableMotor.config_kD(0, TurntableConstants.motorkD);
    m_intakeProcessRunning = false;
    m_gamePieceDetected = false;
    // TODO: Current limit + tuning
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Color Distance", m_colorSensor.getProximity());
    SmartDashboard.putNumber("Red", m_colorSensor.getRed());
    SmartDashboard.putNumber("Blue", m_colorSensor.getBlue());
    SmartDashboard.putNumber("Green", m_colorSensor.getGreen());
    SmartDashboard.putBoolean("FindCube", isCube());
    SmartDashboard.putBoolean("FindCone", isCone());
    SmartDashboard.putBoolean("Something?", isSomething());
    SmartDashboard.putNumber("RedNormal", (double) m_colorSensor.getRed() / m_colorSensor.getProximity());
    SmartDashboard.putNumber("BlueNormal", (double) m_colorSensor.getBlue() / m_colorSensor.getProximity());
    SmartDashboard.putNumber("GreenNormal", (double) m_colorSensor.getGreen() / m_colorSensor.getProximity());

    SmartDashboard.putNumber("Cone Counter", coneCounter);
    SmartDashboard.putNumber("Past Encoder Value", pastEncoderValue);
    SmartDashboard.putNumber("Present Encoder Value", presentEncoderValue);
    SmartDashboard.putNumber("Real Encoder Value", m_turntableMotor.getSelectedSensorPosition());

    if (m_intakeProcessRunning) {
      if (isSomething()) {
        m_gamePieceDetected = true;
      }
      if (m_gamePieceDetected) {
        correctPosition();
      }
    }
  }

  public void spinClockwise() {
    m_turntableMotor.set(ControlMode.PercentOutput, TurntableConstants.clockwiseSpeed);
  }

  public void spinCounterClockwise() {
    m_turntableMotor.set(ControlMode.PercentOutput, TurntableConstants.counterClockwiseSpeed);
  }

  public void stopSpinning() {
    m_turntableMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isCube() {
    double blueNormal = (double) m_colorSensor.getBlue() / m_colorSensor.getProximity();
    return blueNormal > TurntableConstants.kCubeBThreshold && isSomething();
  }

  public boolean isCone() {
    if (!isCube()) {
      double greenNormal = (double) m_colorSensor.getGreen() / m_colorSensor.getProximity();
      return greenNormal > TurntableConstants.kConeGThreshold && isSomething();
    }
    return false;
  } //TODO: tune colors, these are for sure not right

  public boolean isSomething() {
    return m_colorSensor.getProximity() > TurntableConstants.kDistanceThreshold;
  }

  public boolean correctPosition() {
    if (isCone()) {
      coneCounter++;
      if (coneCounter >= 2) {
        pastEncoderValue = presentEncoderValue;
      }
      presentEncoderValue = m_turntableMotor.getSelectedSensorPosition();
      if (coneCounter >= 2) {
        if (Math.abs(presentEncoderValue - pastEncoderValue) <= TurntableConstants.ticksThresholdMax
            && Math.abs(presentEncoderValue - pastEncoderValue) >= TurntableConstants.ticksThresholdMin) {
          m_turntableMotor.set(ControlMode.Position,
              m_turntableMotor.getSelectedSensorPosition() + TurntableConstants.coneCorrectionTicks);
          SmartDashboard.putNumber("Cone Set Positon",
              m_turntableMotor.getSelectedSensorPosition() + TurntableConstants.coneCorrectionTicks);
          return true;
        } else {
          coneCounter--;
        }
      }

    } else if (isCube()) {
      m_turntableMotor.set(ControlMode.Position,
          m_turntableMotor.getSelectedSensorPosition() + TurntableConstants.cubeCorrectionTicks);
      SmartDashboard.putNumber("Cube Set Positon",
          m_turntableMotor.getSelectedSensorPosition() + TurntableConstants.cubeCorrectionTicks);
      return true;
    }
    return false;
  }

  public void intakeOn() {
    m_intakeProcessRunning = true;
  }

  public void intakeOff() {
    m_intakeProcessRunning = false;
  }
}
