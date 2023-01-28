// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurntableConstants;

public class TurntableSubsystem extends SubsystemBase {
  public final TalonFX m_turntableMotor;

  /** Creates a new TurntablesSubsystem. */
  public TurntableSubsystem() {
    m_turntableMotor = new TalonFX(TurntableConstants.motorID);
    m_turntableMotor.configFactoryDefault();
    m_turntableMotor.setNeutralMode(NeutralMode.Brake);

    m_turntableMotor.config_kP(0, TurntableConstants.motorkP);
    m_turntableMotor.config_kI(0, TurntableConstants.motorkI);
    m_turntableMotor.config_kD(0, TurntableConstants.motorkD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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
}
