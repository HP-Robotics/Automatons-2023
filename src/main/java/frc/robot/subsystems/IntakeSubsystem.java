// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public final TalonFX m_intakeMotor;

  /** Creates a new TurntablesSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new TalonFX(IntakeConstants.motorID);
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setNeutralMode(NeutralMode.Brake);

    m_intakeMotor.config_kP(0, IntakeConstants.motorkP);
    m_intakeMotor.config_kI(0, IntakeConstants.motorkI);
    m_intakeMotor.config_kD(0, IntakeConstants.motorkD);
    // TODO: Current limit + tuning

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void intake() {
    // TODO: Use velocity control
    m_intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeSpeed);
  }

  public void intakeOnly() {
    m_intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.intakeOnlySpeed);
  }

  public void outake() {
    // TODO: Use velocity control
    m_intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.outakeSpeed);
  }

  public void stopSpinning() {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

}
