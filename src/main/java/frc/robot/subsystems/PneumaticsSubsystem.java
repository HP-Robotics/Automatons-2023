// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase {

  // DoubleSolenoid corresponds to a double solenoid.
  private final DoubleSolenoid m_chomp = new DoubleSolenoid(PneumaticsConstants.hubID, PneumaticsModuleType.REVPH, 2,
      3);
  private final DoubleSolenoid m_intake = new DoubleSolenoid(PneumaticsConstants.hubID, PneumaticsModuleType.REVPH, 0,
      1);
  Compressor m_compressor;

  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {

    m_compressor = new Compressor(PneumaticsConstants.hubID, PneumaticsModuleType.REVPH);
    m_compressor.enableAnalog(PneumaticsConstants.minPressure, PneumaticsConstants.maxPressure);
  }

  public void forward() {
    m_chomp.set(DoubleSolenoid.Value.kForward);
  }

  public void backward() {
    m_chomp.set(DoubleSolenoid.Value.kReverse);
  }

  public void intakeOut() {
    m_intake.set(DoubleSolenoid.Value.kForward);
  }

  public void intakeIn() {
    m_intake.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
