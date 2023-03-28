// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase {

  // DoubleSolenoid corresponds to a double solenoid.
  private final DoubleSolenoid m_chomp = new DoubleSolenoid(PneumaticsConstants.hubID, PneumaticsModuleType.REVPH, 2,
      3);

  Compressor m_compressor;
  public boolean m_chompClosed;

  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {

    m_compressor = new Compressor(PneumaticsConstants.hubID, PneumaticsModuleType.REVPH);
    m_compressor.enableAnalog(PneumaticsConstants.minPressure, PneumaticsConstants.maxPressure);
    ChompClose();
  }

  public void ChompClose() {
    m_chomp.set(DoubleSolenoid.Value.kForward);
    m_chompClosed = true;
  }

  public void ChompOpen() {
    m_chomp.set(DoubleSolenoid.Value.kReverse);
    m_chompClosed = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", m_compressor.getPressure());
  }
}
