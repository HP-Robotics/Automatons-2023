// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class AltBalanceCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_subsystem;
  double startPos;
  private double m_pastAngle;
  private double m_currentAngle;
  private double m_rate;

  /*
   * Creates a new ResetFalconCommand.
   ***
   * @param subsystem The subsystem used by this command.
   */
  public AltBalanceCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_subsystem.getPoseX();
    m_pastAngle = m_subsystem.getCombinedRoll();
    m_rate = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO MENTOR: put our PID values to the smartdashboard for pid tuning
    double power = DriveConstants.altBalanceSpeed;
    m_currentAngle = m_subsystem.getCombinedRoll();
    m_rate = ((m_currentAngle - m_pastAngle) * DriveConstants.altBalanceAlpha)
        + (1 - DriveConstants.altBalanceAlpha) * m_rate;
    m_pastAngle = m_currentAngle;
    SmartDashboard.putNumber("Combined Roll", m_currentAngle);
    SmartDashboard.putNumber("Balance Rate", m_rate);
    if (m_currentAngle < 0) {
      power = power * -1;
    }
    if (Math.abs(m_currentAngle) < 1.5) {
      power = 0;
    }

    // if (m_subsystem.getPoseX() > startPos + 2 || m_subsystem.getPoseX() < startPos - 2) {
    //   m_subsystem.drive(0, 0, 0, true);
    //   SmartDashboard.putNumber("Balance Power", 0);
    // } else {
    m_subsystem.drive(power, 0, 0, true);
    SmartDashboard.putNumber("Balance Power", power * -1);
    // }
    m_pastAngle = m_currentAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_rate) > DriveConstants.altRateThreshold) {
      SmartDashboard.putNumber("Balance Power", 0);
      return true;
    }
    return false;
  }
}
