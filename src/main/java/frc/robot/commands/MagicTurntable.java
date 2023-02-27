// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurntableSubsystem;

public class MagicTurntable extends CommandBase {
  private final TurntableSubsystem m_subsystem;
  private boolean m_done = false;

  public MagicTurntable(TurntableSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.magicTurntableStart();
    m_subsystem.spinClockwise();
    m_subsystem.coneCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.correctPosition()) {
      m_done = true;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_done) {
      m_subsystem.goToCorrectPosition();
    } else {
      m_subsystem.stopSpinning();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_done || !m_subsystem.m_magicTurntableOn;
  }
}
