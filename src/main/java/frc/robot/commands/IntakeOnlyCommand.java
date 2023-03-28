// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOnlyCommand extends CommandBase {
  private final IntakeSubsystem m_subsystem;
  private int m_intakeCounter;

  /** Creates a new IntakeYuck. */
  public IntakeOnlyCommand(IntakeSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeCounter++;
    if (m_intakeCounter < 20) {
      m_subsystem.intakeOnly();
    } else {
      m_subsystem.outake();
    }
    if (m_intakeCounter == 20 + 2) {
      m_intakeCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopSpinning();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
