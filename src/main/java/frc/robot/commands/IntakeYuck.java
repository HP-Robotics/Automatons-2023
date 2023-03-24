// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class IntakeYuck extends CommandBase {
  private final IntakeSubsystem m_subsystem;
  private final PneumaticsSubsystem m_intake;

  /** Creates a new IntakeYuck. */
  public IntakeYuck(IntakeSubsystem subsystem, PneumaticsSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_intake = intake;
    addRequirements(m_subsystem, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.outake();
    m_intake.intakeOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopSpinning();
    m_intake.intakeIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
