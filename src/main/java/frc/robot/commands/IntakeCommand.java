// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final PneumaticsSubsystem m_pneumatic;

  /** Creates a new spinClockwise. */
  public IntakeCommand(IntakeSubsystem subsystem, PneumaticsSubsystem psubsystem) {
    m_intake = subsystem;
    m_pneumatic = psubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, psubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumatic.intakeOut();
    m_intake.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopSpinning();
    m_pneumatic.intakeIn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
