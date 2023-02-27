// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmChangeStateCommand extends CommandBase {
  /** Creates a new ArmChangeStateCommand. */
  ArmSubsystem m_subsystem;
  int m_state;

  public ArmChangeStateCommand(ArmSubsystem subsystem, int state) {
    m_subsystem = subsystem;
    m_state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setTargetState(m_state);
    int direction = m_subsystem.getTargetState() - m_subsystem.getCurrentState();
    if (direction > 0) {
      m_subsystem.moveUpState();
    } else if (direction < 0) {
      m_subsystem.moveDownState();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.getIsChanging()) {
      return;
    }
    if (ArmConstants.useAbsoluteEncoders) {
      m_subsystem.setFalconEncoders();
    }

    int direction = m_subsystem.getTargetState() - m_subsystem.getCurrentState();
    if (direction > 0) {
      m_subsystem.moveUpState();
    } else if (direction < 0) {
      m_subsystem.moveDownState();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int direction = m_subsystem.getTargetState() - m_subsystem.getCurrentState();
    return direction == 0 && m_subsystem.getDoneChanging();
  }
}
