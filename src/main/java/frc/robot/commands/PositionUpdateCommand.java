// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class PositionUpdateCommand extends CommandBase {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;

  /** Creates a new PositionUpdateCommand. */
  public PositionUpdateCommand(VisionSubsystem vision, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_drive = drive;
    addRequirements(vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.m_lastPosition != null)
      m_drive.resetOdometry(m_vision.m_lastPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
