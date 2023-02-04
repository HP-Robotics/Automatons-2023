// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class MoveSetDistanceCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_subsystem;
  private Command m_swerveControllerCommand;
  private final double m_X;

  /*
   * Creates a new ResetFalconCommand.
   ***
   * @param subsystem The subsystem used by this command.
   */
  public MoveSetDistanceCommand(DriveSubsystem subsystem, double X) {
    m_subsystem = subsystem;
    m_X = X;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Success");

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // An example trajectory to follow. All units in meters.
    Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(m_subsystem.getPoseX(), m_subsystem.getPoseY(), (m_subsystem.getPoseRot())),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(m_subsystem.getPoseX() + m_X, m_subsystem.getPoseY() + 0, m_subsystem.getPoseRot()),
        config); //Added robot poseX and y to this command
    m_swerveControllerCommand = new SwerveControllerCommand(
        forwardTrajectory,
        m_subsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_subsystem::setModuleStates,
        m_subsystem);

    m_swerveControllerCommand.initialize();
    if (m_swerveControllerCommand == null) {
      System.out.println("ItsVoid!");
    } else {
      System.out.println("It's working");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_swerveControllerCommand != null) {
      m_swerveControllerCommand.execute();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveControllerCommand.end(interrupted);
    m_subsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_swerveControllerCommand != null) {
      return m_swerveControllerCommand.isFinished();
    }
    return false;
  }
}
