// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveSetDistanceCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_subsystem;
  private Command m_swerveControllerCommand;
  private final double m_X;
  private final double m_Y;
  private final Rotation2d m_Rot;
  private final double m_Velocity;
  private final double m_Acceleration;
  private final List<Translation2d> m_MidPoints;
  private final List<PathPoint> m_wayPoints;
  private PIDController m_thetaController;
  private PIDController m_XController;
  private PIDController m_YController;

  /*
   * Creates a new ResetFalconCommand.
   ***
   * @param subsystem The subsystem used by this command.
   */
  public MoveSetDistanceCommand(DriveSubsystem subsystem, double X, double Y, Rotation2d Rot, double Velocity,
      double Acceleration, List<Translation2d> midPoints) {
    m_subsystem = subsystem;
    m_X = X;
    m_Y = Y;
    m_Rot = Rot;
    m_Velocity = Velocity;
    m_Acceleration = Acceleration;
    m_MidPoints = midPoints;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_wayPoints = new ArrayList();
  }

  public MoveSetDistanceCommand(DriveSubsystem subsystem, Pose2d Destination) {
    this(subsystem, Destination.getX(), Destination.getY(), Destination.getRotation(), AutoConstants.kMaxAutoVelocity,
        AutoConstants.kMaxAutoAcceleration, List.of());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      System.out.println("Success");

      TrajectoryConfig config = new TrajectoryConfig(m_Velocity, m_Acceleration)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);

      m_thetaController = new PIDController(
          //var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0);
      m_XController = new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController);
      m_YController = new PIDController(AutoConstants.kPYController, 0, 0);

      m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

      m_wayPoints.add(new PathPoint(new Translation2d(m_subsystem.getPoseX(), m_subsystem.getPoseY()),
          Rotation2d.fromDegrees(0), m_subsystem.getPoseRot()));
      for (int i = 0; i < m_MidPoints.size(); i++) {
        m_wayPoints.add(new PathPoint(m_MidPoints.get(i), new Rotation2d(0)));
      }
      m_wayPoints.add(new PathPoint(new Translation2d(m_X, m_Y), new Rotation2d(0), m_Rot));

      // An example trajectory to follow. All units in meters.
      PathPlannerTrajectory forwardTrajectory = PathPlanner.generatePath(
          new PathConstraints(m_Velocity, m_Acceleration),
          m_wayPoints
      // Start at the origin facing the +X direction
      //new PathPoint(new Translation2d(m_subsystem.getPoseX(), m_subsystem.getPoseY()), Rotation2d.fromDegrees(0), m_subsystem.getPoseRot()),
      // Pass through these two interior waypoints, making an 's' curve path
      // End 3 meters straight ahead of where we started, facing forward
      //new Pose2d(m_X, m_Y, m_Rot),
      //config); //Added robot poseX and y to this command
      );
      System.out.println("initialized");
      m_swerveControllerCommand = new PPSwerveControllerCommand(
          forwardTrajectory,
          m_subsystem::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          m_XController, m_YController, m_thetaController,
          m_subsystem::setModuleStates,
          false,
          m_subsystem);

      m_swerveControllerCommand.initialize();
      if (m_swerveControllerCommand == null) {
        System.out.println("ItsVoid!");
      } else {
        System.out.println("It's working");
      }
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println(e);
      m_swerveControllerCommand = null;

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_swerveControllerCommand != null) {
      m_swerveControllerCommand.execute();
      SmartDashboard.putNumber("thetaSetpoint", m_thetaController.getSetpoint());
      SmartDashboard.putNumber("thetaError", m_thetaController.getPositionError());
      SmartDashboard.putNumber("XsetPoint", m_XController.getSetpoint());
      SmartDashboard.putNumber("XError", m_XController.getPositionError());
      SmartDashboard.putNumber("YsetPoint", m_YController.getSetpoint());
      SmartDashboard.putNumber("YError", m_YController.getPositionError());
      SmartDashboard.putNumber("XPosition", m_XController.getSetpoint() - m_XController.getPositionError());
      SmartDashboard.putNumber("YPosition", m_YController.getSetpoint() - m_YController.getPositionError());
      SmartDashboard.putNumber("thetaPosition",
          m_thetaController.getSetpoint() - m_thetaController.getPositionError());

    }
    // if (m_vision != null) {
    //   m_subsystem.resetOdometry(m_vision.getRobotAbsolute());
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_swerveControllerCommand != null) {
      m_swerveControllerCommand.end(interrupted);
    }
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
