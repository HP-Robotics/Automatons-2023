// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.ResetFalconCommand;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final Joystick m_joystick = new Joystick(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
              MathUtil.applyDeadband(m_joystick.getRawAxis(1),0.1) * -1 * DriveConstants.kMaxSpeed,
              MathUtil.applyDeadband(m_joystick.getRawAxis(0),0.1) * -1 * DriveConstants.kMaxSpeed,
              MathUtil.applyDeadband(m_joystick.getRawAxis(2),0.05) * -1 * DriveConstants.kMaxSpeed,
               true),
           m_robotDrive));
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // TODO: put stuff in here
    new JoystickButton(m_joystick, 2).onTrue(new ResetFalconCommand(m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
// private void driveWithJoystick(boolean fieldRelative) {
// // Get the x speed. We are inverting this because Xbox controllers return
// // negative values when we push forward.
// final var xSpeed =
// -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_joystick.getRawAxis(1),
// 0.02))
// * DriveSubsystem.kMaxSpeed;

// // Get the y speed or sideways/strafe speed. We are inverting this because
// // we want a positive value when we pull to the left. Xbox controllers
// // return positive values when you pull to the right by default.
// final var ySpeed =
// -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_joystick.getRawAxis(0),
// 0.02))
// * DriveSubsystem.kMaxSpeed;
// // Get the rate of angular rotation. We are inverting this because we want a
// // positive value when we pull to the left (remember, CCW is positive in
// // mathematics). Xbox controllers return positive values when you pull to
// // the right by default.
// final var rot =
// -m_rotLimiter.calculate(MathUtil.applyDeadband(m_joystick.getRawAxis(2),
// 0.02))
// * DriveSubsystem.kMaxAngularSpeed;

// m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
// }
