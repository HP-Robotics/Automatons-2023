// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.ArmChangeStateCommand;
import frc.robot.commands.ArmMoveElbowCommand;
import frc.robot.commands.ArmMoveShoudlerCommand;
import frc.robot.commands.BackToNormalCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ChickenCommand;
import frc.robot.commands.ChompForward;
import frc.robot.commands.ChompReverse;
import frc.robot.commands.DriveTrackGamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveSetDistanceCommand;
import frc.robot.commands.ResetFalconCommand;
import frc.robot.commands.SpinClockwiseCommand;
import frc.robot.commands.SpinCounterClockwiseCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.TurntableSubsystem;

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
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final VisionSubsystem m_exampleSubsystem = new VisionSubsystem();
  public final Command getCamera = new RunCommand(() -> m_exampleSubsystem.getCameraAbsolute(), m_exampleSubsystem);
  // The robot's subsystems and commands are defined here...
  private final Joystick m_joystick = new Joystick(1);
  private final Joystick m_opJoystick = new Joystick(0);

  private final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem() : null;
  private final ArmSubsystem m_robotArm = SubsystemConstants.useArm ? new ArmSubsystem(m_opJoystick) : null;
  private final PneumaticsSubsystem m_pneumatics = SubsystemConstants.usePneumatics ? new PneumaticsSubsystem() : null;
  private final TurntableSubsystem m_turntables = SubsystemConstants.useTurnTables ? new TurntableSubsystem() : null;
  private final IntakeSubsystem m_intake = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;
  // The driver's controller

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    if (SubsystemConstants.useDrive) {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.

          new RunCommand(

              () -> m_robotDrive.drive(
                  Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.1), 1) * -1 * DriveConstants.kMaxSpeed,
                  Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 1) * -1 * DriveConstants.kMaxSpeed,
                  MathUtil.applyDeadband(m_joystick.getRawAxis(2), 0.2) * -1
                      * DriveConstants.kMaxAngularSpeed,
                  //0.2 * DriveConstants.kMaxSpeed, 0, 0,
                  m_robotDrive.m_fieldRelative),
              m_robotDrive));
    }
  }

  public Command getAutonomousCommand() {
    if (Constants.autonomousMode == "Vision") {
      return getCamera;
    }
    if (SubsystemConstants.useDrive) {

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
          List.of(new Translation2d(1, .1), new Translation2d(2, -.1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

      // An example trajectory to follow. All units in meters.
      Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(),
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
    } else
      return new InstantCommand();
  }

  public void runResetFalconCommand() {
    m_robotDrive.run(() -> new ResetFalconCommand(m_robotDrive));
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
    if (SubsystemConstants.useDrive) {
      new JoystickButton(m_joystick, 2).onTrue(new InstantCommand(m_robotDrive::forceRobotRelative, m_robotDrive));
      new JoystickButton(m_joystick, 2).onFalse(new InstantCommand(m_robotDrive::forceFieldRelative, m_robotDrive));
      new JoystickButton(m_joystick, 16).onTrue(new InstantCommand(m_robotDrive::resetYaw, m_robotDrive));
      new JoystickButton(m_joystick, 7).onTrue(new ResetFalconCommand(m_robotDrive));
      new JoystickButton(m_joystick, 14).whileTrue(new BalanceCommand(m_robotDrive));

      new JoystickButton(m_joystick, 12).onTrue(new MoveSetDistanceCommand(m_robotDrive, 1));

    }

    if (SubsystemConstants.useArm) {
      //new JoystickButton(m_opJoystick, 4).onTrue(new ChickenCommand(m_robotArm));
      //new JoystickButton(m_opJoystick, 7).onTrue(new BackToNormalCommand(m_robotArm));

      System.out.println("Im alive!");

      //new RunCommand(() -> m_robotArm.moveShoulder(m_opJoystick.getRawAxis(1) * 0.2), m_robotArm);
      //new RunCommand(() -> m_robotArm.moveElbow(m_opJoystick.getRawAxis(5) * 0.2), m_robotArm);
      new JoystickButton(m_opJoystick, 1).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.intakeState));
      new JoystickButton(m_opJoystick, 4).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.highState));
    }
    if (SubsystemConstants.usePneumatics) {
      new JoystickButton(m_opJoystick, 2).onTrue(new ChompForward(m_pneumatics));
      new JoystickButton(m_opJoystick, 3).onTrue(new ChompReverse(m_pneumatics));
    }
    if (SubsystemConstants.useTurnTables) {
      new JoystickButton(m_opJoystick, 9).whileTrue(new SpinClockwiseCommand(m_turntables));
      new JoystickButton(m_opJoystick, 10).whileTrue(new SpinCounterClockwiseCommand(m_turntables));
    }
    if (SubsystemConstants.useIntake) {
      new JoystickButton(m_opJoystick, 7).whileTrue(new IntakeCommand(m_intake));

    }

    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight) {
      new JoystickButton(m_joystick, 4).whileTrue(new DriveTrackGamePiece(m_robotDrive, m_joystick, true));
      new JoystickButton(m_joystick, 3).whileTrue(new DriveTrackGamePiece(m_robotDrive, m_joystick, false));
    }

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
