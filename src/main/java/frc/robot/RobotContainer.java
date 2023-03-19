// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.AltBalanceCommand;
import frc.robot.commands.ArmChangeStateCommand;
import frc.robot.commands.ArmCycleStateCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ChompOpenCommand;
import frc.robot.commands.ChompToggleCommand;
import frc.robot.commands.DriveParkingBrakeCommand;
import frc.robot.commands.ChompCloseCommand;
import frc.robot.commands.DriveTrackGamePiece;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MagicTurntable;
import frc.robot.commands.MoveSetDistanceCommand;
import frc.robot.commands.MoveWithVisionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.TurntableSubsystem;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;

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
  public Command leftAutoButton = null;
  public Command rightAutoButton = null;
  public Command middleAutoButton = null;

  // The robot's subsystems and commands are defined here...
  private final Joystick m_joystick = new Joystick(1);
  private final Joystick m_opJoystick = new Joystick(0);

  private final DriveSubsystem m_robotDrive = SubsystemConstants.useDrive ? new DriveSubsystem() : null;
  private final VisionSubsystem m_visionSubsystem = SubsystemConstants.useVision ? new VisionSubsystem() : null;
  private final ArmSubsystem m_robotArm = SubsystemConstants.useArm ? new ArmSubsystem() : null;
  private final PneumaticsSubsystem m_pneumatics = SubsystemConstants.usePneumatics ? new PneumaticsSubsystem() : null;
  private final TurntableSubsystem m_turntables = SubsystemConstants.useTurnTables ? new TurntableSubsystem() : null;
  private final IntakeSubsystem m_intake = SubsystemConstants.useIntake ? new IntakeSubsystem() : null;

  private final SendableChooser<String> m_startPosition;
  private final SendableChooser<Boolean> m_grabPiece1;
  private final SendableChooser<Boolean> m_grabPiece2;
  private final SendableChooser<Boolean> m_chargeBalance;

  private Boolean m_manualArm = false;
  // The driver's controller

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (SubsystemConstants.useVision) {
      leftAutoButton = new RunCommand(
          () -> m_visionSubsystem.getDestination(m_robotDrive.getPose(), "left"),
          m_visionSubsystem);
      rightAutoButton = new RunCommand(
          () -> m_visionSubsystem.getDestination(m_robotDrive.getPose(), "right"),
          m_visionSubsystem);
      middleAutoButton = new RunCommand(
          () -> m_visionSubsystem.getDestination(m_robotDrive.getPose(), "middle"),
          m_visionSubsystem);
    }

    m_startPosition = new SendableChooser<String>();
    m_startPosition.addOption("Station Side", "station");
    m_startPosition.addOption("Middle", "middle");
    m_startPosition.addOption("Drive Side", "drive");
    m_startPosition.setDefaultOption("Middle", "middle");
    SmartDashboard.putData("Starting Position?", m_startPosition);

    m_grabPiece1 = new SendableChooser<Boolean>();
    m_grabPiece1.addOption("Yes", true);
    m_grabPiece1.addOption("No", false);
    m_grabPiece1.setDefaultOption("No", false);
    SmartDashboard.putData("Grabbing First Piece?", m_grabPiece1);

    m_grabPiece2 = new SendableChooser<Boolean>();
    m_grabPiece2.addOption("Yes", true);
    m_grabPiece2.addOption("No", false);
    m_grabPiece2.setDefaultOption("No", false);
    SmartDashboard.putData("Grabbing Second Piece?", m_grabPiece2);

    m_chargeBalance = new SendableChooser<Boolean>();
    m_chargeBalance.addOption("Yes", true);
    m_chargeBalance.addOption("No", false);
    m_chargeBalance.setDefaultOption("No", false);
    SmartDashboard.putData("Balancing on Charge Station?", m_chargeBalance);

    if (SubsystemConstants.useDataManger) {
      DataLogManager.start();
    }

    // Configure default commands
    if (SubsystemConstants.useDrive) {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.

          new RunCommand(

              () -> {
                m_robotDrive.m_allowVisionUpdates = false;
                m_robotDrive.drive(
                    // TODO MENTOR:  are deadbands good?  Do we want to try to tweak turning so it's easier to turn a small amount?
                    Math.signum(m_joystick.getRawAxis(1))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.05), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(0))
                        * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.05), 2) * -1
                        * DriveConstants.kMaxSpeed,
                    MathUtil.applyDeadband(m_joystick.getRawAxis(2), 0.1) * -1
                        * DriveConstants.kMaxAngularSpeed,
                    //0.2 * DriveConstants.kMaxSpeed, 0, 0,
                    m_robotDrive.m_fieldRelative);
              },
              m_robotDrive));
    }

    // Configure the trigger bindings
    configureBindings();
  }

  public Command getAutonomousCommand() {
    m_robotDrive.m_allowVisionUpdates = false;

    SequentialCommandGroup ret = new SequentialCommandGroup();

    ret.addCommands(
        new ArmChangeStateCommand(m_robotArm, ArmConstants.scoreState), new WaitCommand(0.1),
        new ChompOpenCommand(m_pneumatics), new WaitCommand(0.1));

    if (m_startPosition.getSelected() == "middle") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 2.19, new Rotation2d(getAllianceTheta())));
      SequentialCommandGroup temp = new SequentialCommandGroup();
      temp.addCommands(
          new ArmChangeStateCommand(m_robotArm, ArmConstants.stowState),
          new MoveSetDistanceCommand(m_robotDrive, getAllianceX(3.1196), 2.7615, //X value is not actually correct, goes over, but should end in roughly the right place because the robot thinks it is a different place than where it is.
              new Rotation2d(getAllianceTheta()),
              AutoConstants.kMaxChargeStationVelocity, AutoConstants.kMaxChargeStationAcceleration,
              List.of()),
          new BalanceCommand(m_robotDrive));

      ret.addCommands(temp.withTimeout(10.5), new DriveParkingBrakeCommand(m_robotDrive));
    }

    if (m_startPosition.getSelected() == "station") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 5.20, new Rotation2d(getAllianceTheta())));
      ret.addCommands(
          new ArmChangeStateCommand(m_robotArm, ArmConstants.stowState),
          new ParallelRaceGroup(
              new MoveSetDistanceCommand(m_robotDrive, getAllianceX(7.0196), 4.58,
                  new Rotation2d(getAllianceTheta()),
                  AutoConstants.kFastAutoVelocity, AutoConstants.kfastAutoAcceleration, List.of()),
              new SequentialCommandGroup(new WaitCommand(0.5), new IntakeCommand(m_intake, m_pneumatics))));
    }

    if (m_startPosition.getSelected() == "drive") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 0.65, new Rotation2d(getAllianceTheta())));
      ret.addCommands(
          new ArmChangeStateCommand(m_robotArm, ArmConstants.stowState),
          new ParallelRaceGroup(
              new MoveSetDistanceCommand(m_robotDrive, getAllianceX(7.0196), 0.82,
                  new Rotation2d(getAllianceTheta()),
                  AutoConstants.kFastAutoVelocity, AutoConstants.kfastAutoAcceleration,
                  List.of(new PathPoint(new Translation2d((getAllianceX(1.8)), (0.75)),
                      new Rotation2d(getAllianceTheta()), new Rotation2d(getAllianceTheta())))),
              new SequentialCommandGroup(new WaitCommand(0.5), new IntakeCommand(m_intake, m_pneumatics))));

    }

    return ret;

  }

  public Command getAutonomousCommandGood() {
    SequentialCommandGroup ret = new SequentialCommandGroup();
    ParallelCommandGroup move = new ParallelCommandGroup();

    ret.addCommands(
        new ArmChangeStateCommand(m_robotArm, ArmConstants.scoreState), new WaitCommand(0.1),
        new ChompOpenCommand(m_pneumatics), new WaitCommand(0.1),
        move);
    move.addCommands(new ArmChangeStateCommand(m_robotArm, ArmConstants.stowState));

    m_robotDrive.m_allowVisionUpdates = false;

    // System.out.println(m_startPosition.getSelected() + " " + m_grabPiece1.getSelected() + " "
    //     + m_grabPiece2.getSelected() + " " + m_chargeBalance.getSelected());
    if (m_startPosition.getSelected() == "middle") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 2.19, new Rotation2d(getAllianceTheta())));
      if (m_grabPiece1.getSelected()) {
        move.addCommands(
            new ParallelRaceGroup(
                new MoveSetDistanceCommand(m_robotDrive, getAllianceX(7.1196), 2.19,
                    new Rotation2d(getAllianceTheta()),
                    AutoConstants.kMaxChargeStationVelocity, AutoConstants.kMaxChargeStationAcceleration, List.of()),
                new SequentialCommandGroup(new WaitCommand(0.5), new IntakeCommand(m_intake, m_pneumatics))));
      } else {
        //deliberatly adding nothing for else
      }
    }

    if (m_startPosition.getSelected() == "station") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 5.20, new Rotation2d(getAllianceTheta())));
      if (m_grabPiece1.getSelected()) {
        move.addCommands(
            new ParallelRaceGroup(
                new MoveSetDistanceCommand(m_robotDrive, getAllianceX(7.1196), 4.58,
                    new Rotation2d(getAllianceTheta()),
                    AutoConstants.kFastAutoVelocity, AutoConstants.kfastAutoAcceleration, List.of()),
                new SequentialCommandGroup(new WaitCommand(0.5), new IntakeCommand(m_intake, m_pneumatics))));
      } else {

      }
    }

    if (m_startPosition.getSelected() == "drive") {
      m_robotDrive.resetOdometry(new Pose2d(getAllianceX(1.36), 0.65, new Rotation2d(getAllianceTheta())));
      if (m_grabPiece1.getSelected()) {
        move.addCommands(
            new ParallelRaceGroup(
                new MoveSetDistanceCommand(m_robotDrive, getAllianceX(7.1196), 0.92,
                    new Rotation2d(getAllianceTheta()),
                    AutoConstants.kFastAutoVelocity, AutoConstants.kfastAutoAcceleration, List.of()),
                new SequentialCommandGroup(new WaitCommand(0.5), new IntakeCommand(m_intake, m_pneumatics))));
      } else {

      }
    }

    if (m_chargeBalance.getSelected()) {
      if (m_grabPiece1.getSelected()) {
        ret.addCommands(
            new MoveSetDistanceCommand(m_robotDrive, getAllianceX(3.485), 2.7615, new Rotation2d(getAllianceTheta()),
                AutoConstants.kMaxChargeStationVelocity, AutoConstants.kMaxAutoAcceleration,
                List.of(
                    new PathPoint(new Translation2d(getAllianceX(5.1196), 2.7615), new Rotation2d(0),
                        new Rotation2d(getAllianceTheta())))),
            new BalanceCommand(m_robotDrive));
      } else {
        move.addCommands(
            new SequentialCommandGroup(
                new MoveSetDistanceCommand(m_robotDrive, getAllianceX(3.485), 2.7615,
                    new Rotation2d(getAllianceTheta()),
                    AutoConstants.kMaxChargeStationVelocity, AutoConstants.kMaxChargeStationAcceleration,
                    List.of(
                        new PathPoint(new Translation2d(getAllianceX(5.1196), 2.7615), new Rotation2d(0),
                            new Rotation2d(getAllianceTheta())))),
                new BalanceCommand(m_robotDrive)));
      }

    }

    return ret;
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
      //new JoystickButton(m_joystick, 10).onTrue(new InstantCommand(m_robotDrive::resetYaw, m_robotDrive)); No go north for now
      new JoystickButton(m_joystick, 16).whileTrue(new DriveParkingBrakeCommand(m_robotDrive));
      new JoystickButton(m_joystick, 14).whileTrue(new BalanceCommand(m_robotDrive));
      new JoystickButton(m_joystick, 11).whileTrue(new AltBalanceCommand(m_robotDrive));
      new JoystickButton(m_joystick, 5).whileTrue(new RunCommand(
          () -> {
            m_robotDrive.m_allowVisionUpdates = false;
            m_robotDrive.drive(
                // TODO MENTOR:  are deadbands good?  Do we want to try to tweak turning so it's easier to turn a small amount? Also this is slowmode
                Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.1), 1) * -1 * DriveConstants.kSlowSpeed,
                Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 1) * -1
                    * DriveConstants.kSlowAngularspeed,
                MathUtil.applyDeadband(m_joystick.getRawAxis(2), 0.2) * -1
                    * DriveConstants.kMaxAngularSpeed,
                //0.2 * DriveConstants.kMaxSpeed, 0, 0,
                m_robotDrive.m_fieldRelative);
          },
          m_robotDrive));

      // Create config for trajectory

      //   new JoystickButton(m_joystick, 13)
      //       .onTrue(new MoveSetDistanceCommand(m_robotDrive, 0.5, 0.75, new Rotation2d(Math.PI / 16),
      //           AutoConstants.kMaxAutoVelocity, AutoConstants.kMaxAutoAcceleration,
      //           List.of()));
      //   new JoystickButton(m_joystick, 12)
      //       .onTrue(new MoveSetDistanceCommand(m_robotDrive, 0, 0, new Rotation2d(0),
      //           AutoConstants.kMaxAutoVelocity, AutoConstants.kMaxAutoAcceleration, // List.of()));
      //           List.of()));
      //   new JoystickButton(m_joystick, 11)
      //       .onTrue(new MoveSetDistanceCommand(m_robotDrive, 0, 0.0, new Rotation2d(-Math.PI / 16),
      //           AutoConstants.kMaxAutoVelocity, AutoConstants.kMaxAutoAcceleration,
      //           List.of()));
    }

    if (SubsystemConstants.useArm) {
      //new JoystickButton(m_opJoystick, 4).onTrue(new ChickenCommand(m_robotArm));
      //new JoystickButton(m_opJoystick, 7).onTrue(new BackToNormalCommand(m_robotArm));
      new Trigger(() -> m_manualArm).whileTrue(
          new RunCommand(() -> {
            m_robotArm.moveShoulder(MathUtil.applyDeadband(m_opJoystick.getRawAxis(1), 0.1) * 500);
            m_robotArm.moveElbow(MathUtil.applyDeadband(m_opJoystick.getRawAxis(5), 0.1) * 500);
          }, m_robotArm));

      new JoystickButton(m_opJoystick, 7).onTrue(new InstantCommand(() -> m_manualArm = true));
      new JoystickButton(m_opJoystick, 8).onTrue(new InstantCommand(() -> m_manualArm = false));

      //new JoystickButton(m_opJoystick, 1).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.intakeState));
      new JoystickButton(m_opJoystick, 4).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.highState));
      new JoystickButton(m_opJoystick, 2).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.midState));
      new JoystickButton(m_opJoystick, 1).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.lowState));
      new JoystickButton(m_opJoystick, 3).onTrue(new ArmChangeStateCommand(m_robotArm, ArmConstants.stowState));

      new Trigger(() -> m_robotArm.m_movingFromIntake).onTrue(new InstantCommand(m_pneumatics::intakeIn));//This doesn't work when using ArmCycleStateCommand

      new Trigger(() -> {
        return m_opJoystick.getRawAxis(2) > 0.95;
      }).onTrue(new ArmCycleStateCommand(m_robotArm, false));
      new Trigger(() -> {
        return m_opJoystick.getRawAxis(3) > 0.95;
      }).onTrue(new ArmCycleStateCommand(m_robotArm, true));

      new JoystickButton(m_opJoystick, 10).onTrue(new InstantCommand(m_robotArm::resetArmEncoders));

      if (SubsystemConstants.usePneumatics) {
        new JoystickButton(m_opJoystick, 6).onTrue( //drop n chomp
            new SequentialCommandGroup(new ChompOpenCommand(m_pneumatics),
                new InstantCommand(m_pneumatics::intakeOut),
                new ArmChangeStateCommand(m_robotArm, ArmConstants.intakeState),
                new ChompCloseCommand(m_pneumatics)));

        new JoystickButton(m_opJoystick, 5).onTrue(new ChompToggleCommand(m_pneumatics));
      }
    }
    if (SubsystemConstants.useTurnTables) {
      new Trigger(() -> {
        return m_opJoystick.getPOV() == 270;
      }).whileTrue(new RunCommand(m_turntables::spinCounterClockwise))
          .onFalse(new InstantCommand(m_turntables::stopSpinning));
      new Trigger(() -> {
        return m_opJoystick.getPOV() == 90;
      }).whileTrue(new RunCommand(m_turntables::spinClockwise)).onFalse(new InstantCommand(m_turntables::stopSpinning));
      new Trigger(() -> {
        return m_opJoystick.getPOV() == 0;
      }).onTrue(new InstantCommand(m_turntables::stopSpinning));

      new JoystickButton(m_joystick, 9).onTrue(new MagicTurntable(m_turntables));
      new JoystickButton(m_joystick, 9).debounce(TurntableConstants.waitTime, DebounceType.kFalling)
          .onFalse(new InstantCommand(m_turntables::stopMagicTurntable));
    }

    if (SubsystemConstants.useIntake && SubsystemConstants.useTurnTables && SubsystemConstants.usePneumatics) {
      new JoystickButton(m_joystick, 1).whileTrue(new IntakeCommand(m_intake, m_pneumatics))
          .onTrue(new InstantCommand(() -> m_turntables.spinClockwise(), m_turntables)); // TODO MENTOR: we need a lot of new logic

    }
    // if (SubsystemConstants.useDrive && SubsystemConstants.useVision) {
    //   new JoystickButton(m_joystick, 16).whileTrue(new SequentialCommandGroup(
    //       new MoveWithVisionCommand(m_robotDrive, m_visionSubsystem, "left"),
    //       new RunCommand(
    //           () -> m_robotDrive.drive(0, 0, 0, m_robotDrive.m_fieldRelative), m_robotDrive)));
    //   new JoystickButton(m_joystick, 15).whileTrue(new SequentialCommandGroup(
    //       new MoveWithVisionCommand(m_robotDrive, m_visionSubsystem, "middle"),
    //       new RunCommand(
    //           () -> m_robotDrive.drive(0, 0, 0, m_robotDrive.m_fieldRelative), m_robotDrive)));
    //   new JoystickButton(m_joystick, 14).whileTrue(new SequentialCommandGroup(
    //       new MoveWithVisionCommand(m_robotDrive, m_visionSubsystem, "right"),
    //       new RunCommand(
    //           () -> m_robotDrive.drive(0, 0, 0, m_robotDrive.m_fieldRelative), m_robotDrive)));
    // }

    //new JoystickButton(m_opJoystick, 7).whileTrue(new TestCommand(m_turntables));

    if (SubsystemConstants.useDrive && SubsystemConstants.useLimelight) {
      new JoystickButton(m_joystick, 4).whileTrue(new DriveTrackGamePiece(m_robotDrive, m_joystick, true));
      new JoystickButton(m_joystick, 3).whileTrue(new DriveTrackGamePiece(m_robotDrive, m_joystick, false));

    }
  }

  public void resetDriveOffsets() {

    if (SubsystemConstants.useDrive) {
      m_robotDrive.resetOffsets();

    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

  }

  public double getAllianceX(double X) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return X;
    } else {
      return AutoConstants.fieldLength - X;
    }
  }

  public double getAllianceTheta() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return 0;
    } else {
      return Math.PI;
    }
  }

  public void updateDriveFromVision() {
    if (SubsystemConstants.useDrive && SubsystemConstants.useVision) {
      if (m_visionSubsystem.m_lastPosition != null && m_robotDrive.m_allowVisionUpdates)
        m_robotDrive.resetOdometry(m_visionSubsystem.m_lastPosition);
    }
  }
}
