package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrackGamePiece extends CommandBase {
    private final DriveSubsystem m_subsystem;
    private final boolean m_isCone;
    private final Joystick m_joystick;

    public DriveTrackGamePiece(DriveSubsystem subsytem, Joystick joystick, boolean isCone) {
        m_subsystem = subsytem;
        m_joystick = joystick;
        m_isCone = isCone;
        addRequirements(m_subsystem);
        System.out.println("IsCone: " + isCone);

    }

    @Override
    public void initialize() {
        if (m_isCone) {
            m_subsystem.switchConePipeline();
        } else {
            m_subsystem.switchCubePipeline();
        }
    }

    @Override
    public void execute() {
        if (m_subsystem.gamePieceSeen()) {
            m_subsystem.drive(
                    Math.signum(m_joystick.getRawAxis(1))
                            * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.1), 2) * -1
                            * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(1))
                            * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 2) * -1
                            * DriveConstants.kMaxSpeed,
                    m_subsystem.getGamePieceX() * LimelightConstants.turnValue, false);
        } else {
            m_subsystem.drive(
                    Math.signum(m_joystick.getRawAxis(1))
                            * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(1), 0.1), 2) * -1
                            * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(1))
                            * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 2) * -1
                            * DriveConstants.kMaxSpeed,
                    Math.signum(m_joystick.getRawAxis(1))
                            * Math.pow(MathUtil.applyDeadband(m_joystick.getRawAxis(0), 0.1), 2) * -1
                            * DriveConstants.kMaxAngularSpeed,
                    true);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}