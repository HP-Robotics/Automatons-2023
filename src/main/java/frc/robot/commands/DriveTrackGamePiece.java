package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTrackGamePiece extends CommandBase {
    private final DriveSubsystem m_subsystem;
    private final boolean m_isCone;

    public DriveTrackGamePiece(DriveSubsystem subsytem, boolean isCone) {
        m_subsystem = subsytem;
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
            m_subsystem.drive(0, 0, m_subsystem.getGamePieceX() * LimelightConstants.turnValue, false);
        } else {
            m_subsystem.drive(0, 0, 0, false);
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