package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveShoudlerCommand extends CommandBase {
    private final ArmSubsystem m_subsystem;
    private final double m_speed;

    public ArmMoveShoudlerCommand(ArmSubsystem subsystem, double speed) {
        m_subsystem = subsystem;
        m_speed = speed;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_subsystem.moveShoulder(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.moveShoulder(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
