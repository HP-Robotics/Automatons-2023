// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class BalanceCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    private final PIDController m_PidController;
    double startPos;

    /*
     * Creates a new ResetFalconCommand.
     ***
     * @param subsystem The subsystem used by this command.
     */
    public BalanceCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        m_PidController = new PIDController(DriveConstants.balancekP, DriveConstants.balancekI,
                DriveConstants.balancekD);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_PidController.setSetpoint(0);
        m_PidController.setTolerance(1);
        startPos = m_subsystem.getPoseX();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Starting Position", startPos);
        SmartDashboard.putNumber("Current Position", m_subsystem.getPoseX());
        SmartDashboard.putNumber("Current Position Y", m_subsystem.getPoseY());
        double power = m_PidController.calculate(m_subsystem.getCombinedRoll());
        if (power > DriveConstants.balanceThreshold) {
            power = DriveConstants.balanceThreshold;
        }
        if (power < -DriveConstants.balanceThreshold) {
            power = -DriveConstants.balanceThreshold;
        }
        if (m_subsystem.getPoseX() > startPos + 2 || m_subsystem.getPoseX() < startPos - 2) {
            m_subsystem.drive(0, 0, 0, true);
        } else {
            m_subsystem.drive(power * -1, 0, 0, true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return m_PidController.atSetpoint();
        return false;
    }
}
