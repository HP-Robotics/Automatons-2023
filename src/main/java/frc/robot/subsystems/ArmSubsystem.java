// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
public class ArmSubsystem extends SubsystemBase {
  private final TalonFX m_shoulderMotor;
  //private final TalonFX m_elbowMotor;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_shoulderMotor = new TalonFX(ArmConstants.shoulderID);
    m_shoulderMotor.configFactoryDefault();
    m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
    

  
    m_shoulderMotor.config_kP(0, ArmConstants.shoulderkP);
    m_shoulderMotor.config_kI(0, ArmConstants.shoulderkI);
    m_shoulderMotor.config_kD(0, ArmConstants.shoulderkD);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void chickenMove() {
    m_shoulderMotor.set(ControlMode.Position, 1000); 
  

  }

  public void backToNormal() {
    m_shoulderMotor.set(ControlMode.Position, 0); 

  }
}
