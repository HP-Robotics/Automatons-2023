// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX m_shoulderMotor;
  private final TalonFX m_elbowMotor;
  private final DutyCycleEncoder m_shoulderEncoder;
  private final DutyCycleEncoder m_elbowEncoder;
  private int m_targetState;
  private int m_currentState;
  private boolean m_isChanging;
  private int m_pastState = -1;
  private int m_frameCounter;
  private boolean m_doneChanging;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_shoulderMotor = new TalonFX(ArmConstants.shoulderID);
    m_shoulderMotor.configFactoryDefault();
    m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
    m_shoulderMotor.config_kP(0, ArmConstants.shoulderkP);
    m_shoulderMotor.config_kI(0, ArmConstants.shoulderkI);
    m_shoulderMotor.config_kD(0, ArmConstants.shoulderkD);
    m_shoulderMotor.config_IntegralZone(0, ArmConstants.shoulderIZone);
    m_shoulderMotor.configAllowableClosedloopError(0, ArmConstants.shoulderMaxAllowableError);
    m_shoulderMotor.configPeakOutputForward(1);
    m_shoulderMotor.configPeakOutputReverse(-1); //TODO MENTOR: remove these eventually, once tuning is finished
    m_shoulderMotor.configMotionAcceleration(ArmConstants.shoulderAcceleration);
    m_shoulderMotor.configMotionCruiseVelocity(ArmConstants.shoulderMaxVelocity);
    m_shoulderMotor.configMotionSCurveStrength(ArmConstants.shoulderSCurve);

    m_shoulderEncoder = new DutyCycleEncoder(1);

    m_elbowMotor = new TalonFX(ArmConstants.elbowID);
    m_elbowMotor.configFactoryDefault();
    m_elbowMotor.setNeutralMode(NeutralMode.Brake);
    m_elbowMotor.config_kP(0, ArmConstants.shoulderkP);
    m_elbowMotor.config_kI(0, ArmConstants.shoulderkI);
    m_elbowMotor.config_kD(0, ArmConstants.shoulderkD);
    m_elbowMotor.configPeakOutputForward(1);
    m_elbowMotor.configPeakOutputReverse(-1); //TODO MENTOR: remove these eventually, once tuning is finished
    m_elbowMotor.configMotionAcceleration(ArmConstants.elbowAcceleration);
    m_elbowMotor.configMotionCruiseVelocity(ArmConstants.elbowMaxVelocity);
    m_elbowMotor.configMotionSCurveStrength(ArmConstants.elbowSCurve);

    m_elbowEncoder = new DutyCycleEncoder(0);

    SmartDashboard.putNumber("Elbow Mid", ArmConstants.elbowMid);
    SmartDashboard.putNumber("Shoulder Mid", ArmConstants.shoulderMid);

    SmartDashboard.putNumber("Elbow High", ArmConstants.elbowHigh);
    SmartDashboard.putNumber("Shoulder High", ArmConstants.shoulderHigh);

    SmartDashboard.putNumber("Elbow Score", ArmConstants.elbowScore);
    SmartDashboard.putNumber("Shoulder Score", ArmConstants.shoulderScore);

    m_targetState = 1;
    m_currentState = 1;
    m_isChanging = false;
    m_frameCounter = 0;

    // TODO MENTOR:  If the arm is all the way out and we deploy new code, at enable, we would move the arm to starting state.  Is that safe?
    // m_shoulderMotor.set(ControlMode.Position, ArmConstants.shoulderPositions[m_currentState]);
    // m_elbowMotor.set(ControlMode.Position, ArmConstants.elbowPositions[m_currentState]);
    // m_shoulderMotor.setSelectedSensorPosition(0);
    // m_elbowMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //moveShoulder(m_joystick.getRawAxis(1) * -0.3);
    //moveElbow(m_joystick.getRawAxis(5) * 0.3);
    SmartDashboard.putNumber("Shoulder Absolute Encoder", m_shoulderEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elbow Absolute Encoder", m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shoulder Falcon Encoder", m_shoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow Falcon Encoder", m_elbowMotor.getSelectedSensorPosition());

    ArmConstants.elbowPositions[ArmConstants.midState] = SmartDashboard.getNumber("Elbow Mid",
        ArmConstants.elbowMid);
    ArmConstants.shoulderPositions[ArmConstants.midState] = SmartDashboard.getNumber("Shoulder Mid",
        ArmConstants.shoulderMid);

    ArmConstants.elbowPositions[ArmConstants.highState] = SmartDashboard.getNumber("Elbow High",
        ArmConstants.elbowHigh);
    ArmConstants.shoulderPositions[ArmConstants.highState] = SmartDashboard.getNumber("Shoulder High",
        ArmConstants.shoulderHigh);

    ArmConstants.elbowPositions[ArmConstants.scoreState] = SmartDashboard.getNumber("Elbow Score",
        ArmConstants.elbowScore);
    ArmConstants.shoulderPositions[ArmConstants.scoreState] = SmartDashboard.getNumber("Shoulder Score",
        ArmConstants.shoulderScore);

    SmartDashboard.putNumber("Past State", m_pastState);

    SmartDashboard.putNumber("Target State", m_targetState);
    SmartDashboard.putNumber("Current State", m_currentState);
    SmartDashboard.putBoolean("Is Changing?", m_isChanging);
    SmartDashboard.putNumber("Shoulder Error",
        m_shoulderMotor.getSelectedSensorPosition() - ArmConstants.shoulderPositions[m_currentState]);
    SmartDashboard.putNumber("Elbow Error",
        m_elbowMotor.getSelectedSensorPosition() - ArmConstants.elbowPositions[m_currentState]);
    m_isChanging = Math.abs(m_shoulderMotor.getSelectedSensorPosition()
        - ArmConstants.shoulderPositions[m_currentState]) > ArmConstants.errorThreshold
        || Math.abs(m_elbowMotor.getSelectedSensorPosition()
            - ArmConstants.elbowPositions[m_currentState]) > ArmConstants.errorThreshold;

    m_doneChanging = !m_isChanging && m_frameCounter >= ArmConstants.frameCounterThreshold;
    if (!m_isChanging) {
      m_pastState = m_currentState;
      m_frameCounter++;
    }

  }

  public void moveShoulder(double newSetpoint) {
    m_shoulderMotor.set(ControlMode.MotionMagic, m_shoulderMotor.getClosedLoopTarget() + newSetpoint);
    System.out.println("shoulder running at " + newSetpoint);
  }

  public void moveElbow(double newSetpoint) {
    m_elbowMotor.set(ControlMode.MotionMagic, m_elbowMotor.getClosedLoopTarget() - newSetpoint);
    System.out.println("elbow running at " + newSetpoint);
  }

  public double absoluteToAngle(double absValue) {
    return 0;
  }

  public void changeState(int state) {
    if (state < ArmConstants.intakeState) {
      state = ArmConstants.intakeState;
    } else if (state > ArmConstants.scoreState) {
      state = ArmConstants.scoreState;
    }
    m_targetState = state;

  }

  public int getCurrentState() {
    return m_currentState;
  }

  public int getTargetState() {
    return m_targetState;
  }

  public boolean getIsChanging() {
    return m_isChanging;
  }

  public boolean getDoneChanging() {
    return m_doneChanging;
  }

  public void moveDownState() {
    if (m_currentState == m_pastState && m_currentState != ArmConstants.intakeState) {
      m_currentState--;
    }

    m_isChanging = true;
    m_doneChanging = false;
    m_frameCounter = 0;
    m_shoulderMotor.set(ControlMode.MotionMagic, ArmConstants.shoulderPositions[m_currentState]);
    SmartDashboard.putNumber("Shoulder Target", ArmConstants.shoulderPositions[m_currentState]);
    m_elbowMotor.set(ControlMode.MotionMagic, ArmConstants.elbowPositions[m_currentState]);
    SmartDashboard.putNumber("Elbow Target", ArmConstants.elbowPositions[m_currentState]);
  }

  public void moveUpState() {
    if (m_currentState == m_pastState && m_currentState != ArmConstants.scoreState) {
      m_currentState++;
    }

    m_isChanging = true;
    m_doneChanging = false;
    m_frameCounter = 0;
    m_shoulderMotor.set(ControlMode.MotionMagic, ArmConstants.shoulderPositions[m_currentState]);
    SmartDashboard.putNumber("Shoulder Target", ArmConstants.shoulderPositions[m_currentState]);
    m_elbowMotor.set(ControlMode.MotionMagic, ArmConstants.elbowPositions[m_currentState]);
    SmartDashboard.putNumber("Elbow Target", ArmConstants.elbowPositions[m_currentState]);
    //TODO MENTOR: We may need to set relative velocities to get the right movement
  }

  public void setTargetState(int state) {
    if (state < ArmConstants.intakeState) {
      state = ArmConstants.intakeState;
    } else if (state > ArmConstants.scoreState) {
      state = ArmConstants.scoreState;
    }
    m_targetState = state;
  }

  public void setFalconEncoders() {
    double deltaE = ArmConstants.elbowStarting - m_elbowEncoder.getAbsolutePosition();
    double ticksE = deltaE * ArmConstants.elbowGearRatio * DriveConstants.kEncoderResolution;
    double deltaS = (ArmConstants.shoulderStarting - m_shoulderEncoder.getAbsolutePosition()) * -1;
    double ticksS = deltaS * ArmConstants.shoulderGearRatio * DriveConstants.kEncoderResolution;
    m_elbowMotor.setSelectedSensorPosition(ticksE);
    m_shoulderMotor.setSelectedSensorPosition(ticksS);

  }
}
