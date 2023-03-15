// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.spline.Spline;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX m_shoulderMotor;
  private final TalonFX m_elbowMotor;
  private final DutyCycleEncoder m_shoulderEncoder;
  private final DutyCycleEncoder m_elbowEncoder;
  private int m_targetState;
  private int m_currentState;
  private AHRS m_shoulderGyro;
  private boolean m_isChanging;
  private int m_pastState = -1;
  private int m_frameCounter;
  private boolean m_doneChanging;
  public boolean m_movingFromIntake;

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

    m_shoulderEncoder = new DutyCycleEncoder(ArmConstants.shoulderEncoderID);
    m_shoulderGyro = new AHRS(Port.kUSB);

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

    m_elbowEncoder = new DutyCycleEncoder(ArmConstants.elbowEncoderID);

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
    m_movingFromIntake = false;

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
    SmartDashboard.putNumber("Shoulder Absolute Encoder", m_shoulderEncoder.get());
    SmartDashboard.putNumber("Elbow Absolute Encoder", m_elbowEncoder.get());
    SmartDashboard.putNumber("Shoulder Falcon Encoder", m_shoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow Falcon Encoder", m_elbowMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("Shoulder Pitch", m_shoulderGyro.getPitch());
    SmartDashboard.putNumber("Shoulder Yaw", m_shoulderGyro.getYaw());
    SmartDashboard.putNumber("Shoulder Roll", m_shoulderGyro.getRoll());

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

  //This is Trajectory for the arm
  public BufferedTrajectoryPointStream generateArmTrajectory(Translation2d start, Translation2d end,
      List<Translation2d> waypoints) {
    TrajectoryConfig config = new TrajectoryConfig(ArmConstants.trajectoryMaxVelocity,
        ArmConstants.trajectoryMaxAcceleration);
    Spline.ControlVector startPos = new Spline.ControlVector(new double[] { start.getX(), 0 },
        new double[] { start.getY(), 0 });
    Spline.ControlVector endPos = new Spline.ControlVector(new double[] { end.getX(), 0 },
        new double[] { end.getY(), 0 });
    Trajectory armTrajectory = TrajectoryGenerator.generateTrajectory(startPos, waypoints, endPos, config);
    int trajectorylength = armTrajectory.getStates().size();
    BufferedTrajectoryPointStream armTrajectoryPoints = new BufferedTrajectoryPointStream();
    for (int i = 0; i <= trajectorylength; i++) {
      TrajectoryPoint pointI = new TrajectoryPoint();
      pointI.position = armTrajectory.getStates().get(i).poseMeters.getX();//do this again with getY() for elbow/arm
      pointI.velocity = armTrajectory.getStates().get(i).velocityMetersPerSecond;
      pointI.isLastPoint = (i == trajectorylength);
      pointI.timeDur = 20; //or zero
      armTrajectory.getStates().get(i);
      armTrajectoryPoints.Write(pointI);
    }

    return null;
  }

  public void followTrajectory(Translation2d start, Translation2d end, List<Translation2d> waypoints) {
    m_elbowMotor.startMotionProfile(generateArmTrajectory(start, end, waypoints), 0, ControlMode.MotionProfile);
    new BufferedTrajectoryPointStream();

  }

  public void moveShoulder(double newSetpoint) {
    m_shoulderMotor.set(ControlMode.MotionMagic, m_shoulderMotor.getClosedLoopTarget() + newSetpoint);
    // System.out.println("shoulder running at " + newSetpoint);
  }

  public void moveElbow(double newSetpoint) {
    m_elbowMotor.set(ControlMode.MotionMagic, m_elbowMotor.getClosedLoopTarget() - newSetpoint);
    // System.out.println("elbow running at " + newSetpoint);
  }

  public double absoluteToAngle(double absValue) {
    return 0;
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
      if (m_targetState == ArmConstants.stowState && m_currentState == ArmConstants.highState) {
        m_currentState = ArmConstants.lowState;
      } else {
        m_currentState--;
      }
    }

    m_isChanging = true;
    m_doneChanging = false;
    m_frameCounter = 0;
    m_shoulderMotor.set(ControlMode.MotionMagic, ArmConstants.shoulderPositions[m_currentState]);
    SmartDashboard.putNumber("Shoulder Target", ArmConstants.shoulderPositions[m_currentState]);
    m_elbowMotor.set(ControlMode.MotionMagic, ArmConstants.elbowPositions[m_currentState]);
    SmartDashboard.putNumber("Elbow Target", ArmConstants.elbowPositions[m_currentState]);
    SmartDashboard.putBoolean("Leaving Intake", m_movingFromIntake);
  }

  public void moveUpState() {
    if (m_currentState == m_pastState && m_currentState != ArmConstants.scoreState) {
      if (m_targetState == ArmConstants.highState && m_pastState == ArmConstants.lowState) {
        m_currentState = ArmConstants.highState;
      } else {
        m_currentState++;
      }
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
    if (m_targetState == ArmConstants.stowState) {
      m_movingFromIntake = true;
    }
  }

  public void setFalconEncoders() {
    double deltaE = ArmConstants.elbowStarting - m_elbowEncoder.get();
    double ticksE = -1 * deltaE * ArmConstants.elbowGearRatio * DriveConstants.kEncoderResolution;
    double deltaS = (ArmConstants.shoulderStarting - m_shoulderEncoder.get()) * -1;
    double ticksS = deltaS * ArmConstants.shoulderGearRatio * DriveConstants.kEncoderResolution;

    System.out.println("Elbow target " + ticksE);
    System.out.println("Elbow delta " + deltaE);
    System.out.println("Elbow current " + m_elbowMotor.getSelectedSensorPosition());
    // System.out.println("Shoulder target " + ticksS);
    // System.out.println("Shoulder current " + m_shoulderMotor.getSelectedSensorPosition());
    if (m_elbowEncoder.getAbsolutePosition() != 0.0) {
      //  m_elbowMotor.setSelectedSensorPosition(ticksE);
    }
    // m_shoulderMotor.setSelectedSensorPosition(ticksS);

  }

  public void resetArmEncoders() {
    if (m_currentState == ArmConstants.stowState) {
      m_shoulderMotor.setSelectedSensorPosition(0);
      m_elbowMotor.setSelectedSensorPosition(0);
    }
  }
}
