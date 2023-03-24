// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
  private boolean m_isChanging;
  private int m_pastState = -1;
  private int m_frameCounter;
  private boolean m_doneChanging;
  public boolean m_movingFromIntake;
  private Timer m_start = new Timer();
  private Boolean m_initializedEncoders = false;

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

    m_elbowMotor = new TalonFX(ArmConstants.elbowID);
    m_elbowMotor.configFactoryDefault();
    m_elbowMotor.setNeutralMode(NeutralMode.Brake);
    m_elbowMotor.config_kP(0, ArmConstants.elbowkP);
    m_elbowMotor.config_kI(0, ArmConstants.elbowkI);
    m_elbowMotor.config_kD(0, ArmConstants.elbowkD);
    m_elbowMotor.config_IntegralZone(0, ArmConstants.elbowIZone);
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

    m_start.start();

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
    double deltaE = ArmConstants.elbowStarting - getAdjustedAbsoluteElbow();
    double ticksE = -1 * deltaE * ArmConstants.elbowGearRatio * DriveConstants.kEncoderResolution;
    double deltaS = ArmConstants.shoulderStarting - getAdjustedAbsoluteShoulder();
    double ticksS = deltaS * ArmConstants.shoulderGearRatio * DriveConstants.kEncoderResolution;
    double fancyEquation = 395650.6045342109 * getAdjustedAbsoluteElbow() - 307888.2646918395;
    double fancyFunction = 340626.7792820861 - 721413.6886682262 * getAdjustedAbsoluteShoulder();
    double fancyFunction2 = 302129.742102882 - 662435.677872067 * getAdjustedAbsoluteShoulder();

    if (m_start.hasElapsed(2.5)) {
      m_start.stop();
      m_start.reset();
      if (!m_initializedEncoders) {
        setFalconEncoders();
        m_initializedEncoders = true;
      }
    }

    SmartDashboard.putNumber("Elbow ticksE", ticksE);
    SmartDashboard.putNumber("ElbowFancy-30", fancyEquation - 30000);
    SmartDashboard.putNumber("ElbowFancyequation", fancyEquation);
    SmartDashboard.putNumber("Shoulder Drop-Low Equation", fancyFunction);
    SmartDashboard.putNumber("Shoulder High-Score Equation", fancyFunction2);
    SmartDashboard.putNumber("ElbowVoltage", m_elbowMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("ElbowCurrent", m_elbowMotor.getStatorCurrent());
    SmartDashboard.putNumber("Shoulder ticksS", ticksS);
    SmartDashboard.putNumber("Shoulder Absolute Encoder", getAdjustedAbsoluteShoulder());
    SmartDashboard.putNumber("Elbow Absolute Encoder", getAdjustedAbsoluteElbow());
    SmartDashboard.putNumber("Elbow Real Absolute Encoder", m_elbowEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Shoulder Falcon Encoder", m_shoulderMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elbow Falcon Encoder", m_elbowMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Shoulder Motion Position", m_shoulderMotor.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Elbow Motion Position", m_elbowMotor.getActiveTrajectoryPosition());

    SmartDashboard.putNumber("Shoulder Output Percent", m_shoulderMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Elbow Output Percent", m_elbowMotor.getMotorOutputPercent());

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

    ArmConstants.elbowPositions[ArmConstants.intakeState] = SmartDashboard.getNumber("Elbow Chomp",
        ArmConstants.elbowIntake);
    ArmConstants.shoulderPositions[ArmConstants.intakeState] = SmartDashboard.getNumber("Shoulder Chomp",
        ArmConstants.shoulderIntake);

    SmartDashboard.putNumber("Past State", m_pastState);

    SmartDashboard.putNumber("Target State", m_targetState);
    SmartDashboard.putNumber("Current State", m_currentState);
    SmartDashboard.putBoolean("Is Changing?", m_isChanging);
    SmartDashboard.putNumber("Shoulder Error", m_shoulderMotor.getClosedLoopError(0));
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

    if (m_isChanging) {
      //setFalconEncoders();
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
    // MENTOR TODO:  if (Math.abs(newSetPoint) > 1) {}, both here and in moveElbow()
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
    if (m_currentState == m_pastState && m_isChanging) {
      return;
    }
    if (m_currentState != ArmConstants.intakeState) {
      if ((m_currentState - m_pastState) > 0) {
        m_currentState = m_pastState;
      } else if (m_targetState <= ArmConstants.lowState && m_pastState == ArmConstants.highState) {
        m_currentState = ArmConstants.lowState;
      } else if (m_pastState == ArmConstants.lowState && m_targetState == ArmConstants.intakeState) {
        m_currentState = ArmConstants.intakeState;

      } else {
        m_currentState = m_pastState - 1;
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
    if (m_currentState == m_pastState && m_isChanging) {
      return;
    }
    if (m_currentState != ArmConstants.scoreState) {
      if ((m_currentState - m_pastState) < 0) {
        m_currentState = m_pastState;
      } else if (m_targetState >= ArmConstants.highState && m_pastState == ArmConstants.lowState) {
        m_currentState = ArmConstants.highState;
      } /*else if (m_currentState == ArmConstants.intakeState && m_targetState == ArmConstants.highState) {
        m_currentState = ArmConstants.highState;
        } */else if (m_pastState == ArmConstants.intakeState && m_targetState == ArmConstants.midState) {
        m_currentState = ArmConstants.midState;
      } else if (m_pastState == ArmConstants.intakeState && m_targetState >= ArmConstants.lowState) {
        m_currentState = ArmConstants.lowState;
      } else {
        m_currentState = m_pastState + 1;
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
    double deltaE = ArmConstants.elbowStarting - getAdjustedAbsoluteElbow();
    double ticksE = -1 * deltaE * ArmConstants.elbowGearRatio * DriveConstants.kEncoderResolution;
    double deltaS = (ArmConstants.shoulderStarting - getAdjustedAbsoluteShoulder()) * -1;
    double ticksS = -1 * deltaS * ArmConstants.shoulderGearRatio * DriveConstants.kEncoderResolution;

    double elbowHighAbsTicks = 395650.6045342109 * getAdjustedAbsoluteElbow() - 307888.2646918395;
    double shoulderLowAbsTicks = 340626.7792820861 - 721413.6886682262 * getAdjustedAbsoluteShoulder();
    double shoulderHighAbsTicks = 302129.742102882 - 662435.677872067 * getAdjustedAbsoluteShoulder();
    // System.out.println("Elbow target " + ticksE);
    // System.out.println("Elbow delta " + deltaE);
    // System.out.println("Elbow current " + m_elbowMotor.getSelectedSensorPosition());
    System.out.println("Elbow Error: " + (ticksE - m_elbowMotor.getSelectedSensorPosition()));
    System.out.println("Shoulder Error: " + (ticksS - m_shoulderMotor.getSelectedSensorPosition()));
    // System.out.println("Shoulder target " + ticksS);
    // System.out.println("Shoulder current " + m_shoulderMotor.getSelectedSensorPosition());
    if (m_elbowEncoder.getAbsolutePosition() != 0.0
        && (m_currentState == ArmConstants.lowState || !m_initializedEncoders)) {
      m_elbowMotor.setSelectedSensorPosition(elbowHighAbsTicks);
    }
    if (m_shoulderEncoder.getAbsolutePosition() != 0.0
        && (m_currentState == ArmConstants.lowState || m_currentState == ArmConstants.intakeState)
        || !m_initializedEncoders) {
      m_shoulderMotor.setSelectedSensorPosition(shoulderLowAbsTicks);
    }
    // if (m_shoulderEncoder.getAbsolutePosition() != 0.0
    //     && (m_currentState == ArmConstants.highState || m_currentState == ArmConstants.scoreState)) {
    //   m_shoulderMotor.setSelectedSensorPosition(shoulderHighAbsTicks);
    // }
    // m_shoulderMotor.setSelectedSensorPosition(ticksS);
    // if (m_shoulderEncoder.getAbsolutePosition() != 0.0) {
    //   m_shoulderMotor.setSelectedSensorPosition(ticksS);
    //   System.out.println("Setting shoulder to " + ticksS);
    // }
    // if (m_elbowEncoder.getAbsolutePosition() != 0.0) {
    //   m_elbowMotor.setSelectedSensorPosition(ticksE);
    //   System.out.println("Setting elbow to " + ticksE);
    // }
  }

  public double getAdjustedAbsoluteElbow() {
    if (m_elbowEncoder.getAbsolutePosition() < 0.4) {
      return m_elbowEncoder.getAbsolutePosition() + 1 + ArmConstants.elbowOffset;
    } else {
      return m_elbowEncoder.getAbsolutePosition() + ArmConstants.elbowOffset;
    }
  }

  public double getAdjustedAbsoluteShoulder() {
    if (m_shoulderEncoder.getAbsolutePosition() < 0.0) { //TODO: 0.0 is a filler number, this needs to be tuned
      return m_shoulderEncoder.getAbsolutePosition() + 1 + ArmConstants.shoulderOffset;
    } else {
      return m_shoulderEncoder.getAbsolutePosition() + ArmConstants.shoulderOffset;
    }
  }
}
