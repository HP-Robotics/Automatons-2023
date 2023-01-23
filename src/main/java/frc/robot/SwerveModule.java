// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import javax.swing.plaf.basic.BasicBorders.RadioButtonBorder;
import javax.xml.namespace.QName;
import javax.xml.transform.TransformerConfigurationException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveModule {
  private static final double kWheelRadius = 0.04445;
  private static final int kEncoderResolution = 2048;

  private static final double driveGearRatio = 6.75;

  private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  


  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor.
   * 
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel
     ) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.configFactoryDefault();
    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_turningMotor = new TalonFX(turningMotorChannel);
  
    m_turningMotor.configFactoryDefault();
    m_turningMotor.config_kP(0, .1);
    m_turningMotor.config_kD(0, 1);
    m_turningMotor.setInverted(true);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / (kEncoderResolution*driveGearRatio));

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
   // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

  
  }

  public double getTurningPosition() {
    return m_turningMotor.getSensorCollection().getIntegratedSensorAbsolutePosition();
  }

  public double radiansToTicks(double radians) {
    //drive ratio: 6.75:1
    //rotation ratio: 15.429:1
    return radians * ((2048*15.429)/(2*Math.PI));
  }

  public double ticksToRadians(double ticks) {
    return ticks * ((2*Math.PI)/(2048*15.429));
  } 

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(ticksToRadians(m_turningMotor.getSelectedSensorPosition())));
        Rotation2d currentState = new Rotation2d(ticksToRadians(m_turningMotor.getSelectedSensorPosition()));

        //System.out.println(m_turningMotor.getSelectedSensorPosition() + " | " + radiansToTicks(state.angle.getRadians()) + " | " + m_turningMotor.getClosedLoopError());

    //m_driveMotor.set(ControlMode.Velocity, metersToTicks(state.speedMetersPerSecond));
     //System.out.println(radiansToTicks(desiredState.angle.getDegrees()));
     
  // m_turningMotor.set(ControlMode.Position, radiansToTicks(state.angle.getRadians()));
    m_driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond/10);
    //System.out.println("Output: " + -radiansToTicks(state.angle.getRadians()) + " | Input: " + desiredState.angle.getDegrees() + " | In-Between: " + state.angle.getDegrees() + " | Current Wheel Position: " + currentState.getDegrees() + "Speed Output: " + state.speedMetersPerSecond/10 + " | Speed In-Between: " + desiredState.speedMetersPerSecond/10);
    //System.out.println("Speed Output: " + state.speedMetersPerSecond/10 + " | Speed In-Between: " + desiredState.speedMetersPerSecond/10 + " | Current Wheel Position: " + currentState.getDegrees());
   
   


  
  }
}
