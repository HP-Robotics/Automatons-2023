// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class VisionSubsystem extends SubsystemBase {

  GenericEntry robotPose;
  NetworkTableEntry yEntry;
  NetworkTableEntry targetPose;
  Pose2d apriltagPose;
  Pose2d absolutePose;
  Transform2d cameraTrans;
  Map<Integer, Pose2d> apriltags;
  int apriltageid;
  int loopCount = 0;
  List<Pose2d> m_blueScoring;
  List<Pose2d> m_redScoring;
  public Pose2d m_lastPosition;

  PhotonCamera camera = new PhotonCamera(VisionConstants.kcameraName);

  ShuffleboardTab Tab = Shuffleboard.getTab("Field");
  // Tab.add("CameraPose", null).withwidget;

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {

    apriltags = new HashMap<Integer, Pose2d>();
    apriltags.put(1, new Pose2d(new Translation2d(15.514, 1.072), new Rotation2d(Math.PI)));
    apriltags.put(2, new Pose2d(new Translation2d(15.514, 2.748), new Rotation2d(Math.PI)));
    apriltags.put(3, new Pose2d(new Translation2d(15.514, 4.424), new Rotation2d(Math.PI)));
    apriltags.put(4, new Pose2d(new Translation2d(16.179, 6.75), new Rotation2d(Math.PI)));
    apriltags.put(5, new Pose2d(new Translation2d(0.362, 6.75), new Rotation2d(0)));
    apriltags.put(6, new Pose2d(new Translation2d(1.027, 4.424), new Rotation2d(0)));
    apriltags.put(7, new Pose2d(new Translation2d(1.027, 2.748), new Rotation2d(0)));
    apriltags.put(8, new Pose2d(new Translation2d(1.027, 1.072), new Rotation2d(0)));

    m_blueScoring = new ArrayList<Pose2d>();
    m_redScoring = new ArrayList<Pose2d>();

    m_blueScoring.add(apriltags.get(6));
    m_blueScoring.add(apriltags.get(7));
    m_blueScoring.add(apriltags.get(8));

    m_redScoring.add(apriltags.get(1));
    m_redScoring.add(apriltags.get(2));
    m_redScoring.add(apriltags.get(3));

    // robotPose = Shuffleboard.getTab("Field")
    // .add("CameraPose", new Pose2d())
    // .withWidget(BuiltInWidgets.kField)
    // .getEntry();

    // Get the default instance of NetworkTables that was created automatically
    // when your program starts
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    // NetworkTable table = inst.getTable("photonvision/Microsoft_LifeCam_HD-3000");

    // Get the entries within that table that correspond to the X and Y values
    // for some operation in your program.
    // xEntry = table.getEntry("X");
    // yEntry = table.getEntry("Y");
    // targetPose = table.getEntry("targetPose");
  }

  // double x = 0;
  // double y = 0;

  public Pose2d getCameraAbsolute() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        //System.out.println(target.getBestCameraToTarget().getX());
        //System.out.println(target.getBestCameraToTarget().getY());
        //System.out.println(target.getBestCameraToTarget().getZ());
        int apriltagid = target.getFiducialId();

        apriltagPose = apriltags.get(apriltagid);
        if (apriltagPose == null) {
          return null;
        }
        cameraTrans = new Transform2d(
            new Translation2d(target.getBestCameraToTarget().getX(), target.getBestCameraToTarget().getY()),
            (target.getBestCameraToTarget().getRotation().toRotation2d()));
        // robotPose.setValue(absolutePose);
        absolutePose = apriltagPose.plus(cameraTrans);

        return new Pose2d(absolutePose.getTranslation(), apriltagPose.getRotation().minus(cameraTrans.getRotation()));
      }

    }
    return null;

  }

  public Pose2d getRobotAbsolute() {
    Pose2d camPose = getCameraAbsolute();
    if (camPose != null) {
      return camPose.transformBy(VisionConstants.cameraToRobot);
    }
    return null;
  }

  public Pose2d getDestination(Pose2d ourPos, String direction) {
    ///////////////////////////////////////////////////////////////////
    //System.out.println(direction);

    Pose2d target = bestApriltag(ourPos);

    Pose2d destination;
    if (direction == "left") {
      destination = target.transformBy(VisionConstants.leftTrans);
      System.out.println(destination.getX() + "x" + destination.getY());
      return destination;

    } else if (direction == "right") {
      destination = target.transformBy(VisionConstants.rightTrans);
      System.out.println(destination.getX() + "x" + destination.getY());
      return destination;

    } else {
      destination = target.transformBy(VisionConstants.centerTrans);
      System.out.println(destination.getX() + "x" + destination.getY());
      return destination;

    }

  }

  public Pose2d bestApriltag(Pose2d ourPose) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return ourPose.nearest(m_blueScoring);
    } else {
      return ourPose.nearest(m_redScoring);
    }
  }

  @Override
  public void periodic() {
    loopCount += 1;
    if (loopCount % 5 == 1) {
      Pose2d position = getRobotAbsolute();
      if (position != null) {
        m_lastPosition = position;
      }

      // if (m_updateDrive && position != null) {
      //updateRobotPos()
      //}

    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
