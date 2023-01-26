// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTableEntry targetPose;
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  /** Creates a new ExampleSubsystem. */
  public VisionSubsystem() {

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
  






  

  public void GetData() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
      
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        System.out.println();
        System.out.println("ID: " + target.getFiducialId());
        System.out.println("Best Target: "+ target.getBestCameraToTarget());
        System.out.println("Ambiguity: " + target.getPoseAmbiguity());
      }
    }
    else {
      System.out.println("No target found");
    }
    //System.out.println(result.getTimestampSeconds());

    // xEntry.setDouble(x);
    // yEntry.setDouble(y);
    // x += 0.05;
    // y += 1.0;
    // Using the entry objects, set the value to a double that is constantly
    // increasing. The keys are actually "/datatable/X" and "/datatable/Y".
    // If they don't already exist, the key/value pair is added.
    // This method will be called once per scheduler run
    //Number[] numArray = {};
    //numArray = targetPose.getNumberArray(numArray);
    //System.out.println(Arrays.toString(numArray));
    // x, y, z?, z-angle?, _, _, _
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
