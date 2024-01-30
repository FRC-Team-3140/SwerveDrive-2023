// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private PhotonCamera april;
  private PhotonCamera notes;

  private boolean connected = false;

  private Camera() {
    april = new PhotonCamera(inst, "april");
    notes = new PhotonCamera(inst, "notes");

    while (connected == false) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in 5 seconds.");
        Timer.delay(5);
      }
    }
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera();
    }
    return instance;
  }

  @Override
  public void periodic() {}

  public int getAprilTagID() {
    // If this function returns a 0, that means there is not any detected targets

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  public double getAprilTagYaw() {
    // If this function returns a 0, that means there is not any detected targets

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getYaw();
    } else {
      return -1;
    }
  }

  public double getAprilTagPitch() {
    // If this function returns a 0, that means there is not any detected targets

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getPitch();
    } else {
      return -1;
    }
  }

  public double getApriltagDistX() {
    if (april.getLatestResult().hasTargets()) {
      // Negative because it's distance towards Apriltag
      return -0.8 * april.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    } else {
      return 0;
    }
  }

  public double getApriltagDistY() {
    if (april.getLatestResult().hasTargets()) {
      // Negative because it's distance towards Apriltag
      return -0.8 * april.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
    } else {
      return 0;
    }
  }

  public void turnToFaceAprilTagID(int ID, boolean verbose) {
    // Probably should return # of degrees to face apriltag for Pose2d instead of Actually turning the robot
    // so Pathplanner can handle turning.


    if (verbose == true) {
      System.out.println("----------------\nID: " + getAprilTagID() + "\nYaw:" + getAprilTagYaw() + "\nPitch: "
          + getAprilTagPitch() + "\n----------------");
    }

    if (getAprilTagYaw() > 0) {
      // turn left
    } else if (getAprilTagID() < 0) {
      // turn right
    } else {
      // set speed 0
    }
  }

  public void strafeToFaceAprilTagID(int ID, boolean verbose) {
    if (verbose == true) {
      System.out.println("----------------\nID: " + getAprilTagID() + "\nYaw:" + getAprilTagYaw() + "\nPitch: "
          + getAprilTagPitch() + "\n----------------");
    }

    if (getAprilTagYaw() > 0) {
      // slide left
    } else if (getAprilTagID() < 0) {
      // slide right
    } else {
      // set speed 0
    }
  }

  /*
   * public Transform2d moveToAprilTag() {
   * 
   * }
   */

  public double getNoteDistance() {
    // If this function returns a 0, that means there is not any detected targets

    // Need to wait until on Final Robot because calculation requires specific
    // measurements

    notes.getLatestResult().getBestTarget();
    PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);

    return 0.0;
  }

  public Pose2d getUpdatedPose() {
    double x = getApriltagDistX(); 
    double y = getApriltagDistY(); 

    System.out.println("X: " + x + "Y: " + y);
    
    Pose2d newPose = new Pose2d(x, y, new Rotation2d(0)); 
    
    return newPose;
  }
}