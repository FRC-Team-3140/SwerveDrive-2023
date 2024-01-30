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
  public void periodic() {
    if (connected == false) {
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
  }

  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  public double getApriltagYaw() {
    // If this function returns a 0, that means there is not any detected targets

    if (april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getYaw();
    } else {
      return -1;
    }
  }

  public double getApriltagPitch() {
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

  public double getDegToApriltag() {
    // Calculate the difference in yaw angles
    double targetYaw = targetPose.getRotation().getDegrees();
    double requiredTurnDegrees = targetYaw - currentHeading;

    // Ensure the angle is within the range of -180 to 180 degrees
    requiredTurnDegrees = (requiredTurnDegrees + 180) % 360 - 180;

    return requiredTurnDegrees;
  }


  public Pose2d getApriltagPose2d() {
    // Make sure this returns the proper pose. I'm writing this without code checking...
    return new Pose2d(new Translation2d(getApriltagDistX(), getApriltagDistY()), getDegToApriltag());
  }

  public double getNoteDistance() {
    // If this function returns a 0, that means there is not any detected targets

    // Need to wait until cameras are on Final Robot because calculation requires specific
    // measurements to the camera.

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

  /*
  // Probably won't need methods like this that move the Robot manually while using Pathplanner

  public void turnToFaceApriltagID(int ID, boolean verbose) {
    // Probably should return # of degrees to face apriltag for Pose2d instead of Actually turning the robot
    // so Pathplanner can handle turning.


    if (verbose == true) {
      System.out.println("----------------\nID: " + getApriltagID() + "\nYaw:" + getApriltagYaw() + "\nPitch: "
          + getApriltagPitch() + "\n----------------");
    }

    if (getApriltagYaw() > 0) {
      // turn left
    } else if (getApriltagYaw() < 0) {
      // turn right
    } else {
      // set speed 0
    }
  }

  public void strafeToFaceApriltagID(int ID, boolean verbose) {
    if (verbose == true) {
      System.out.println("----------------\nID: " + getApriltagID() + "\nYaw:" + getApriltagYaw() + "\nPitch: "
          + getApriltagPitch() + "\n----------------");
    }

    if (getApriltagYaw() > 0) {
      // slide left
    } else if (getApriltagID() < 0) {
      // slide right
    } else {
      // set speed 0
    }
  }

  
  public Transform2d moveToApriltag() {
    
  }
  */
}