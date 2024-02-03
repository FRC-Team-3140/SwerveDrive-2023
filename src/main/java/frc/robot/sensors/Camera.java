// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrive;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private PhotonCamera april = null;

  private PhotonCamera notes = null;

  private boolean connected = false;
  private int connectionAttempts = 0;

  // The heartbeat is a value in the Photonvision Networktable that continually
  // changes.
  private double heartbeat = 0;
  private double previousResult = -1;

  private SwerveDrive swerveDrive;
  private Pose2d currentSwervePose2d;

  private Camera(SwerveDrive swerve, int PhotonvisionConnectionAttempts) {
    while (connected == false && connectionAttempts > PhotonvisionConnectionAttempts) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        connectionAttempts++;
        System.out.println("Checking for PhotonVision connection in 5 seconds.");
        Timer.delay(5);
      }
    }

    if (connected == true) {
      aprilGetInstance();
      notesGetInstance();
    }

    swerveDrive = swerve;
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(RobotContainer.m_robotDrive, 5);
    }
    return instance;
  }

  private PhotonCamera aprilGetInstance() {
    if (april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  private PhotonCamera notesGetInstance() {
    if (notes == null) {
      notes = new PhotonCamera(inst, "notes");
    }
    return april;
  }

  private boolean testConnection() {
    // Gets new result from april camera and test if it's equal to the previous
    // result
    if (heartbeat == previousResult) {
      connected = false;
      heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
    } else { 
      connected = true;
    }

    return connected;
  }

  private void attemptToReconnect() {
    System.out.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");

    while (connected == false) {
      if (testConnection() == true) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in 5 seconds.");
        Timer.delay(5);
      }
    }
    System.out.println(heartbeat);
  }

  public boolean getStatus() {
    return connected;
  }

  @Override
  public void periodic() {

    previousResult = heartbeat;
    heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);

    if (testConnection() == false) {
      attemptToReconnect();
    }

    Timer.delay(1);
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
      return 0.8 * april.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    } else {
      return 0;
    }
  }

  public double getApriltagDistY() {
    if (april.getLatestResult().hasTargets()) {
      // Negative because it's distance towards Apriltag
      return 0.8 * april.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
    } else {
      return 0;
    }
  }

  public double getDegToApriltag() {
    // Calculate the difference in yaw angles
    if (april.getLatestResult().hasTargets()) {
      double targetYaw = getApriltagYaw();
      double requiredTurnDegrees = targetYaw - currentSwervePose2d.getRotation().getDegrees();

      // Ensure the angle is within the range of -180 to 180 degrees
      requiredTurnDegrees = (requiredTurnDegrees + 180) % 360 - 180;

      return requiredTurnDegrees;
    } else {
      return 0;
    }
  }

  public Pose2d getApriltagPose2d() {
    // Make sure this returns the proper pose. I'm writing this without code
    // checking...
    return new Pose2d(new Translation2d(getApriltagDistX(), getApriltagDistY()), new Rotation2d(getDegToApriltag()));
  }

  public double getNoteDistance() {
    // If this function returns a 0, that means there is not any detected targets

    // Need to wait until cameras are on Final Robot because calculation requires
    // specific
    // measurements to the camera.

    notes.getLatestResult().getBestTarget();
    PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);

    return 0.0;
  }

  public Pose2d getUpdatedPose() {
    currentSwervePose2d = swerveDrive.getPose();
    double currentX = currentSwervePose2d.getX();
    double currentY = currentSwervePose2d.getY();
    double newX = getApriltagDistX();
    double newY = getApriltagDistY();
    double degs = getDegToApriltag();

    Pose2d newPose = new Pose2d((currentX + newX), (currentY + newY), new Rotation2d((degs * (Math.PI / 180)))); // Rotation2d
                                                                                                                 // yearns
                                                                                                                 // for
                                                                                                                 // Radians
                                                                                                                 // so
                                                                                                                 // conversion
                                                                                                                 // is
                                                                                                                 // neccesary.

    System.out.println(
        "Pose:\nX: " + newPose.getX() + "\nY: " + newPose.getY() + "\nDeg: " + newPose.getRotation().getDegrees());

    return newPose;
  }

  /*
   * // Probably won't need methods like this that move the Robot manually while
   * using Pathplanner
   * 
   * public void turnToFaceApriltagID(int ID, boolean verbose) {
   * // Probably should return # of degrees to face apriltag for Pose2d instead of
   * Actually turning the robot
   * // so Pathplanner can handle turning.
   * 
   * 
   * if (verbose == true) {
   * System.out.println("----------------\nID: " + getApriltagID() + "\nYaw:" +
   * getApriltagYaw() + "\nPitch: "
   * + getApriltagPitch() + "\n----------------");
   * }
   * 
   * if (getApriltagYaw() > 0) {
   * // turn left
   * } else if (getApriltagYaw() < 0) {
   * // turn right
   * } else {
   * // set speed 0
   * }
   * }
   * 
   * public void strafeToFaceApriltagID(int ID, boolean verbose) {
   * if (verbose == true) {
   * System.out.println("----------------\nID: " + getApriltagID() + "\nYaw:" +
   * getApriltagYaw() + "\nPitch: "
   * + getApriltagPitch() + "\n----------------");
   * }
   * 
   * if (getApriltagYaw() > 0) {
   * // slide left
   * } else if (getApriltagID() < 0) {
   * // slide right
   * } else {
   * // set speed 0
   * }
   * }
   * 
   * 
   * public Transform2d moveToApriltag() {
   * 
   * }
   */
}