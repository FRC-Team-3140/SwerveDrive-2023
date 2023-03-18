// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
//impirt edu.wpi.first.math.controller.Bang;

public class Arm extends SubsystemBase {
  // Min and Max angle || NOT SET YET DON'T USE!!!
  private final double minAngleArm = 0;
  private final double maxAngleArm = .581149;
  
  // Arm and wrist motor and pid controller definitions
  public final CANSparkMax ArmSparkMax;
  

  // private final DigitalInput limitSwitch = RobotContainer.getLimitSwitch();

  // 0.473388 max wrist encoder output(top) 0.892239 (bottom)
  // Yay!

  double xCoord;
  double yCoord;
  double armLength = Units.feetToMeters(31 / 12);
  DigitalInput limitSwitchArmUpper = RobotContainer.getLimitSwitchUper();
  DigitalInput limitSwitchLower = RobotContainer.getLimitSwitchLower();
 

  /** Creates a new Arm. */
  public Arm(int ArmID) {
    ArmSparkMax = new CANSparkMax(ArmID, MotorType.kBrushless);
   
    ArmSparkMax.restoreFactoryDefaults();
    ArmSparkMax.setSmartCurrentLimit(60);
    ArmSparkMax.setIdleMode(IdleMode.kBrake);
    ArmSparkMax.burnFlash();
  }

  // .035 P only arm
  //
  double lastP = .0075;
  double lastI = .025;

  @Override
  public void periodic() {
    // Update the arm and wrist setpoints
    /*
     * double ArmSetpoint = ArmAngle;
     * double WristSetpoint = 2;
     * ArmAngle =
     * NetworkTableInstance.getDefault().getTable("Arm").getEntry("Arm Angle").
     * getDouble(0);
     * // This method will be called once per scheduler run
     * if(NetworkTableInstance.getDefault().getTable("PID").getEntry("P").getDouble(
     * 0) != lastP ||
     * NetworkTableInstance.getDefault().getTable("PID").getEntry("I").getDouble(0)
     * != lastI){
     * armPidController = new
     * PIDController(NetworkTableInstance.getDefault().getTable("PID").getEntry("P")
     * .getDouble(0),
     * NetworkTableInstance.getDefault().getTable("PID").getEntry("I").getDouble(0),
     * 0);
     * }
     * NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngle").
     * setDouble(getWristAngle());
     */
    // wristPidController.calculate(getWristAngle(), ArmAngle);
    // double armSpeed = armPidController.calculate(getArmAngle(),ArmAngle);
    // ArmSparkMax.set(armSpeed);
    // wristSparkMax.setVoltage(
    // Math.abs(ArmAngle - getWristAngle()) > 3 ?
    // Math.copySign(2, ArmAngle - getWristAngle()) : 0
    // );
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("ArmAngle").setDouble(getArmAngle());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Arm Voltage").setDouble(ArmSparkMax.getBusVoltage());

  }

  // private void wristAngleFromCoords(double xCoord, double yCoord) {
  //   double bruh = Math
  //       .acos((sqr(xCoord) + sqr(yCoord) - sqr(armLength) - sqr(wristLength)) / (2 * armLength * wristLength));
  //   WristAngle = bruh + 36;
  //   ArmAngle = Math.atan2(wristLength * Math.sin(bruh) * yCoord + (armLength + wristLength * Math.cos(bruh)) * xCoord,
  //       (armLength + wristLength * Math.cos(bruh)) * yCoord - wristLength * Math.sin(bruh) * xCoord);
  // }
  //
  // private double sqr(double valToBeSquared) {
  //   return valToBeSquared * valToBeSquared;
  // }

  // Setter Methods
  public void setArmAngle(double aAngle) {
    Math.min(Math.max(aAngle, minAngleArm), maxAngleArm);
  }


  // mechanical was here
 
  

  public void setArmVoltage(double armVoltage) {
    if(!limitSwitchArmUpper.get()){
      ArmSparkMax.setVoltage(Math.min(armVoltage, 0));
    }else if(!limitSwitchLower.get()){
      ArmSparkMax.setVoltage(Math.max(armVoltage, 0));
    }else{
      ArmSparkMax.setVoltage(armVoltage);
    }
  }

  public void setArmSpeed(double armSpeed) {
    if(!limitSwitchArmUpper.get()){
      ArmSparkMax.set(Math.min(armSpeed, 0));
    }else if(!limitSwitchLower.get()){
      ArmSparkMax.set(Math.max(armSpeed, 0));
    }else{
      ArmSparkMax.set(armSpeed);
    }
  }

  // Getter Methods
  public RelativeEncoder getArmEncoder() {
    return ArmSparkMax.getEncoder();
  }


  public double getArmAngle() {
    return (getArmEncoder().getPosition() * 360) / (296) ;
  }


  

  // public boolean atSetpoint(){
  // return wristPidController.atSetpoint() && armPidController.atSetpoint();
  // }

}
