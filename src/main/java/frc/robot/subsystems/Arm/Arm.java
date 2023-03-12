// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Functions;
import frc.robot.RobotContainer;
//impirt edu.wpi.first.math.controller.Bang;

public class Arm extends SubsystemBase {
  // Min and Max angle || NOT SET YET DON'T USE!!!
  private final double minAngleArm = 0;
  private final double maxAngleArm = .581149;
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 209;

  // Wrist Absolute Encoder
  public final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);

  // Arm and wrist motor and pid controller definitions
  public final CANSparkMax ArmSparkMax;
  private PIDController armPidController = new PIDController(.035, 0, 0);

  public final CANSparkMax wristSparkMax;
  private PIDController wristPidController = new PIDController(0, 0, 0);
  // private final DigitalInput limitSwitch = RobotContainer.getLimitSwitch();

  // 0.473388 max wrist encoder output(top) 0.892239 (bottom)
  // Yay!

  // Set Angles
  private double ArmAngle;
  private double WristAngle;
  double xCoord;
  double yCoord;
  double armLength = Units.feetToMeters(31 / 12);
  double wristLength = Units.feetToMeters(23.75 / 12);
  DigitalInput limitSwitchArmUpper = RobotContainer.getLimitSwitchUper();
  DigitalInput limitSwitchLower = RobotContainer.getLimitSwitchLower();
  DigitalInput limitSwitchWrist = RobotContainer.getlimitSwitchWrist();

  /** Creates a new Arm. */
  public Arm(int ArmID, int WristID) {
    ArmSparkMax = new CANSparkMax(ArmID, MotorType.kBrushless);
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);

    ArmSparkMax.restoreFactoryDefaults();
    ArmSparkMax.setSmartCurrentLimit(60);
    ArmSparkMax.setIdleMode(IdleMode.kBrake);
    ArmSparkMax.burnFlash();
    
    wristSparkMax.restoreFactoryDefaults();
    wristSparkMax.setSmartCurrentLimit(20);
    wristSparkMax.setIdleMode(IdleMode.kBrake);
    wristSparkMax.burnFlash();

    wristSparkMax.getEncoder().setPosition(0.0);
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
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("WristAngle").setDouble(getWristAngle());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("ArmAngle").setDouble(getArmAngle());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Wrist Voltage")
        .setDouble(wristSparkMax.getBusVoltage());
    NetworkTableInstance.getDefault().getTable("Arm").getEntry("Arm Voltage").setDouble(ArmSparkMax.getBusVoltage());

  }

  private void wristAngleFromCoords(double xCoord, double yCoord) {
    double bruh = Math
        .acos((sqr(xCoord) + sqr(yCoord) - sqr(armLength) - sqr(wristLength)) / (2 * armLength * wristLength));
    WristAngle = bruh + 36;
    ArmAngle = Math.atan2(wristLength * Math.sin(bruh) * yCoord + (armLength + wristLength * Math.cos(bruh)) * xCoord,
        (armLength + wristLength * Math.cos(bruh)) * yCoord - wristLength * Math.sin(bruh) * xCoord);
  }

  private double sqr(double valToBeSquared) {
    return valToBeSquared * valToBeSquared;
  }

  // Setter Methods
  public void setArmAngle(double aAngle) {
    ArmAngle = Math.min(Math.max(aAngle, minAngleArm), maxAngleArm);
  }


  // mechanical was here
  public void setWristAngle(double angle) {
    WristAngle = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
  }

  
  public void setWristVoltage(double wristVoltage) {
    System.out.println(getWristAngle());
    if(!limitSwitchWrist.get()){
      wristSparkMax.setVoltage(Math.min(wristVoltage,0));
    }else if(getWristAngle() < 2){
      wristSparkMax.setVoltage(Math.max(wristVoltage, 0));
    }else{
      wristSparkMax.setVoltage(wristVoltage);
    }
  }

  public void setArmVoltage(double armVoltage) {
    if(!limitSwitchArmUpper.get()){
      ArmSparkMax.setVoltage(Math.min(armVoltage, 0));
    }else if(!limitSwitchLower.get()){
      ArmSparkMax.setVoltage(Math.max(armVoltage, 0));
    }else{
      ArmSparkMax.setVoltage(armVoltage);
    }
  }

  // Getter Methods
  public RelativeEncoder getArmEncoder() {
    return ArmSparkMax.getEncoder();
  }

  public DutyCycleEncoder getWristEncoder() {
    return wristEncoder;
  }

  public double getArmAngle() {
    return getArmEncoder().getPosition();
  }

  public double getWristAngle() {
    double a = ((getWristEncoder().getAbsolutePosition() - .892239) % 1) * 360;
    return a < 0 ? a + 360 : a;
  }

  // public boolean atSetpoint(){
  // return wristPidController.atSetpoint() && armPidController.atSetpoint();
  // }

}