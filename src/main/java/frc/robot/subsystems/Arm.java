// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.RobotContainer;
//impirt edu.wpi.first.math.controller.Bang;

public class Arm extends SubsystemBase {
  //Min and Max angle || NOT SET YET DON'T USE!!!
  private final double minAngleArm = Double.MIN_VALUE;
  private final double maxAngleArm = Double.MAX_VALUE;
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 0;

  //Wrist Absolute Encoder
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);

  //Arm and wrist motor and pid controller definitions
  private final CANSparkMax ArmSparkMax;
  private PIDController armPidController = new PIDController(0, 0, 0);

  private final CANSparkMax wristSparkMax; 
  private PIDController wristPidController = new PIDController(0, 0, 0);
  private final DigitalInput limitSwitch = RobotContainer.getLimitSwitch();
  

  //Set Angles
  private double ArmAngle; 
  private double WristAngle;
  double xCoord;
  double yCoord;
  double armLength = Units.feetToMeters(31/12);
  double wristLength = Units.feetToMeters(23.75/12);
  /** Creates a new Arm. */
  public Arm(int ArmID, int WristID) {
    ArmSparkMax = new CANSparkMax(ArmID, MotorType.kBrushless);
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);
    
     
    ArmSparkMax.setSmartCurrentLimit(80);
    ArmSparkMax.setIdleMode(IdleMode.kBrake);
    ArmSparkMax.burnFlash();

    wristSparkMax.setSmartCurrentLimit(20);
    wristSparkMax.setIdleMode(IdleMode.kBrake);
    wristSparkMax.burnFlash();
    
    wristSparkMax.getEncoder().setPosition(0.0);
  }
  double lastP = .0075;
  double lastI = .025;
  @Override
  public void periodic() {
  //Update the arm and wrist setpoints 
    double ArmSetpoint = ArmAngle;
    double WristSetpoint = 2;
    ArmAngle = NetworkTableInstance.getDefault().getTable("Arm").getEntry("Arm Angle").getDouble(0);    
    // This method will be called once per scheduler run
    if(NetworkTableInstance.getDefault().getTable("PID").getEntry("P").getDouble(0) != lastP || NetworkTableInstance.getDefault().getTable("PID").getEntry("I").getDouble(0) != lastI){
      armPidController = new PIDController(NetworkTableInstance.getDefault().getTable("PID").getEntry("P").getDouble(0), NetworkTableInstance.getDefault().getTable("PID").getEntry("I").getDouble(0), 0);
    }
     

    wristPidController.calculate(getWristAngle(),WristSetpoint);
    double armSpeed = armPidController.calculate(getArmAngle(),ArmSetpoint);
   
    ArmSparkMax.set(armSpeed);
    
  }
  private void wristAngleFromCoords(double xCoord, double yCoord){
    WristAngle = Math.acos(
    (sqr(xCoord) + sqr(yCoord)- sqr(armLength) - sqr(wristLength))/(2*armLength*wristLength) ) - ArmAngle;
    Math.atan2(wristLength * Math.sin(WristAngle) * yCoord + (armLength + wristLength * Math.cos(wristLength)) * xCoord,
     (armLength + wristLength * Math.cos(wristLength)) * yCoord - wristLength * Math.sin(WristAngle) * xCoord);
    
  }
  private double sqr(double valToBeSquared){
    return valToBeSquared * valToBeSquared;
  }

  
  //Setter Methods
  public void setArmAngle(double aAngle){
    //if(!limitSwitch.get()){
      ArmAngle = Math.min(Math.max(aAngle, minAngleArm),maxAngleArm);
   // }
  }

  public void setWristAngle(double angle){

    WristAngle = Math.min(Math.max(angle, minAngleWrist), maxAngleWrist);
  }
  
  public void setWristVoltage(double wristVoltage){
     wristSparkMax.setVoltage(wristVoltage);
  }
  
  public void setArmVoltage(double armVoltage){
   // if(!limitSwitch.get()){
     ArmSparkMax.setVoltage(armVoltage);
   // }
  }

  //Getter Methods
  public RelativeEncoder getArmEncoder(){
    return ArmSparkMax.getEncoder();
  }

  public DutyCycleEncoder getWristEncoder(){
    return wristEncoder;
  }

  public double getArmAngle(){
    return getArmEncoder().getPosition();
  }

  public double getWristAngle(){
    return getWristEncoder().getAbsolutePosition();
  }

  public boolean atSetpoint(){
    return wristPidController.atSetpoint() && armPidController.atSetpoint();
  }
}
