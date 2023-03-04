// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.BangBangController; 

public class Arm extends SubsystemBase {
  //Min and Max angle || NOT SET YET DON'T USE!!!
  private final double minAngleArm = 0;
  private final double maxAngleArm = 0;
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 0;

  //Wrist Absolute Encoder
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(1);

  //Arm and wrist motor and pid controller definitions
  private final CANSparkMax ArmSparkMax;
  private final PIDController armPidController = new PIDController(0, 0, 0);

  private final CANSparkMax wristSparkMax; 
  private final PIDController wristPidController = new PIDController(0, 0, 0);

  //Set Angles
  private double ArmAngle;
  private double WristAngle;

  /** Creates a new Arm. */
  public Arm(int ArmID, int WristID, int compressorID, int PneumaticsModuleID, int forwardCh, int reverseCh) {
    ArmSparkMax = new CANSparkMax(ArmID, MotorType.kBrushless);
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);

    ArmSparkMax.setSmartCurrentLimit(35);
    ArmSparkMax.setIdleMode(IdleMode.kBrake);
    ArmSparkMax.burnFlash();

    wristSparkMax.setSmartCurrentLimit(35);
    wristSparkMax.setIdleMode(IdleMode.kBrake);
    wristSparkMax.burnFlash();

    wristSparkMax.getEncoder().setPosition(0.0);
  }

  @Override
  public void periodic() {
  //Update the arm and wrist setpoints 
    double ArmSetpoint = 2;
    double WristSetpoint = 2;

    // This method will be called once per scheduler run
    wristPidController.calculate(getWristAngle(),WristSetpoint);
    armPidController.calculate(getArmAngle(),ArmSetpoint);
  }

  //Setter Methods
  public void setArmAngle(double aAngle){
    aAngle = Math.min(Math.max(aAngle, minAngleArm),maxAngleArm);
    ArmAngle = aAngle;
  }

  public void setWristAngle(double Wangle){
    Wangle = Math.min(Math.max(Wangle, minAngleWrist), maxAngleWrist);
    WristAngle = Wangle;
  }

  //Getter Methods
  public RelativeEncoder getArmEncoder(){
    return ArmSparkMax.getEncoder();
  }

  public DutyCycleEncoder getWristEncoder(){
    return wristEncoder;
  }

  public double getArmAngle(){
    double armAngle = getArmEncoder().getPosition();
    return armAngle;
  }

  public double getWristAngle(){
    double wristAngle = getWristEncoder().getAbsolutePosition();
    return wristAngle; 
  }


}
