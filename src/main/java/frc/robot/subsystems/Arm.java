// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  //Min and Max angle || NOT SET YET DON'T USE!!!
  private final double minAngleArm = 0;
  private final double maxAngleArm = 0;
  private final double minAngleWrist = 0;
  private final double maxAngleWrist = 0;

  //Wrist Absolute Encoder
  private final Encoder wristEncoder = new Encoder(null, null);

  //Arm and wrist motor and pid controller definitions
  private final CANSparkMax ArmSparkMax;
  private final PIDController armPidController = new PIDController(0, 0, 0);

  private final CANSparkMax wristSparkMax; 
  private final PIDController wrisPidController = new PIDController(0, 0, 0);

  //Compressor & Solenoid
  private final Compressor m_Compressor;
  private final DoubleSolenoid m_DoubleSolenoid;

  //Set Angles
  private double ArmAngle;
  private double WristAngle;

  /** Creates a new Arm. */
  public Arm(int ArmID, int WristID, int compressorID, int PneumaticsModuleID, int forwardCh, int reverseCh) {
    ArmSparkMax = new CANSparkMax(ArmID, MotorType.kBrushless);
    wristSparkMax = new CANSparkMax(WristID, MotorType.kBrushless);
    m_Compressor = new Compressor(compressorID, PneumaticsModuleType.CTREPCM);
    m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleID, PneumaticsModuleType.CTREPCM, forwardCh, reverseCh);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(WristAngle > minAngleWrist && WristAngle < maxAngleWrist){
      armPidController.calculate(ArmAngle);
    }
    if(ArmAngle > minAngleArm && ArmAngle < maxAngleArm){
      armPidController.calculate(WristAngle);
    }
  }

  //Methods
  public void openClaw(){
    m_DoubleSolenoid.set(Value.kForward);
  }

  public void closeClaw(){
    m_DoubleSolenoid.set(Value.kReverse);
  }

  //Setter Methods
  public void setArmAngle(double Aangle){
    ArmAngle = Aangle;
  }

  public void setWristAngle(double Wangle){
    WristAngle = Wangle;
  }

  //Getter Methods
  public RelativeEncoder getArmEncoder(){
    return ArmSparkMax.getEncoder();
  }

  public Encoder getWristEncoder(){
    return wristEncoder;
  }
}
