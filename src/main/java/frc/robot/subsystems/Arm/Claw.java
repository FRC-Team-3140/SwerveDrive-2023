
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;/////////////////////////////////////////////////////////////////////////////////////                                                
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;
// import frc.robot.subsystems.Comms3140;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {
  // Compressor & Solenoid
  private final DoubleSolenoid m_DoubleSolenoid;
  private static boolean closed = false;
  public CANSparkMax motor1;
  public CANSparkMax motor2;
  //private final Ultrasonic ultraDistSensor = new Ultrasonic(6, 5);
  // private RobotContainer m_RobotContainer =  RobotContainer.getInstance();
  // private final XboxController controller1;
  private double rumbleVal = 0;
  private double range = 0;
  private double percentageOfMaxRange = 0;   
  /** Creates a new Arm. */
  public Claw(int PneumaticsModuleID1, int forwardCh, int reverseCh) {
    // controller1 = RobotContainer.getInstance().getController1();
    m_DoubleSolenoid = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, reverseCh, forwardCh);
    motor1 = new CANSparkMax(15, MotorType.kBrushless);
    motor2 = new CANSparkMax(14, MotorType.kBrushless);

    }

  

  @Override
  public void periodic() {
   // System.out.println(ultraDistSensor.getRangeMM());
    // Methods
   // m_RobotContainer = RobotContainer.getInstance();
    //range = ultraDistSensor.getRangeMM();
    //Comms3140.getInstance().sendDoubleTelemetry("Claw", "Claw Distance", ultraDistSensor.getRangeMM());

    // if the range is greater than 3.5ft then ignore it
    // 1066.8 = 3.5ft in mm

    if(range > 1066.8){
      range = -1;
    }

    percentageOfMaxRange = range / 1066.8;

    if(range != -1){
      rumbleVal *= percentageOfMaxRange;
    } else {
      rumbleVal = 0;
    }

    RobotContainer.getInstance().getController2().setRumble(RumbleType.kBothRumble, rumbleVal);
  }
  // Setter method

  // public RunCommand openClaw(){
  // new RuncCom
  // }

  public void clawClosed() {
    m_DoubleSolenoid.set(Value.kForward);
    motor1.setVoltage(6.1);
    motor2.setVoltage(-6);

  }

  public void clawOpen() {
    m_DoubleSolenoid.set(Value.kReverse);
    motor1.setVoltage(-6.1);
    motor2.setVoltage(6);


  }
  public double getAmps(){
    return motor1.getOutputCurrent();
  }
  //motor 1 is sucky so it needs more voltage
  public void clawOff(){
    motor1.setVoltage(0);
    motor2.setVoltage(0);
    
  }
  public void holdCube(){
    motor1.setVoltage(-1.2);
    motor2.setVoltage(1);
  }

  public void toggleClaw() {
    System.out.println(closed);
    if (!closed) {
      clawClosed();
    } else {
      clawOpen();
    }
    closed = !closed;
  }
}
