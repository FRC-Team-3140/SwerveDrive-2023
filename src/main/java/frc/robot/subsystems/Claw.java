

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  //Compressor & Solenoid
  private final DoubleSolenoid m_DoubleSolenoid;
  private static boolean closed = false;

  /** Creates a new Arm. */
  public Claw(int PneumaticsModuleID1, int forwardCh, int reverseCh) {
    m_DoubleSolenoid = new DoubleSolenoid(PneumaticsModuleID1, PneumaticsModuleType.CTREPCM, forwardCh, reverseCh);
  }

  @Override
  public void periodic() {
  //Methods

  }
  //Setter method

  public void clawOpen() {
    m_DoubleSolenoid.set(Value.kForward);
  }
  public void clawClosed(){
    m_DoubleSolenoid.set(Value.kReverse);
  }

  public void toggleClaw() {
    System.out.println(closed);
    if(!closed){clawClosed();}else{clawOpen();}
    closed = !closed;
  }
}

