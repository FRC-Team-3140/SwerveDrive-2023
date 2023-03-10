package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class TargetAlign extends CommandBase {
    SwerveDrive m_Drive;
    public TargetAlign(SwerveDrive swerveDrive){
        m_Drive = swerveDrive;
        addRequirements(swerveDrive);
    }
    double setpoint;
    double deadband = .05;

    @Override
    public void initialize() {
        m_Drive.setLocked(false);
        if(m_Drive.getTarget() != null)
        setpoint = m_Drive.getTarget().getBestCameraToTarget().getY() ;
    }

    @Override
    public void execute() {
        
        if(m_Drive.hasTarget()){
            
        Transform3d relativePosition = m_Drive.getTarget().getBestCameraToTarget();
        double DIST = relativePosition.getY();
        //driving_pid.setIntegratorRange(-turnIntegratorRange, turnIntegratorRange);
        //hi
        //
        m_Drive.setChassisSpeeds(0,Math.copySign(.1, relativePosition.getY()) , 0);
        double drivingVelocity = Math.pow(DIST,2) * 5;
        //m_Drive.setChassisSpeeds(0, Math.copySign(drivingVelocity, relativePosition.getY()),0);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method sub
        if(m_Drive.getTarget() != null){ 
            if(Math.abs(m_Drive.getTarget().getBestCameraToTarget().getY()) < deadband){
                m_Drive.setLocked(true);
                return true;
            }
            return false;
        }
        System.out.println("Lost target");
        return true;
    }

    @Override
    public void end(boolean interrupt) {
        m_Drive.setChassisSpeeds(0, 0, 0);
    }
}
    
