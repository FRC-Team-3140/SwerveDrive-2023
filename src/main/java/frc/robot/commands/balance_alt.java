package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class balance_alt extends CommandBase{
    //Welecome to Balance Alt...
    SwerveDrive m_drive;
    RelativeEncoder encoder;
    double startPosition = Double.MAX_VALUE;
    public balance_alt(SwerveDrive swerveDrive) {
        m_drive = swerveDrive;
        encoder = m_drive.getBRModule().getEncoder();
        addRequirements(swerveDrive);
    }
     
    boolean hasTilted = false;
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_drive.setLocked(false);
        startPosition = Double.MAX_VALUE;
        hasTilted = false;
    }
    @Override
    public void execute() {
        
        m_drive.setChassisSpeeds(.075, 0, 0);
        if(Math.abs(SwerveDrive.m_gyro.getPitch()) > 2 && !hasTilted){
            hasTilted = true;
            startPosition = encoder.getPosition();
        }
    }
    double traveled;
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub = m_drive.getBRModule().getEncoder();
        traveled = encoder.getPosition() - startPosition;
        System.out.println("What is my mans doing" + ", " + SwerveDrive.m_gyro.getPitch() + ", " + traveled + ", " + encoder.getPositionConversionFactor());
        return Math.abs(encoder.getPosition() - startPosition) > 17 && hasTilted;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_drive.setChassisSpeeds(0, 0, 0);
        m_drive.setLocked(true);
    }
}
