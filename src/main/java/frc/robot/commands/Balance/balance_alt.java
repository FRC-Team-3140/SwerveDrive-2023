//ARE WE EVEN USING THIS???

package frc.robot.commands.Balance;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class balance_alt extends CommandBase{
    //Welecome to Balance Alt...
    SwerveDrive m_drive;
    RelativeEncoder[] encoder;
    double startPosition[] = new double[4];
    public balance_alt(SwerveDrive swerveDrive) {
        m_drive = swerveDrive;
        encoder = m_drive.getEncoders();
        addRequirements(swerveDrive);
    }
    double angle = 0;
    double dist = .0927;
    boolean hasTilted = false;
    @Override
    public void initialize() {
        angle = NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Angle (Alt)").getDouble(0.0);
        dist = NetworkTableInstance.getDefault().getTable("Extra Support").getEntry("Distance (Alt)").getDouble(0.0);
        SwerveDrive.m_gyro.zeroYaw();
        // TODO Auto-generated method stub
        m_drive.setLocked(false);
        for(int i = 0; i < 4; i++) startPosition[i] = encoder[i].getPosition();
        hasTilted = false;
    }
    @Override
    public void execute() {
        if(!hasTilted){
            m_drive.setChassisSpeeds(.5, 0, 0);
        }else{
            m_drive.setChassisSpeeds(.3, 0, 0);
        }
        
        if(Math.abs(SwerveDrive.m_gyro.getRoll()) > angle && !hasTilted){
            hasTilted = true;
            for(int i = 0; i< 4; i++){
                startPosition[i] = encoder[i].getPosition();
            }   
        }
    }
    double traveled;
    boolean[] hasTraveledToEnd = new boolean[]{false, false, false, false};
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub = m_drive.getBRModule().getEncoder();
        for(int i = 0; i < 4; i++){
            traveled = encoder[i].getPosition() - startPosition[i];
            hasTraveledToEnd[i] = Math.abs(encoder[i].getPosition() - startPosition[i]) > dist;
            //System.out.println("What is my mans doing" + ", " + SwerveDrive.m_gyro.getPitch() + ", " + traveled + ", " + encoder[i].getPosition());
        }
        System.out.println(traveled);
        return  hasTilted && hasTraveledToEnd[0] && hasTraveledToEnd[1] &&  hasTraveledToEnd[2] && hasTraveledToEnd[3] || traveled >= 3;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_drive.setChassisSpeeds(0, 0, 0);
    }
}
