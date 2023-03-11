package frc.robot.commands.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class SwerveController extends CommandBase{

    SwerveDrive m_drive;

    Supplier<Double> m_x_axis;
    Supplier<Double> m_y_axis;
    Supplier<Double> m_r_axis;

    public SwerveController(SwerveDrive swerve_drive, Supplier<Double> x_axis, Supplier<Double> y_axis, Supplier<Double> r_axis){
        assert(swerve_drive != null);

        m_drive = swerve_drive;

        m_x_axis = x_axis;
        m_y_axis = y_axis;
        m_r_axis = r_axis;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub

        super.execute();
        double deadband = .1;
        double dx = m_x_axis.get();
        double dy = m_y_axis.get();
        double dr = m_r_axis.get();

        m_drive.setChassisSpeeds(Math.abs(dx) < deadband ? 0 : dx, Math.abs(dy) < deadband ? 0 : dy, Math.abs(dr) < deadband ? 0: dr);
        
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    
}
