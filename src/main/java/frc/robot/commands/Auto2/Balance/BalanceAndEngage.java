package frc.robot.commands.Auto2.Balance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;


public class BalanceAndEngage extends CommandBase{
    public double stopPosition = 0.0;
    public double m_max_power = 0.7;
    public int m_count = 0;
    PIDController pid = new PIDController(0,0,0);

    // Compute how much the angle is changing
    double last_angle = 0.0;

    SwerveDrive swerveDrive;
    public BalanceAndEngage(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

    }
    
    @Override
    public void initialize() {
   //     stopPosition = swerveDrive.getPosition();
    }

    @Override
    public void execute() {
        
        
    }

    @Override
    public boolean isFinished() {
        return false;
        
    }

    @Override
    public void end(boolean interrupted) {
        
        
    }

    
}
