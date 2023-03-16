package frc.robot.commands.Auto2.Balance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class BalanceAndEngage extends CommandBase{
    public double stopPosition = 0.0;
    public double m_max_power = 0.7;
    public int m_count = 0;
    private NetworkTable m_navx_table;
    PIDController pid = new PIDController(0,0,0);

    // Compute how much the angle is changing
    double last_angle = 0.0;

    SwerveDrive swerveDrive;
    public BalanceAndEngage(SwerveDrive swerveDrive){
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_navx_table = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

    }
    
    @Override
    public void initialize() {
        stopPosition = swerveDrive.getPosition();
    }

    @Override
    public void execute() {
        double position = swerveDrive.getPosition();
        double roll = m_navx_table.getEntry("navx_filtered_roll").getDouble(0.0);
        //SwerveDrive.m_gyro.getRoll();
        //.getEntry("navx_filtered_roll").getDouble(0.0);
        m_count += 1;
        if(m_count % 100 == 0){
            if(roll > 2.0) stopPosition -= 0.1;
            else if(roll < -2.0) stopPosition+= 0.1;
        }
        
        double power = pid.calculate(position - stopPosition);

        power = Math.min(Math.max(power,-m_max_power), m_max_power); // limit the speed

        System.out.printf("set_pos:%.3f  pos:%.3f  angle:%.3f  power:%.3f\n", stopPosition, position, roll, power);

        swerveDrive.setChassisSpeeds(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
        
    }

    @Override
    public void end(boolean interrupted) {
        
        
    }

    
}
