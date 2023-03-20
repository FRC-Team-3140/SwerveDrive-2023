package frc.robot.commands.Balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BalanceAndEngage extends CommandBase {
    public double stopPosition = 0.0;
    public double m_max_speed = 0.4;
    public int m_count = 0;
    private NetworkTable m_navx_table;
    private double balanceP;
    private double balanceD;
    private double power;
    PIDController pid = new PIDController(balanceP, 0, balanceD);

    // Compute how much the angle is changing
    double last_angle = 0.0;

    SwerveDrive swerveDrive;

    public BalanceAndEngage(SwerveDrive swerveDrive) {
        addRequirements(swerveDrive);
        this.swerveDrive = swerveDrive;
        
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
        double pitch = m_navx_table.getEntry("navx_filtered_pitch").getDouble(0.0);

        m_count++;
        if (m_count % 50 == 0) {
            if (pitch > 2.0)
                stopPosition = position - 0.1;
            else if (pitch < -2.0)
                stopPosition = position + 0.1;
        }

        power = pid.calculate(position - stopPosition);

        power = Math.min(Math.max(power, -m_max_speed), m_max_speed); // limit the speed

        System.out.printf("set_pos:%.3f  pos:%.3f  angle:%.3f  power:%.3f\n", stopPosition, position, pitch, power);

        swerveDrive.setChassisSpeeds(power, 0, 0);

        balanceP = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance P").getDouble(0.0);
        balanceD = NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance D").getDouble(0.0);

        if (pid.getP() != balanceP || pid.getD() != balanceD) {
            pid.setP(balanceP);
            pid.setD(balanceD);
        }

        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance Position").setDouble(position);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance stopPosition").setDouble(stopPosition);
        NetworkTableInstance.getDefault().getTable("Balance").getEntry("Balance Power").setDouble(power);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {

    }

}
