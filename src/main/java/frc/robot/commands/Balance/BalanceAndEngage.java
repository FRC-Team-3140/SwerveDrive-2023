package frc.robot.commands.Balance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Comms3140;
import frc.robot.subsystems.SwerveOdometer;
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
        Comms3140.getInstance().sendDoubleTelemetry("Balance","be_state", 0);
        this.swerveDrive = swerveDrive;
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_navx_table = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

    }

    @Override
    public void initialize() {
        Comms3140.getInstance().sendDoubleTelemetry("Balance","be_state", 1);

        // TODO: switch to odometer if it is tested and values are correct
        SwerveOdometer.getInstance().reset(); // Reset should zero the position of the robot at this location
        stopPosition = SwerveOdometer.getInstance().getX(); // Robot should now just move in the x dimension relative to the current position

        //stopPosition = swerveDrive.getPosition();
    }

    @Override
    public void execute() {
        Comms3140.getInstance().sendDoubleTelemetry("Balance","be_state", 2);

        // TODO: switch to odometer if it is tested and values are correct
        double position = SwerveOdometer.getInstance().getX();

        //double position = swerveDrive.getPosition();

        double pitch = m_navx_table.getEntry("navx_filtered_pitch").getDouble(0.0);

        m_count++;
        Comms3140.getInstance().sendDoubleTelemetry("Balance","be_m_count", m_count);
        if (m_count % 100 == 0) {
            if (pitch > 2.0)
                stopPosition = position - 0.15;
            else if (pitch < -2.0)
                stopPosition = position + 0.15;
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
        Comms3140.getInstance().sendDoubleTelemetry("Balance","be_state", 3);

    }

}
