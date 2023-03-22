package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve.SwerveDrive;

/*
 * Track the location of the robot using odometry.
 */
public class SwerveOdometer extends SubsystemBase {

    private static SwerveOdometer s_global_instance = null;

    /**
     * Get a global instace of the swerve object.
     */
    public static SwerveOdometer getInstance() {
        assert s_global_instance != null;
        return s_global_instance;
    }

    private final SwerveDriveOdometry odometer;

    private final SwerveDrive m_drive_train;

    private double m_last_reset;

    /**
     * This function should only be called once.
     * 
     * @param drivetrain
     */
    public SwerveOdometer(SwerveDrive drivetrain) {
        assert s_global_instance == null;

        // initialize the singleton
        s_global_instance = this;

        m_drive_train = drivetrain;
        odometer = new SwerveDriveOdometry(m_drive_train.getKinematics(), m_drive_train.getYaw(),
                m_drive_train.getModulePositions(), new Pose2d());
    }

    @Override
    public void periodic() {
        super.periodic();

        double current_time = 1000.0 * System.currentTimeMillis() - m_last_reset;

        odometer.update(m_drive_train.getYaw(), m_drive_train.getModulePositions());

        Comms3140 comms = Comms3140.getInstance();
        comms.sendDoubleTelemetry("Odometer","timestamp", current_time);
        comms.sendDoubleTelemetry("Odometer","x_pos", getX());
        comms.sendDoubleTelemetry("Odometer","y_pos", getY());
        comms.sendDoubleTelemetry("Odometer","yaw", getYaw());
    }

    // Reset the origin 
    public void reset() {
        m_last_reset = 1000.0 * System.currentTimeMillis();

        odometer.resetPosition(m_drive_train.getYaw(), m_drive_train.getModulePositions(), new Pose2d());
    }

    public Pose2d getPose2d() {
        return odometer.getPoseMeters();
    }

    public double getX() {
        return odometer.getPoseMeters().getX();
    }

    public double getY() {
        return odometer.getPoseMeters().getY();
    }

    public double getYaw() {
        return odometer.getPoseMeters().getRotation().getDegrees();
    }

}
