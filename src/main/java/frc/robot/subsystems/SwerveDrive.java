package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 13, 3, 4, 285.0 + 45, -45);
    private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 10, 1, 2, 315, 45);
    private final SwerveModule m_swerveModule_br = new SwerveModule("br", 12, 7, 8, 30.0 + 135, 45);
    private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 11, 5, 6, 7.0 + 225, -45);

    // Locations for the swerve drive modules relative to the robot center.
    //0.27305, 0.3302
    Translation2d m_frontLeftLocation = new Translation2d(0.3302, .27305);
    Translation2d m_frontRightLocation = new Translation2d(0.3302, -.27305);
    Translation2d m_backLeftLocation = new Translation2d(-.3302, .27305);
    Translation2d m_backRightLocation = new Translation2d(-.3302, -0.27305);
    AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    final private NetworkTable swerve_table;
    final private NetworkTableEntry x_velocity;
    final private NetworkTableEntry y_velocity;
    final private NetworkTableEntry r_velocity;
    final private NetworkTableEntry drive_enabled;

    private boolean locked = false;

    public SwerveDrive() {
        swerve_table = NetworkTableInstance.getDefault().getTable("swerve_chassis");
        x_velocity = swerve_table.getEntry("x_velocity");
        x_velocity.setDouble(0.0);
        y_velocity = swerve_table.getEntry("y_velocity");
        y_velocity.setDouble(0.0);
        r_velocity = swerve_table.getEntry("r_velocity");
        r_velocity.setDouble(0.0);
        drive_enabled = swerve_table.getEntry("enabled");
        drive_enabled.setBoolean(true);
    }

    // This method will run repetitively while the robot is running
    @Override
    public void periodic() {
        // TODO Auto-generated method stub        
        double dx = x_velocity.getDouble(0);
        double dy = y_velocity.getDouble(0);
        double rads_per_sec = r_velocity.getDouble(0);

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(dx, dy, rads_per_sec);
        m_gyro.zeroYaw();
        Rotation2d angle = new Rotation2d(m_gyro.getYaw());
        ChassisSpeeds botSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(dx, dy, rads_per_sec, angle);
        System.out.println(angle);
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldSpeeds);
        
        int angleBL = (int) NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BL").getInteger(0);
        int angleBR = (int)NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BR").getInteger(0);
        int angleFL = (int) NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FL").getInteger(0);
        int angleFR = (int) NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FR").getInteger(0);
        //NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BL").setDouble(states[2].angle.getDegrees());
        //NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BR").setDouble(states[3].angle.getDegrees());
        //NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FL").setDouble(states[0].angle.getDegrees());
        //NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FR").setDouble(states[1].angle.getDegrees());
        if (drive_enabled.getBoolean(true)) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.maxDriveSpeed);
            m_swerveModule_fl.setStates(states[0], locked, angleFL);
            m_swerveModule_fr.setStates(states[1], locked, angleFR);
            m_swerveModule_bl.setStates(states[2], locked, angleBL);
            m_swerveModule_br.setStates(states[3], locked, angleBR);

            m_swerveModule_bl.periodic();
            m_swerveModule_br.periodic();
            m_swerveModule_fl.periodic();
            m_swerveModule_fr.periodic();
        }
    }

    // ----------------- Setter Methods ----------------- \\
    public void setChassisSpeeds(double x_vel, double y_vel, double r_vel) {
        //If val < deadband then set it to 0, else ignore
        double deadband = .1;
        x_velocity.setDouble(Math.abs(x_vel) < deadband ? 0 : x_vel);
        y_velocity.setDouble(Math.abs(y_vel) < deadband ? 0 : y_vel);
        r_velocity.setDouble(Math.abs(r_vel) < deadband ? 0 : r_vel);
    }
    
    public void setLocked(boolean locked) {
        this.locked = locked;
    }


    // ----------------- Getter Methods ----------------- \\

    public boolean getLocked() {
        return locked;
    }
    
}
