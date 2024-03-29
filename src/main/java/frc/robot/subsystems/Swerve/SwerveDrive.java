package frc.robot.subsystems.Swerve;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Comms3140;

public class SwerveDrive extends SubsystemBase {
    // Cameras
    public static PhotonCamera aprilTagCamera = new PhotonCamera("Apriltag-Cam");
    public static PhotonCamera colorCam = new PhotonCamera("Wrist-Cam");

    private final SwerveModule m_swerveModule_fr = new SwerveModule("fr", 10, 1, 2, 131, 86);
    private final SwerveModule m_swerveModule_fl = new SwerveModule("fl", 13, 3, 4, 308, 171);
    private final SwerveModule m_swerveModule_br = new SwerveModule("br", 11, 5, 6, 140, 183);
    private final SwerveModule m_swerveModule_bl = new SwerveModule("bl", 12, 7, 8, 295, 249);
    public static boolean headless = true;
    // Locations for the swerve drive modules relative to the robot center.
    // 0.27305, 0.3302
    Translation2d m_frontLeftLocation = new Translation2d(.3302, .2794);
    Translation2d m_frontRightLocation = new Translation2d(.3302, -.2794);
    Translation2d m_backLeftLocation = new Translation2d(-.3302, .2794);
    Translation2d m_backRightLocation = new Translation2d(-.3302, -.2794);

    public static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    NetworkTable DataNAVX = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("DataNAVX");
    double m_last_pitch = 0.0;

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
            m_backLeftLocation, m_backRightLocation);

    final private NetworkTable swerve_table;
    final private NetworkTableEntry x_velocity;
    final private NetworkTableEntry y_velocity;
    final private NetworkTableEntry r_velocity;
    final private NetworkTableEntry drive_enabled;
    final private NetworkTableEntry position; 

    private boolean locked = false;

    // Get Angle Filtered
    double accel_angle = 0.0;
    double angle_filtered = 0.0;
    LinearFilter angle_filter = LinearFilter.singlePoleIIR(0.2, 0.02);; // for get velocity function
    LinearFilter angle_filter2 = LinearFilter.singlePoleIIR(0.1, 0.02); // for pitch
    LinearFilter angle_filter3 = LinearFilter.singlePoleIIR(.1, .02); // for roll

    private Accelerometer accelerometer;

    // Get Velocity
    private static double avgVelocity = 0.0;
    private static double FRVelocity = 0.0;
    private static double FLVelocity = 0.0;
    private static double BRVelocity = 0.0;
    private static double BLVelocity = 0.0;

    // Get Position
    private static double FRPosition = 0.0;

    public SwerveDrive() {

        m_gyro.reset();
        swerve_table = NetworkTableInstance.getDefault().getTable("swerve_chassis");
        x_velocity = swerve_table.getEntry("x_velocity");
        x_velocity.setDouble(0.0);
        y_velocity = swerve_table.getEntry("y_velocity");
        y_velocity.setDouble(0.0);
        r_velocity = swerve_table.getEntry("r_velocity");
        r_velocity.setDouble(0.0);
        drive_enabled = swerve_table.getEntry("enabled");
        drive_enabled.setBoolean(true);
        position = swerve_table.getEntry("Position Thing");
        position.setDouble(0.0);

        accelerometer = new BuiltInAccelerometer();
    }

    // This method will run repetitively while the robot is running
    @Override
    public void periodic() {
        updateNavX();

        double dx = x_velocity.getDouble(0);
        double dy = y_velocity.getDouble(0);
        double rads_per_sec = r_velocity.getDouble(0);

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(dx, dy, rads_per_sec);
        ChassisSpeeds botSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(dx, dy, rads_per_sec, m_gyro.getRotation2d());
        SwerveModuleState[] states = headless ? m_kinematics.toSwerveModuleStates(botSpeeds)
                : m_kinematics.toSwerveModuleStates(fieldSpeeds);

        // int angleBL = (int)
        // NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BL").getInteger(0);
        // int angleBR =
        // (int)NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_BR").getInteger(0);
        // int angleFL = (int)
        // NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FL").getInteger(0);
        // int angleFR = (int)
        // NetworkTableInstance.getDefault().getTable("Angle").getEntry("Angle_FR").getInteger(0);
        NetworkTableInstance.getDefault().getTable("Angle").getEntry("Pos_BL")
                .setDouble(-(m_swerveModule_bl.getEncoder().getPosition()));
        NetworkTableInstance.getDefault().getTable("Angle").getEntry("Pos_BR")
                .setDouble(m_swerveModule_br.getEncoder().getPosition());
        NetworkTableInstance.getDefault().getTable("Angle").getEntry("Pos_FL")
                .setDouble(-(m_swerveModule_fl.getEncoder().getPosition()));
        NetworkTableInstance.getDefault().getTable("Angle").getEntry("Pos_FR")
                .setDouble(m_swerveModule_fr.getEncoder().getPosition());

        NetworkTableInstance.getDefault().getTable("swerve_chassis").getEntry("Position Thing").setDouble(getPosition());
        
        if (drive_enabled.getBoolean(true)) {
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveModule.maxDriveSpeed);
            m_swerveModule_fl.setStates(states[0], locked);
            m_swerveModule_fr.setStates(states[1], locked);
            m_swerveModule_bl.setStates(states[2], locked);
            m_swerveModule_br.setStates(states[3], locked);

            m_swerveModule_bl.periodic();
            m_swerveModule_br.periodic();
            m_swerveModule_fl.periodic();
            m_swerveModule_fr.periodic();
        }

        // angle_filtered = angle_filter.calculate(accel_angle);
        // accel_angle = -Math.atan2(accelerometer.getX(), accelerometer.getY()) * 180 /
        // Math.PI;
        // if (angle_filtered > 15)
        // angle_filtered = 15;
        // if (angle_filtered < -15)
        // angle_filtered = -15;

        avgVelocity = (m_swerveModule_bl.driveSparkMax.getEncoder().getVelocity()
                + m_swerveModule_br.driveSparkMax.getEncoder().getVelocity()
                + m_swerveModule_fr.driveSparkMax.getEncoder().getVelocity()
                + m_swerveModule_fl.driveSparkMax.getEncoder().getVelocity()) / 4;
        FRVelocity = m_swerveModule_fr.driveSparkMax.getEncoder().getVelocity();
        FLVelocity = m_swerveModule_fl.driveSparkMax.getEncoder().getVelocity();
        BRVelocity = m_swerveModule_br.driveSparkMax.getEncoder().getVelocity();
        BLVelocity = m_swerveModule_bl.driveSparkMax.getEncoder().getVelocity();
        FRPosition = m_swerveModule_fr.driveSparkMax.getEncoder().getPosition();

        Comms3140.getInstance().sendDoubleTelemetry("SwerveDrive", "Position", getPosition());
    }

    public void updateNavX() {
        // System.out.println("Update NAVX");
        DataNAVX.getEntry("navx_yaw").setNumber(m_gyro.getYaw());
        DataNAVX.getEntry("navx_pitch").setNumber(m_gyro.getPitch());
        DataNAVX.getEntry("navx_roll").setNumber(m_gyro.getRoll());
        DataNAVX.getEntry("navx_compass").setNumber(m_gyro.getCompassHeading());

        DataNAVX.getEntry("navx_x_pos").setNumber(m_gyro.getDisplacementX());

        DataNAVX.getEntry("navx_gyrox").setNumber(m_gyro.getRawGyroX());
        DataNAVX.getEntry("navx_gyroy").setNumber(m_gyro.getRawGyroY());
        DataNAVX.getEntry("navx_gyroz").setNumber(m_gyro.getRawGyroZ());

        double filtered_pitch = angle_filter2.calculate(m_gyro.getPitch());
        DataNAVX.getEntry("navx_filtered_pitch").setNumber(filtered_pitch);
        double filtered_roll = angle_filter3.calculate(m_gyro.getRoll());

        DataNAVX.getEntry("navx_filtered_roll").setNumber(filtered_roll);

        double pitch_change = Math.abs(50.0 * (filtered_pitch - m_last_pitch)); // Estimate the pitch change per second
        m_last_pitch = filtered_pitch;

        DataNAVX.getEntry("navx_pitch_change").setNumber(pitch_change);

    }

    public static void zeroNavx() {
        m_gyro.calibrate();
        m_gyro.reset();
    }

    // ----------------- Setter Methods ----------------- \\
    public void setChassisSpeeds(double x_vel, double y_vel, double r_vel) {

        // If val < deadband then set it to 0, else ignore
        x_velocity.setDouble(x_vel);
        y_velocity.setDouble(y_vel);
        r_velocity.setDouble(r_vel);

    }

    public void arcadeDrive(double x_vel, double r_vel) {
        setChassisSpeeds(x_vel, 0, r_vel);

    }

    public void setLocked(boolean locked) {
        this.locked = locked;
    }

    // cone=0 cube=1
    public void changePipeline() {
        if (colorCam.getPipelineIndex() == 0) {
            colorCam.setPipelineIndex(1);
        } else if (colorCam.getPipelineIndex() == 1) {
            colorCam.setPipelineIndex(0);
        }
    }

    public void setIdleModes(IdleMode bMode) {
        m_swerveModule_bl.setIdleMode(bMode);
        m_swerveModule_br.setIdleMode(bMode);
        m_swerveModule_fl.setIdleMode(bMode);
        m_swerveModule_fr.setIdleMode(bMode);
    }

    // ----------------- Getter Methods ----------------- \\

    public double getPosition() {
        RelativeEncoder[] encoders = getEncoders();
        double[] signs = new double[4];
        double[] encoderPositions = new double[4];
        double smallestDistTraveled = 999;
        int encoderIndex = 0;
        
        for (int i = 0; i < encoders.length; i++){
            signs[i] = Math.signum(encoders[i].getPosition());
            encoderPositions[i] = Math.abs(encoders[i].getPosition());
            if (encoderPositions[i] < smallestDistTraveled){
                smallestDistTraveled = encoderPositions[i];
                encoderIndex = i;
            }
        }   

        for (int i = 0; i < encoders.length; i++){
            smallestDistTraveled = signs[encoderIndex] * encoderPositions[encoderIndex];
        }

        return smallestDistTraveled;
        //return (encoders[0].getPosition());
    }


    public void homeWheels(){
        m_swerveModule_fl.setAngle(0, false);
        m_swerveModule_fr.setAngle(0, false);
        m_swerveModule_bl.setAngle(0, false);
        m_swerveModule_br.setAngle(0, false);
    }

    public boolean getLocked() {
        return locked;
    }

    public double getAngleFiltered() {
        return angle_filtered;
    }

    public static double getAvgVelocity() {
        return avgVelocity;
    }

    public static double getFRVelocity() {
        return FRVelocity;
    }

    public static double getFLVelocity() {
        return FLVelocity;
    }

    public static double getBRVelocity() {
        return BRVelocity;
    }

    public static double getBLVelocity() {
        return BLVelocity;
    }

    public static double getFRPosition() {
        return FRPosition;
    }

    public RelativeEncoder[] getEncoders() {
        return new RelativeEncoder[] {
                m_swerveModule_bl.getEncoder(),
                m_swerveModule_br.getEncoder(),
                m_swerveModule_fl.getEncoder(),
                m_swerveModule_fr.getEncoder() };
    }

    public PhotonTrackedTarget getTarget() {
        return aprilTagCamera.getLatestResult().getBestTarget();
    }

    public PhotonTrackedTarget getColor() {
        return colorCam.getLatestResult().getBestTarget();
    }

    public boolean hasTarget() {
        return aprilTagCamera.getLatestResult().hasTargets();
    }

    public boolean hasColor() {
        return colorCam.getLatestResult().hasTargets();
    }

    public double getDistanceToConeFromCamera(){
        if(colorCam.getPipelineIndex() == 1){
            colorCam.setPipelineIndex(0);
        }
        if(hasColor()){
            return -1;
        }
        return 3.27975 * Math.pow(getColor().getArea(),-0.402222) - 0.520129;
    }
    public SwerveModule getBRModule() {
        return m_swerveModule_br;
    }

    public Accelerometer getAccelerometer() {
        return accelerometer;
    }

    // Added to support odometer
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // Added to support odometer
    public Rotation2d getYaw() {
        return new Rotation2d(Math.PI*m_gyro.getYaw()/180.0);
    }

    // Added to support odometer
    public SwerveModulePosition[] getModulePositions() {
        // m_frontLeftLocation,
        // m_frontRightLocation,
        // m_backLeftLocation,
        // m_backRightLocation
        SwerveModulePosition pose_fl = m_swerveModule_fl.getSwerveModulePosition();
        SwerveModulePosition pose_fr = m_swerveModule_fr.getSwerveModulePosition();
        SwerveModulePosition pose_bl = m_swerveModule_bl.getSwerveModulePosition();
        SwerveModulePosition pose_br = m_swerveModule_br.getSwerveModulePosition();
        
        SwerveModulePosition[] positions = {pose_fl, pose_fr, pose_bl, pose_br};
        return positions;
    }

}

// rotatation/ gear ratio