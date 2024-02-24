package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

/**
 * Represents a swerve module, which is a combination of a drive motor and a turn motor.
 * This class provides methods to control the angle and speed of the module.
 */
public class SwerveModule extends SubsystemBase { // TODO: This probably does not need to be a subsystem

    private static final double kTurnD = .02;
    private static final double kDriveP = 0.09;
    private static double kTurnP = .008;

    private static double kDriveSetpointTolerance = .5;
    private static double kTurnSetpointTolerance = .2;
    private static double kTurnVelocityTolerance = .1;

    private static final double kDriveFeedforwardKs = 0.084706;
    private static final double kDriveFeedforwardKv = 2.4433;
    private static final double kDriveFeedforwardKa = 0.10133;

    private String moduleID;
    private CANSparkMax turnMotor;
    private CANSparkMax driveMotor;
    private PIDController turnPID;
    private PIDController drivePID;
    private AbsoluteEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

    private final NetworkTable moduleTable; // NetworkTable for the module

    /**
     * Represents a feedforward controller for a simple motor.
     * 
     * The SimpleMotorFeedforward class calculates the feedforward term for a motor controller
     * based on the motor's static gain (kS), velocity gain (kV), and acceleration gain (kA).
     * It is commonly used to compensate for the dynamics of a motor and achieve better control
     * performance.
     */
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(kDriveFeedforwardKs, kDriveFeedforwardKv, kDriveFeedforwardKa);

    // Conversion Factor for the motor encoder output to wheel output
    // (Circumference / Gear Ratio) * Inches to meters conversion

    boolean encoderSets = false;

    SwerveModuleState thisState;

    /**
     * Represents a swerve module on a robot's drivetrain.
     * Each swerve module consists of a drive motor, a turn motor, and associated sensors and controllers.
     *
     * @param moduleID    The unique identifier for the swerve module.
     * @param analogID    The analog input ID for the turn encoder.
     * @param driveMotorID    The CAN ID for the drive motor.
     * @param turnMotorID    The CAN ID for the turn motor.
     * @param baseAngle    The base angle offset for the turn encoder.
     */
    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle) {
        this.moduleID = moduleID;

        moduleTable = NetworkTableInstance.getDefault().getTable(moduleID);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(false);
        driveMotor.setSmartCurrentLimit(25);
        driveMotor.burnFlash();

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(false);
        turnMotor.setSmartCurrentLimit(15);
        turnMotor.burnFlash();

        turnEncoder = new AbsoluteEncoder(analogID);
        turnEncoder.setPositionOffset(baseAngle);
        driveEncoder = driveMotor.getEncoder();

        turnPID = new PIDController(kTurnP, 0, 0);
        // we don't use I or D
        turnPID.enableContinuousInput(0, 360);
        turnPID.setTolerance(kTurnSetpointTolerance, kTurnVelocityTolerance);

        // determined from a SYSID scan
        drivePID = new PIDController(kDriveP, 0, kTurnD);
        drivePID.setTolerance(kDriveSetpointTolerance);

    }

    /**
        * This method is called periodically to update the state of the SwerveModule.
        * If the encoder sets have not been initialized, it sets the velocity and position conversion factors for the drive encoder.
        */
    @Override
    public void periodic() {

        moduleTable.getEntry("Speed Setpoint").setDouble(drivePID.getSetpoint());
        moduleTable.getEntry("Speed Actual").setDouble(driveEncoder.getVelocity());
        moduleTable.getEntry("Speed Error").setDouble(drivePID.getSetpoint() - driveEncoder.getVelocity());

        moduleTable.getEntry("Angle Setpoint").setDouble(turnPID.getSetpoint());
        moduleTable.getEntry("Angle Actual").setDouble(turnEncoder.getPos());
        moduleTable.getEntry("Angle Error").setDouble(getAngleError());

        if (!encoderSets) {
            driveEncoder.setVelocityConversionFactor(Constants.encoderRotationToMeters);
            driveEncoder.setPositionConversionFactor(42 * Constants.encoderRotationToMeters);
        }
    }

    /**
     * Sets the states of the Swerve module.
     * 
     * @param state The desired state of the Swerve module.
     * @param locked A boolean indicating whether the module is locked or not.
     */
    public void setStates(SwerveModuleState state, boolean locked) {
        thisState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getPos()));
        setAngle(thisState.angle.getDegrees());
        setDriveSpeed(thisState.speedMetersPerSecond);
    }


    /**
     * Sets the desired angle for the swerve module.
     * 
     * @param angle the desired angle in degrees
     */
    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getPos()));
    }

    /**
     * Sets the drive speed of the swerve module.
     * 
     * @param velocity the desired velocity of the drive motor
     */
    public void setDriveSpeed(double velocity) {
        // TODO: drivePID added too much instability
        drivePID.setSetpoint(velocity); // TODO remove?

        double voltagePercent = driveFeedforward.calculate(velocity);

        // Power should be adjusted based on the current angle error in the turn motor
        double turnError = getAngleError();

        // If the turn error is greater than 90 degrees, the drive motor should be 0
        if (Math.abs(turnError) > 90) {
            voltagePercent = 0;
        }
        
        voltagePercent = Math.cos( Math.toRadians(turnError) ) * voltagePercent;

        driveMotor.setVoltage(voltagePercent);

        // drivePID.calculate(driveEncoder.getVelocity())); 
    }

    /**
     * Calculates the error between the target angle and the current angle.
     * The error is normalized to be within the range of -180 to 180 degrees.
     *
     * @return the angle error in degrees
     */
    private double getAngleError() {
        double error = turnPID.getPositionError();
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    /**
     * Sets the turn speed of the swerve module.
     * 
     * @param speed the desired turn speed, ranging from -1.0 to 1.0
     */
    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, Constants.maxTurnSpeed), -Constants.maxTurnSpeed);
        turnMotor.set(speed);
    }

    /**
     * Retrieves the position of the swerve module.
     *
     * @return The swerve module position.
     */
    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getPos(); // Get the angle from the turn encoder.
        double distance = driveEncoder.getPosition(); // Get the distance from the drive encoder.
        double angleRadians = angle * Math.PI / 180; // Convert the angle to radians.
        return new SwerveModulePosition(distance, new Rotation2d(angleRadians));
    }

    /**
     * Returns the drive encoder for the swerve module.
     *
     * @return the drive encoder
     */
    private RelativeEncoder getDriveEncoder() { // TODO: Remove?
        return this.driveEncoder;
    }

    /**
     * Returns the CANcoder used for measuring the turn angle of the swerve module.
     *
     * @return the CANcoder used for measuring the turn angle
     */
    private CANcoder getTurnEncoder() { // TODO: Remove?
        return this.turnEncoder;
    }

    /**
     * Returns the current state of the Swerve module.
     *
     * @return the current state of the Swerve module
     */
    public SwerveModuleState getState() { 
        return thisState;
    }
}
