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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase implements Constants {

    // Zero : 0.697578
    // One : 0.701239
    // Two: 0.467096
    // Three : 0.207867
    public String moduleID;
    public int analogID;
    public int pwmID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public CANSparkMax turnMotor;
    public CANSparkMax driveMotor;
    public PIDController turnPID;
    public PIDController drivePID;
    public AbsoluteEncoder turnEncoder;
    public RelativeEncoder driveEncoder;

    public double botMass = 24.4;

    public double P = .008;

    public double driveSetpointTolerance = .5;
    public double turnSetpointTolerance = .2;
    public double turnVelocityTolerance = .1;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706, 2.4433, 0.10133);

    // private TrapezoidProfile.Constraints constraints = new
    // TrapezoidProfile.Constraints(maxDriveSpeed, maxAcceleration);
    // private State initialState = new TrapezoidProfile.State(0, 0);
    // private TrapezoidProfile trapezoidProfile;

    // Conversion Factor for the motor encoder output to wheel output
    // (Circumference / Gear Ratio) * Inches to meters conversion

    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle) {
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;
        this.analogID = analogID;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(false);
        driveMotor.setSmartCurrentLimit(60);
        driveMotor.burnFlash();

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(false);
        turnMotor.setSmartCurrentLimit(40);
        turnMotor.burnFlash();

        turnEncoder = new AbsoluteEncoder(analogID);
        turnEncoder.setPositionOffset(baseAngle);
        driveEncoder = driveMotor.getEncoder();

        turnPID = new PIDController(P, 0, 0);
        // we don't use I or D
        turnPID.enableContinuousInput(0, 360);
        turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);
        // determined from a SYSID scan
        drivePID = new PIDController(0.09, 0, .02);
        drivePID.setTolerance(driveSetpointTolerance);

    }

    boolean encoderSets = false;

    // runs while the bot is running
    @Override
    public void periodic() {
        if (!encoderSets) {
            driveEncoder.setVelocityConversionFactor(encoderRotationToMeters);
            driveEncoder.setPositionConversionFactor(42 * encoderRotationToMeters);

        }

    }

    SwerveModuleState thisState;

    public void setStates(SwerveModuleState state, boolean locked) {
        thisState = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getPos()));
        // thisState.speedMetersPerSecond *=
        // thisState.angle.minus(Rotation2d.fromDegrees(turnEncoder.getPos())).getCos();
        setAngle(thisState.angle.getDegrees());
        setDriveSpeed(thisState.speedMetersPerSecond);
        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);
    }

    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getPos()));
    }

    public void setDriveSpeed(double velocity) {
        drivePID.setSetpoint(velocity);
        driveMotor.setVoltage(driveFeedforward.calculate(velocity));
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(velocity);
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed")
                .setDouble(driveEncoder.getVelocity());
        // drivePID.calculate(driveEncoder.getVelocity())); ///drivePID added too much
        // instability
    }

    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, maxTurnSpeed), -maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getPos();
        double distance = driveEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d((angle/* + 270*/) * Math.PI / 180));
        // ^ 3*3.14/2+3.14 * angle/ 180)); Erm... Idk what's going on here || This math
        // is the equivalent of adding 270 degs in Radians
    }

    public RelativeEncoder getDriveEncoder() {
        return this.driveEncoder;
    }

    public CANcoder getTurnEncoder() {
        return this.turnEncoder;
    }

    public String getModuleID() {
        return this.moduleID;
    }

    public SwerveModuleState getState() {
        return thisState;// new SwerveModuleState(driveEncoder.getVelocity(), new
                         // Rotation2d(turnEncoder.getPosRadians()));
    }
}
