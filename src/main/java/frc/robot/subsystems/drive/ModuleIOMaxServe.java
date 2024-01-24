package frc.robot.subsystems.drive;
   
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import static frc.robot.Constants.SwerveModuleConstants.*;

public class ModuleIOMaxServe implements ModuleIO {

  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredModuleState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs and configures the driving and turning motors, encoder, PIDs.
   * 
   * @param drivingCANId  The CAN ID of the driving motor.
   * @param turningCANId  The CAN ID of the turning motor.
   * @param angularOffset The angular offset of the module.
   */
  public ModuleIOMaxServe(int index) {

    if (index > 3) {
        throw new RuntimeException("Invalid module index");
    }

    // ModuleIO flModuleIO,
    // ModuleIO frModuleIO,
    // ModuleIO blModuleIO,
    // ModuleIO brModuleIO
    int[] driveIds = {  CANID.FRONT_LEFT_DRIVING_SPARKMAX,
                        CANID.FRONT_RIGHT_DRIVING_SPARKMAX,
                        CANID.REAR_LEFT_DRIVING_SPARKMAX,
                        CANID.REAR_RIGHT_DRIVING_SPARKMAX};
    int[] turnIds = {   CANID.FRONT_LEFT_TURNING_SPARKMAX,
                        CANID.FRONT_RIGHT_TURNING_SPARKMAX,
                        CANID.REAR_LEFT_TURNING_SPARKMAX,
                        CANID.REAR_RIGHT_TURNING_SPARKMAX};
    double[] offsets = {(-Math.PI / 2),
                        0,
                        Math.PI,
                        (Math.PI / 2)};

    drivingSparkMax = new CANSparkMax(driveIds[index], MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turnIds[index], MotorType.kBrushless);

    // Factory reset, set the SPARKS MAX(s) to a known state before configuration.
    // This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup driving encoder
    drivingEncoder = drivingSparkMax.getEncoder();
    // Setup turning encoder
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    // Calculations required for driving motor conversion factors and feed forward.
    double drivingMotorFreeSpinRPS = DRIVING_MOTOR_FREE_SPIN_RPM / 60;
    double wheelCircumferenceInMeters = WHEEL_DIAMETER_IN_METERS * Math.PI;

    // 45 teeth on the driving wheel's bevel gear.
    // 22 teeth on the first-stage spur gear.
    // 15 teeth on the bevel pinion.
    double drivingMotorReduction = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    double driveWheelFreeSpinRPS = (drivingMotorFreeSpinRPS * wheelCircumferenceInMeters) / drivingMotorReduction;
    double drivingEncoderPositionFactor = (WHEEL_DIAMETER_IN_METERS * Math.PI) / drivingMotorReduction;
    double drivingEncoderVelocityFactor = ((WHEEL_DIAMETER_IN_METERS * Math.PI) / drivingMotorReduction) / 60.0;

    double turningEncoderPositionFactor = (2 * Math.PI);
    double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0;

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but WPILib's swerve APIs want meters and meters per second.
    drivingEncoder.setPositionConversionFactor(drivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(drivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder.
    // WPILib's swerve APIs want these vals in radians and radians per second.
    turningEncoder.setPositionConversionFactor(turningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(turningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the REV MAXSwerve Module.
    turningEncoder.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(0);
    turningPIDController.setPositionPIDWrappingMaxInput(turningEncoderPositionFactor);

    // Set the idle mode (brake or coast).
    drivingSparkMax.setIdleMode(DRIVING_MOTOR_IDLE_MODE);
    turningSparkMax.setIdleMode(TURNING_MOTOR_IDLE_MODE);

    // Current Limiting
    drivingSparkMax.setSmartCurrentLimit(DRIVING_MOTOR_CURRENT_LIMIT);
    turningSparkMax.setSmartCurrentLimit(TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations.
    // Note: If a SPARK MAX browns out it will maintain these configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    chassisAngularOffset = offsets[index];
    desiredModuleState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state (speed and angle) for the module.
   * 
   * @param desiredState The desired state of the swerve modules.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    desiredModuleState = desiredState;
  }



  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivingEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(drivingEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = drivingSparkMax.getAppliedOutput() * drivingSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {drivingSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turningSparkMax.getAppliedOutput() * turningSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turningSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    drivingSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turningSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    drivingSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turningSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }  
}
