package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  // TODO: add beambreak and network tables
  private final FlywheelIO intake;
  private final FlywheelIO outtake;
  private final SimpleMotorFeedforward ffModel;

  /** Creates a new End Effector. */
  public EndEffector(FlywheelIO intake, FlywheelIO outtake) {
    this.intake = intake;
    this.outtake = outtake;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        intake.configurePID(1.0, 0.0, 0.0);
        outtake.configurePID(1.0, 0.0, 0.0);
        break;
      case ROBOT_SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        intake.configurePID(0.5, 0.0, 0.0);
        outtake.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {}

  /** Run open loop at the specified voltage. */
  public void runIntakeVolts(double volts) {
    intake.setVoltage(volts);
  }

  /** Run open loop at the specified voltage. */
  public void runOuttakeVolts(double volts) {
    outtake.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void runIntakeVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    intake.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Intake Flywheel/SetpointRPM", velocityRPM);
  }

  /** Run closed loop at the specified velocity. */
  public void runOuttakeVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    outtake.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Outtake Flywheel/SetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stopIntake() {
    intake.stop();
  }

  /** Stops the flywheel. */
  public void stopOuttake() {
    outtake.stop();
  }

  public void playFiddle() {
    if (outtake instanceof FlywheelIOTalonFX) ((FlywheelIOTalonFX) outtake).playFiddle();
  }

  public void pauseFiddle() {
    if (outtake instanceof FlywheelIOTalonFX) ((FlywheelIOTalonFX) outtake).pauseFiddle();
  }

  public void stopFiddle() {
    if (outtake instanceof FlywheelIOTalonFX) ((FlywheelIOTalonFX) outtake).stop();
  }

  public boolean isPlayingFiddle() {
    if (outtake instanceof FlywheelIOTalonFX)
      return ((FlywheelIOTalonFX) outtake).isPlayingFiddle();
    return false;
  }

  public void nextSong() {
    if (outtake instanceof FlywheelIOTalonFX) ((FlywheelIOTalonFX) outtake).nextSong();
  }
}
