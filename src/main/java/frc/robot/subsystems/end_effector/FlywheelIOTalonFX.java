package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = Constants.EffectorTalonConstants.kGearRatio;

  private final TalonFX motor = new TalonFX(Constants.CAN.kEffectorTalonCanId);

  private final StatusSignal<Double> position = motor.getPosition();
  private final StatusSignal<Double> velocity = motor.getVelocity();
  private final StatusSignal<Double> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Double> current = motor.getStatorCurrent();

  private final Orchestra orchestra;
  private final String[] songs = {
    Constants.EffectorTalonConstants.FiddleSongs.ALL_STAR,
    Constants.EffectorTalonConstants.FiddleSongs.IMPERIAL_MARCH,
    Constants.EffectorTalonConstants.FiddleSongs.WII_SONG
  };
  private int songIndex;

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.EffectorTalonConstants.kCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);

    motor.setInverted(Constants.EffectorTalonConstants.kIsInverted);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();

    orchestra = new Orchestra();
    orchestra.addInstrument(motor);

    songIndex = 0;
    orchestra.loadMusic(songs[songIndex]);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor.setControl(
        // TODO: talk with Mr. Greier about this
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    motor.getConfigurator().apply(config);
  }

  public void playFiddle() {
    orchestra.play();
  }

  public void pauseFiddle() {
    orchestra.pause();
  }

  public void stopFiddle() {
    orchestra.stop();
  }

  public boolean isPlayingFiddle() {
    return orchestra.isPlaying();
  }

  public void nextSong() {
    songIndex = (songIndex + 1) % songs.length;
    orchestra.loadMusic(songs[songIndex]);
  }
}
