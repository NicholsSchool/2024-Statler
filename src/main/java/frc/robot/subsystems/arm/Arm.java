package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CurrentDrawDesparity;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double manuelInput = 0.0;

  private ArmFeedforward ARM_FF = new ArmFeedforward(ARM_FF_KS, ARM_FF_KG, ARM_FF_KV, ARM_FF_KA);

  private double previousVelocity = 0.0;
  private double acclerationRad = 0.0;

  private double voltageCmdFF = 0.0;

  // tunable booleans

  private static final LoggedTunableNumber currentThreshold =
      new LoggedTunableNumber("Arm/DesparityThreshold");

  // tunable parameters
  private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
  private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
  private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

  private static final LoggedTunableNumber armMaxVelocityRad =
      new LoggedTunableNumber("Arm/MaxVelocityRad");
  private static final LoggedTunableNumber armMaxAccelerationRad =
      new LoggedTunableNumber("Arm/MaxAccelerationRad");

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;

    armKg.initDefault(ARM_FF_KG);
    armKv.initDefault(ARM_FF_KV);
    armKa.initDefault(ARM_FF_KA);
    armMaxVelocityRad.initDefault(Constants.ArmConstants.ARM_VEL_LIMIT);
    armMaxAccelerationRad.initDefault(Constants.ArmConstants.ARM_ACCEL_LIMIT);
    currentThreshold.initDefault(0.25);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    updateTunables();

    acclerationRad = (inputs.velocityRadsPerSec - previousVelocity) / 0.02;
    previousVelocity = inputs.velocityRadsPerSec;

    // Reset when disabled
    if (DriverStation.isDisabled()) {}

    voltageCmdFF =
        ARM_FF.calculate(
            inputs.angleRads, Constants.ArmConstants.ARM_VEL_LIMIT * softLimit(manuelInput));
    io.setVoltage(voltageCmdFF);
  }

  // set soft limits on the input velocity of the arm to make
  // sure arm does not extend passed danger position.
  public double softLimit(double inputVel) {
    // weird ranges due to [0, 360] angle range of the arm angle input
    if ((inputs.angleDegs >= 90.0 && inputs.angleDegs <= 200.0) && inputVel > 0
        || (inputs.angleDegs <= 2.0 || inputs.angleDegs >= 200.0) && inputVel < 0) {
      return 0.0;
    }
    return inputVel;
  }

  // called from run command on every cycle when manual is running.
  public void setManuel(double manuelInput) {
    this.manuelInput = manuelInput;
  }

  private void updateTunables() {
    // Update from tunable numbers
    if (armKg.hasChanged(hashCode())
        || armKv.hasChanged(hashCode())
        || armKa.hasChanged(hashCode())) {
      ARM_FF = new ArmFeedforward(0.0, armKg.get(), armKv.get(), armKa.get());
    }
  }

  // THINGS TO LOG IN ADV SCOPE

  @AutoLogOutput
  public double getAngleDeg() {
    return inputs.angleDegs;
  }

  @AutoLogOutput
  public double getAcceleration() {
    return acclerationRad;
  }

  @AutoLogOutput
  public double getVoltageCommandFF() {
    return voltageCmdFF;
  }

  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isCurrnetProblem() {
    return CurrentDrawDesparity.isDesparity(
        inputs.currentAmps[0], inputs.currentAmps[1], currentThreshold.get());
  }
}
