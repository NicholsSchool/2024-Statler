package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.arm_commands.ArmExtend;
import frc.robot.commands.arm_commands.ArmManuel;
import frc.robot.commands.arm_commands.ArmRetract;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.example_flywheel.*;
import frc.robot.subsystems.intake.*;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Arm arm;
  private final Intake intake;
  private final ExampleFlywheel exampleFlywheel;

  @SuppressWarnings("unused")
  private final PowerDistribution pdh;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  // Auto Commands
  private final AutoCommands autoCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOMaxSwerve(0),
                new ModuleIOMaxSwerve(1),
                new ModuleIOMaxSwerve(2),
                new ModuleIOMaxSwerve(3));
        // We have no flywheel, so create a simulated just for example.
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIOSim());
        pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kRev);
        arm = new Arm(new ArmIOReal());
        intake = new Intake(new IntakeIOReal());
        break;

      case ROBOT_SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIOSim());
        pdh = new PowerDistribution();
        arm = new Arm(new ArmIOSim());
        intake = new Intake(new IntakeIOSim());
        break;

      case ROBOT_REPLAY:
      default:
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        exampleFlywheel = new ExampleFlywheel(new ExampleFlywheelIO() {});
        pdh = new PowerDistribution();
        arm = new Arm(new ArmIO() {}); // TODO: make interfaces
        intake = new Intake(new IntakeIOSim());
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> exampleFlywheel.runVelocity(flywheelSpeedInput.get()),
                exampleFlywheel::stop,
                exampleFlywheel)
            .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Create auto commands
    autoCommands = new AutoCommands(drive);

    autoChooser.addOption("Score Four", autoCommands.amplifierScoreFour());
    autoChooser.addOption("auto field test", autoCommands.autoTest());
    autoChooser.addOption("Drive to note", autoCommands.driveToNote());
    autoChooser.addOption("drivenote pickup", autoCommands.driveNotePickup());
    autoChooser.addOption(
        "Drive To Amplifier",
        autoCommands.driveToPose(
            AllianceFlipUtil.apply(
                (new Pose2d(FieldConstants.amplifierTranslation, Rotation2d.fromDegrees(-90.0))))));

    // add testing auto functions
    addTestingAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));
    driveController.back().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driveController.start().onTrue(new ResetFieldOrientation(drive));
    driveController
        .leftTrigger(0.9)
        .onFalse(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getRightX() * Constants.DriveConstants.lowGearScaler));
    //for testing purposes setting to 45 degrees
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 45.0,
                () -> drive.getYaw()));
    driveController.rightTrigger(0.9).onTrue(new IntakeCommand(intake));
    driveController.leftBumper().whileTrue(new DriveToAmplifier(drive));
    // TOOD: add autoalign and nudge to swerve

    // Arm controls
    arm.setDefaultCommand(
        new ArmManuel(
            arm,
            () ->
                MathUtil.applyDeadband(
                    -operatorController.getRightY(), Constants.JOYSTICK_DEADBAND)));
    operatorController.a().onTrue(arm.runGoToPosCommand(ArmConstants.armIntakePosDeg));
    operatorController.b().onTrue(arm.runGoToPosCommand(ArmConstants.armDrivePosDeg));
    operatorController.x().onTrue(arm.runGoToPosCommand(ArmConstants.armTrapPosDeg));
    operatorController.y().onTrue(arm.runGoToPosCommand(ArmConstants.armAmpPosDeg));

    operatorController.back().onTrue(new ArmExtend(arm));
    operatorController.start().onTrue(new ArmRetract(arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void addTestingAutos() {
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addOption(
        "Flywheel FF Characterization",
        new FeedForwardCharacterization(
            exampleFlywheel,
            exampleFlywheel::runVolts,
            exampleFlywheel::getCharacterizationVelocity));

    autoChooser.addOption(
        "Module Drive Ramp Test",
        new VoltageCommandRamp(drive, drive::runDriveCommandRampVolts, 0.5, 5.0));
    autoChooser.addOption(
        "Module Turn Ramp Test",
        new VoltageCommandRamp(drive, drive::runTurnCommandRampVolts, 0.5, 5.0));
  }
}
