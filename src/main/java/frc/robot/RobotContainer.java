package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToAmplifier;
import frc.robot.commands.EndEffectorTest;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.VoltageCommandRamp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONAVX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMaxSwerve;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.FlywheelIO;
import frc.robot.subsystems.end_effector.FlywheelIOSim;
import frc.robot.subsystems.end_effector.FlywheelIOSparkMax;
import frc.robot.subsystems.end_effector.FlywheelIOTalonFX;
import frc.robot.subsystems.example_flywheel.ExampleFlywheel;
import frc.robot.subsystems.example_flywheel.ExampleFlywheelIO;
import frc.robot.subsystems.example_flywheel.ExampleFlywheelIOSim;
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
  private final ExampleFlywheel exampleFlywheel;
  private final EndEffector endEffector;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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
        endEffector = new EndEffector(new FlywheelIOSparkMax(), new FlywheelIOTalonFX());
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
        endEffector = new EndEffector(new FlywheelIOSim(), new FlywheelIOSim());
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
        endEffector = new EndEffector(new FlywheelIO() {}, new FlywheelIO() {});
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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.y().whileTrue(new DriveToAmplifier(drive).withName("DriveToAmplifier"));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller
        .a()
        .whileTrue(
            Commands.startEnd(
                () -> exampleFlywheel.runVelocity(flywheelSpeedInput.get()),
                exampleFlywheel::stop,
                exampleFlywheel));

    endEffector.setDefaultCommand(new EndEffectorTest(endEffector, controller));

    controller.povUp().onTrue(new InstantCommand(() -> endEffector.playFiddle(), endEffector));
    controller.povDown().onTrue(new InstantCommand(() -> endEffector.pauseFiddle(), endEffector));
    controller.povLeft().onTrue(new InstantCommand(() -> endEffector.stopFiddle(), endEffector));
    controller.povRight().onTrue(new InstantCommand(() -> endEffector.nextSong(), endEffector));
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
