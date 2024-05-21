package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.VoltageCommandRamp;
import frc.robot.commands.arm_commands.ArmManuel;
import frc.robot.commands.drive_commands.DriveCommands;
import frc.robot.commands.drive_commands.ParabolicSplineToPoint;
import frc.robot.commands.drive_commands.ResetPoseCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandIOReal;
import frc.robot.subsystems.hand.HandIOSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private final Hand hand;

  @SuppressWarnings("unused")
  private PowerDistribution pdh;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static final LoggedTunableNumber startX0 = new LoggedTunableNumber("Start X(m)", 0.0);
  public static final LoggedTunableNumber startY0 = new LoggedTunableNumber("Start Y(m)", 0.0);
  public static final LoggedTunableNumber startTheta0 =
      new LoggedTunableNumber("Start Theta(deg)", 0.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        // Real robot, instantiate hardware IO implementations
        pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kCTRE);

        // drive =
        //     new Drive(
        //         new GyroIONAVX(),
        //         new ModuleIOMaxSwerve(0),
        //         new ModuleIOMaxSwerve(1),
        //         new ModuleIOMaxSwerve(2),
        //         new ModuleIOMaxSwerve(3));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        arm = new Arm(new ArmIOSim());
        hand = new Hand(new HandIOReal());
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
        arm = new Arm(new ArmIOSim());
        hand = new Hand(new HandIOSim());
        break;

      default:
        // case ROBOT_REPLAY:
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        arm = new Arm(new ArmIOSim());
        hand = new Hand(new HandIOSim());
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Wait 5 seconds", new WaitCommand(5.0));

    // add testing auto functions
    addTestingAutos();

    // Configure the button bindings
    configureButtonBindings();

    // set starting position of robot
    setStartingPose();
  }

  /**
   * Set the starting pose of the robot based on position index. This should be called only when
   * robot is disabled.
   */
  public void setStartingPose() {
    // Set starting position only if operating robot in field-relative control.
    // Otherwise, robot starts at 0, 0, 0.
    if (!Constants.driveRobotRelative) {
      Pose2d startPosition =
          new Pose2d(
              startX0.get(), startY0.get(), new Rotation2d(Math.toRadians(startTheta0.get())));

      drive.setPose(startPosition);
    }
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
            () -> driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
            () -> driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getRightX() * Constants.DriveConstants.lowGearScaler,
            () -> Constants.driveRobotRelative));

    driveController.start().onTrue(new ResetPoseCommand(drive));
    driveController.back().onTrue(new ResetPoseCommand(drive));

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 180,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 0,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> -90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));

    driveController.povUp().whileTrue(new ParabolicSplineToPoint(drive));

    // Arm Controls
    arm.setDefaultCommand(new ArmManuel(arm, () -> -operatorController.getLeftY()));

    // intake
    hand.setDefaultCommand(new InstantCommand(() -> hand.stop(), hand));
    operatorController.leftTrigger().whileTrue(new RunCommand(() -> hand.intake(), hand));
    operatorController.rightTrigger().whileTrue(hand.runShootCommand());
  }

  /**
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
        "Module Drive Ramp Test",
        new VoltageCommandRamp(drive, drive::runDriveCommandRampVolts, 0.5, 5.0));

    autoChooser.addOption(
        "Module Turn Ramp Test",
        new VoltageCommandRamp(drive, drive::runTurnCommandRampVolts, 0.5, 5.0));
  }
}
