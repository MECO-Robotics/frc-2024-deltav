// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.NoAutomationIntakieCommand;
import frc.robot.commands.intake.StartIntakingCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swervedrive.auto.Test;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // creates variable for controllerSubsystem
  private final ControllerSubsystem controllerSubsystem = new ControllerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  
  private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

  public RobotContainer() {

    //Commands for Pathplanner
    NamedCommands.registerCommand("Shoot", new ShooterCommand(armSubsystem));  
    NamedCommands.registerCommand("Intake", new StartIntakingCommand(armSubsystem, intakeSubsystem)); 


    //Auto selection choices
    autoCommandChoice.addOption("7 note auto", "7 note auto");
    SmartDashboard.putData("PathPlannerAuto", autoCommandChoice);

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox::getYButtonPressed,
        driverXbox::getAButtonPressed,
        driverXbox::getXButtonPressed,
        driverXbox::getBButtonPressed);

    TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2), () -> true);
    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(driverController.getY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(3), () -> true);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Old YAGSL Controls
    // new JoystickButton(driverXbox, 1).onTrue((new
    // InstantCommand(drivebase::zeroGyro)));
    // new JoystickButton(driverXbox, 3).onTrue(new
    // InstantCommand(drivebase::addFakeVisionReading));

    // Driver controls

    // Pilot Controller
    XboxController pilot = controllerSubsystem.getPilotController();
    JoystickButton pilotAButton = new JoystickButton(pilot, XboxController.Button.kA.value);
    JoystickButton pilotBButton = new JoystickButton(pilot, XboxController.Button.kB.value);
    JoystickButton pilotXButton = new JoystickButton(pilot, XboxController.Button.kX.value);
    JoystickButton pilotYButton = new JoystickButton(pilot, XboxController.Button.kY.value);
    JoystickButton pilotStartButton = new JoystickButton(pilot, XboxController.Button.kStart.value);

    // CoPilot Controller
    XboxController coPilot = controllerSubsystem.getCopilotController();
    JoystickButton coPilotAButton = new JoystickButton(coPilot, XboxController.Button.kA.value);
    JoystickButton coPilotBButton = new JoystickButton(coPilot, XboxController.Button.kB.value);
    JoystickButton coPilotXButton = new JoystickButton(coPilot, XboxController.Button.kX.value);
    JoystickButton coPilotYButton = new JoystickButton(coPilot, XboxController.Button.kY.value);
    JoystickButton coPilotRightBumper = new JoystickButton(coPilot, XboxController.Button.kRightBumper.value);
    JoystickButton coPilotStartButton = new JoystickButton(coPilot, XboxController.Button.kStart.value);
    JoystickButton coPilotLeftBumper = new JoystickButton(coPilot, XboxController.Button.kLeftBumper.value);

    JoystickButton triggerJoystickButton = new JoystickButton(pilot, XboxController.Button.kY.value);


    //pilotAButton.onTrue(new StartIntakingCommand(armSubsystem, intakeSubsystem));
    //pilotBButton.onTrue(new NoAutomationIntakieCommand(intakeSubsystem));
    pilotXButton.whileTrue(new ShooterCommand(armSubsystem)); 
    
    

    // new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    // InstantCommand(drivebase::lock, drivebase)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    

    if (autoCommandChoice != null && autoCommandChoice.getSelected() != null) {

      return new PathPlannerAuto(autoCommandChoice.getSelected());
    }

    return null;
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
