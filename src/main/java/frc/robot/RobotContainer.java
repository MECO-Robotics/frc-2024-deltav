// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.ConstantsJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.arm.ManualArmControlCommand;
import frc.robot.commands.arm.SetPointControlCommand;
import frc.robot.commands.indexer.IndexingCommand;
//import frc.robot.commands.HandoffCommand;
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
import java.lang.invoke.ConstantCallSite;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.IndexingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IndexingSubsystem indexingSubsystem = new IndexingSubsystem();

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));
    // CommandJoystick rotationController = new CommandJoystick(1);
    // Replace with CommandPS4Controller or CommandJoystick if needed

    XboxController pilotController = new XboxController(0);
    CommandXboxController pilotCommandController = new CommandXboxController(0);
    XboxController coPilotController = new XboxController(1);
    CommandXboxController coPilotCommandController = new CommandXboxController(1);
    CommandXboxController tuningCommandXboxController = new CommandXboxController(2);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private final SendableChooser<String> autoCommandChoice = new SendableChooser<String>();

    public RobotContainer() {

        // Commands for Pathplanner
        NamedCommands.registerCommand("Shoot", new ShooterCommand(shooterSubsystem,
                Constants.Shooter.Presets.kLeftSpeaker, Constants.Shooter.Presets.kRightSpeaker));
        NamedCommands.registerCommand("Intake", new NoAutomationIntakieCommand(intakeSubsystem, null));
        NamedCommands.registerCommand("Handoff", new HandoffCommand(indexingSubsystem, intakeSubsystem));
        NamedCommands.registerCommand("RunIndexer", new IndexingCommand(indexingSubsystem, 12));
        NamedCommands.registerCommand("StopIndexer", new IndexingCommand(indexingSubsystem, 0));

        // Auto selection choices
        autoCommandChoice.addOption("7 note auto", "7 note auto");
        SmartDashboard.putData("PathPlannerAuto", autoCommandChoice);
        SmartDashboard.putData("4 note(3 close) middle auto", autoCommandChoice);
        SmartDashboard.putData("4 note(3 close) bottom auto", autoCommandChoice);

        // Configure the trigger bindingss
        configureBindings();

        AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
                // Applies deadbands and inverts controls because joysticks
                // are back-right positive while robot
                // controls are front-left positive
                () -> MathUtil.applyDeadband(pilotController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(pilotController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -pilotController.getRightX(),
                () -> -pilotController.getRightY());

        AbsoluteFieldDrive closedFieldAbsoluteDrive = new AbsoluteFieldDrive(drivebase,
                () -> MathUtil.applyDeadband(pilotController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(pilotController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> pilotController.getRawAxis(2));

        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                () -> MathUtil.applyDeadband(pilotController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(pilotController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(pilotController.getRightX(),
                        OperatorConstants.RIGHT_X_DEADBAND),
                pilotController::getYButtonPressed,
                pilotController::getAButtonPressed,
                pilotController::getXButtonPressed,
                pilotController::getBButtonPressed);

        TeleopDrive simClosedFieldRel = new TeleopDrive(drivebase,
                () -> MathUtil.applyDeadband(pilotController.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(pilotController.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> pilotController.getRawAxis(2), () -> true);

        TeleopDrive closedFieldRel = new TeleopDrive(
                drivebase,
                () -> MathUtil.applyDeadband(pilotCommandController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(pilotCommandController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -pilotCommandController.getRawAxis(3), () -> true);

        //ManualArmControlCommand manualArm = new ManualArmControlCommand(armSubsystem,
                //() -> MathUtil.applyDeadband(coPilotController.getRightY() * -12, 0.01));

        drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
        intakeSubsystem.setDefaultCommand(
               new NoAutomationIntakieCommand(intakeSubsystem, () -> pilotController.getRightTriggerAxis() * -12));
        indexingSubsystem.setDefaultCommand(new IndexingCommand(indexingSubsystem, () -> pilotController.getLeftTriggerAxis() * 12));
   
        //armSubsystem.setDefaultCommand(manualArm);
                //new IndexingCommand(indexingSubsystem, () -> pilotController.getLeftTriggerAxis() * -12));
        //armSubsystem.setDefaultCommand(manualArm);
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
        // new JoystickButton(pilot, 1).onTrue((new
        // InstantCommand(drivebase::zeroGyro)));
        // new JoystickButton(pilot, 3).onTrue(new
        // InstantCommand(drivebase::addFakeVisionReading));

        // pilotAButton.onTrue(new StartIntakingCommand(armSubsystem, intakeSubsystem));
        pilotCommandController.b().whileTrue(new NoAutomationIntakieCommand(intakeSubsystem, () -> -12));
        coPilotCommandController.x().whileTrue(new ShooterCommand(shooterSubsystem, Constants.Shooter.Presets.kLeftSpeaker,
                Constants.Shooter.Presets.kRightSpeaker));
        //coPilotCommandController.a().onTrue(new InstantCommand(shooterSubsystem::disable));
        coPilotCommandController.a().onTrue(new InstantCommand(shooterSubsystem::disable));
        pilotCommandController.x().whileTrue(new HandoffCommand(indexingSubsystem, intakeSubsystem));
        coPilotCommandController.povDown().onTrue(new SetPointControlCommand(armSubsystem, Constants.Arm.SetPointPositions.kStowPosition));
        coPilotCommandController.povUp().onTrue(new SetPointControlCommand(armSubsystem, Constants.Arm.SetPointPositions.kAmpPosition));
        
        //SysId controls
        tuningCommandXboxController.x().whileTrue(armSubsystem.sysIdQuasistaticc(SysIdRoutine.Direction.kForward)
                .finallyDo(armSubsystem::disable));
        tuningCommandXboxController.a().whileTrue(armSubsystem.sysIdQuasistaticc(SysIdRoutine.Direction.kReverse)
                .finallyDo(armSubsystem::disable));
        tuningCommandXboxController.y().whileTrue(
                armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward).finallyDo(armSubsystem::disable));
        tuningCommandXboxController.b().whileTrue(
                armSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse).finallyDo(armSubsystem::disable));

        // pilotAButton.onTrue(new HandoffCommand(armSubsystem, intakeSubsystem));

        // new JoystickButton(pilot, 3).whileTrue(new RepeatCommand(new
        // InstantCommand(drivebase::lock, drivebase)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
/* 
        if (autoCommandChoice != null && autoCommandChoice.getSelected() != null) {

            return new PathPlannerAuto(autoCommandChoice.getSelected());
        }

        return null;
        */
        return new PathPlannerAuto("test");
    }

    public void setDriveMode() {
        // drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
