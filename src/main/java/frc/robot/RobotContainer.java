// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extend;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Arm arm = new Arm();
  private final Extend extend = new Extend();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  // Command armMove = Commands.startEnd(() -> arm.moveArm(0.6), () -> arm.moveArm(0), arm); 

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    arm.setDefaultCommand(Commands.run(() -> arm.moveArm(-0.21), arm));
    extend.setDefaultCommand(Commands.run(() -> extend.moveExtend(-0.01), extend));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue
    // (Commands.parallel(Commands.run(() -> arm.moveArm(0.6), arm), 
    // Commands.run(() -> extend.moveExtend(0.6), extend)));

    // m_driverController.b().whileTrue
    //  (Commands.startRun(() -> arm.moveArm(0.6), arm.position(0), arm));
    //L4 0.474
    //L3 0.538
    //L2 0.628

    // m_driverController.b().whileTrue
    // (Commands.sequence(Commands.run(() -> arm.moveArm(0.538), arm)
    // .until(() -> arm.armAtSetPoint(0.538)).andThen(
    // Commands.run(() -> extend.moveExtend(0.0), extend)))); //L2

    m_driverController.y().whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.277), arm)
    .until(() -> arm.armAtSetPoint(0.277)).andThen(
    Commands.run(() -> extend.moveExtend(1.634), extend)))); //L4


    // m_driverController.a().whileTrue
    // (Commands.sequence(Commands.run(() -> arm.moveArm(0.474), arm)
    // .until(() -> arm.armAtSetPoint(0.474)).andThen(
    // Commands.run(() -> extend.moveExtend(0.0), extend)))); //L2

    // m_driverController.leftBumper().or(m_driverController.a())
    // .or(m_driverController.b()).whileTrue
    // (Commands.sequence(Commands.run(() -> arm.moveArm(0.6))
    // .until(() -> arm.armAtSetPoint(0.6)).andThen(
    //  Commands.run(() -> extend.moveExtend(0.0))))); //L2 and L3

    // m_driverController.leftTrigger().whileTrue
    // (Commands.sequence(Commands.run(() -> arm.moveArm(0.1), arm)
    // .until(() -> arm.armAtSetPoint(0.1)).andThen(
    //  Commands.run(() -> extend.moveExtend(0.0), extend))));

     m_driverController.leftBumper().and(m_driverController.y()).whileTrue
     (Commands.sequence(Commands.run(() -> arm.moveArm(0.29))
     .until(() -> arm.armAtSetPoint(0.29)).andThen(
      Commands.run(() -> extend.moveExtend(0.0))))); //score L2

      // m_driverController.start().whileTrue(Commands.runOnce(()-> extend.setZero(), extend));

    // m_driverController.a().whileTrue
    // (Commands.run(() -> extend.moveExtend(0.6), extend));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
