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

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControlller);

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

    //Default Commands

    extend.setDefaultCommand(Commands.run(() -> extend.moveExtend(-0.5), extend).until(() -> extend.extendAtSetPoint(-0.5))
    .andThen(() -> arm.moveArm(0), arm));

    //Prep Sequences

    m_operatorController.a().whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.52), arm)
    .until(() -> arm.armAtSetPoint(0.52)).andThen(
    Commands.run(() -> extend.moveExtend(-0.55), extend))));

    m_operatorController.b().whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.523), arm)
    .until(() -> arm.armAtSetPoint(0.523)).andThen(
    Commands.run(() -> extend.moveExtend(0), extend))));

    m_operatorController.y().whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.47), arm)
    .until(() -> arm.armAtSetPoint(0.47)).andThen(
    Commands.run(() -> extend.moveExtend(1.57), extend))));

    //Score Sequences

    m_operatorController.leftBumper().and(m_operatorController.a()).whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.7))
    .until(() -> arm.armAtSetPoint(0.7)).andThen(
     Commands.run(() -> extend.moveExtend(-0.65)))));

     m_operatorController.leftBumper().and(m_operatorController.b()).whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.6))
    .until(() -> arm.armAtSetPoint(0.6)).andThen(
     Commands.run(() -> extend.moveExtend(-0.7)).until(() -> extend.extendAtSetPoint(-0.7)))
     .andThen(Commands.run(() -> arm.moveArm(0.45)))));

     m_operatorController.leftBumper().and(m_operatorController.y()).whileTrue
    (Commands.sequence(Commands.run(() -> arm.moveArm(0.52))
    .until(() -> arm.armAtSetPoint(0.52)).andThen(
     Commands.run(() -> extend.moveExtend(1)))));
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
