package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaePivotStates.Barge;
import frc.robot.commands.AlgaePivotStates.Descore;
import frc.robot.commands.AlgaePivotStates.Processor;
import frc.robot.commands.AlgaePivotStates.Storage;
import frc.robot.commands.AlgaePivotStates.Tuck;
import frc.robot.commands.AlgaeShooterStates.AlgaeIntake;
import frc.robot.commands.AlgaeShooterStates.AlgaeShoot;
import frc.robot.commands.ElevatorStates.L2State;
import frc.robot.commands.ElevatorStates.L3State;
import frc.robot.commands.ElevatorStates.L4State;
import frc.robot.commands.ElevatorStates.ManualElev;
import frc.robot.commands.ElevatorStates.TuckState;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AlgaeShooter;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final CommandXboxController d_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SparkMax coralPivot = new SparkMax(7, MotorType.kBrushless);
  private final AlgaePivot algaePivotSub = new AlgaePivot(coralPivot.getForwardLimitSwitch());
  private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();

  private final AlgaeShooter algaeShooter = new AlgaeShooter();

  public RobotContainer() {
    // elevatorSub.setDefaultCommand(new ManualElev(elevatorSub, () -> d_controller.getLeftY()));
    configureBindings();
  }  
  
  private void configureBindings() {
    

    d_controller.y().onTrue(new L2State(elevatorSub));
    d_controller.a().onTrue(new TuckState(elevatorSub));
    d_controller.x().onTrue(new L3State(elevatorSub));
    // d_controller.b().onTrue(new L2State(elevatorSub));

    d_controller.leftBumper().onTrue(new Tuck(algaePivotSub));
    // d_controller.rightBumper().onTrue(new Processor(algaePivotSub));
    // d_controller.a().onTrue(new Storage(algaePivotSub));
    // d_controller.b().onTrue(new Descore(algaePivotSub));
    // d_controller.rightBumper().onTrue(new Barge(algaePivotSub));

    // d_controller.x().whileTrue(new AlgaeIntake(algaeShooter));
    // d_controller.b().onTrue(new AlgaeShoot(algaeShooter));

    //SysId Routines
    // d_controller.leftBumper().onTrue(algaePivotSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // d_controller.rightBumper().onTrue(algaePivotSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // d_controller.leftTrigger().onTrue(algaePivotSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // d_controller.rightTrigger().onTrue(algaePivotSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // d_controller.x().onTrue(Commands.runOnce(SignalLogger::start));
    // d_controller.y().onTrue(Commands.runOnce(SignalLogger::stop));
  }
}
