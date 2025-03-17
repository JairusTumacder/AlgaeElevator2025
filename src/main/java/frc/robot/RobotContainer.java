package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorStates.ManualElev;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.fakeCoral;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
  private final CommandXboxController d_controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final fakeCoral coral = new fakeCoral();
  private final AlgaePivot algaePivotSub = new AlgaePivot(coral.getLimitSwitch());
  private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();

  public RobotContainer() {
    elevatorSub.setDefaultCommand(new ManualElev(elevatorSub, () -> d_controller.getLeftY()));
    configureBindings();
  }
  
  private void configureBindings() {
    
    //SysId Routines
    // d_controller.leftBumper().onTrue(algaePivotSub.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // d_controller.rightBumper().onTrue(algaePivotSub.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // d_controller.leftTrigger().onTrue(algaePivotSub.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // d_controller.rightTrigger().onTrue(algaePivotSub.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // d_controller.x().onTrue(Commands.runOnce(SignalLogger::start));
    // d_controller.y().onTrue(Commands.runOnce(SignalLogger::stop));
  }
}
