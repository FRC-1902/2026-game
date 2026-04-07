package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.Timer;
import java.util.TimerTask;

public class ControllerSubsystem extends SubsystemBase {
  private final CommandXboxController commandDriveController;
  private final CommandXboxController commandManipController;
  private final XboxController driveController;
  private final XboxController manipController;

  private static ControllerSubsystem controllerInstance;

  public static ControllerSubsystem getInstance() {
    if (controllerInstance == null) {
      controllerInstance = new ControllerSubsystem();
    }
    return controllerInstance;
  }

  private ControllerSubsystem() {
    commandDriveController = new CommandXboxController(Constants.Controller.DRIVE_CONTROLLER_PORT);
    commandManipController = new CommandXboxController(Constants.Controller.MANIP_CONTROLLER_PORT);
    driveController = commandDriveController.getHID();
    manipController = commandManipController.getHID();
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    LS(9),
    RS(10);

    public final int id;

    Button(int id) {
      this.id = id;
    }
  }

  public enum Axis {
    LX(0),
    LY(1),
    RX(4),
    RY(5),
    LT(2),
    RT(3);

    public final int id;

    Axis(int id) {
      this.id = id;
    }
  }

  public enum ControllerName {
    DRIVE,
    MANIP
  }

  public boolean get(ControllerName name, Button button) {
    switch (name) {
      case DRIVE:
        return driveController.getRawButton(button.id);
      case MANIP:
        return manipController.getRawButton(button.id);
      default:
        return false;
    }
  }

  public double get(ControllerName name, Axis axis) {
    switch (name) {
      case DRIVE:
        return driveController.getRawAxis(axis.id);
      case MANIP:
        return manipController.getRawAxis(axis.id);
      default:
        return 0.0;
    }
  }

  public Trigger getTrigger(ControllerName name, Button button) {
    switch (name) {
      case DRIVE:
        return commandDriveController.button(button.id);
      case MANIP:
        return commandManipController.button(button.id);
      default:
        return null;
    }
  }

  public int getDPAD(ControllerName name) {
    switch (name) {
      case DRIVE:
        return driveController.getPOV();
      case MANIP:
        return manipController.getPOV();
      default:
        return 0;
    }
  }

  public void vibrate(ControllerName name, long msDuration, double intensity) {
    XboxController targetController =
        name == ControllerName.DRIVE ? driveController : manipController;

    targetController.setRumble(RumbleType.kBothRumble, intensity);
    Timer timer = new Timer(true);
    timer.schedule(
        new TimerTask() {
          @Override
          public void run() {
            targetController.setRumble(RumbleType.kBothRumble, 0);
            timer.cancel();
          }
        },
        msDuration);
  }

  public void setRumble(ControllerName name, double intensity) {
    XboxController targetController =
        name == ControllerName.DRIVE ? driveController : manipController;
    targetController.setRumble(RumbleType.kBothRumble, intensity);
  }

  public CommandXboxController getCommandController(ControllerName name) {
    switch (name) {
      case DRIVE:
        return commandDriveController;
      case MANIP:
        return commandManipController;
      default:
        return null;
    }
  }
}
