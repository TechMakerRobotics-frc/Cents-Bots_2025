package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ChoreoAuto;
import frc.robot.commands.reef.DeliveryCoral;
import frc.robot.commands.reef.IntakeCoral;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.util.LoggedSequentialDashboardChooser;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoTrajetorys {

  private final LoggedDashboardChooser<Supplier<Command>> start =
      new LoggedDashboardChooser<>("Start");
  private final LoggedSequentialDashboardChooser<Supplier<Command>> Collect =
      new LoggedSequentialDashboardChooser<>("Collect", 5);
  private final LoggedSequentialDashboardChooser<Supplier<Command>> delivery =
      new LoggedSequentialDashboardChooser<>("Delivery", 4);

  private final Cradle cradle;
  private final Funnel funnel;

  public AutoTrajetorys(Drive drive, Cradle cradle, Funnel funnel) {
    this.cradle = cradle;
    this.funnel = funnel;
    start();
    Collect();
    delivery();
  }

  private void start() {
    start.addDefaultOption("None", () -> Commands.none());

    start.addOption("Left-A", () -> new ChoreoAuto("Left-A"));
    start.addOption("Left-B", () -> new ChoreoAuto("Left-B"));
    start.addOption("Left-C", () -> new ChoreoAuto("Left-C"));
    start.addOption("Left-D", () -> new ChoreoAuto("Left-D"));
    start.addOption("Left-E", () -> new ChoreoAuto("Left-E"));
    start.addOption("Left-F", () -> new ChoreoAuto("Left-F"));
    start.addOption("Left-G", () -> new ChoreoAuto("Left-G"));
    start.addOption("Left-H", () -> new ChoreoAuto("Left-H"));
    start.addOption("Left-I", () -> new ChoreoAuto("Left-I"));
    start.addOption("Left-J", () -> new ChoreoAuto("Left-J"));
    start.addOption("Left-K", () -> new ChoreoAuto("Left-K"));
    start.addOption("Left-L", () -> new ChoreoAuto("Left-L"));

    start.addOption("Center-A", () -> new ChoreoAuto("Center-A"));
    start.addOption("Center-B", () -> new ChoreoAuto("Center-B"));
    start.addOption("Center-C", () -> new ChoreoAuto("Center-C"));
    start.addOption("Center-D", () -> new ChoreoAuto("Center-D"));
    start.addOption("Center-E", () -> new ChoreoAuto("Center-E"));
    start.addOption("Center-F", () -> new ChoreoAuto("Center-F"));
    start.addOption("Center-G", () -> new ChoreoAuto("Center-G"));
    start.addOption("Center-H", () -> new ChoreoAuto("Center-H"));
    start.addOption("Center-I", () -> new ChoreoAuto("Center-I"));
    start.addOption("Center-J", () -> new ChoreoAuto("Center-J"));
    start.addOption("Center-K", () -> new ChoreoAuto("Center-K"));
    start.addOption("Center-L", () -> new ChoreoAuto("Center-L"));

    start.addOption("Right-A", () -> new ChoreoAuto("Right-A"));
    start.addOption("Right-B", () -> new ChoreoAuto("Right-B"));
    start.addOption("Right-C", () -> new ChoreoAuto("Right-C"));
    start.addOption("Right-D", () -> new ChoreoAuto("Right-D"));
    start.addOption("Right-E", () -> new ChoreoAuto("Right-E"));
    start.addOption("Right-F", () -> new ChoreoAuto("Right-F"));
    start.addOption("Right-G", () -> new ChoreoAuto("Right-G"));
    start.addOption("Right-H", () -> new ChoreoAuto("Right-H"));
    start.addOption("Right-I", () -> new ChoreoAuto("Right-I"));
    start.addOption("Right-J", () -> new ChoreoAuto("Right-J"));
    start.addOption("Right-K", () -> new ChoreoAuto("Right-K"));
    start.addOption("Right-L", () -> new ChoreoAuto("Right-L"));
  }

  public void Collect() {
    Collect.addDefaultOption("None", () -> Commands.none());

    Collect.addOption("A-StationLeft", () -> new ChoreoAuto("A-StationLeft"));

    Collect.addOption("B-StationLeft", () -> new ChoreoAuto("B-StationLeft"));
    Collect.addOption("C-StationLeft", () -> new ChoreoAuto("C-StationLeft"));
    Collect.addOption("D-StationLeft", () -> new ChoreoAuto("D-StationLeft"));
    Collect.addOption("E-StationLeft", () -> new ChoreoAuto("E-StationLeft"));
    Collect.addOption("F-StationLeft", () -> new ChoreoAuto("F-StationLeft"));
    Collect.addOption("G-StationLeft", () -> new ChoreoAuto("G-StationLeft"));
    Collect.addOption("H-StationLeft", () -> new ChoreoAuto("H-StationLeft"));
    Collect.addOption("I-StationLeft", () -> new ChoreoAuto("I-StationLeft"));
    Collect.addOption("J-StationLeft", () -> new ChoreoAuto("J-StationLeft"));
    Collect.addOption("K-StationLeft", () -> new ChoreoAuto("K-StationLeft"));
    Collect.addOption("L-StationLeft", () -> new ChoreoAuto("L-StationLeft"));

    Collect.addOption("A-StationRight", () -> new ChoreoAuto("A-StationRight"));
    Collect.addOption("B-StationRight", () -> new ChoreoAuto("B-StationRight"));
    Collect.addOption("C-StationRight", () -> new ChoreoAuto("C-StationRight"));
    Collect.addOption("D-StationRight", () -> new ChoreoAuto("D-StationRight"));
    Collect.addOption("E-StationRight", () -> new ChoreoAuto("E-StationRight"));
    Collect.addOption("F-StationRight", () -> new ChoreoAuto("F-StationRight"));
    Collect.addOption("G-StationRight", () -> new ChoreoAuto("G-StationRight"));
    Collect.addOption("H-StationRight", () -> new ChoreoAuto("H-StationRight"));
    Collect.addOption("I-StationRight", () -> new ChoreoAuto("I-StationRight"));
    Collect.addOption("J-StationRight", () -> new ChoreoAuto("J-StationRight"));
    Collect.addOption("K-StationRight", () -> new ChoreoAuto("K-StationRight"));
    Collect.addOption("L-StationRight", () -> new ChoreoAuto("L-StationRight"));
  }

  public void delivery() {
    delivery.addDefaultOption("None", () -> Commands.none());

    delivery.addOption("StationLeft-A", () -> new ChoreoAuto("StationLeft-A"));
    delivery.addOption("StationLeft-B", () -> new ChoreoAuto("StationLeft-B"));
    delivery.addOption("StationLeft-C", () -> new ChoreoAuto("StationLeft-C"));
    delivery.addOption("StationLeft-D", () -> new ChoreoAuto("StationLeft-D"));
    delivery.addOption("StationLeft-E", () -> new ChoreoAuto("StationLeft-E"));
    delivery.addOption("StationLeft-F", () -> new ChoreoAuto("StationLeft-F"));
    delivery.addOption("StationLeft-G", () -> new ChoreoAuto("StationLeft-G"));
    delivery.addOption("StationLeft-H", () -> new ChoreoAuto("StationLeft-H"));
    delivery.addOption("StationLeft-I", () -> new ChoreoAuto("StationLeft-I"));
    delivery.addOption("StationLeft-J", () -> new ChoreoAuto("StationLeft-J"));
    delivery.addOption("StationLeft-K", () -> new ChoreoAuto("StationLeft-K"));
    delivery.addOption("StationLeft-L", () -> new ChoreoAuto("StationLeft-L"));

    delivery.addOption("StationRight-A", () -> new ChoreoAuto("StationRight-A"));
    delivery.addOption("StationRight-B", () -> new ChoreoAuto("StationRight-B"));
    delivery.addOption("StationRight-C", () -> new ChoreoAuto("StationRight-C"));
    delivery.addOption("StationRight-D", () -> new ChoreoAuto("StationRight-D"));
    delivery.addOption("StationRight-E", () -> new ChoreoAuto("StationRight-E"));
    delivery.addOption("StationRight-F", () -> new ChoreoAuto("StationRight-F"));
    delivery.addOption("StationRight-G", () -> new ChoreoAuto("StationRight-G"));
    delivery.addOption("StationRight-H", () -> new ChoreoAuto("StationRight-H"));
    delivery.addOption("StationRight-I", () -> new ChoreoAuto("StationRight-I"));
    delivery.addOption("StationRight-J", () -> new ChoreoAuto("StationRight-J"));
    delivery.addOption("StationRight-K", () -> new ChoreoAuto("StationRight-K"));
    delivery.addOption("StationRight-L", () -> new ChoreoAuto("StationRight-L"));
  }

  public SequentialCommandGroup auto() {
    return new SequentialCommandGroup(
        start.get().get(),
        new DeliveryCoral(cradle),
        Collect.get().get(0).get(),
        new IntakeCoral(cradle, funnel),
        delivery.get().get(0).get(),
        new DeliveryCoral(cradle),
        Collect.get().get(1).get(),
        new IntakeCoral(cradle, funnel),
        delivery.get().get(1).get(),
        new DeliveryCoral(cradle),
        Collect.get().get(2).get(),
        new IntakeCoral(cradle, funnel),
        delivery.get().get(2).get(),
        new DeliveryCoral(cradle),
        Collect.get().get(3).get(),
        new IntakeCoral(cradle, funnel),
        delivery.get().get(3).get(),
        new DeliveryCoral(cradle),
        Collect.get().get(4).get(),
        new IntakeCoral(cradle, funnel));
  }

  public Command getStart() {
    return start.get().get();
  }

  /*
   * gets collect (min is 1 max is 5)
   */
  public Command getCollect(int trajetory) {
    return Collect.get().get(trajetory - 1).get();
  }

  /*
   * gets delivery (min is 1 max is 4)
   */
  public Command getDelivery(int trajetory) {
    return delivery.get().get(trajetory - 1).get();
  }
}
