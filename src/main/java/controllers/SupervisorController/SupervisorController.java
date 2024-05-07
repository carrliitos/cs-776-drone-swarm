import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class SupervisorController extends Supervisor {
  private final int TIME_STEP = (int) Math.round(getBasicTimeStep());
  private Node drone1Node;
  private Field trans1Field;

  public SupervisorController() {
    super();
    getDevices();
  }
  
  private void getDevices() {
    drone1Node = getFromDef("Drone1");
    trans1Field = drone1Node.getField("translation");
  }
  
  public void run() {
    while (step(TIME_STEP) != -1) {
      final double[] values1 = trans1Field.getSFVec3f();
      double xPosition = values1[0];
      double yPosition = values1[1];
      double zPosition = values1[2];
    }
  }

  public static void main(String[] args) {
    SupervisorController supervisorController = new SupervisorController();
    supervisorController.run();
  }
}
