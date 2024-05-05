import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Receiver;

public class SupervisorController extends Supervisor {
  final int TIME_STEP = 32; // Simulation time step in milliseconds
  private Receiver receiver;

  public SupervisorController() {
    receiver = getReceiver("receiver");
    receiver.enable(TIME_STEP);
  }

  public static double distance(double currentPosX, double currentPosY, double targetPosX, double targetPosY) {
    return Math.sqrt(Math.pow((targetPosX - currentPosX), 2) + Math.pow((targetPosY - currentPosY), 2));
  }
  
  public void run() {
    receiver = getReceiver("receiver");
    receiver.enable(TIME_STEP);
    final Supervisor supervisor = new Supervisor();
    final Node drone1Node = supervisor.getFromDef("Drone1");
    final Node drone2Node = supervisor.getFromDef("Drone2");
    final Field trans1Field = drone1Node.getField("translation");
    final Field trans2Field = drone2Node.getField("translation");
    
    while (supervisor.step(TIME_STEP) != -1) {
      final double[] values1 = trans1Field.getSFVec3f();
      final double[] values2 = trans2Field.getSFVec3f();
      // System.out.format("drone1Node is at position: %g %g %g\n", values1[0], values1[1], values1[2]);
      // System.out.format("drone2Node is at position: %g %g %g\n", values2[0], values2[1], values2[2]);
      // double[] targetPositions = { 0, 0 };
      // System.out.println(distance(values1[0], values1[1], targetPositions[0], targetPositions[1]));
      
      if (receiver.getQueueLength() > 0) {
        String message = new String(receiver.getData());
        receiver.nextPacket();
        System.out.println("Position X: " + message);
      }
    }
  }

  public static void main(String[] args) {
    SupervisorController supervisorController = new SupervisorController();
    supervisorController.run();
  }
}
