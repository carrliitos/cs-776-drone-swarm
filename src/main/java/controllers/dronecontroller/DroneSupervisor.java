import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;

public class DroneSupervisor {
  public static void main(String[] args) {
    final int TIME_STEP = 64; // Simulation time step in milliseconds
    final Supervisor supervisor = new Supervisor();
    final Node drone1Node = supervisor.getFromDef("DRONE_1");
    final Node drone2Node = supervisor.getFromDef("DRONE_2");
    final Field trans1Field = drone1Node.getField("translation");
    final Field trans2Field = drone2Node.getField("translation");
    
    while (supervisor.step(TIME_STEP) != -1) {
      // this is done repeatedly
      final double[] values1 = trans1Field.getSFVec3f();
      final double[] values2 = trans2Field.getSFVec3f();
      System.out.format("drone1Node is at position: %g %g %g\n", values1[0], values1[1], values1[2]);
      System.out.format("drone2Node is at position: %g %g %g\n", values2[0], values2[1], values2[2]);
    }
  }
}
