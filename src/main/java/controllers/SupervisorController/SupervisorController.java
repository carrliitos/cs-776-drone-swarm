import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Keyboard;
import java.util.Arrays;
import java.util.List;

public class SupervisorController extends Supervisor {
  private final int TIME_STEP = (int) Math.round(getBasicTimeStep());
  private Node[] droneNodes = new Node[6];
  private Field[] transFields = new Field[6];
  private Keyboard keyboard;
  private double[][] currentFormation;
  private double[][] baseFormation = {
      { 0.0, 0.0, 1.0 },
      { 0.0, 0.5, 1.0 },
      { 0.0, 1.0, 1.0 },
      { 0.0, 0.5, 1.0 },
      { 0.0, 2.0, 1.0 },
      { 0.0, 0.5, 1.0 }
  };
  private int[][][] grid = {
      {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
      {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
      {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
  };

  public SupervisorController() {
    super();
    getDevices();
  }
  
  private void getDevices() {
    for (int i = 0; i < 6; i++) {
      droneNodes[i] = getFromDef("Drone" + (i + 1));
      transFields[i] = droneNodes[i].getField("translation");
    }
    
    keyboard = new Keyboard();
    keyboard.enable(TIME_STEP);
  }
  
  public void run() {
    currentFormation = baseFormation;
    while (step(TIME_STEP) != -1) {
      int key = keyboard.getKey();
      while (key > 0) {
        switch (key) {
          case (Keyboard.SHIFT + 'T'):
            System.out.println("T Formation");
            formTShape();
            break;
          case (Keyboard.SHIFT + 'L'):
            System.out.println("L Formation");
            formLShape();
            break;
          case (Keyboard.SHIFT + 'B'):
            System.out.println("Box Formation");
            formBoxShape();
            break;
        }
        key = keyboard.getKey();
      }
      if (keyboard.getKey() == 0 && !Arrays.deepEquals(currentFormation, baseFormation)) {
        currentFormation = baseFormation;
        moveDronesToTarget(baseFormation);
      }
    }
  }

  private void moveDronesToTarget(double[][] targetPositions) {
    for (int i = 0; i < 6; i++) {
      double[] currentPosition = transFields[i].getSFVec3f();

      int startX = (int) Math.round(currentPosition[0]);
      int startY = (int) Math.round(currentPosition[1]);
      int startZ = (int) Math.round(currentPosition[2]);

      int targetX = (int) Math.round(targetPositions[i][0]);
      int targetY = (int) Math.round(targetPositions[i][1]);
      int targetZ = (int) Math.round(targetPositions[i][2]);

      List<AStar.Node> path = AStar.findPath(grid, startX, startY, startZ, targetX, targetY, targetZ);

      if (path != null && !path.isEmpty()) {
        // Move drone along the path
        for (int j = 1; j < path.size(); j++) {
          AStar.Node node = path.get(j);
          double[] newPosition = { node.x, node.y, node.z };
          transFields[i].setSFVec3f(newPosition);
          System.out.println("Drone" + (i + 1) + " moved to: (" + newPosition[0] + ", " + newPosition[1] + ", " + newPosition[2] + ")");
          // Delay for movement speed
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      } else {
        System.out.println("No path found for Drone" + (i + 1));
      }
    }
  }

  private void formTShape() {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 0.0, 1.0, 1.2 },
        { 0.0, -1.0, 1.4 },
        { 0.0, -2.0, 1.6 },
        { -2.0, 1.0, 1.8 },
        { 2.0, 1.0, 2.0 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions);
  }

  private void formLShape() {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 1.0, 0.0, 1.2 },
        { 2.0, 0.0, 1.4 },
        { 3.0, 0.0, 1.5 },
        { 3.0, 1.0, 1.6 },
        { 3.0, 2.0, 1.8 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions);
  }

  private void formBoxShape() {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 1.0, 0.0, 1.0 },
        { 1.0, 1.0, 1.0 },
        { 0.0, 1.0, 1.0 },
        { 0.0, 0.0, 2.0 },
        { 1.0, 0.0, 2.0 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions);
  }

  public static void main(String[] args) {
    SupervisorController supervisorController = new SupervisorController();
    supervisorController.run();
  }
}
