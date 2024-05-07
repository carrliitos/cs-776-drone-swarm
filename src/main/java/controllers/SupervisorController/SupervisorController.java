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
    double[][] currentPosition = new double[6][3];
    while (step(TIME_STEP) != -1) {
      int key = keyboard.getKey();
      if (key > 0) {
        switch (key) {
          case Keyboard.SHIFT + 'T':
            System.out.println("T Formation");
            for (int i = 0; i < 6; i++) {
              currentPosition[i] = transFields[i].getSFVec3f();
              System.out.println("Drone " + (i + 1) + " current position: (" + currentPosition[i][0] + ", " + currentPosition[i][1] + ", " + currentPosition[i][2] + ")");
              formTShape(currentPosition);
            }
            break;
        }
      }
    }
  }

  private void moveDronesToTarget(double[][] targetPositions, double[][] currentPosition) {
    double distanceThreshold = 0.01;
    for (int i = 0; i < 6; i++) {
      double[] currentPos = currentPosition[i];
      double[] targetPos = targetPositions[i];

      double distance = Math.sqrt(Math.pow(targetPos[0] - currentPos[0], 2)
          + Math.pow(targetPos[1] - currentPos[1], 2)
          + Math.pow(targetPos[2] - currentPos[2], 2));

      while (distance > distanceThreshold) {
        double[] directionVector = {
            (targetPos[0] - currentPos[0]) / distance,
            (targetPos[1] - currentPos[1]) / distance,
            (targetPos[2] - currentPos[2]) / distance
        };

        double speed = 0.02;
        double[] newPosition = {
            currentPos[0] + directionVector[0] * speed,
            currentPos[1] + directionVector[1] * speed,
            currentPos[2] + directionVector[2] * speed
        };

        // Move the drone to the new position
        transFields[i].setSFVec3f(newPosition);
        System.out.println("Drone" + (i + 1) + " moved to: (" + newPosition[0] + ", " + newPosition[1] + ", " + newPosition[2] + ")");

        // Update current position and distance
        currentPos = newPosition;
        distance = Math.sqrt(Math.pow(targetPos[0] - currentPos[0], 2)
            + Math.pow(targetPos[1] - currentPos[1], 2)
            + Math.pow(targetPos[2] - currentPos[2], 2));

        // Delay for movement speed
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }

  private void formTShape(double[][] currentPosition) {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 0.0, 1.0, 1.2 },
        { 0.0, -1.0, 1.4 },
        { 0.0, -2.0, 1.6 },
        { -2.0, 1.0, 1.8 },
        { 2.0, 1.0, 2.0 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions, currentPosition);
  }

  private void formLShape(double[][] currentPosition) {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 1.0, 0.0, 1.2 },
        { 2.0, 0.0, 1.4 },
        { 3.0, 0.0, 1.5 },
        { 3.0, 1.0, 1.6 },
        { 3.0, 2.0, 1.8 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions, currentPosition);
  }

  private void formBoxShape(double[][] currentPosition) {
    double[][] targetPositions = {
        { 0.0, 0.0, 1.0 },
        { 1.0, 0.0, 1.0 },
        { 1.0, 1.0, 1.0 },
        { 0.0, 1.0, 1.0 },
        { 0.0, 0.0, 2.0 },
        { 1.0, 0.0, 2.0 }
    };
    currentFormation = targetPositions;
    moveDronesToTarget(targetPositions, currentPosition);
  }

  public static void main(String[] args) {
    SupervisorController supervisorController = new SupervisorController();
    supervisorController.run();
  }
}
