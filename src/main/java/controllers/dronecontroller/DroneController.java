import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;

import java.io.IOException;

public class DroneController extends Robot {
  private String positionsCSVFile = System.getProperty("user.dir") + "/data/positions.csv";
  private String inputsCSVFile = System.getProperty("user.dir") + "/data/inputs.csv";
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds
  private static final double TARGET_ALTITUDE = 1.0;
  private static final double TARGET_X = 0.0;
  private static final double TARGET_Y = 0.0;

  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
  private Motor motors[];
  private LED frontLeftLED, frontRightLED;
  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  private CsvWriter positionCsvWriter;
  private CsvWriter inputsCsvWriter;
  private RealTimeDataApp positionsDataVisualizer;
  private PID rollPID;
  private PID pitchPID;
  private PID yawPID;

  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_VERTICAL_OFFSET = 0.6; // Vertical offset where the robot actually targets to stabilize itself.
  private static final double K_VERTICAL_P = 2.0;
  private static final double K_ROLL_P = 1.0;
  private static final double K_ROLL_I = 0.0;
  private static final double K_ROLL_D = 1.0;
  private static final double K_PITCH_P = 0.0;
  private static final double K_PITCH_I = 0.0;
  private static final double K_PITCH_D = 0.0;
  private static final double K_YAW_P = 0.0;
  private static final double K_YAW_I = 0.0;
  private static final double K_YAW_D = 0.0;
  
  public DroneController() {
    super();
    initializeDevices();
  }
  
  public static double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
  }
  
  private void initializeDevices() {
    System.out.println(inputsCSVFile);
    imu = getInertialUnit("inertial unit");
    imu.enable(TIME_STEP);
    gps = getGPS("gps");
    gps.enable(TIME_STEP);
    gyro = getGyro("gyro");
    gyro.enable(TIME_STEP);
    frontLeftLED = new LED("front left led");
    frontRightLED = new LED("front right led");
    frontRightPropeller = getMotor("front right propeller");
    frontLeftPropeller = getMotor("front left propeller");
    rearRightPropeller = getMotor("rear right propeller");
    rearLeftPropeller = getMotor("rear left propeller");
    motors = new Motor[] {
      frontRightPropeller,
      frontLeftPropeller,
      rearRightPropeller,
      rearLeftPropeller
    };
    
    for (Motor motor : motors) {
      motor.setPosition(Double.POSITIVE_INFINITY);
      motor.setVelocity(1); // 10% of velocity to start
    }
    
    cameraRollMotor = getMotor("camera roll");
    cameraPitchMotor = getMotor("camera pitch");
    try {
      positionCsvWriter = new CsvWriter(positionsCSVFile);
      inputsCsvWriter = new CsvWriter(inputsCSVFile);
    } catch (IOException e) {
      System.err.println("Error creating a new CsvWriter: " + e.getMessage());
    }
    positionsDataVisualizer = new RealTimeDataApp("Drone State Visualization");

    rollPID = new PID(K_ROLL_P, K_ROLL_I, K_ROLL_D, 0.0);
    pitchPID = new PID(K_PITCH_P, K_PITCH_I, K_PITCH_D, 0.0);
    yawPID = new PID(K_YAW_P, K_YAW_I, K_YAW_D, 0.0);
  }
  
  private void displayWelcomeMessage() {
    // Wait one second
    double previousTime = 0.0;
    while (step(TIME_STEP) != -1) {
      if (getTime() - previousTime > 1.0) { break; }
    }
  }
  
  private void blinkLEDS() {
    final double timeInSecs = getTime(); // in seconds.
    final boolean ledState = ((int) timeInSecs) % 2 == 0;
  
    frontLeftLED.set(ledState ? 1 : 0);
    frontRightLED.set(ledState ? 0 : 1);
  }
  
  private double[] getRobotState() {
    try {
      final double roll = imu.getRollPitchYaw()[0];
      final double pitch = imu.getRollPitchYaw()[1];
      final double yaw = imu.getRollPitchYaw()[2];
      final double posX = gps.getValues()[0];
      final double posY = gps.getValues()[1];
      final double altitude = gps.getValues()[2];
      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];
      double[] positions = { roll, pitch, yaw, posX, posY, altitude, rollVelocity, pitchVelocity };
      positionCsvWriter.writeData(positions);
      return positions;
    } catch (Exception e) {
      System.err.println("Error retrieving robot state: " + e.getMessage());
      return new double[]{0.0, 0.0, 0.0, 0.0, 0.0}; // Default values
    }
  }
  
  private void stabilizeCamera(double rollVelocity, double pitchVelocity) {
    cameraRollMotor.setPosition(-0.115 * rollVelocity);
    cameraPitchMotor.setPosition(-0.1 * pitchVelocity);
  }

  private double[] computeInputs(double roll, double altitude, double rollVelocity, double rollDisturbance, 
                                 double pitch, double pitchVelocity, double pitchDisturbance, 
                                 double yaw, double yawDisturbance, double xPos, double yPos) {
    double dt = 0.1;
    double deltaX = TARGET_X - xPos;
    double deltaY = TARGET_Y - yPos;
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    double targetAngle = Math.atan2(deltaY, deltaX);
    double currentYaw = yaw;
    double yawError = targetAngle - currentYaw;
    if (yawError > Math.PI) {
        yawError -= 2 * Math.PI;
    } else if (yawError < -Math.PI) {
        yawError += 2 * Math.PI;
    }
    double rollError = distance;
    double pitchError = distance;

    final double rollInput = rollPID.update(rollError, dt);
    final double pitchInput = pitchPID.update(pitchError, dt);
    final double yawInput = yawPID.update(yawError, dt);
    final double clampedDiffAltitude = clamp(TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET, -1.0, 1.0);
    final double verticalInput = K_VERTICAL_P * Math.pow(clampedDiffAltitude, 3);
    double[] allInputs = { rollInput, pitchInput, yawInput, verticalInput };
    inputsCsvWriter.writeData(allInputs);

    positionsDataVisualizer.visualize(rollInput);

    return allInputs;
  }
  
  private void activateActuators(double verticalInput, double rollInput, double pitchInput, double yawInput) {
    final double frontLeftPropellerInput = K_VERTICAL_THRUST + verticalInput - rollInput + pitchInput - yawInput;
    final double frontRightPropellerInput = K_VERTICAL_THRUST + verticalInput + rollInput + pitchInput + yawInput;
    final double rearLeftPropellerInput = K_VERTICAL_THRUST + verticalInput - rollInput - pitchInput + yawInput;
    final double rearRightPropellerInput = K_VERTICAL_THRUST + verticalInput + rollInput - pitchInput - yawInput;

    frontLeftPropeller.setVelocity(frontLeftPropellerInput);
    frontRightPropeller.setVelocity(-frontRightPropellerInput);
    rearLeftPropeller.setVelocity(-rearLeftPropellerInput);
    rearRightPropeller.setVelocity(rearRightPropellerInput);
  }
  
  // Main control loop
  public void run() {
    displayWelcomeMessage();
    String[] positionHeaders = { "ROLL", "PITCH", "YAW", "POSITION_X", "POSITION_Y", "ALTITUDE", "ROLL_VELOCITY", "PITCH_VELOCITY" };
    String[] inputHeaders = { "ROLL", "PITCH", "YAW", "VERTICAL" };
    positionCsvWriter.writeHeaders(positionHeaders);
    inputsCsvWriter.writeHeaders(inputHeaders);
    
    while (step(TIME_STEP) != -1) {
      double rollDisturbance = 0.0;
      double pitchDisturbance = 0.0;
      double yawDisturbance = 0.0;

      double[] robotState = getRobotState();
      double roll = robotState[0];
      double pitch = robotState[1];
      double yaw = robotState[2];
      double posX = robotState[3];
      double posY = robotState[4];
      double altitude = robotState[5];
      double rollVelocity = robotState[6];
      double pitchVelocity = robotState[7];

      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(rollVelocity, pitchVelocity);

      // Compute the roll, pitch, yaw and vertical inputs.
      double[] rpyvInputs = computeInputs(roll, altitude, rollVelocity, rollDisturbance, 
                                          pitch, pitchVelocity, pitchDisturbance, 
                                          yaw, yawDisturbance, posX, posY);
      double rollInput = rpyvInputs[0];
      double pitchInput = rpyvInputs[1];
      double yawInput = rpyvInputs[2];
      double verticalInput = rpyvInputs[3];
      
      // Actuate the motors taking into consideration all the computed inputs.
      activateActuators(verticalInput, rollInput, pitchInput, yawInput);
    }
    try {
      positionCsvWriter.close();
      inputsCsvWriter.close();
    } catch (IOException e) {
      System.err.println("Error closing CsvWriter: " + e.getMessage());
    }
  }
  
  public static void main(String[] args) {
    DroneController droneController = new DroneController();
    droneController.run();
  }
}