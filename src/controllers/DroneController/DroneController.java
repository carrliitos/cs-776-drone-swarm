import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;

public class DroneController extends Robot {
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds
  private static final double MAX_VELOCITY = 100;
  private static final double TARGET_ALTITUDE = 1.0;
  private static final double TARGET_X = 0.0;
  private static final double TARGET_Z = 0.0;
  private static final double TARGET_YAW = -1;

  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
  private Motor motors[];
  private LED frontLeftLED, frontRightLED;
  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  private CsvWriter positionCsvWriter;
  private CsvWriter inputsCsvWriter;
    
  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_VERTICAL_OFFSET = 0.2; // Vertical offset where the robot actually targets to stabilize itself.
  private static final double K_VERTICAL_P = 2.0; // P constant of the vertical PID.
  private static final double K_ROLL_P = 20.0; // P constant of the roll PID.
  private static final double K_PITCH_P = 22.5; // P constant of the pitch PID.
  
  public DroneController() {
    super();
    initializeDevices();
  }
  
  private void initializeDevices() {
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
    positionCsvWriter = new CsvWriter("../data/positions_data.csv");
    inputsCsvWriter = new CsvWriter("../data/inputs_data.csv");
  }
  
  private void displayWelcomeMessage() {
    System.out.println("Starting the drone...");
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
      final double altitude = gps.getValues()[2];

      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];

      double[] positions = { roll, pitch, altitude, rollVelocity, pitchVelocity };
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
                                 double pitch, double pitchVelocity, double pitchDisturbance, double yawDisturbance) {

    final double pitchInput = K_PITCH_P * Math.min(Math.max(pitch, -1.0), 1.0) + pitchVelocity + pitchDisturbance;
    final double rollInput = K_ROLL_P * Math.min(Math.max(roll, -1.0), 1.0) + rollVelocity + rollDisturbance;
    final double yawInput = yawDisturbance;
    final double clampedDiffAltitude = Math.min(Math.max(TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET, -1.0), 1.0);
    final double verticalInput = K_VERTICAL_P * Math.pow(clampedDiffAltitude, 3);
    
    double[] allInputs = { rollInput, pitchInput, yawInput, verticalInput };
    inputsCsvWriter.writeData(allInputs);

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
    String[] positionHeaders = { "currRoll", "currPitch", "currYaw", "currAltitude", "currRollVelocity", 
                                 "currPitchVelocity", "currPositionX", "currPositionz" };
    String[] inputHeaders = { "roll", "pitch", "yaw", "vertical" };
    positionCsvWriter.writeHeaders(positionHeaders);
    inputsCsvWriter.writeHeaders(inputHeaders);
    
    while (step(TIME_STEP) != -1) {
      double rollDisturbance = 0.0;
      double pitchDisturbance = 0.0;
      double yawDisturbance = 0.0;

      double[] robotState = getRobotState();
      double roll = robotState[0];
      double pitch = robotState[1];
      double altitude = robotState[2];
      double rollVelocity = robotState[3];
      double pitchVelocity = robotState[4];
      
      System.out.printf("Roll Current State: %.8f %n", roll);
      System.out.printf("Pitch Current State: %.8f %n", pitch);
      System.out.printf("Roll Velocity Current State: %.8f %n", rollVelocity);
      System.out.printf("Pitch Velocity Current State: %.8f %n", pitchVelocity);
      System.out.printf("Current Altitude: %.8f %n", altitude);

      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(rollVelocity, pitchVelocity);

      // Compute the roll, pitch, yaw and vertical inputs.
      double[] rpyvInputs = computeInputs(roll, altitude, rollVelocity, rollDisturbance, 
                                          pitch, pitchVelocity, pitchDisturbance, yawDisturbance);
      double rollInput = rpyvInputs[0];
      double pitchInput = rpyvInputs[1];
      double yawInput = rpyvInputs[2];
      double verticalInput = rpyvInputs[3];
      
      System.out.printf("Roll Input: %.8f %n", rollInput);
      System.out.printf("Pitch Input: %.8f %n", pitchInput);
      System.out.printf("Yaw Input: %.8f %n", yawInput);
      System.out.printf("Vertical Input: %.8f %n", verticalInput);

      // Actuate the motors taking into consideration all the computed inputs.
      activateActuators(verticalInput, rollInput, pitchInput, yawInput);
      System.out.println("==============================");
    }
    positionCsvWriter.close();
    inputsCsvWriter.close();
  }
  
  public static void main(String[] args) {
    DroneController droneController = new DroneController();
    droneController.run();
  }
}
