import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.Compass;

public class DroneController extends Robot {
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds

  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
  private Motor motors[];
  private LED frontLeftLED, frontRightLED;
  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  private Compass compass;
  private CsvWriter positionCsvWriter;
  private CsvWriter inputsCsvWriter;
  private PID pitchPID;
  private PID rollPID;
  private PID throttlePID;
  private PID yawPID;
    
  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_ROLL_P = 0.0;
  private static final double K_PITCH_P = 0.0;
  private static final double TARGET_X = 0.0;
  private static final double TARGET_Y = 0.0;
  private static final double TARGET_ALTITUDE = 1.0;

  private static final double pitchKp = 0.0;
  private static final double pitchKi = 0.0;
  private static final double pitchKd = 0.0;

  private static final double rollKp = 0.0;
  private static final double rollKi = 0.0;
  private static final double rollKd = 0.0;

  private static final double throttleKp = 0.0;
  private static final double throttleKi = 0.0;
  private static final double throttleKd = 0.0;

  private static final double yawKp = 0.0;
  private static final double yawKi = 0.0;
  private static final double yawKd = 0.0;
  private static final double yawSetpoint = 0.0;

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
    compass = getCompass("compass");
    compass.enable(TIME_STEP);
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
      motor.setVelocity(1);
    }
    
    cameraRollMotor = getMotor("camera roll");
    cameraPitchMotor = getMotor("camera pitch");
    positionCsvWriter = new CsvWriter("../data/positions_data.csv");
    inputsCsvWriter = new CsvWriter("../data/inputs_data.csv");
    pitchPID = new PID(pitchKp, pitchKi, pitchKd, 0.0);
    rollPID = new PID(rollKp, rollKi, rollKd, 0.0);
    throttlePID = new PID(throttleKp, throttleKi, throttleKd, 0.0);
    yawPID = new PID(yawKp, yawKi, yawKd, yawSetpoint);
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
      final double yaw = imu.getRollPitchYaw()[2];

      final double xGPS = gps.getValues()[0];
      final double yGPS = gps.getValues()[1];
      final double zGPS = gps.getValues()[2];

      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];

      double[] positions = { roll, pitch, yaw, xGPS, yGPS, zGPS, rollVelocity, pitchVelocity };
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

  private double[] computeInputs(double roll, double rollVelocity, double pitch, double pitchVelocity, 
                                 double yaw, double xGPS, double yGPS, double zGPS) {
    double verticalInput = throttlePID.update(zGPS, 0.0);
    double yawInput = yawPID.update(yaw, 0.0);

    rollPID.setPoint(TARGET_X);
    pitchPID.setPoint(TARGET_Y);

    double rollInput = K_ROLL_P * roll + rollVelocity + rollPID.update(xGPS, 0.0);
    double pitchInput = K_PITCH_P * pitch + pitchVelocity - pitchPID.update(-yGPS, 0.0);

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
    String[] positionHeaders = { "ROLL", "PITCH", "YAW", "GPS_X", "GPS_Y", "GPS_Z", "ROLL_VELOCITY", "PITCH_VELOCITY" };
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
      double xGPS = robotState[3];
      double yGPS = robotState[4];
      double zGPS = robotState[5];
      double rollVelocity = robotState[6];
      double pitchVelocity = robotState[7];
      
      System.out.printf("Roll Current State: %.8f %n", roll);
      System.out.printf("Pitch Current State: %.8f %n", pitch);
      System.out.printf("Roll Velocity Current State: %.8f %n", rollVelocity);
      System.out.printf("Pitch Velocity Current State: %.8f %n", pitchVelocity);

      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(rollVelocity, pitchVelocity);

      // Compute the roll, pitch, yaw and vertical inputs.
      double[] rpyvInputs = computeInputs(roll, rollVelocity, pitch, pitchVelocity, yaw, xGPS, yGPS, zGPS);
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
