import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;

/**
 * PID Reading: https://oscarliang.com/pid/
 **/
public class DroneController extends Robot {
  private static final int TIME_STEP = 64; // Simulation time step in milliseconds
  // private static final double MAX_VELOCITY = 100;
  private static final double TARGET_ALTITUDE = 1.0; // 1 meter
  private static final double TARGET_X_POSITION = 0.0;
  private static final double TARGET_Z_POSITION = 0.0;
  private static final double TARGET_YAW = -1;

  private Motor frontRightPropeller, frontLeftPropeller, rearRightPropeller, rearLeftPropeller, cameraRollMotor, cameraPitchMotor;
  private Motor motors[];
  private LED frontLeftLED, frontRightLED;
  private InertialUnit imu;
  private GPS gps;
  private Gyro gyro;
  private Clamp clamp;

  private double last_error_x = 0.0;
  private double last_error_z = 0.0;
  private double last_error_altitude = 0.0;
  private double last_error_yaw = 0.0;
  
  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_VERTICAL_OFFSET = 0.2; // Vertical offset where the robot actually targets to stabilize itself.
  private static final double K_VERTICAL_P = 3.0; // P constant of the vertical PID.
  private static final double K_ROLL_P = 50.0; // P constant of the roll PID.
  private static final double K_PITCH_P = 30.0; // P constant of the pitch PID.
  
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
      motor.setVelocity(1);
     }
  
    cameraRollMotor = getMotor("camera roll");
    cameraPitchMotor = getMotor("camera pitch");
    cameraPitchMotor.setPosition(0.7);
    
    clamp = new Clamp();
  }
  
  private void displayWelcomeMessage() {
    System.out.println("Starting the drone...");
    while (step(TIME_STEP) != -1) {
      if (getTime() > 1.0) { break; }
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
      
      final double gps_position_x = gps.getValues()[0];
      final double gps_position_y = gps.getValues()[1]; // Altitude
      final double gps_position_z = gps.getValues()[2];
      
      final double rollAcceleration = gyro.getValues()[0];
      final double pitchAcceleration = gyro.getValues()[1];

      return new double[]{roll, pitch, yaw, gps_position_x, gps_position_y, gps_position_z, rollAcceleration, pitchAcceleration};
    } catch (Exception e) {
      System.err.println("Error retrieving robot state: " + e.getMessage());
      return new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Default values
    }
  }
  
  private void stabilizeCamera(double rollAcceleration, double pitchAcceleration) {
    cameraRollMotor.setPosition(-0.115 * rollAcceleration);
    cameraPitchMotor.setPosition(-0.1 * pitchAcceleration);
  }
  
  private double[] computeInputs(double roll, double altitude, double rollAcceleration, double rollDisturbance,
                                  double pitch, double pitchAcceleration, double pitchDisturbance, double yaw, 
                                  double yawDisturbance, double kp_x, double kd_x, double kp_z, double kd_z,
                                  double kp_yaw, double kd_yaw, double kp_altitude, double kd_altitude,
                                  double gps_position_x, double gps_position_z) {
    // PID Control for x position
    clamp.setValue(TARGET_X_POSITION + gps_position_x, -1.0, 1.0);
    double errorX = clamp.getValue();
    double d_ErrorX = errorX - last_error_x;
    last_error_x = errorX;
    clamp.setValue(pitch, -1.0, 1.0);
    double clampedPitch = clamp.getValue();
    double initialPitchInput = (K_PITCH_P * clampedPitch) + 5.0 * pitchAcceleration + pitchDisturbance;
    final double pitchInput = initialPitchInput + (kp_x  * errorX) + (kd_x * d_ErrorX);

    // PID Control for z position
    clamp.setValue(TARGET_Z_POSITION + gps_position_z, -1.0, 1.0);
    double errorZ = clamp.getValue();
    double d_ErrorZ = errorZ - last_error_z;
    last_error_z = errorZ;
    clamp.setValue(roll, -1.0, 1.0);
    double clampedRoll = clamp.getValue();
    double initialRollInput = (K_ROLL_P * clampedRoll) - 5.0 * rollAcceleration + rollDisturbance;
    final double rollInput = initialRollInput + (kp_z * errorZ) + (kd_z * d_ErrorZ);

    // PID Control for altitude position
    clamp.setValue(TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET, -1.0, 1.0);
    double clampedDifferenceAltitude = clamp.getValue();
    double errorAltitude = clampedDifferenceAltitude;
    double d_ErrorAltitude = errorAltitude - last_error_altitude;
    last_error_altitude = errorAltitude;
    final double verticalInput = (kp_altitude * errorAltitude) + (kd_altitude * d_ErrorAltitude);

    // PID Control for rotation
    double errorYaw = TARGET_YAW - yaw;
    double d_ErrorYaw = errorYaw - last_error_yaw;
    last_error_yaw = errorYaw;
    clamp.setValue(errorYaw, -1.0, 1.0);
    double clampedErrorYaw = clamp.getValue();
    final double yawInput = (kp_yaw * clampedErrorYaw) + (kd_yaw * d_ErrorYaw);

    return new double[] { rollInput, pitchInput, yawInput, verticalInput };
  }
  
  private void activateActuators(double verticalInput, double rollInput, double pitchInput, double yawInput) {
    final double frontLeftPropellerInput = K_VERTICAL_THRUST + verticalInput - yawInput + pitchInput - rollInput;
    final double frontRightPropellerInput = K_VERTICAL_THRUST + verticalInput + yawInput + pitchInput + rollInput;
    final double rearLeftPropellerInput = K_VERTICAL_THRUST + verticalInput - yawInput - pitchInput - rollInput;
    final double rearRightPropellerInput = K_VERTICAL_THRUST + verticalInput - yawInput - pitchInput + rollInput;
  
    frontLeftPropeller.setVelocity(frontLeftPropellerInput);
    frontRightPropeller.setVelocity(-frontRightPropellerInput);
    rearLeftPropeller.setVelocity(-rearLeftPropellerInput);
    rearRightPropeller.setVelocity(rearRightPropellerInput);
    
    System.out.println("frontLeftPropeller velocity: " + frontLeftPropeller.getVelocity());
    System.out.println("frontRightPropeller velocity: " + frontRightPropeller.getVelocity());
    System.out.println("rearLeftPropeller velocity: " + rearLeftPropeller.getVelocity());
    System.out.println("rearRightPropeller velocity: " + rearRightPropeller.getVelocity());
    System.out.println("==========");
  }
  
  // Main control loop
  public void run() {
    double rollDisturbance = 0.0;
    double pitchDisturbance = 0.0;
    double yawDisturbance = 0.0;
    double kp_x = 10;
    double kd_x = 10;
    double kp_z = 10;
    double kd_z = 10;
    double kp_altitude = 3;
    double kd_altitude = 10;
    double kp_yaw = 1;
    double kd_yaw = 1;
    
    displayWelcomeMessage();
    while (step(TIME_STEP) != -1) {
      double[] robotState = getRobotState();
      double roll = robotState[0];
      double pitch = robotState[1];
      double yaw = robotState[2];
      double gps_position_x = robotState[3];
      double gps_position_y = robotState[4]; // Altitude
      double gps_position_z = robotState[5];
      double rollAcceleration = robotState[6];
      double pitchAcceleration = robotState[7];

      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(rollAcceleration, pitchAcceleration);

      // Compute the roll, pitch, yaw and vertical inputs.
      double[] rpyvInputs = computeInputs(roll, gps_position_y, rollAcceleration, rollDisturbance, pitch, 
                                          pitchAcceleration, pitchDisturbance, yaw, yawDisturbance, 
                                          kp_x, kd_x, kp_z, kd_z, kp_yaw, kd_yaw, kp_altitude, kd_altitude,
                                          gps_position_x, gps_position_z);
      double rollInput = rpyvInputs[0];
      double pitchInput = rpyvInputs[1];
      double yawInput = rpyvInputs[2];
      double verticalInput = rpyvInputs[3];

      // Actuate the motors taking into consideration all the computed inputs.
      activateActuators(verticalInput, rollInput, pitchInput, yawInput);
    }
  }
  
  public static void main(String[] args) {
    DroneController droneController = new DroneController();
    droneController.run();
  }
}
