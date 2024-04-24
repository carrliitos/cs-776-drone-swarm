import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.InertialUnit;
import com.cyberbotics.webots.controller.GPS;
import com.cyberbotics.webots.controller.Gyro;

import java.io.IOException;

public class DroneController extends Robot {
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
    
  // Constants
  private static final double K_VERTICAL_THRUST = 68.5; // with this thrust, the drone lifts.
  private static final double K_VERTICAL_OFFSET = 0.6; // Vertical offset where the robot actually targets to stabilize itself.
  private static final double K_VERTICAL_P = 2.0; // P constant of the vertical PID.
  private static final double K_POSITION_P = 0.05; // P constant of the position PID.
  private static final double K_ROLL_P = 22.0; // P constant of the roll PID.
  private static final double K_PITCH_P = 17.0; // P constant of the pitch PID.
  
  public DroneController() {
    super();
    initializeDevices();
  }
  
  public static double clamp(double value, double low, double high) {
    return value < low ? low : (value > high ? high : value);
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
  }
  
  private void displayWelcomeMessage() {
    // Wait one second
    double previousTime = 0.0;
    while (step(TIME_STEP) != -1) {
      if (getTime() - previousTime > 3.0) { break; }
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
      final double posX = gps.getValues()[0];
      final double posY = gps.getValues()[1];
      final double altitude = gps.getValues()[2];
      final double rollVelocity = gyro.getValues()[0];
      final double pitchVelocity = gyro.getValues()[1];
      double[] positions = { roll, pitch, posX, posY, altitude, rollVelocity, pitchVelocity };

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
                                 double pitch, double pitchVelocity, double pitchDisturbance, double yawDisturbance,
                                 double xPos, double yPos) {

    double pitchInput = K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitchVelocity + pitchDisturbance;
    double rollInput = K_ROLL_P * clamp(roll, -1.0, 1.0) + rollVelocity + rollDisturbance;
    final double yawInput = yawDisturbance;
    
    // Calculate position error from the target position (0, 0)
    double errorX = TARGET_X - xPos;
    double errorY = TARGET_Y - yPos;
    
    // Calculate the velocity adjustments based on the errors
    double velocityX = K_POSITION_P * errorX;
    double velocityY = K_POSITION_P * errorY;

    final double clampedDiffAltitude = clamp(TARGET_ALTITUDE - altitude + K_VERTICAL_OFFSET, -1.0, 1.0);
    final double verticalInput = K_VERTICAL_P * Math.pow(clampedDiffAltitude, 3);
    
    // Apply the position control adjustments to the pitch and roll inputs
    pitchInput += velocityY;
    rollInput += velocityX;
    
    double[] allInputs = { rollInput, pitchInput, yawInput, verticalInput };

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
    
    while (step(TIME_STEP) != -1) {
      double rollDisturbance = 0.0;
      double pitchDisturbance = 0.0;
      double yawDisturbance = 0.0;

      double[] robotState = getRobotState();
      double roll = robotState[0];
      double pitch = robotState[1];
      double posX = robotState[2];
      double posY = robotState[3];
      double altitude = robotState[4];
      double rollVelocity = robotState[5];
      double pitchVelocity = robotState[6];

      // Blink the front LEDs alternatively with a 1 second rate.
      blinkLEDS();

      // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
      stabilizeCamera(rollVelocity, pitchVelocity);

      // Compute the roll, pitch, yaw and vertical inputs.
      double[] rpyvInputs = computeInputs(roll, altitude, rollVelocity, rollDisturbance, 
                                          pitch, pitchVelocity, pitchDisturbance, yawDisturbance,
                                          posX, posY);
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