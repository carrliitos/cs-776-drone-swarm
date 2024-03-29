/**
 * A PID controller for yaw control in a drone.
 */
public class YawController {
  private final double yaw_Kp;
  private final double yaw_Ki;
  private final double yaw_Kd;

  private double yawIntegral = 0.0;
  private double previousYawError = 0.0;

  /**
   * Constructs a new YawController with the given PID constants.
   *
   * @param yaw_pid an array containing the PID constants [Kp, Ki, Kd]
   */
  public YawController(double[] yaw_pid) {
    this.yaw_Kp = yaw_pid[0];
    this.yaw_Ki = yaw_pid[1];
    this.yaw_Kd = yaw_pid[2];
  }

  /**
   * Calculates the PID output for yaw control based on the current yaw angle and the setpoint.
   *
   * @param currentYawAngle the current yaw angle of the drone
   * @param setpoint the desired yaw angle (setpoint)
   * @return the PID output for yaw control
   */
  public double calculateYawPID(double currentYawAngle, double setpoint) {
    // Calculate the error
    double yawError = setpoint - currentYawAngle;

    // Calculate the proportional term
    double P = yaw_Kp * yawError;

    // Calculate the integral term
    yawIntegral += yawError;
    double I = yaw_Ki * yawIntegral;

    // Calculate the derivative term
    double yawDerivative = yawError - previousYawError;
    double D = yaw_Kd * yawDerivative;

    // Update the previous error
    previousYawError = yawError;

    // Calculate the PID output
    double yawPID = P + I + D;

    return yawPID;
  }
}
