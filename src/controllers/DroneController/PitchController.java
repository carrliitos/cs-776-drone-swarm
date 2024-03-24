/**
 * A PID controller for pitch control in a drone.
 */
public class PitchController {
  private final double pitch_Kp;
  private final double pitch_Ki;
  private final double pitch_Kd;

  private double pitchIntegral = 0.0;
  private double previousPitchError = 0.0;

  /**
   * Constructs a new PitchController with the given PID constants.
   *
   * @param pitch_pid an array containing the PID constants [Kp, Ki, Kd]
   */
  public PitchController(double[] pitch_pid) {
    this.pitch_Kp = pitch_pid[0];
    this.pitch_Ki = pitch_pid[1];
    this.pitch_Kd = pitch_pid[2];
  }

  /**
   * Calculates the PID output for pitch control based on the current pitch angle and the setpoint.
   *
   * @param currentPitchAngle the current pitch angle of the drone
   * @param setpoint the desired pitch angle (setpoint)
   * @return the PID output for pitch control
   */
  public double calculatePitchPID(double currentPitchAngle, double setpoint) {
    // Calculate the error
    double pitchError = setpoint - currentPitchAngle;

    // Calculate the proportional term
    double P = pitch_Kp * pitchError;

    // Calculate the integral term
    pitchIntegral += pitchError;
    double I = pitch_Ki * pitchIntegral;

    // Calculate the derivative term
    double pitchDerivative = pitchError - previousPitchError;
    double D = pitch_Kd * pitchDerivative;

    // Update the previous error
    previousPitchError = pitchError;

    // Calculate the PID output
    double pitchPID = P + I + D;

    return pitchPID;
  }
}
