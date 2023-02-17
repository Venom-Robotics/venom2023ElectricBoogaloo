package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Venom's [Trash] Custom PID Class
 */
public class PIDController {
    private final double K_p;
    private final double K_i;
    private final double K_d;
    private final ElapsedTime timer;

    private double integralSum = 0;
    private double lastError = 0;
    private double error;
    private double derivative;

    /**
     * Construct PID Controller
     *
     * @param K_p Kp Constant
     * @param K_i Ki Constant
     * @param K_d Kd Constant
     */
    public PIDController(double K_p, double K_i, double K_d) {
        this.K_p = K_p;
        this.K_i = K_i;
        this.K_d = K_d;
        this.timer = new ElapsedTime();
    }

    /**
     * Resets the Timer; Should be Called Before PID Loop
     */
    public void startController() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    /**
     * Should be called once per loop.
     *
     * @param current_position Current Position in Ticks
     * @param target_position  Target Position in Ticks
     * @return Motor Power in Power Unit
     */
    public double updateController(double current_position, double target_position) {
        // Calculate Error
        error = target_position - current_position;

        // Calculate D and I Sum
        derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        // Reset/Update Variables Before Return
        lastError = error;
        timer.reset();

        return (K_p * error) + (K_i * integralSum) + (K_d * derivative);
    }
}
