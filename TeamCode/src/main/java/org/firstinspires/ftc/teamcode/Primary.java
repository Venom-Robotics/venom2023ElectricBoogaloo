package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.helper.Constants.Drivetrain.Calculations;
import org.firstinspires.ftc.teamcode.helper.PIDController;
import org.firstinspires.ftc.teamcode.helper.Robot;
import org.firstinspires.ftc.teamcode.helper.Utils;

@Config
@TeleOp(name = "Primary")
public class Primary extends OpMode {
    // Declare OpMode members
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors
    private ElapsedTime runtime = new ElapsedTime();
    PIDController turretController = new PIDController(0.02, 0, 0);

    double[] drivetrainMotors;

    double negative_left_stick_y;
    double negative_right_stick_y;
    double half_left_stick_x;
    double half_right_stick_x;
    double dpad_up_down;
    double dpad_right;
    double dpad_left;
    double topLeftPower;
    double topRightPower;
    double bottomLeftPower;
    double bottomRightPower;

    public static volatile double K_arm = 0.00028;

    boolean last_left = false;
    boolean last_up = false;
    boolean last_right = false;
    boolean last_down = false;
    boolean last_x = false;
    boolean last_y = false;
    boolean last_b = false;
    boolean last_a = false;

    boolean running_to_turret_preset = false;
    boolean running_to_arm_preset = false;

    double turretTarget = 0;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize Robot
        robot.init(hardwareMap);
        runtime.reset();

        // Initialize Telemetry
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Update Telemetry to Reflect that Initialization is Complete
        telemetry.addData("Status", Utils.fontText("Initialized!", "green"));
        telemetry.addLine(Utils.fontText("Waiting for Start...", "blue", "+3"));
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        int armpos = robot.armMotor.getCurrentPosition();
        int turretpos = robot.turretMotor.getCurrentPosition();
        double pow = -gamepad2.right_stick_y/2 + (K_arm * armpos * 9.81);

        // ----- Controller 1 -----

        // Drivetrain
        drivetrainMotors = calculateDrivetrain();
        robot.topLeftMotor.setPower(drivetrainMotors[0]);
        robot.topRightMotor.setPower(drivetrainMotors[1]);
        robot.bottomLeftMotor.setPower(drivetrainMotors[2]);
        robot.bottomRightMotor.setPower(drivetrainMotors[3]);

        robot.clawServo.setPosition(gamepad1.a ? 0.0 : 1.0);

        if (!last_left && gamepad2.dpad_left) {
            running_to_turret_preset = true;
            runToPosition(robot.turretMotor, Constants.Presets.Turret.LEFT, .75);
            turretTarget = Constants.Presets.Turret.LEFT;
        } else if (!last_up && gamepad2.dpad_up) {
            running_to_turret_preset = true;
            runToPosition(robot.turretMotor, Constants.Presets.Turret.FORWARD, .75);
            turretTarget = Constants.Presets.Turret.FORWARD;
        } else if (!last_right && gamepad2.dpad_right) {
            running_to_turret_preset = true;
            runToPosition(robot.turretMotor, Constants.Presets.Turret.RIGHT, .75);
            turretTarget = Constants.Presets.Turret.RIGHT;
        } else if (!last_down && gamepad2.dpad_down) {
            running_to_turret_preset = true;
            runToPosition(robot.turretMotor, Constants.Presets.Turret.BACK, .75);
            turretTarget = Constants.Presets.Turret.BACK;
        } else if (!last_x && gamepad2.x) {
            running_to_arm_preset = true;
            runToPosition(robot.bottomLiftMotor, Constants.Presets.Lift.LOW, .75);
            runToPosition(robot.topLiftMotor, Constants.Presets.Lift.LOW, .75);
            runToPosition(robot.armMotor, Constants.Presets.Arm.LOW, .75);
        } else if (!last_y && gamepad2.y) {
            running_to_arm_preset = true;
            runToPosition(robot.bottomLiftMotor, Constants.Presets.Lift.MED, .75);
            runToPosition(robot.topLiftMotor, Constants.Presets.Lift.MED, .75);
            runToPosition(robot.armMotor, Constants.Presets.Arm.MED, .75);
        } else if (!last_b && gamepad2.b) {
            running_to_arm_preset = true;
            runToPosition(robot.bottomLiftMotor, Constants.Presets.Lift.HIGH, .75);
            runToPosition(robot.topLiftMotor, Constants.Presets.Lift.HIGH, .75);
            runToPosition(robot.armMotor, Constants.Presets.Arm.HIGH, .75);
        } else if (!last_a && gamepad2.a) {
            running_to_arm_preset = true;
            runToPosition(robot.bottomLiftMotor, Constants.Presets.Lift.GROUND, .75);
            runToPosition(robot.topLiftMotor, Constants.Presets.Lift.GROUND, .75);
            runToPosition(robot.armMotor, Constants.Presets.Arm.GROUND, .75);
        }
        last_left = gamepad2.dpad_left;
        last_up = gamepad2.dpad_up;
        last_right = gamepad2.dpad_right;
        last_down = gamepad2.dpad_down;
        last_x = gamepad2.x;
        last_y = gamepad2.y;
        last_b = gamepad2.b;
        last_a = gamepad2.a;

        if (!turretIsBusy()) {
            resetTurret();
            running_to_turret_preset = false;
        }

        if (!armIsBusy()) {
            resetArm();
            running_to_arm_preset = false;
        }

        if (!running_to_turret_preset) {
            turretTarget += (gamepad2.left_trigger - gamepad2.right_trigger) * 6;
            robot.turretMotor.setPower(turretController.updateController(turretpos, turretTarget));
        }
        if (!running_to_arm_preset) {
            robot.topLiftMotor.setPower(-gamepad2.left_stick_y + 0.1);
            robot.bottomLiftMotor.setPower(-gamepad2.left_stick_y + 0.1);

            robot.armMotor.setPower(pow);
        }

        // Driver Station Telemetry
        telemetry.addData("Status", "<font color='purple' font-weight=\"bold\">Running...</font>");
        telemetry.addData("turretMotor", robot.turretMotor.getCurrentPosition());
        telemetry.addData("topLiftMotor", robot.topLiftMotor.getCurrentPosition());
        telemetry.addData("bottomLiftMotor", robot.bottomLiftMotor.getCurrentPosition());
        telemetry.addData("armpower", pow);
    }
    //  telemetry.addLine("Run Time: " + runtime.toString());

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Updates Robot Status
        telemetry.addData("Status", "<font color='red' font-weight=\"bold\">Stopped</font>");
        telemetry.update();
    }

    private double[] calculateDrivetrain() {
        // Pre-Calculated Stick and Button Values
        negative_left_stick_y = -gamepad1.left_stick_y / 2;
        negative_right_stick_y = -gamepad1.right_stick_y / 2;

        half_left_stick_x = gamepad1.left_stick_x / 2;
        half_right_stick_x = gamepad1.right_stick_x / 2;

        dpad_up_down = ((gamepad1.dpad_up) ? Calculations.DPAD_SPEED : 0) + ((gamepad1.dpad_down) ? -Calculations.DPAD_SPEED : 0);
        dpad_right = ((gamepad1.dpad_right) ? Calculations.DPAD_SPEED : 0);
        dpad_left = ((gamepad1.dpad_left) ? Calculations.DPAD_SPEED : 0);

        // Calculate Joystick Movement for Each Wheel (and Discreetly Clamp Each Value to a Range of (-1.0, 1.0))
        topLeftPower = negative_left_stick_y + half_left_stick_x + half_right_stick_x;
        topRightPower = negative_right_stick_y - half_left_stick_x - half_right_stick_x;
        bottomLeftPower = negative_left_stick_y - half_left_stick_x - half_right_stick_x;
        bottomRightPower = negative_right_stick_y + half_left_stick_x + half_right_stick_x;

        // Dpad Inputs
        topLeftPower += dpad_up_down + dpad_right - dpad_left;
        topRightPower += dpad_up_down - dpad_right + dpad_left;
        bottomLeftPower += dpad_up_down - dpad_right + dpad_left;
        bottomRightPower += dpad_up_down + dpad_right - dpad_left;

        // Trigger Rotation
        topLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        topRightPower += gamepad1.left_trigger - gamepad1.right_trigger;
        bottomLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        bottomRightPower += gamepad1.left_trigger - gamepad1.right_trigger;

        // Common Multiplier
        topLeftPower *= Calculations.TOTAL_MULTIPLIER;
        topRightPower *= Calculations.TOTAL_MULTIPLIER;
        bottomLeftPower *= Calculations.TOTAL_MULTIPLIER;
        bottomRightPower *= Calculations.TOTAL_MULTIPLIER;

        return new double[]{topLeftPower, topRightPower, bottomLeftPower, bottomRightPower};
    }

    private boolean turretIsBusy() {
        return robot.turretMotor.isBusy();
    }

    private boolean armIsBusy() {
        return robot.topLiftMotor.isBusy() || robot.bottomLiftMotor.isBusy() || robot.armMotor.isBusy();
    }

    private void runToPosition(DcMotorEx motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void resetTurret() {
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetArm() {
        robot.topLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bottomLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}