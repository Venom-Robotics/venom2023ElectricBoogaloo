package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Class that Declares and Initializes Robot Motors to Simplify Access
 */
public class Robot {
    // Declare Drivetrain Members
    public DcMotorEx topLeftMotor;
    public DcMotorEx topRightMotor;
    public DcMotorEx bottomLeftMotor;
    public DcMotorEx bottomRightMotor;
    public DcMotorEx[] driveMotors;

    public DcMotorEx topLiftMotor;
    public DcMotorEx bottomLiftMotor;
    public DcMotorEx turretMotor;
    public DcMotorEx armMotor;

    public ServoImplEx clawServo;

    public List<LynxModule> allHubs;
    public BNO055IMU imu;

    // Create HardwareMap
    HardwareMap hwMap;

    // Initialize standard Hardware interfaces
    public void init(HardwareMap _hardwareMap) {
        // Save Reference to Hardware Map
        hwMap = _hardwareMap;

        // Define and Initialize Motors
        topLeftMotor = hwMap.get(DcMotorEx.class, "front_left_motor");
        topRightMotor = hwMap.get(DcMotorEx.class, "front_right_motor");
        bottomLeftMotor = hwMap.get(DcMotorEx.class, "back_left_motor");
        bottomRightMotor = hwMap.get(DcMotorEx.class, "back_right_motor");

        topLiftMotor = hwMap.get(DcMotorEx.class, "top_lift_motor");
        bottomLiftMotor = hwMap.get(DcMotorEx.class, "bottom_lift_motor");
        turretMotor = hwMap.get(DcMotorEx.class, "turret_motor");
        armMotor = hwMap.get(DcMotorEx.class, "arm_motor");

        clawServo = hwMap.get(ServoImplEx.class, "claw_servo");

        // Set Hubs to Bulk Read
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        // Widen Servo Ranges
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        rotationServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Set Motor Directions
        topLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topRightMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomRightMotor.setDirection(DcMotor.Direction.REVERSE);


        // Reset all Motor Powers
        topLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        bottomLeftMotor.setPower(0);
        bottomRightMotor.setPower(0);

        topLiftMotor.setPower(0);
        bottomLiftMotor.setPower(0);
        turretMotor.setPower(0);
        armMotor.setPower(0);

        // Set Motors to Brake
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Motor Encoders
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motors' Encoder Status
        topLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Assign Drive Motors to Array
        driveMotors = new DcMotorEx[]{topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor};
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}