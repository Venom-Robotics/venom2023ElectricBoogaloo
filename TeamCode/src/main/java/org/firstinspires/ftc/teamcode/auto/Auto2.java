/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helper.Constants;
import org.firstinspires.ftc.teamcode.helper.PIDController;
import org.firstinspires.ftc.teamcode.helper.Robot;

@Config
@Autonomous(name="_Auto2")
public class Auto2 extends LinearOpMode {

    // Declare OpMode members.
    Robot robot = new Robot();
    PIDController driveController = new PIDController(0.003, 0.000001, 0);
    PIDController turnController = new PIDController(0.03, 0.00001, 0);
    double delta_inches = 0.0;
    double delta_position = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        dashboard.setTelemetryTransmissionInterval(25);

        packet.put("calculated_power", 0.0);
        packet.put("target_position", (int) (Constants.Drivetrain.COUNTS_PER_INCH * delta_inches));
        packet.put("current_position", (int) ((robot.topLeftMotor.getCurrentPosition() + robot.bottomRightMotor.getCurrentPosition()) / 2.0));
        dashboard.sendTelemetryPacket(packet);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        packet.put("calculated_power", 0.0);
        packet.put("target_position", (int) (Constants.Drivetrain.COUNTS_PER_INCH * delta_inches));
        packet.put("current_position", (int) ((robot.topLeftMotor.getCurrentPosition() + robot.bottomRightMotor.getCurrentPosition()) / 2.0));
        dashboard.sendTelemetryPacket(packet);


//        moveToPosition(44.0);
//        sleep(500);
//        turnToPosition(0);
//        sleep(500);
//        turnToPosition(-87);
//        sleep(500);
//        moveToPosition(13.0);

        moveToPosition(8);
        sleep(500);
        turnToPosition(-90);
        sleep(1000);

    }

    private void moveToPosition(double target) {
        delta_position += target;
        // Calculate Target Position in Ticks and Initialize calculated_power for Loop
        int target_position = (int) (Constants.Drivetrain.COUNTS_PER_INCH * delta_position);
        double calculated_power;
        int position = (int) ((robot.topLeftMotor.getCurrentPosition() + robot.bottomRightMotor.getCurrentPosition()) / 2.0);

        // PID Loop for Movement
        driveController.startController();
        while (opModeIsActive() && (Math.abs(target_position - position) > 3)) {
            calculated_power = driveController.updateController(position, target_position);

            robot.topLeftMotor.setPower(calculated_power);
            robot.topRightMotor.setPower(calculated_power);
            robot.bottomLeftMotor.setPower(calculated_power);
            robot.bottomRightMotor.setPower(calculated_power);

            packet.put("calculated_power", calculated_power);
            packet.put("target_position", target_position);
            packet.put("current_position", position);
            dashboard.sendTelemetryPacket(packet);

            position = (int) ((robot.topLeftMotor.getCurrentPosition() + robot.bottomRightMotor.getCurrentPosition()) / 2.0);
        }
        robot.topLeftMotor.setPower(0);
        robot.topRightMotor.setPower(0);
        robot.bottomLeftMotor.setPower(0);
        robot.bottomRightMotor.setPower(0);
    }

    private void turnToPosition(double angle) {

            // Calculate Target Position in Ticks and Initialize calculated_power for Loop
            double calculated_power;
            double position = robot.getHeading();

            // PID Loop for Movement
            turnController.startController();
            while (opModeIsActive() && (Math.abs(angle - position) > 3)) {
                calculated_power = turnController.updateController(position, angle);

                robot.topLeftMotor.setPower(-calculated_power);
                robot.topRightMotor.setPower(calculated_power);
                robot.bottomLeftMotor.setPower(-calculated_power);
                robot.bottomRightMotor.setPower(calculated_power);

                packet.put("calculated_power", calculated_power);
                packet.put("target_angle", angle);
                packet.put("current_angle", position);
                dashboard.sendTelemetryPacket(packet);

                position = robot.getHeading();
            }
            robot.topLeftMotor.setPower(0);
            robot.topRightMotor.setPower(0);
            robot.bottomLeftMotor.setPower(0);
            robot.bottomRightMotor.setPower(0);
        }
}
