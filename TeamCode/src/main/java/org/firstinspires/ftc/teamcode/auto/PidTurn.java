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

import org.firstinspires.ftc.teamcode.helper.PIDController;
import org.firstinspires.ftc.teamcode.helper.Robot;

@Config
@Autonomous(name="PidTurn")
public class PidTurn extends LinearOpMode {

    // Declare OpMode members.
    Robot robot = new Robot();

    public static volatile double p_turn = 0.03; // Formerly 0.0004 // Increase by 0.0001 until achieved
    public static volatile double i_turn = 0.00001; // Formerly 0
    public static volatile double d_turn = 0.0; // 0.00004
    public static volatile double delta_angle = 90.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

        packet.put("calculated_power", 0.0);
        packet.put("target_angle", delta_angle);
        packet.put("current_angle", robot.getHeading());
        dashboard.sendTelemetryPacket(packet);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            PIDController controller = new PIDController(p_turn, i_turn, d_turn);

            // Calculate Target Position in Ticks and Initialize calculated_power for Loop
            double target_angle;
            double calculated_power;
            double position = robot.getHeading();

            // PID Loop for Movement
            controller.startController();
            while (opModeIsActive() && !gamepad1.a) {
                target_angle = delta_angle;
                calculated_power = controller.updateController(position, target_angle);

                robot.topLeftMotor.setPower(-calculated_power);
                robot.topRightMotor.setPower(calculated_power);
                robot.bottomLeftMotor.setPower(-calculated_power);
                robot.bottomRightMotor.setPower(calculated_power);

                packet.put("calculated_power", calculated_power);
                packet.put("target_angle", target_angle);
                packet.put("current_angle", position);
                dashboard.sendTelemetryPacket(packet);

                position = robot.getHeading();
            }
        }
    }
}
