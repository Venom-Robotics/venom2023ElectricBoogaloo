package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    private WebcamName webcamName       = null;

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AUNQPmr/////AAABmdbAYr/hPEDeoUk1ceK1SRYSEqDeKKjgLQJl4VvddHRtL3DHGXxCmEERDRwQgxDUrDX4PtnVhKc8TDo2HlSfdcPqW9eFzRzkx0nChdSk340Iq0QkgGQC4a/WYO8IVmpc5emf8Xguwzoyo0nEnO9ROhAgm7gc/XU/thB2F4qc0/zdkAKvV06XgKXrTl5s1eN2PxdVqffTLKOcIpwMcTjMT0zpnIURZDaGkd58nX+HdapxNJdclF4xNpTPHT+SMJYXu102kyOdvgraR890V0HkuRk8/LgEjJvSaBrFDx2avFqcjOFrksoPan1gSSRDrM3cpAb3sCDDfL/OlmzfB8ZDdsJyl0JyRf6uIIgAWfuJtZcL ";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.cameraName = webcamName;
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
//        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}