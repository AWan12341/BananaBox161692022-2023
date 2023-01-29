package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.symbiotes.vision.CombinedDetectorHandler;
import org.firstinspires.ftc.teamcode.symbiotes.vision.utility.TrackType;

@Autonomous
public class testing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CombinedDetectorHandler vision = new CombinedDetectorHandler(hardwareMap, "webcam 1", "webcam 2", "webcam 3");
        vision.init();
        vision.setTrackingTypes(TrackType.SLEEVE, TrackType.POLE, TrackType.POLE);

        telemetry.addData("Waiting for Start", "");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Pole Parallax: ", vision.getPoleParallax());
//            telemetry.addData("April Tag ID: ", vision.getAprilTagPipeOutput());
            telemetry.update();
        }
    }
}
