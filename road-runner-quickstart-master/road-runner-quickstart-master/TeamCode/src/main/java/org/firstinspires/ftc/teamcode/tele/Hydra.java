package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.Gobiidae;
import org.firstinspires.ftc.teamcode.symbiotes.ViolaStandardSymbiote;
import org.firstinspires.ftc.teamcode.symbiotes.ZitherArrayBasedDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(group = "drive")
public class Hydra extends LinearOpMode {

    boolean switchSys = false;

    boolean automaticMode = false;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        ZitherArrayBasedDetection cameraSymbiote1 = new ZitherArrayBasedDetection(); //
        cameraSymbiote1.setup(hardwareMap);

        //Set up the drive train
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set up the symbiote
        ViolaStandardSymbiote symbiote = new ViolaStandardSymbiote();

        symbiote.setup(drive);

        double speedMod = .5;

        //Initialize FTC Lib
//        GamepadEx driver = new GamepadEx(gamepad1);
//        GamepadEx operator = new GamepadEx(gamepad2);
        telemetry.addData("Waiting for Start", "");
        telemetry.update();

//        symbiote.test();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            /** Update all symbiotes and drivers */
            drive.update();
            symbiote.updateViolaSymbiote();



            /** Driver Controls */
            if(gamepad1.x)
                speedMod = 1;
            else if(gamepad1.a)
                speedMod = .75;
            else if (gamepad1.b)
                speedMod = .5;
            else if (gamepad1.y)
                speedMod = .25;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speedMod,
                            -gamepad1.left_stick_x * speedMod,
                            -gamepad1.right_stick_x * speedMod
//                            ((cameraSymbiote1.getCell() - 25) * -.01)

                    )
            );

            //Automatic lift Controls
            if(automaticMode){
                if(cameraSymbiote1.getCell() == 25 && !symbiote.isBusy()) {
                    if(drive.forwardPoleSensor.getDistance(DistanceUnit.INCH) < 10 && drive.forwardPoleSensor.getDistance(DistanceUnit.INCH) > 5) {
                        symbiote.sendStatusRequest(0);
                    }
                }
            }

            //Manual Lift Controls
            if(!automaticMode){
                if(gamepad2.dpad_up && !symbiote.isBusy())
                    symbiote.sendStatusRequest(0);

                else if (gamepad2.dpad_right && !symbiote.isBusy())
                    symbiote.sendStatusRequest(1);

                else if (gamepad2.dpad_down && switchSys && clock1.milliseconds() > 300 && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(2);
                    switchSys = false;
                    clock1.reset();
                }

                else if (gamepad2.dpad_down && !switchSys && clock1.milliseconds() > 300 && !symbiote.isBusy()){
                    symbiote.sendStatusRequest(3);
                    switchSys = true;
                    clock1.reset();
                }

                else if(gamepad2.dpad_left && !symbiote.isBusy())
                    symbiote.sendStatusRequest(4);
            }



            /** Operator Controls */
            //Automatic Lift Controls/Overrides
            if(gamepad2.a){
                symbiote.setAutomaticMode(true);
                automaticMode = true;
            }
            else if(gamepad2.b){
                symbiote.setAutomaticMode(false);
                automaticMode = false;
            }






            /** Telemetry */
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Cell Pos: ", cameraSymbiote1.getCell());
            telemetry.addData("Forward Pole Sensor Distance: ", drive.forwardPoleSensor.getDistance(DistanceUnit.INCH));

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            telemetry.update();
        }
    }
}
