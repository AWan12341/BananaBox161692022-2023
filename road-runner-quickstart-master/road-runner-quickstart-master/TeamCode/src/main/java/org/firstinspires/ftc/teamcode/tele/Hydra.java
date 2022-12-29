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

//    OpenCvWebcam webcam;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//
//
//        webcam.setPipeline(new ZitherArrayBasedDetection());
//
//        /*
//         * Open the connection to the camera device. New in v1.4.0 is the ability
//         * to open the camera asynchronously, and this is now the recommended way
//         * to do it. The benefits of opening async include faster init time, and
//         * better behavior when pressing stop during init (i.e. less of a chance
//         * of tripping the stuck watchdog)
//         *
//         * If you really want to open synchronously, the old method is still available.
//         */
//        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                sleep(1000);
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        //Set up camera one. This is the camera that faces the pole.
//
//       ZitherArrayBasedDetection zitherPipeline1 = new ZitherArrayBasedDetection();

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

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * speedMod,
//                            -gamepad1.left_stick_x * speedMod,
//                            -gamepad1.right_stick_x * speedMod
//                    )
//            );

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * speedMod,
                            -gamepad1.left_stick_x * speedMod,
                            ((cameraSymbiote1.getCell() - 25) * -.02)

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
