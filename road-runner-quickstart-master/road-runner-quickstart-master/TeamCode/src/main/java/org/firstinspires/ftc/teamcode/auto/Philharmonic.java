package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.symbiotes.ViolaStandardSymbiote;

@Autonomous(group = "harmony")
public class Philharmonic extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {



        //Set up the drive train
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set up the symbiote
        ViolaStandardSymbiote symbiote = new ViolaStandardSymbiote();
        symbiote.setup(drive);
        symbiote.setAutomaticMode(true);

        Pose2d startPose = new Pose2d(35, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Generate trajectories for Blue Left
        Trajectory traj1BlueLeft = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(35, 5, Math.toRadians(10)))



                .build();

        Trajectory traj1BlueLeft1 = drive.trajectoryBuilder(traj1BlueLeft.end())
                .lineToLinearHeading(new Pose2d(35, 10, Math.toRadians(10)))
                .build();

        Trajectory traj1BlueLeft2 = drive.trajectoryBuilder(traj1BlueLeft1.end())
                .lineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(-190)))
                .build();

        Trajectory traj1BlueLeft3 = drive.trajectoryBuilder(traj1BlueLeft2.end())
                .lineToLinearHeading(new Pose2d(-35, 15, Math.toRadians(-190)))
                .build();

        telemetry.addData("Waiting for Start", "");
        telemetry.update();

        waitForStart();

        drive.followTrajectory(traj1BlueLeft);

        symbiote.updateAutonomousViolaSymbiote(3, 1); //Deliver the preload

        symbiote.updateAutonomousViolaSymbiote(1, 4);
        symbiote.updateAutonomousViolaSymbiote(2, 4);

        symbiote.updateAutonomousViolaSymbiote(1, 3);
        symbiote.updateAutonomousViolaSymbiote(2, 3);

        symbiote.updateAutonomousViolaSymbiote(1, 2);
        symbiote.updateAutonomousViolaSymbiote(2, 2);

        symbiote.updateAutonomousViolaSymbiote(1, 1);
        symbiote.updateAutonomousViolaSymbiote(2, 1);

        symbiote.updateAutonomousViolaSymbiote(4, 1); //Send all appendages to rest
//
        drive.followTrajectory(traj1BlueLeft1);
        drive.followTrajectory(traj1BlueLeft2);
//        drive.followTrajectory(traj1BlueLeft3);

    }
}


