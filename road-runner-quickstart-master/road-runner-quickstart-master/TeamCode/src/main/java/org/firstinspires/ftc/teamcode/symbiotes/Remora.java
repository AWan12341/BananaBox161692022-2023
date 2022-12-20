package org.firstinspires.ftc.teamcode.symbiotes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Remora {

    private SampleMecanumDrive drive;

    //Lift Positions
    private int liftRestPosition = 500;
    private int liftUpPosition = 4700 - liftRestPosition;

    //Gripper Positions
    private double gripperClosedPos = .1;
    private double gripperMidPos = .5;
    private double gripperOpenPos = .9;

    //Intake Extension Positions
    private double intakeExtendedPos = 1;
    private double intakeExtendedPos2 = 0;
    private double intakeRetractedPos = .8;
    private double intakeRetractedPos2 = .2;                                                                                                                 ;

    //Intake Wrist Positions
    private double intakeWristOutPos = .7;
    private double intakeWristInPos = .15;

    //Outtake wrist
    private double outtakeWristOutPos = 0;
    private double outtakeWristInPos = 1;

    //Outtake Claw
    private double outtakeClawOpenPos = 1;
    private double outtakeClawClosedPos = 0;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();



    public void setup(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable
        drive.setLiftTargetPosition(liftRestPosition, 1); //Setup the lifter
        drive.setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
    }

    public void updateAppendages(){

    }

    public void primeIntake(double speedMod, float lsy, float lsx, float rsx){
        drive.setIntakeGrippersPos(.5, .5); //close the gripper
        clock1.reset();
        while (clock1.milliseconds() < 400) {        }
        drive.setIntakeExtension(intakeExtendedPos, intakeExtendedPos2); //Extend the intake
        drive.setIntakeWristPos(intakeWristOutPos);//Lower the wrist
        drive.setIntakeGrippersPos(-gripperOpenPos, gripperOpenPos); //Open the grippers


    }

    public void grabAndPrep(double speedMod, float lsy, float lsx, float rsx){
        drive.setIntakeGrippersPos(.5, .5); //close the gripper
        clock1.reset(); //Reset the timer
        while(clock1.milliseconds() < 500){
        }
        drive.setIntakeWristPos(.4);//Lower the wrist
        drive.setIntakeExtension(intakeRetractedPos, intakeRetractedPos2); //Pull back the intake
        clock1.reset();
        while(clock1.milliseconds() < 500){
        }
        drive.setIntakeWristPos(intakeWristInPos); //Pull the wrist up
        clock1.reset();
        while(clock1.milliseconds() < 500) {
        }
        drive.setIntakeGrippersPos(-gripperOpenPos, gripperOpenPos); //Open the grippers

        clock1.reset();
        while(clock1.milliseconds() < 400){
        }
        drive.setLiftTargetPosition(liftUpPosition, 1); //Lift up the lift
        clock1.reset();
        while(clock1.milliseconds() < 200){
        }
        drive.setIntakeGrippersPos(.5, .5); //close the gripper
        drive.setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
        drive.setIntakeWristPos(.4);//Lower the wrist

    }

    public void lesserDelivery(){
        drive.setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
    }

    public void deliver(double speedMod, float lsy, float lsx, float rsx){

        drive.setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);



        clock1.reset();
        while(clock1.milliseconds() < 1000){
            drive.setWeightedDrivePower(
                new Pose2d(
                        -lsy * speedMod,
                        -lsx * speedMod,
                        -rsx * speedMod
                )
        );
            drive.update();
        }

        drive.setLiftTargetPosition(liftRestPosition, 1); //Return the lift to rest position

        drive.setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
    }

    public void restMode(){
        drive.setLiftTargetPosition(liftRestPosition, 1); //Return the lift to rest position
        drive.setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
        drive.setIntakeExtension(intakeRetractedPos, intakeRetractedPos2); //Pull back the intake
        drive.setIntakeWristPos(.4);//Mid point of wrist
        drive.setIntakeGrippersPos(-gripperOpenPos, gripperOpenPos); //Open the grippers
    }


}
