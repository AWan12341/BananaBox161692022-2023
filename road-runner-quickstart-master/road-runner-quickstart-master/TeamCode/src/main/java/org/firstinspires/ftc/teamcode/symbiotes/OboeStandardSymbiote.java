package org.firstinspires.ftc.teamcode.symbiotes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class OboeStandardSymbiote {

    //Imports
    private SampleMecanumDrive drive;

    //Lift positions
    private int liftRestPosition = 100;
    private int liftHeight1Pos = 300, liftHeight2Pos = 600, liftHeight3Pos = 905;
    private int tempHeight = 905;
    private int liftTargetHeight = 3;
    private double rampIncrement = .05, ramp = 1;

    //Outtake wrist
    private double outtakeWristOutPos = .2;
    private double outtakeWristInPos = .86;

    //Outtake Claw
    private double outtakeClawOpenPos = .75;
    private double outtakeClawClosedPos = 0;

    //Intake wrist
    private double intakeWristInPos = .1;

    //grippers
    private double leftGripperOpenPos = .05;
    private double leftGripperClosedPos = .75;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime rampDelay = new ElapsedTime();
    ElapsedTime requestDelay = new ElapsedTime();


    //Other
    boolean automaticMode = false;

    //Flags
    boolean wasClockReset = false;
    boolean shouldPrime = false, shouldGrab = false, shouldPrepareDelivery = false, shouldDeliver = false, shouldRest = false;
    boolean isBusy = false;
    boolean flag = false;
    public boolean readyFlag = false;



    public void setup(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable

        setLiftTargetPosition(0, 1); //Return the lift to rest position
        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
        setIntakeExtension(0, 1); //Pull back the intake
        setIntakeWristPos(1);
        setIntakeGrippersPos(0);
        setIntakeWristUpperPos(0);
        rampDelay.reset();

        readyFlag = true;
    }


    public void updateViolaSymbiote(){

            if(shouldPrime){

                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }

                if(wasClockReset && clock1.milliseconds() < 40 && clock1.milliseconds() < 100){
                    setIntakeGrippersPos(2); //Close the gripper
                    setIntakeWristPos(0); //Lower the wrist
                    setLiftTargetPosition(0, 1);
                }

                if(wasClockReset && clock1.milliseconds() > 400 && clock1.milliseconds() < 700) { //Wait for a bit
//                    setIntakeExtension(3);

                    if(!automaticMode){
                        setIntakeExtension(1, 1); //Extend the intake
                    }
                    else if (automaticMode){
                        setIntakeExtension(4, 1);
                    }


                    setIntakeGrippersPos(0); //Open the grippers
//                    drive.intakeGrip1.setPosition(.2);

                }

//                if(drive.intakeSlide.getCurrentPosition() > 75){
//                    setIntakeWristPos(0);//Lower the wrist
//                }
//                if(automaticMode && wasClockReset && clock1.milliseconds() > 800) {
//                    shouldGrab = true;
//                    wasClockReset = false;
//                    shouldPrime = false;
//                }
                else if (!automaticMode && wasClockReset && clock1.milliseconds() > 800){
                    shouldPrime = false;
                    wasClockReset = false;
                    isBusy = false;
                }
            }

            else if(shouldGrab){

                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }

//                if(wasClockReset && clock1.milliseconds() < 400){
//                    setIntakeGrippersPos(2); //close the gripper
//                }
//
//                if (drive.intakeSlide.getCurrentPosition() > 40 && wasClockReset && clock1.milliseconds() > 300) {
////                    setIntakeWristPos(1); //Mid point of wrist
//                    setIntakeExtension(2, 1); //Pull back the intake
//                }
//
//                else if (drive.intakeSlide.getCurrentPosition() < 40) {
//                    setIntakeWristPos(2); //Pull the wrist up
//                }
//
//                else if ((drive.intakeWrist.getPosition() > intakeWristInPos -.05 || drive.intakeWrist.getPosition() < intakeWristInPos + .05)) {
//                    setIntakeGrippersPos(3); //Open the grippers
////                    clock1.reset();
//                }

//                else if (wasClockReset && clock1.milliseconds() > 2000 && clock1.milliseconds() < 2200) {
//                    setLiftTargetPosition(liftTargetHeight, 1); //Lift up the lift
//                    setIntakeGrippersPos(3); //close the gripper
//                    setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
//                    setIntakeWristPos(1);//Wrist mid point
//                }
                //drive.intakeGrip2.getPosition() > leftGripperOpenPos -.05 || drive.intakeGrip2.getPosition() < leftGripperOpenPos + .05

//                else if (drive.lift.getTargetPosition() > tempHeight -4) {
//
//                }

                if(wasClockReset && clock1.milliseconds() < 400){
                    setIntakeGrippersPos(2); //close the gripper
                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                }
                if (wasClockReset && clock1.milliseconds() > 500 && clock1.milliseconds() < 950) {
                    setIntakeWristPos(1);//Mid point
//                    if(rampDelay.milliseconds() > 20) {
//                        ramp -= rampIncrement;
//                        rampDelay.reset();
//                    }
                    setIntakeExtension(0, .5); //Pull back the intake
                }
                else if (wasClockReset && clock1.milliseconds() > 1000 && clock1.milliseconds() < 1200) {
                    setIntakeWristPos(2); //Pull the wrist up
//                    ramp = 1;
                }
                else if (wasClockReset && clock1.milliseconds() > 2000 && clock1.milliseconds() < 2100) {
                    setIntakeGrippersPos(3); //Open the grippers

                }
                else if (wasClockReset && clock1.milliseconds() > 2800 && clock1.milliseconds() < 2900) {
                    setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
                    setIntakeExtension(3, 1);
                    setLiftTargetPosition(liftTargetHeight, 1); //Lift up the lift
                }
                else if (wasClockReset && clock1.milliseconds() > 3200 && clock1.milliseconds() < 3300) {
                    setIntakeGrippersPos(2); //close the gripper
                    setOuttakePos(outtakeWristInPos, outtakeClawClosedPos); //Clamp the part
                    setIntakeWristPos(1);//Wrist mid point
                }

//                if(automaticMode && drive.lift.getTargetPosition() > tempHeight -4) {
//                    shouldPrepareDelivery = true;
//                    wasClockReset = false;
//                    shouldGrab = false;
//                }
                else if (!automaticMode && drive.lift.getTargetPosition() > tempHeight -4){
                    shouldGrab = false;
                    wasClockReset = false;
                    isBusy = false;
                }

            }
            else if (shouldPrepareDelivery){

                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }

                if(wasClockReset && clock1.milliseconds() < 40) {
                    setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                    setIntakeGrippersPos(2); //close the gripper
                }

//                if(automaticMode && wasClockReset && clock1.milliseconds() > 400){
//                    shouldPrepareDelivery = false;
//                    shouldDeliver = true;
//                    wasClockReset = false;
//                }
                if (!automaticMode && wasClockReset && clock1.milliseconds() > 400){
                    shouldPrepareDelivery = false;
                    wasClockReset = false;
                    isBusy = false;
                }


            }
            else if (shouldDeliver){

                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }

                if(wasClockReset && clock1.milliseconds() < 400){
                    setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);
                    setIntakeGrippersPos(2); //close the gripper
                }

                if(wasClockReset && clock1.milliseconds() > 1000 && clock1.milliseconds() < 1100){
                    setLiftTargetPosition(0, .25); //Return the lift to rest position

                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                }

//                if(automaticMode && wasClockReset && clock1.milliseconds() > 1200){
//                    shouldRest = true;
//                    wasClockReset = false;
//                    shouldDeliver = false;
//                    isBusy = false;
//                }
                if (!automaticMode && wasClockReset && clock1.milliseconds() > 1200){
                    wasClockReset = false;
                    shouldDeliver = false;
                    isBusy = false;
                }
            }
            else if(shouldRest){
                setLiftTargetPosition(0, 1); //Return the lift to rest position
                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                setIntakeExtension(0, .5); //Pull back the intake
                setIntakeWristPos(1);
                setIntakeGrippersPos(0);
                shouldRest = false;
            }

    }

    public void updateAutonomousViolaSymbiote(int stage, int height){

        if(stage == 1){

//            setIntakeExtension(1); //Extend the intake
            setIntakeWristPos(height+2);//Lower the wrist
            setIntakeWristUpperPos(height); //Angle the upper wrist
            setIntakeGrippersPos(0); //Open the grippers

            wait(800);

            setIntakeGrippersPos(2); //close the gripper

            wait(400);

            setIntakeWristPos(1);//Mid point
            setIntakeWristUpperPos(0); // Return the upper wrist to default

            wait(200);

            if(height == 5) {
//                setIntakeExtension(2); //Pull back the intake
            }
            else {
//                drive.intakeExtension.setPosition(.22);
//                drive.intakeExtensionRight.setPosition(.82);
            }

            wait(100);

            setIntakeWristPos(2); //Pull the wrist up

            wait(600);

            setIntakeGrippersPos(0); //Open the grippers

            wait(600);

            setLiftTargetPosition(3, 1); //Lift up the lift

            wait(1600);

//            while(drive.lift.getTargetPosition() < (4600 - liftRestPosition)){drive.update();}

            setIntakeGrippersPos(2); //close the gripper
            setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
            setIntakeWristPos(1); //Wrist mid point

            wait(700);


        }
        else if (stage == 2 || stage == 3){

            if(stage == 3) {
                setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                setLiftTargetPosition(3, 1); //Lift up the lift

//                while(drive.lift.getTargetPosition() <= (4600 - liftRestPosition)){drive.update();}

                wait(2000);
            }

            setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);

            wait(800);

            setLiftTargetPosition(0, 1); //Return the lift to rest position
            setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);

        }
        else if(stage == 4){
            setLiftTargetPosition(0, 1); //Return the lift to rest position
            setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
//            setIntakeExtension(0); //Pull back the intake
            setIntakeWristPos(1);
            setIntakeGrippersPos(0);
            shouldRest = false;
        }

    }

    public boolean isBusy(){
        return isBusy;
    }

    public void wait(int time){
        clock1.reset();
        while(clock1.milliseconds() < time){}
    }

    public void setLiftTargetHeight(int targ){
        liftTargetHeight = targ;
        switch(targ){
            case 1:
                tempHeight = liftHeight1Pos;
                break;
            case 2:
                tempHeight = liftHeight2Pos;
                break;
            case 3:
                tempHeight = liftHeight3Pos;
                break;
        }
    }

    public void sendStatusRequest(int status){
        if(requestDelay.milliseconds() > 200) {
            switch (status) {
                case 0:
                    shouldPrime = true;
                    break;
                case 1:
                    shouldGrab = true;
                    break;
                case 2:
                    shouldPrepareDelivery = true;
                    break;
                case 3:
                    shouldDeliver = true;
                    break;
                case 4:
                    shouldRest = true;
                    break;
            }
            requestDelay.reset();
        }
    }

    public void setAutomaticMode(boolean mode){
        automaticMode = mode;
    }

    //0 = rest
    //1 = Level 1
    //2 = Level 2
    //3 = Level 3
    //4 = Level 4
    public void setIntakeWristUpperPos(int position){
        switch(position){
            case 0:
                drive.intakeWristUpper.setPosition(.48);
                break;
            case 1:
                drive.intakeWristUpper.setPosition(.48);
                break;
            case 2:
                drive.intakeWristUpper.setPosition(.56);
                break;
            case 3:
                drive.intakeWristUpper.setPosition(.58);
                break;
            case 4:
                drive.intakeWristUpper.setPosition(.6);
                break;
            case 5:
                drive.intakeWristUpper.setPosition(.6);
                break;
        }
    }

    //0 = open
    //1 = mid
    //2 = closed
    //3 one side open, one side closed
    public void setIntakeGrippersPos(int position){
        switch(position){
            case 0:
                drive.intakeGrip1.setPosition(.05);
                drive.intakeGrip2.setPosition(1.0);
                break;
            case 1:
                //Todo: Find a mid position. Low importance
                drive.intakeGrip1.setPosition(.2);
                drive.intakeGrip2.setPosition(.8);
                break;
            case 2:
                drive.intakeGrip1.setPosition(.33);
                drive.intakeGrip2.setPosition(.75);
                break;
            case 3:
                drive.intakeGrip1.setPosition(.05);
                drive.intakeGrip2.setPosition(1.0);
                break;
        }
    }

    //0 = out
    //1 = mid
    //2 = in
    //3 = Level 1
    //4 = Level 2
    //5 = Level 3
    //6 = Level 4
    //7 = Level 5
    public void setIntakeWristPos(int position){
        switch(position){
            case 0:
                drive.intakeWrist.setPosition(.76);
                break;
            case 1:
                drive.intakeWrist.setPosition(.5);
                break;
            case 2:
                drive.intakeWrist.setPosition(.15);//todo:test
                break;
            case 3:
                drive.intakeWrist.setPosition(.78);
                break;
            case 4:
                drive.intakeWrist.setPosition(.7);
                break;
            case 5:
                drive.intakeWrist.setPosition(.67);
                break;
            case 6:
                drive.intakeWrist.setPosition(.6);
                break;
            case 7:
                drive.intakeWrist.setPosition(.56);
                break;
        }
    }

    //Left as two separate values because we do weird things with it
    public void setOuttakePos(double pos, double pos2){
        drive.outtakeWrist.setPosition(pos);
        drive.outtakeClaw.setPosition(pos2);
    }

    //0 = retracted
    //1 = extended
    //2 = delivery
    //3 = delivery mid
    public void setIntakeExtension(int position, double pow){

        drive.intakeSlide.setPower(pow);

        switch (position){
            case 0:
                drive.intakeSlide.setTargetPosition(0);
                break;
            case 1:
                drive.intakeSlide.setTargetPosition(170);
                break;
            case 2:
                drive.intakeSlide.setTargetPosition(60); //TODO: tune
                break;
            case 3:
                drive.intakeSlide.setTargetPosition(70);
//            case 4:
//                if(!flag) {
//                    drive.intakeSlide.setTargetPosition(0);
//                }
//                else if (flag){
//                    drive.intakeSlide.setTargetPosition((int)(drive.forwardSensor.getDistance(DistanceUnit.INCH) * 8.95));
//                }
//                if(drive.intakeSlide.getCurrentPosition() >= 0 && !flag){
//                    flag = true;
//                }
//                break;
        }
    }

    //0 = Reset
    //1 = Junction height 1
    //2 = Junction height 2
    //3 = Junction height 3;
    public void setLiftTargetPosition(int level, double pow){

        drive.lift.setPower(pow);



        switch(level){
            case 0:
//                drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drive.lift.setTargetPosition(300);
                break;
            case 1:
                //Todo: Find actual junction height 1 height
                drive.lift.setTargetPosition(liftHeight1Pos);
                break;
            case 2:
                //Todo: Find actual junction height 2 height
                drive.lift.setTargetPosition(liftHeight2Pos);
                break;
            case 3:
//                drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.lift.setTargetPosition(liftHeight3Pos);
                break;
        }
    }


}
