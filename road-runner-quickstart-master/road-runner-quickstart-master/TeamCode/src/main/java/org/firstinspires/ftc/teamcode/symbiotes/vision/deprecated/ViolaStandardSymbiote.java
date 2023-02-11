package org.firstinspires.ftc.teamcode.symbiotes.vision.deprecated;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ViolaStandardSymbiote {

    //Imports
    private SampleMecanumDrive drive;

    //Lift positions
    private int liftRestPosition = -240;

    //Outtake wrist
    private double outtakeWristOutPos = 0;
    private double outtakeWristInPos = 1;

    //Outtake Claw
    private double outtakeClawOpenPos = .8;
    private double outtakeClawClosedPos = .2;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime requestDelay = new ElapsedTime();

    //Other
    boolean automaticMode = false;

    //Flags
    boolean wasClockReset = false;
    boolean shouldPrime = false, shouldGrab = false, shouldPrepareDelivery = false, shouldDeliver = false, shouldRest = false;
    boolean isBusy = false;



    public void setup(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable

        setLiftTargetPosition(0, 1); //Return the lift to rest position
        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
//        setIntakeExtension(0); //Pull back the intake
        setIntakeWristPos(1);
        setIntakeGrippersPos(0);
        setIntakeWristUpperPos(0);
    }


    public void updateViolaSymbiote(){

            if(shouldPrime){

                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }

                if(wasClockReset && clock1.milliseconds() < 50 && clock1.milliseconds() < 100){
                    setIntakeGrippersPos(0); //Close the gripper
                }

                if(wasClockReset && clock1.milliseconds() > 200 && clock1.milliseconds() < 500) { //Wait for a bit
//                    setIntakeExtension(3);

                }

                if(wasClockReset && clock1.milliseconds() > 400 && clock1.milliseconds() < 700) { //Wait for a bit
//                    setIntakeExtension(3);
//                    setIntakeExtension(1); //Extend the intake


                    setIntakeGrippersPos(0); //Open the grippers

                }

                if(wasClockReset && clock1.milliseconds() > 700 ){
                    setIntakeWristPos(0);//Lower the wrist
                }
                if(automaticMode && wasClockReset && clock1.milliseconds() > 800) {
                    shouldGrab = true;
                    wasClockReset = false;
                    shouldPrime = false;
                }
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

                if(wasClockReset && clock1.milliseconds() < 400){
                    setIntakeGrippersPos(2); //close the gripper
                }
                if (wasClockReset && clock1.milliseconds() > 500 && clock1.milliseconds() < 950) {
                    setIntakeWristPos(1);//Mid point
//                    setIntakeExtension(2); //Pull back the intake
                }
                else if (wasClockReset && clock1.milliseconds() > 1000 && clock1.milliseconds() < 1200) {
                    setIntakeWristPos(2); //Pull the wrist up
                }
                else if (wasClockReset && clock1.milliseconds() > 1500 && clock1.milliseconds() < 1700) {
                    setIntakeGrippersPos(0); //Open the grippers
                }
                else if (wasClockReset && clock1.milliseconds() > 2000 && clock1.milliseconds() < 2200) {
                    setLiftTargetPosition(3, 1); //Lift up the lift
                }
                else if (wasClockReset && clock1.milliseconds() > 2500 && clock1.milliseconds() < 2700) {
                    setIntakeGrippersPos(2); //close the gripper
                    setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
                    setIntakeWristPos(1);//Wrist mid point
                }

                if(automaticMode && wasClockReset && clock1.milliseconds() > 3000) {
                    shouldPrepareDelivery = true;
                    wasClockReset = false;
                    shouldGrab = false;
                }
                else if (!automaticMode && wasClockReset && clock1.milliseconds() > 3000){
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

                if(wasClockReset && clock1.milliseconds() < 50)
                    setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);

                if(automaticMode && wasClockReset && clock1.milliseconds() > 400){
                    shouldPrepareDelivery = false;
                    shouldDeliver = true;
                    wasClockReset = false;
                }
                else if (!automaticMode && wasClockReset && clock1.milliseconds() > 400){
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
                }

                if(wasClockReset && clock1.milliseconds() > 1000){
                    setLiftTargetPosition(0, 1); //Return the lift to rest position

                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                }

                if(automaticMode && wasClockReset && clock1.milliseconds() > 1200){
                    shouldRest = true;
                    wasClockReset = false;
                    shouldDeliver = false;
                    isBusy = false;
                }
                else if (!automaticMode && wasClockReset && clock1.milliseconds() > 1200){
                    wasClockReset = false;
                    shouldDeliver = false;
                    isBusy = false;
                }
            }
            else if(shouldRest){
                setLiftTargetPosition(0, 1); //Return the lift to rest position
                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
//                setIntakeExtension(0); //Pull back the intake
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

            wait(500);

            setIntakeGrippersPos(0); //Open the grippers

            wait(500);

            setLiftTargetPosition(3, 1); //Lift up the lift

            wait(1500);

//            while(drive.lift.getTargetPosition() < (4500 - liftRestPosition)){drive.update();}

            setIntakeGrippersPos(2); //close the gripper
            setOuttakePos(outtakeWristInPos, outtakeClawClosedPos);
            setIntakeWristPos(1); //Wrist mid point

            wait(700);


        }
        else if (stage == 2 || stage == 3){

            if(stage == 3) {
                setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                setLiftTargetPosition(3, 1); //Lift up the lift

//                while(drive.lift.getTargetPosition() <= (4500 - liftRestPosition)){drive.update();}

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
                drive.intakeWristUpper.setPosition(.95);
                break;
            case 1:
                drive.intakeWristUpper.setPosition(.96);
                break;
            case 2:
                drive.intakeWristUpper.setPosition(.94);
                break;
            case 3:
                drive.intakeWristUpper.setPosition(.93);
                break;
            case 4:
                drive.intakeWristUpper.setPosition(.86);
                break;
            case 5:
                drive.intakeWristUpper.setPosition(.85);
                break;
        }
    }

    //0 = open
    //1 = mid
    //2 = closed
    public void setIntakeGrippersPos(int position){
        switch(position){
            case 0:
                drive.intakeGrip1.setPosition(.8);
                drive.intakeGrip2.setPosition(.3);
                break;
            case 1:
                //Todo: Find a mid position. Low importance
                drive.intakeGrip1.setPosition(.5);
                drive.intakeGrip2.setPosition(.5);
                break;
            case 2:
                drive.intakeGrip1.setPosition(.55);
                drive.intakeGrip2.setPosition(.45);
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
                drive.intakeWrist.setPosition(1);//.65
                break;
            case 1:
                drive.intakeWrist.setPosition(.5);//.4
                break;
            case 2:
                drive.intakeWrist.setPosition(0);//0
                break;
            case 3:
                drive.intakeWrist.setPosition(.9);
                break;
            case 4:
                drive.intakeWrist.setPosition(.6);
                break;
            case 5:
                drive.intakeWrist.setPosition(.56);
                break;
            case 6:
                drive.intakeWrist.setPosition(.59);
                break;
            case 7:
                drive.intakeWrist.setPosition(.54);
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
//    public void setIntakeExtension(int position){
//        switch (position){
//            case 0:
//                drive.intakeExtension.setPosition(.35);
//                drive.intakeExtensionRight.setPosition(.69);
//                break;
//            case 1:
//                drive.intakeExtension.setPosition(.1);
//                drive.intakeExtensionRight.setPosition(.9);
//                break;
//            case 2:
//                drive.intakeExtension.setPosition(.22);
//                drive.intakeExtensionRight.setPosition(.82);
//                break;
////            case 3:
////                drive.intakeExtension.setPosition(.25);
////                drive.intakeExtensionRight.setPosition(.79);
////                break;
//        }
//    }

    //0 = Reset
    //1 = Junction height 1
    //2 = Junction height 2
    //3 = Junction height 3;
    public void setLiftTargetPosition(int level, double pow){

        drive.lift.setPower(pow);

        switch(level){
            case 0:
                drive.lift.setTargetPosition(liftRestPosition);
                break;
            case 1:
                //Todo: Find actual junction height 1 height
                drive.lift.setTargetPosition(-500);
                break;
            case 2:
                //Todo: Find actual junction height 2 height
                drive.lift.setTargetPosition(-1000);
                break;
            case 3:
                drive.lift.setTargetPosition(-1500);
                break;
        }
    }


}
