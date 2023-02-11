package org.firstinspires.ftc.teamcode.symbiotes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HarpsichordStandardSymbiote {

    //Imports
    private SampleMecanumDrive drive;

    //Slide positions
    private int extendedPos = 340, retractedPos = -10, transferPos = 130, floatingTargetPos = 130;//85
    //Lift positions
    private int liftRestPosition = 100;
    private int liftHeight1Pos = 300, liftHeight2Pos = 600, liftHeight3Pos = 905;
    private int tempHeight = 905;

    //Outtake wrist
    private double outtakeWristOutPos = .2;
    private double outtakeWristInPos = .76;//.86

    //Outtake Claw
    private double outtakeClawOpenPos = .75;
    private double outtakeClawClosedPos = 0;

    //Intake wrist

    //.15
    private double intakeWristInPos = .05, intakeWristOutPos = .78, intakeWristMidPos = .5;

    //grippers
    private double rightGripperOpenPos = .05;
    private double rightGripperClosedPos = .33;

    //Timers
    ElapsedTime clock1 = new ElapsedTime();
    ElapsedTime rampDelay = new ElapsedTime();
    ElapsedTime requestDelay = new ElapsedTime();


    //Other
    boolean automaticMode = false;
    public boolean cyclingMode = false;
    public boolean firstDelivery = false;

    //Flags
    boolean shouldPrime = false, shouldGrab = false, shouldPrepareDelivery = false, shouldDeliver = false, shouldRest = false;
    boolean isBusy = false;
    public boolean readyFlag = false;
    boolean wasClockReset = false;

    //State Machine Enums
    public enum PRIMESTATE{
        EXTEND,
        RESET,
        CANCEL
    }

    public enum GRABSTATE{
        GRAB,
        RETRACT,
        PREPARE_TRANSFER,
        TRANSFER,
        LIFT,
        RESET,
        CANCEL
    }

    public enum DELIVERYSTATE{
        DELIVER,
        RESET,
        CANCEL
    }

    //State Machine Flags
    public PRIMESTATE primeState = PRIMESTATE.RESET;
    public GRABSTATE grabState = GRABSTATE.GRAB;
    public DELIVERYSTATE deliveryState = DELIVERYSTATE.DELIVER;



    public HarpsichordStandardSymbiote(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable

        setLiftTargetPosition(0, 1); //Return the lift to rest position
        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
        setIntakeExtension(0, 1); //Pull back the intake
        setIntakeWristPos(1);
        setIntakeGrippersPos(2);
        setIntakeWristUpperPos(1);
        rampDelay.reset();

        readyFlag = true;
    }


    public void updateHarpsichordSymbiote(){

            if(shouldPrime){
                isBusy = true;
                /*----------*/
                //Should we reset?
                if(drive.intakeWrist.getPosition() < intakeWristOutPos && primeState == PRIMESTATE.RESET) {
                    setIntakeGrippersPos(2); //Close the gripper
                    setIntakeWristPos(0); //Lower the wrist
                    setLiftTargetPosition(0, 1); //Return lift to rest position
                }
                else {
                    primeState = PRIMESTATE.EXTEND;
                }
                /*----------*/
                //Should we extend under automatic mode?
                if (automaticMode && drive.intakeSlide.getCurrentPosition() < floatingTargetPos && primeState == PRIMESTATE.EXTEND){
                    setIntakeExtension(4, 1); //Extend the intake, using the distance sensor
                    setIntakeGrippersPos(0); //Open the grippers
                }
                /*----------*/
                //Should we extend under manual mode?
                else if (!automaticMode && drive.intakeSlide.getCurrentPosition() < extendedPos && primeState == PRIMESTATE.EXTEND){
                    setIntakeExtension(3, 1); //Extend the intake
                    setIntakeGrippersPos(0); //Open the grippers
                }
                /*----------*/
                //Now that we're finished, set everything back up for next time.
                else{
                    primeState = PRIMESTATE.RESET;
                    shouldPrime = false;
                    isBusy = false;
                    wasClockReset = false;
//                    if(cyclingMode){
//                        shouldGrab = true;
//                    }
                }
            }

            else if(shouldGrab){
                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }
                /*----------*/
                //Should we close the gripper?
                if(grabState == GRABSTATE.GRAB && clock1.milliseconds() < 400 && wasClockReset){
                    setIntakeGrippersPos(2); //close the gripper
                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos); //Ensure the outtake is properly set up
                }
                else if(grabState == GRABSTATE.GRAB && clock1.milliseconds() > 400 && wasClockReset){
                    grabState = GRABSTATE.RETRACT;
                    wasClockReset = false;
                }
                /*----------*/
                //Should we retract the intake?
                if(grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() > transferPos){
                    setIntakeWristPos(1);//Send wrist to a mid point
                    setIntakeExtension(2, .5); //Pull back the intake
                }
                else if (grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() <= transferPos){
                    grabState = GRABSTATE.PREPARE_TRANSFER;
                    wasClockReset = false;
                    clock1.reset();
                }
                /*----------*/
                //Should we prepare to transfer?
                if(grabState == GRABSTATE.PREPARE_TRANSFER && clock1.milliseconds() < 400 && wasClockReset){
                    setIntakeWristPos(2); //Pull the wrist up
                }
                else if(grabState == GRABSTATE.PREPARE_TRANSFER && clock1.milliseconds() > 400 && wasClockReset){
                    grabState = GRABSTATE.TRANSFER;
                    wasClockReset = false;
                    clock1.reset();
                }
                /*----------*/
                //Should we transfer?
                if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() < 400 && wasClockReset){
                    setIntakeGrippersPos(0); //Open the grippers
                }
                else if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() > 400 && wasClockReset){
                    grabState = GRABSTATE.LIFT;
                }
                /*----------*/
                //Should we lift up?
                if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() < tempHeight){
                    setIntakeWristPos(1);//Send wrist to a mid point
                    setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                    setIntakeGrippersPos(2); //Close the gripper
                    setLiftTargetPosition(4, 1);
                }
                else if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() >= tempHeight){
                    grabState = GRABSTATE.GRAB;
                    shouldGrab = false;
                    isBusy = false;
                    wasClockReset = false;
                    if(cyclingMode){
                        shouldDeliver = true;
                    }
                }
            }
            else if (shouldPrepareDelivery){
                setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                setIntakeWristPos(0); //Lower the wrist
                shouldPrepareDelivery = false;
                wasClockReset = false;

            }
            else if (shouldDeliver){
                isBusy = true;

                if(!wasClockReset){ //Reset the clock
                    clock1.reset();
                    wasClockReset = true;
                }
                /*----------*/
                //Should we deliver the cone?
                if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() < 800 && wasClockReset){
                    setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);
                    setIntakeGrippersPos(0); //Open the grippers
                    setIntakeWristPos(0); //Lower the wrist
                }
                else if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() > 800 && wasClockReset){
                    deliveryState = DELIVERYSTATE.RESET;
                }
                /*----------*/
                //Should we reset?
                if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() > liftRestPosition){
                    setLiftTargetPosition(0, .75); //Return the lift to rest position

                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);

                    if(cyclingMode){
                        setIntakeExtension(3, 1); //Extend the intake
                    }
                }
                else if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() <= liftRestPosition){
                    deliveryState = DELIVERYSTATE.DELIVER;
                    shouldDeliver = false;
                    isBusy = false;
                    wasClockReset = false;
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

    public void updateAutonomousHarpsichordSymbiote(int cone){

        if(shouldPrime){
            isBusy = true;
            /*----------*/
            //Should we reset?
            if(drive.intakeWrist.getPosition() < intakeWristOutPos && primeState == PRIMESTATE.RESET) {
                setIntakeGrippersPos(2); //Close the gripper
                setIntakeWristPos(cone+2); //Lower the wrist
                setIntakeWristUpperPos(cone);
                setLiftTargetPosition(0, 1); //Return lift to rest position
            }
            else {
                primeState = PRIMESTATE.EXTEND;
            }
            /*----------*/
            //Should we extend under automatic mode?
            if (automaticMode && drive.intakeSlide.getCurrentPosition() < floatingTargetPos && primeState == PRIMESTATE.EXTEND){
                setIntakeExtension(4, 1); //Extend the intake, using the distance sensor
                setIntakeGrippersPos(0); //Open the grippers
            }
            /*----------*/
            //Should we extend under manual mode?
            else if (!automaticMode && drive.intakeSlide.getCurrentPosition() < extendedPos && primeState == PRIMESTATE.EXTEND){
                setIntakeExtension(3, 1); //Extend the intake
                setIntakeGrippersPos(0); //Open the grippers
            }
            /*----------*/
            //Now that we're finished, set everything back up for next time.
            else{
                primeState = PRIMESTATE.RESET;
                shouldPrime = false;
                isBusy = false;
                wasClockReset = false;
                shouldGrab = true;
            }
        }
        else if(shouldGrab){
            isBusy = true;

            if(!wasClockReset){ //Reset the clock
                clock1.reset();
                wasClockReset = true;
            }
            /*----------*/
            //Should we close the gripper?
            if(grabState == GRABSTATE.GRAB && clock1.milliseconds() < 400 && wasClockReset){
                setIntakeGrippersPos(2); //close the gripper
                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos); //Ensure the outtake is properly set up
            }
            else if(grabState == GRABSTATE.GRAB && clock1.milliseconds() > 400 && wasClockReset){
                grabState = GRABSTATE.RETRACT;
                wasClockReset = false;
            }
            /*----------*/
            //Should we retract the intake?
            if(grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() > transferPos){
                setIntakeWristPos(1);//Send wrist to a mid point
                setIntakeExtension(2, .5); //Pull back the intake
            }
            else if (grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() <= transferPos){
                grabState = GRABSTATE.PREPARE_TRANSFER;
                wasClockReset = false;
                clock1.reset();
            }
            /*----------*/
            //Should we prepare to transfer?
            if(grabState == GRABSTATE.PREPARE_TRANSFER && clock1.milliseconds() < 400 && wasClockReset){
                setIntakeWristPos(2); //Pull the wrist up
            }
            else if(grabState == GRABSTATE.PREPARE_TRANSFER && clock1.milliseconds() > 400 && wasClockReset){
                grabState = GRABSTATE.TRANSFER;
                wasClockReset = false;
                clock1.reset();
            }
            /*----------*/
            //Should we transfer?
            if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() < 400 && wasClockReset){
                setIntakeGrippersPos(0); //Open the grippers
            }
            else if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() > 400 && wasClockReset){
                grabState = GRABSTATE.LIFT;
            }
            /*----------*/
            //Should we lift up?
            if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() < tempHeight){
                setIntakeWristPos(1);//Send wrist to a mid point
                setOuttakePos(outtakeWristOutPos, outtakeClawClosedPos);
                setIntakeGrippersPos(2); //Close the gripper
                setLiftTargetPosition(4, 1);
            }
            else if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() >= tempHeight){
                grabState = GRABSTATE.GRAB;
                shouldGrab = false;
                wasClockReset = false;
                shouldDeliver = true;
            }
        }
        else if (shouldDeliver){
            isBusy = true;

            if(!wasClockReset){ //Reset the clock
                clock1.reset();
                wasClockReset = true;
            }
            /*----------*/
            //Should we lift if this is the first delivery?
            if(deliveryState == DELIVERYSTATE.DELIVER && drive.lift.getCurrentPosition() < tempHeight && firstDelivery){
                setLiftTargetPosition(3, 1);
            }
            else if(deliveryState == DELIVERYSTATE.DELIVER && drive.lift.getCurrentPosition() >= tempHeight && firstDelivery){
                firstDelivery = false;
            }
            /*----------*/
            //Should we deliver the cone?
            if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() < 800 && wasClockReset && !firstDelivery){
                setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);
                setIntakeWristPos(0); //Lower the wrist
            }
            else if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() > 800 && wasClockReset && !firstDelivery){
                deliveryState = DELIVERYSTATE.RESET;
            }
            /*----------*/
            //Should we reset?
            if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() > liftRestPosition){
                setLiftTargetPosition(0, .25); //Return the lift to rest position

                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);

                setIntakeExtension(3, 1); //Extend the intake

            }
            else if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() <= liftRestPosition){
                deliveryState = DELIVERYSTATE.DELIVER;
                shouldDeliver = false;
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
            /*----------*/
            //Should we prepare to transfer?
            if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() < 800 && wasClockReset){
                setIntakeWristPos(2); //Pull the wrist up
                setIntakeExtension(1, .7); //Pull back the intake
            }
            else if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() > 800 && wasClockReset){
                deliveryState = DELIVERYSTATE.RESET;
                wasClockReset = false;
                clock1.reset();
            }
            /*----------*/
            //Should we transfer?
            if(deliveryState == DELIVERYSTATE.RESET && clock1.milliseconds() < 400 && wasClockReset){
                setIntakeGrippersPos(0); //Open the grippers
            }
            else if(deliveryState == DELIVERYSTATE.RESET && clock1.milliseconds() > 400 && wasClockReset){
                deliveryState = DELIVERYSTATE.DELIVER;
                shouldPrepareDelivery = false;
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

    public boolean isBusy(){
        return isBusy;
    }

    public void wait(int time){
        clock1.reset();
        while(clock1.milliseconds() < time){}
    }

    public void setLiftTargetHeight(int targ){
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
        if(!isBusy) {
            switch (status) {
                case 0:
                    isBusy = true;
                    shouldPrime = true;
                    break;
                case 1:
                    isBusy = true;
                    shouldGrab = true;
                    break;
                case 2:
                    shouldPrepareDelivery = true;
                    break;
                case 3:
                    isBusy = true;
                    shouldDeliver = true;
                    break;
                case 4:
                    shouldRest = true;
                    break;
            }
        }
    }

    public void setAutomaticMode(boolean mode){
        automaticMode = mode;
    }

    //1 = Level 1
    //2 = Level 2
    //3 = Level 3
    //4 = Level 4
    //5 = Level 5
    public void setIntakeWristUpperPos(int position){
        switch(position){
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

    //INTAKE GRIP 1 IS RIGHT
    public void setIntakeGrippersPos(int position){
        switch(position){
            case 0:
                drive.intakeGrip1.setPosition(rightGripperOpenPos);//.05
                drive.intakeGrip2.setPosition(1.0);//1.0
                break;
            case 1:
                //Todo: Find a mid position. Low importance
                drive.intakeGrip1.setPosition(.2);//.2
                drive.intakeGrip2.setPosition(.8);//.8
                break;
            case 2:
                drive.intakeGrip1.setPosition(rightGripperClosedPos);//.33
                drive.intakeGrip2.setPosition(.75);//.75
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
                drive.intakeWrist.setPosition(intakeWristOutPos);
                break;
            case 1:
                drive.intakeWrist.setPosition(intakeWristMidPos);
                break;
            case 2:
                drive.intakeWrist.setPosition(intakeWristInPos);
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

    //0 = fully retracted
    //1 = transfer pose direct
    //2 = transfer pose with ramp
    //3 = fully extended
    //4 = fully extended position with damping
    public void setIntakeExtension(int position, double pow){



        switch (position){
            case 0:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(retractedPos);
                break;
            case 1:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(transferPos);
                break;
            case 2:
                drive.intakeSlide.setPower(drive.intakeSlide.getCurrentPosition() * .005);
                drive.intakeSlide.setTargetPosition(transferPos);
                break;
            case 3:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(extendedPos);
                break;
            case 4:
                drive.intakeSlide.setPower(pow);
//                if(drive.intakeSlide.getCurrentPosition() < 170 && drive.forwardSensor.getDistance(DistanceUnit.INCH) > 2.5){
//                    drive.intakeSlide.setPower(pow * (drive.forwardSensor.getDistance(DistanceUnit.INCH) * (drive.forwardSensor.getDistance(DistanceUnit.INCH) > 10?.05:.05)));
//                }
//                else {
//                    drive.intakeSlide.setPower(0);
//                }
                floatingTargetPos = (int)(drive.forwardSensor.getDistance(DistanceUnit.INCH) * 8.5);
                drive.intakeSlide.setTargetPosition(floatingTargetPos);
                break;
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
                drive.lift.setTargetPosition(liftRestPosition);
                break;
            case 1:
                drive.lift.setTargetPosition(liftHeight1Pos);
                break;
            case 2:
                drive.lift.setTargetPosition(liftHeight2Pos);
                break;
            case 3:
                drive.lift.setTargetPosition(liftHeight3Pos);
                break;
            case 4:
                drive.lift.setTargetPosition(tempHeight);
                break;
        }
    }


}
