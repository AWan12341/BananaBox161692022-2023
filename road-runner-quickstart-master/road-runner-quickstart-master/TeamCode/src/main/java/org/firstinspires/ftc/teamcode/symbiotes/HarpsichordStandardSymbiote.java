package org.firstinspires.ftc.teamcode.symbiotes;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HarpsichordStandardSymbiote {

    //Imports
    private SampleMecanumDrive drive;

    //Slide positions
    private int extendedPos = 360, retractedPos = -10, transferPos = 160, restPos = 200, floatingTargetPos = 160;//85
    //Lift positions
    private int liftRestPosition = 100;
    private int liftHeight1Pos = 300, liftHeight2Pos = 510, liftHeight3Pos = 780;
    private int tempHeight = 780;

    //Outtake wrist
    private double outtakeWristOutPos = .2;
    private double outtakeWristInPos = .76;//.86
    private double outtakeWristMidPos = .5;

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
    boolean hardwareOverrunProtection = false;

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

    public enum AUTONOMOUSSTATE{
//        INITIAL_PREPARE_STAGE0,
//        INITIAL_PREPARE_STAGE1,
//        INITIAL_PREPARE_STAGE2,
//        INITIAL_PREPARE_STAGE3,
//        INITIAL_PREPARE_STAGE4,
        INITIAL_DELIVERY_STAGE1,
        INITIAL_DELIVERY_STAGE2,
        EXTEND_PREP,
        EXTEND,
        GRIP,
        PREPARE_RETRACTION,
        RETRACT,
        PREPARE_TRANSFER,
        TRANSFER,
        LIFT,
        DELIVER
    }

    //State Machine Flags
    public PRIMESTATE primeState = PRIMESTATE.RESET;
    public GRABSTATE grabState = GRABSTATE.GRAB;
    public DELIVERYSTATE deliveryState = DELIVERYSTATE.DELIVER;
    public AUTONOMOUSSTATE autoState = AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE1;



    public HarpsichordStandardSymbiote(SampleMecanumDrive driveTemp) {
        drive = driveTemp; //Grab the drive variable


        setLiftTargetPosition(0, 1); //Return the lift to rest position
        setOuttakePos(outtakeWristMidPos, outtakeClawOpenPos);
        setIntakeExtension(0, 1); //Pull back the intake
        drive.intakeWrist.setPosition(.4);
        setIntakeGrippersPos(1);
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
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setIntakeGrippersPos(2); //Close the gripper
                        setIntakeWristPos(0); //Lower the wrist
                        setLiftTargetPosition(0, .5); //Return lift to rest position
                    }
                }
                else {
                    hardwareOverrunProtection = false;
                    primeState = PRIMESTATE.EXTEND;
                }
                /*----------*/
                //Should we extend under automatic mode?
//                if (automaticMode && drive.intakeSlide.getCurrentPosition() < floatingTargetPos && primeState == PRIMESTATE.EXTEND){
//                    if(!hardwareOverrunProtection) {
//                        hardwareOverrunProtection = true;
//                        setIntakeExtension(4, 1); //Extend the intake, using the distance sensor
//                        setIntakeGrippersPos(0); //Open the grippers
//                    }
//                }
                /*----------*/
                //Should we extend under manual mode?
                if (!automaticMode && drive.intakeSlide.getCurrentPosition() < extendedPos && primeState == PRIMESTATE.EXTEND){
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setIntakeExtension(3, 1); //Extend the intake
                        setIntakeGrippersPos(0); //Open the grippers
                    }
                }
                /*----------*/
                //Now that we're finished, set everything back up for next time.
                else{
                    hardwareOverrunProtection = false;
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
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setIntakeGrippersPos(2); //close the gripper
                        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos); //Ensure the outtake is properly set up
                    }
                }
                else if(grabState == GRABSTATE.GRAB && clock1.milliseconds() > 400 && wasClockReset){
                    hardwareOverrunProtection = false;
                    grabState = GRABSTATE.RETRACT;
                    wasClockReset = false;
                }
                /*----------*/
                //Should we retract the intake?
                if(grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() > transferPos){
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setIntakeWristPos(1);//Send wrist to a mid point
                        setIntakeExtension(2, 1); //Pull back the intake
                    }
                }
                else if (grabState == GRABSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() <= transferPos){
                    hardwareOverrunProtection = false;
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
                if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() < 200 && wasClockReset){
                    setIntakeGrippersPos(0); //Open the grippers
                }
                else if(grabState == GRABSTATE.TRANSFER && clock1.milliseconds() > 200 && wasClockReset){
                    grabState = GRABSTATE.LIFT;
                }
                /*----------*/
                //Should we lift up?
                if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() < tempHeight){
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setIntakeWristPos(1);//Send wrist to a mid point
                        setOuttakePos(outtakeWristMidPos, outtakeClawClosedPos);
                        setIntakeGrippersPos(2); //Close the gripper
                        setLiftTargetPosition(4, 1);
                    }
                }
                else if(grabState == GRABSTATE.LIFT && drive.lift.getCurrentPosition() >= tempHeight){
                    hardwareOverrunProtection = false;
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
                if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() < 400 && wasClockReset){
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos);
                        setIntakeGrippersPos(0); //Open the grippers
                        setIntakeWristPos(0); //Lower the wrist
                    }
                }
                else if(deliveryState == DELIVERYSTATE.DELIVER && clock1.milliseconds() > 400 && wasClockReset){
                    hardwareOverrunProtection = false;
                    deliveryState = DELIVERYSTATE.RESET;
                }
                /*----------*/
                //Should we reset?
                if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() > liftRestPosition){
                    if(!hardwareOverrunProtection) {
                        hardwareOverrunProtection = true;
                        setLiftTargetPosition(0, .5); //Return the lift to rest position

                        setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);

                        if (cyclingMode) {
                            setIntakeExtension(3, 1); //Extend the intake
                        }
                    }
                }
                else if(deliveryState == DELIVERYSTATE.RESET && drive.lift.getCurrentPosition() <= liftRestPosition){
                    hardwareOverrunProtection = false;
                    deliveryState = DELIVERYSTATE.DELIVER;
                    shouldDeliver = false;
                    isBusy = false;
                    wasClockReset = false;
                }
            }
            else if(shouldRest){
                setLiftTargetPosition(0, .5); //Return the lift to rest position
                setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
                setIntakeExtension(0, .5); //Pull back the intake
                setIntakeWristPos(1);
                setIntakeGrippersPos(0);
                shouldRest = false;
            }

    }

    public void updateAutonomousHarpsichordSymbiote(int cone, int height){

        if(!wasClockReset){ //Reset the clock
            clock1.reset();
            wasClockReset = true;
        }
        if(shouldGrab) {
            isBusy = true;
            if (autoState == AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE1 && drive.lift.getCurrentPosition() < tempHeight) {
                if(!hardwareOverrunProtection) {
                    setLiftTargetPosition(height, 1); //Send the lift to the height
                    setIntakeExtension(5, 1); //Set the intake to a mid way position so the claw can close
                    hardwareOverrunProtection = true;
                }
            }
            else if(autoState == AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE1 && drive.lift.getCurrentPosition() >= tempHeight){
                hardwareOverrunProtection = false;

                autoState = AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE2;
                setIntakeGrippersPos(2); //Close the grippers
                wasClockReset = false;
                hardwareOverrunProtection = false;

            }

            if (autoState == AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE2 && clock1.milliseconds() < 600 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos); //Deliver the first cone
                    setIntakeGrippersPos(2); //Close the grippers
                }

            }
            else if(autoState == AUTONOMOUSSTATE.INITIAL_DELIVERY_STAGE2 && clock1.milliseconds() > 600 && wasClockReset){
                hardwareOverrunProtection = false;
                setIntakeGrippersPos(2); //Close the grippers
                autoState = AUTONOMOUSSTATE.EXTEND_PREP;
                isBusy = false;
                shouldGrab = false;
                wasClockReset = false;
            }
        }
        if(shouldDeliver) {

            isBusy = true;

            if (autoState == AUTONOMOUSSTATE.EXTEND_PREP && clock1.milliseconds() < 500 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setLiftTargetPosition(0, .5); //Lower the delivery lift
                    setIntakeWristPos(cone + 2); //Extend the wrist

                    if(cone == 1){
                        setIntakeWristUpperPos(2);
                    }
                    else{
                        setIntakeWristUpperPos(cone); //Set the upper wrists
                    }
                    setOuttakePos(outtakeWristInPos, outtakeClawOpenPos); //Ensure the outtake is set up
//                    setIntakeExtension(3, 1);
                }
            }
            else if(autoState == AUTONOMOUSSTATE.EXTEND_PREP && clock1.milliseconds() > 500 && wasClockReset){
                hardwareOverrunProtection = false;
                autoState = AUTONOMOUSSTATE.EXTEND;
                wasClockReset = false;
            }

            //Now out of one-time prep stages
            if (autoState == AUTONOMOUSSTATE.EXTEND && drive.intakeSlide.getCurrentPosition() < extendedPos) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setIntakeGrippersPos(0); //Ensure the grippers are open
                    setIntakeExtension(3, 1); //Extend the intake

                }
            }
            else if(autoState == AUTONOMOUSSTATE.EXTEND && drive.intakeSlide.getCurrentPosition() >= extendedPos){
                hardwareOverrunProtection = false;
                autoState = AUTONOMOUSSTATE.GRIP;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.GRIP && clock1.milliseconds() < 500 && wasClockReset) {
                setIntakeGrippersPos(2); //Grip the cone
            }
            else if(autoState == AUTONOMOUSSTATE.GRIP && clock1.milliseconds() > 500 && wasClockReset){
                autoState = AUTONOMOUSSTATE.PREPARE_RETRACTION;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.PREPARE_RETRACTION && clock1.milliseconds() < 400 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    if (cone > 1)
                        drive.intakeWrist.setPosition(.4); //Pop up the cone
                    else
                        drive.intakeWrist.setPosition(.5);
                    setIntakeWristUpperPos(1); //Return the upper wrist to the default position
                }
            }
            else if(autoState == AUTONOMOUSSTATE.PREPARE_RETRACTION && clock1.milliseconds() > 400 && wasClockReset){
                hardwareOverrunProtection = false;
                autoState = AUTONOMOUSSTATE.RETRACT;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() > transferPos && (clock1.milliseconds() < 1200 && wasClockReset)) {
//                if(!hardwareOverrunProtection) {
//                    hardwareOverrunProtection = true;

                    drive.intakeSlide.setPower(1);
                    drive.intakeSlide.setTargetPosition(165);
//                setIntakeExtension(1, 1);
//                }

            }
            else if(autoState == AUTONOMOUSSTATE.RETRACT && drive.intakeSlide.getCurrentPosition() <= transferPos || (clock1.milliseconds() > 1200 && wasClockReset)){
                hardwareOverrunProtection = false;
                autoState = AUTONOMOUSSTATE.PREPARE_TRANSFER;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.PREPARE_TRANSFER && clock1.milliseconds() < 500 && wasClockReset) {
                setIntakeWristPos(2); //Pull back in the wrist
            }
            else if(autoState == AUTONOMOUSSTATE.PREPARE_TRANSFER && clock1.milliseconds() > 500 && wasClockReset){
                autoState = AUTONOMOUSSTATE.TRANSFER;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.TRANSFER && clock1.milliseconds() < 400 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setIntakeGrippersPos(0); //Open the grips
                    setIntakeExtension(5, 1);//Set the extension to a neutral position
                }
            }
            else if(autoState == AUTONOMOUSSTATE.TRANSFER && clock1.milliseconds() > 400 && wasClockReset){
                hardwareOverrunProtection = false;
                autoState = AUTONOMOUSSTATE.LIFT;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.LIFT && clock1.milliseconds() < 400 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setLiftTargetPosition(height, 1);//Lift the elevator
//                    if(clock1.milliseconds() > 200){
//
//                    }
                }
            }
            //drive.lift.getCurrentPosition() >= tempHeight
            else if(autoState == AUTONOMOUSSTATE.LIFT && clock1.milliseconds() > 400 && wasClockReset){
                hardwareOverrunProtection = false;
//                setIntakeExtension(3, 1);
                autoState = AUTONOMOUSSTATE.DELIVER;
                wasClockReset = false;
            }

            if (autoState == AUTONOMOUSSTATE.DELIVER && clock1.milliseconds() < 700 && wasClockReset) {
                if(!hardwareOverrunProtection) {
                    hardwareOverrunProtection = true;

                    setOuttakePos(outtakeWristOutPos, outtakeClawOpenPos); //Deliver the first cone
//                    setIntakeExtension(3, 1);
                    setIntakeGrippersPos(2); //Close the grippers
                }
            }
            else if(autoState == AUTONOMOUSSTATE.DELIVER && clock1.milliseconds() > 700 && wasClockReset){
                hardwareOverrunProtection = false;
                wasClockReset = false;
                autoState = AUTONOMOUSSTATE.EXTEND_PREP;
                shouldGrab = false;
                isBusy = false;
                setIntakeGrippersPos(2); //Close the grips
            }
        }
        if(shouldRest){
            setLiftTargetPosition(0, 1); //Return the lift to rest position
            setOuttakePos(outtakeWristInPos, outtakeClawOpenPos);
            setIntakeExtension(0, 1); //Pull back the intake
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
                drive.intakeWristUpper.setPosition(.48); //.48
                break;
            case 2:
                drive.intakeWristUpper.setPosition(.57); //.56
                break;
            case 3:
                drive.intakeWristUpper.setPosition(.59); //.58
                break;
            case 4:
                drive.intakeWristUpper.setPosition(.6); //.6
                break;
            case 5:
                drive.intakeWristUpper.setPosition(.63);//.63
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
                drive.intakeGrip1.setPosition(.1);//.05
                drive.intakeGrip2.setPosition(.95);//1.0
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
                drive.intakeWrist.setPosition(.75); //.78
                break;
            case 4:
                drive.intakeWrist.setPosition(.7);//.71
                break;
            case 5:
                drive.intakeWrist.setPosition(.65);//.66
                break;
            case 6:
                drive.intakeWrist.setPosition(.61);//.62
                break;
            case 7:
                drive.intakeWrist.setPosition(.54);//.55
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
    //5 = claw open position
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
            case 5:
                drive.intakeSlide.setPower(pow);
                drive.intakeSlide.setTargetPosition(restPos);
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
