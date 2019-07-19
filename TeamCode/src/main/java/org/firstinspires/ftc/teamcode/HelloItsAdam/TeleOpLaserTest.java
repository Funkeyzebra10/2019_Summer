package org.firstinspires.ftc.teamcode.HelloItsAdam;

//Imports (duh)
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "Laser ", group="Main")

public class TeleOpLaserTest extends OpMode {

    //Declaring devices
    DcMotor left, right, shoulder, lift, dumpLift, sweep;
    DigitalChannel magSwitchDown, magSwitchUp, magLatchOpen, magLatchClose;
    AnalogInput pShoulder;
    CRServo latch;
    Servo wrist, dump, dumpDoor;
    DistanceSensor dLeft,dRight;

    //switch drive motor power
    double power = 100;
    boolean positionLift, positionIdle, positionGather;
    boolean dumpUp, dumpDown, dump0, dumpMid, dump1;
    boolean armStartOut;
    boolean isIdle=false;
    boolean isGather=false; boolean isGatherHere = false;
    boolean isLift=false; boolean isLiftHere = false;
    boolean isArmStart=false;
    boolean liftUp, liftDown;
    boolean dumpBreak=false, craterOut= false;

    float sweepOut, sweepIn;
    float dumpLifting,hangLeft,hangRight;;
    float lDrive, rDrive;

    double sVoltReading, sServoValue;
    double initialTime, currentTime;
    double count = 0;

    public void init() {

        //Mapping
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        shoulder = hardwareMap.dcMotor.get("arm");
        lift = hardwareMap.dcMotor.get("lift");
        magSwitchUp = hardwareMap.digitalChannel.get("magSwitchUp");
        magSwitchDown = hardwareMap.digitalChannel.get("magSwitchDown");
        magLatchClose = hardwareMap.digitalChannel.get("magLatchClose");
        magLatchOpen = hardwareMap.digitalChannel.get("magLatchOpen");
        latch = hardwareMap.crservo.get("latch");
        wrist = hardwareMap.servo.get("wrist");
        shoulder = hardwareMap.dcMotor.get("arm");
        dumpLift = hardwareMap.dcMotor.get("dumpLift");
        dump = hardwareMap.servo.get("dump");
        dumpDoor = hardwareMap.servo.get("dumpDoor");
        pShoulder = hardwareMap.analogInput.get("pShoulder");
        sweep =hardwareMap.dcMotor.get("sweep");
        dLeft = hardwareMap.get(DistanceSensor.class, "dLeft");
        dRight = hardwareMap.get(DistanceSensor.class, "dRight");

        //Set direction
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        latch.setDirection(DcMotorSimple.Direction.FORWARD);
        sweep.setDirection(DcMotorSimple.Direction.FORWARD);
        dumpLift.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.REVERSE);
    }

    public void init_loop(){
    }

    //Time to endlessly repeat!
    public void loop(){
        telemetry.addData("right:",dRight.getDistance(DistanceUnit.CM));
        telemetry.addData("left:",dLeft.getDistance(DistanceUnit.CM));


        //Sets gamepad variables to buttons
        //gamepad 1
        lDrive = gamepad1.left_stick_y;
        rDrive= gamepad1.right_stick_y;
        hangLeft = gamepad1.left_trigger;
        hangRight = gamepad1.right_trigger;
        liftUp = gamepad1.right_bumper;
        liftDown = gamepad1.left_bumper;
        dump0 = gamepad1.dpad_left;
        dumpMid = gamepad1.dpad_up;
        dump1 = gamepad1.dpad_right;
        //gamepad 2
        positionGather = gamepad2.dpad_up;
        positionIdle = gamepad2.dpad_left;
        positionLift = gamepad2.dpad_down;
        armStartOut = gamepad2.dpad_right;
        sweepIn = gamepad2.right_trigger;
        sweepOut = gamepad2.left_trigger;
        dumpUp = gamepad2.right_bumper;
        dumpDown = gamepad2.left_bumper;
        dumpLifting = gamepad2.right_stick_y;

        //motor speed for drive
        left.setPower((lDrive * 100) / -power);
        right.setPower((rDrive * 100) / -power);




        //switches the variables that activate the arm to different positions
        if(positionGather) {
            isGather=true;
            //isArmStart=false;
            isIdle=false;
            isLift=false;
            isGatherHere=false;
        }
        else if(positionIdle) {
            isGather=false;
            //isArmStart=false;
            isIdle=true;
            isLift=false;
        }
        else if(positionLift) {
            isGather=false;
            //isArmStart=false;
            isIdle=false;
            isLift=true;
            isLiftHere = false;
        }
        /*else if(armStartOut) {
            isGather=false;
            //isArmStart=true;
            isIdle=false;
            isLift=false;
        }*/




        //moves latch in/out
        if ((hangRight>0.25)&& magLatchClose.getState()) {
            latch.setPower(1);
        }
        else if ((hangLeft>0.25) && magLatchOpen.getState()) {
            latch.setPower(-1);
        }
        else {
            latch.setPower(0);
        }

        ///Manual lift, stops at sensors
        if (liftDown && magSwitchDown.getState()) {
            lift.setPower(-1);
        }
        else if (liftUp && magSwitchUp.getState()) {
            lift.setPower(1);
        }
        else {
            lift.setPower(0);
        }



        //the dump servo go forward and back
        if(dumpUp){
            // dump.setPosition(0);
            dumpBreak = true;
        }
        else if(dumpDown){
            dump.setPosition(0.95);
        }
        else{
            dump.setPosition(0.45);
        }

        //puts the dump up and down
        if(dumpBreak)
        {
            dumpLift.setPower(0.3);
        }
        else if (dumpLifting < -0.25) {
            dumpLift.setPower(0.9);

        }
        else if (dumpLifting > 0.25) {
            dumpLift.setPower(-0.9);
            //System.currentTimeMillis();
        }
        else {
            dumpLift.setPower(0);
        }

        if (dumpLifting > 0.25 || dumpLifting < -0.25) {
            dumpBreak=false;
        }



        //sets dump door positions
        if (dump0) {
            dumpDoor.setPosition(0);
        }
        else if (dumpMid) {
            dumpDoor.setPosition(0.45);
        }
        else if(dump1) {
            dumpDoor.setPosition(1);
        }

        //sweeper
        if(sweepIn > 0.25) {
            sweep.setPower(-1);//-1
        }
        else if(sweepOut > 0.25) {
            sweep.setPower(1);//1
        }
        else {
            sweep.setPower(0);
        }


        //voltage of shoulder potentiometer
        sVoltReading = (float) pShoulder.getVoltage();
        //converts voltage of the potentiometer to servo values
        sServoValue = sVoltReading/3.25;

        if(armStartOut &&sServoValue > 0.2){
            isGather = false;
            isIdle = false;
            isLift = false;
            shoulder.setPower(0.6);
        }else{
            shoulder.setPower(0);
        }

        //sets shoulder to mineral gathering position
        if(isGather){
            wrist.setPosition(0.1);//0.2
            double sGatherPos = .616;//.612;//660

            if(sServoValue < (sGatherPos - .01) || sServoValue > (sGatherPos + .01) ) {
                sVoltReading = (float) pShoulder.getVoltage();
                sServoValue = sVoltReading / 3.25;
                if (sServoValue < (sGatherPos - .01) ) {
                    if(isGatherHere){
                        shoulder.setPower(-0.5);
                    }
                    else{
                        shoulder.setPower(-1);
                    }

                } /*else if (sServoValue > (sGatherPos + .01) ) {
                    shoulder.setPower(0.4);
                }*/
                else {
                    shoulder.setPower(0);
                }
            }
            if((sServoValue > (sGatherPos - .02)) && (sServoValue < (sGatherPos))){
                isGatherHere=true;
            }
        }//end of isGather

        //sets shoulder to slightly over the crater
        else if (isIdle) {
            wrist.setPosition(0.2);
            double sIdlePos = .5;

            if(sServoValue < (sIdlePos - .05) || sServoValue > (sIdlePos + .05) ) {
                sVoltReading = (float) pShoulder.getVoltage();
                sServoValue = sVoltReading / 3.25;
                if (sServoValue < (sIdlePos - .01) ) {
                    shoulder.setPower(-0.6);
                } else if (sServoValue > (sIdlePos + .01) ) {
                    shoulder.setPower(0.6);
                } else {
                    shoulder.setPower(0);
                }
            }
        }//end of isIdle

        //sets shoulder to release minerals in the lift
        else if (isLift) {
            double armStart = 0;

            if (armStart == 0) {
                double sLiftPos = .255;//.270 //.252

                if(sServoValue < (sLiftPos - .01) || sServoValue > (sLiftPos + .005) ) {
                    sVoltReading = (float) pShoulder.getVoltage();
                    sServoValue = sVoltReading / 3.25;
                    if (sServoValue < (sLiftPos - .01) ) {
                        if(isLiftHere){
                            shoulder.setPower(-0.2);
                        }
                        else{
                            shoulder.setPower(-1);
                        }
                    }
                    else if (sServoValue > (sLiftPos + .01) ) {
                        if(isLiftHere){
                            shoulder.setPower(0.2);
                        }
                        else{
                            shoulder.setPower(1);
                        }
                    }
                    else {
                        shoulder.setPower(0);
                        wrist.setPosition(1);
                    }
                    if(sServoValue < 0.58) {
                        wrist.setPosition(1);
                    }
                    armStart++;
                    if((sServoValue > (sLiftPos - .02)) && (sServoValue < (sLiftPos + .02))){
                        isLiftHere=true;
                    }
                }
            }
        }

        //sets shoulder to slowly move out of the robot and activate the servo once done
        /*else if (isArmStart) {
            double armStart = 0;

            if (armStart == 0) {
                double sStartPos = .5;

                if(sServoValue < (sStartPos - .02) || sServoValue > (sStartPos + .02) ) {
                    sVoltReading = (float) pShoulder.getVoltage();
                    sServoValue = sVoltReading / 3.25;
                    if (sServoValue < (sStartPos - .02) ) {
                        shoulder.setPower(-0.6);
                    } else if (sServoValue > (sStartPos + .02) ) {
                        shoulder.setPower(0.6);
                    } else {
                        shoulder.setPower(0);
                        wrist.setPosition(0);
                    }
                    armStart++;
                }
            }
        }//end of isArmStart*/




    }//end of loop

    //Stops all motors
    public void stop(){
        left.setPower(0);
        right.setPower(0);
        shoulder.setPower(0);
        lift.setPower(0);
        shoulder.setPower(0);
        dumpLift.setPower(0);
    }//end of stop
}//end

