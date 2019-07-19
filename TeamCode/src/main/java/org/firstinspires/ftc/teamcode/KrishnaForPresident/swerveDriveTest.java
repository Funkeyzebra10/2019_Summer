package org.firstinspires.ftc.teamcode.KrishnaForPresident;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Swerve Drive Test")
public class swerveDriveTest extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bLeft, bRight;

    //Servos
    Servo sFLeft, sFRight, sBLeft, sBRight;

    double servoPosOutputD;
    double lastServoPosOutput = 0;

    double servoPosOutput;

    //Drive
    float leftX, leftY, rightX, rightY;

    //Functions
    boolean home;

    //Servo positions
    float servoPosL = 0;
    float servoPosR = 0;

    double servoLeftOutput, servoRightOutput;

    boolean driveMode = true; //true is arcade, false is dual joystick.

    public void init(){

        //Initializing/mapping motors and servos
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        sFLeft = hardwareMap.servo.get("sFLeft");
        sFRight = hardwareMap.servo.get("sFRight");
        sBLeft = hardwareMap.servo.get("sBLeft");
        sBRight = hardwareMap.servo.get("sBRight");

        //Directions
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    public void loop(){

        if(driveMode) {
            //Sets the drive function input
            leftX = gamepad1.left_stick_x;
            leftY = 0 - gamepad1.left_stick_y;

            //Sets the servo positioning input
            rightX = gamepad1.right_stick_x;
            rightY = 0 - gamepad1.right_stick_y;

            servoPosOutput = (rightX / 2) + 0.5;


            sFRight.setPosition(servoPosOutput);
            sFLeft.setPosition(servoPosOutput);
            sBRight.setPosition(servoPosOutput);
            sBLeft.setPosition(servoPosOutput);

            if(leftY > 0.15 || leftY < -0.15 /*&& (driveX <= 0.15 && driveX >= -0.15)*/) {
                fLeft.setPower(leftY /2);
                fRight.setPower(leftY /2);
                bLeft.setPower(leftY /2);
                bRight.setPower(leftY /2);
            }
            else if(leftY < 0.15 && leftX > 0.15) {
                fLeft.setPower(leftX);
                fRight.setPower(-leftX);
                bLeft.setPower(leftX);
                bRight.setPower(-leftX);
            }
            else if((leftY < 0.15 && leftY > -0.15)&& leftX < -0.15) {
                fLeft.setPower(leftX);
                fRight.setPower(-leftX);
                bLeft.setPower(leftX);
                bRight.setPower(-leftX);
            }
            else {
                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
            }
            telemetry.addData("servo output: ", servoPosOutput);
        }
        else {
            //Sets the servo positioning input
            servoPosL = gamepad1.left_stick_x;
            servoPosR = gamepad1.right_stick_x;

            leftY = 0 - gamepad1.left_stick_y;
            rightY = 0 - gamepad1.right_stick_y;


            //Uses an exponential function to turn the robot's wheels
            servoLeftOutput = (0.5 * (Math.pow(servoPosL, 7))) + 0.5;
            servoRightOutput = (0.5 * (Math.pow(servoPosR, 7))) + 0.5;

            //Servo positioning
            sFLeft.setPosition(servoLeftOutput);
            sFRight.setPosition(servoRightOutput);
            sBLeft.setPosition(servoLeftOutput);
            sBRight.setPosition(servoRightOutput);

            //Drive
            fLeft.setPower((leftY * 100)/120);
            fRight.setPower((rightY * 100)/120);
            bLeft.setPower((leftY * 100)/120);
            bRight.setPower((rightY * 100)/120);

            telemetry.addData("ServoL: ", servoLeftOutput);
            telemetry.addData("ServoR: ", servoRightOutput);

        }

        if(gamepad1.a && driveMode) {
            driveMode = false;
        }
        else if (gamepad1.a && !driveMode){
            driveMode = true;
        }

        telemetry.addData("x: ", rightX);
        telemetry.addData("y: ", rightY);
        telemetry.addData("drivex: ", leftX);
        telemetry.addData("drivey: ", leftY);
//        telemetry.addData("angle: ", servoPosOutputD);
//        telemetry.addData("last output: ", lastServoPosOutput);


        //Broken TAN method

        /*//Sets the output
        if(rightX == 0 && rightY >= 0) {
            servoPosOutputD = 0;
            servoPosOutput = Math.round(servoPosOutputD / 180 * Math.pow(10, 5)) / Math.pow(10,5);
            lastServoPosOutput = servoPosOutput;
        }
        else if(rightX == 0 && rightY < 0) {
            servoPosOutput = lastServoPosOutput;
        }
        else if(rightY == 0 && rightX > 0) {
            servoPosOutputD = 90;
            servoPosOutput = Math.round(servoPosOutputD / 180 * Math.pow(10, 5)) / Math.pow(10,5);
            lastServoPosOutput = servoPosOutput;
        }
        else if(rightY == 0 && rightX < 0) {
            servoPosOutput = lastServoPosOutput;
        }
        else {
            servoPosOutputD = 90 - (Math.toDegrees(Math.atan(Math.abs(rightY) / Math.abs(rightX))));
            if(rightY > 0) {
                if(rightX > 0) {
                    servoPosOutputD += 90;
                    servoPosOutput = Math.round(servoPosOutputD / 180 * Math.pow(10, 5)) / Math.pow(10,5);
                    lastServoPosOutput = servoPosOutput;
                }
                else {
                    servoPosOutput = (Math.round(servoPosOutputD / 180 * Math.pow(10, 5)) / Math.pow(10,5));
                    lastServoPosOutput = servoPosOutput;
                }
            }
            else {
                servoPosOutput = lastServoPosOutput;
            }
        }
        */

    }

    public void stop(){

    }

}
