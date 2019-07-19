package org.firstinspires.ftc.teamcode.Harryyy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "swerveee")
public class turnyWheelBoi extends OpMode {
/*
* @todo test bot
* @body this is what the issue will say
*/
    DcMotor fLeft, fRight, bLeft, bRight;
    Servo fLeftS, fRightS, bLeftS, bRightS;
    float lDrive, rDrive;
    float positionControllerY = 0;
    float positionControllerX = 0;
    double servoPos;
    float forward;
    double northPower, northeastPower, eastPower,southeastPower, south, power ;



    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        fLeftS = hardwareMap.servo.get("fLeftS");
        fRightS = hardwareMap.servo.get("fRightS");
        bLeftS = hardwareMap.servo.get("bLeftS");
        bRightS = hardwareMap.servo.get("bRightS");

        //Motor Directions
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servo Directions
        fRightS.setDirection(Servo.Direction.FORWARD);
        fLeftS.setDirection(Servo.Direction.FORWARD);
        bRightS.setDirection(Servo.Direction.FORWARD);
        bLeftS.setDirection(Servo.Direction.FORWARD);

    }

    public void loop() {

        lDrive = gamepad1.left_stick_y;
        rDrive= gamepad1.right_stick_y;
        positionControllerY = gamepad1.right_stick_y;
        positionControllerX = gamepad1.right_stick_x;

        telemetry.addData("Controller PositionY",positionControllerY );
        telemetry.addData("Controller PositionX",positionControllerX);

        bLeft.setPower((lDrive * 100) / -120);
        bRight.setPower((rDrive * 100) / -120);
        fLeft.setPower((lDrive * 100) / -120);
        fRight.setPower((rDrive * 100) / -120);

        if(positionControllerX == 0 && positionControllerY == -1){
            fLeftS.setPosition(0);
            fRightS.setPosition(0);
            bLeftS.setPosition(0);
            bRightS.setPosition(0);
        }
        else if((positionControllerX < 1 && positionControllerX > 0) && (positionControllerY < 0 )){
            fLeftS.setPosition(0.125);
            fRightS.setPosition(0.125);
            bLeftS.setPosition(0.125);
            bRightS.setPosition(0.125);
        }
        else if(positionControllerX == 1 && positionControllerY < 0){
            fLeftS.setPosition(0.25);
            fRightS.setPosition(0.25);
            bLeftS.setPosition(0.25);
            bRightS.setPosition(0.25);
        }
        else if((positionControllerX > 0 && positionControllerX < 1) && (positionControllerY > 0 && positionControllerY < 1)){
            fLeftS.setPosition(0.375);
            fRightS.setPosition(0.375);
            bLeftS.setPosition(0.375);
            bRightS.setPosition(0.375);
        }
        else if (positionControllerX == 0 && positionControllerY == 1){
            fLeftS.setPosition(0.5);
            fRightS.setPosition(0.5);
            bLeftS.setPosition(0.5);
            bRightS.setPosition(0.5);
        }
        else if  ((positionControllerX > -1 && positionControllerX < 0)&& (positionControllerY > 0 && positionControllerY < 1)){
            fLeftS.setPosition(0.625);
            fRightS.setPosition(0.625);
            bLeftS.setPosition(0.625);
            bRightS.setPosition(0.625);
        }
        else if (positionControllerX == -1 && positionControllerY == 0){
            fLeftS.setPosition(0.75);
            fRightS.setPosition(0.75);
            bLeftS.setPosition(0.75);
            bRightS.setPosition(0.75);
        }
        else if ((positionControllerX > -1 && positionControllerX < 0) && (positionControllerY < -1 && positionControllerY > 0)){
            fLeftS.setPosition(0.875);
            fRightS.setPosition(0.875);
            bLeftS.setPosition(0.875);
            bRightS.setPosition(0.875);
        }
        else{
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
    }


    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

}
