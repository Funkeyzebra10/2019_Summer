package org.firstinspires.ftc.teamcode.KrishnaForPresident;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "swerve drive auto")
public class SwerveDriveAuto extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bLeft, bRight;

    //Servos
    Servo sFLeft, sFRight, sBLeft, sBRight;

    //Mr. Worldwide (global variables)
    double initSystemTime = System.currentTimeMillis();

    public void init() {
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

        sFLeft.setPosition(0.5);
        sBLeft.setPosition(0.5);
        sFRight.setPosition(0.5);
        sBRight.setPosition(0.5);

    }

    public void init_loop() {
        initSystemTime = System.currentTimeMillis();
    }

    public void loop() {

        boolean exitStatus = false;
        if(System.currentTimeMillis() - initSystemTime < 4000) {
            ServoTurlL(15);
            ServoTurnR(25);
        }
        else {
            ServoTurnAll(180);
            exitStatus = true;
        }

        telemetry.addData("System time: ", System.currentTimeMillis() - initSystemTime);
        telemetry.addData("Exit status: ", exitStatus);
    }

    public void stop() {

    }














    //Methods


    public double DegreesTurnSwerve(double inputAngle) {
        double outputAngle = 0;

        if(inputAngle <= 0 || inputAngle > 360) {
            outputAngle = 0;
        }
        else if (inputAngle > 0 && inputAngle <= 180) {
            outputAngle = (1/180) * inputAngle;
        }
        else {
            outputAngle = ((-1/180) * inputAngle) + 2;
        }

        return outputAngle;
    }

    public void ServoTurnAll (double inputAngle) {
        double turnTo = DegreesTurnSwerve(inputAngle);
        sFLeft.setPosition(turnTo);
        sBLeft.setPosition(turnTo);
        sFRight.setPosition(turnTo);
        sBRight.setPosition(turnTo);
    }

    public void ServoTurlL (double inputAngle) {
        double turnTo = DegreesTurnSwerve(inputAngle);
        sFLeft.setPosition(turnTo);
        sBLeft.setPosition(turnTo);
    }

    public void ServoTurnR (double inputAngle) {
        double turnTo = DegreesTurnSwerve(inputAngle);
        sFRight.setPosition(turnTo);
        sBRight.setPosition(turnTo);
    }

}