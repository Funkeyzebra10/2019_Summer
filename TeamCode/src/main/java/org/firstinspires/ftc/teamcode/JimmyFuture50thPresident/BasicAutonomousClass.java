package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="Jimmy Test")
public class BasicAutonomousClass extends LinearOpMode {


    DcMotor motorRight = null;
    DcMotor motorLeft = null;
    Servo armservo = null;



    static final double ARM_REDACTED_POSITION = 0.2;
    static final double ARM_EXTENDED_POSITION = 0.8;



    @Override
    public void runOpMode() throws InterruptedException {

        motorRight = hardwareMap.dcMotor.get("FrontRight");
        motorLeft = hardwareMap.dcMotor.get("FrontLeft");
        armservo = hardwareMap.servo.get("armservo");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        DriveForward(1);
        Thread.sleep(1000);

        TurnLeft(1);
        Thread.sleep(700);

        DriveForward(0.5);
        Thread.sleep(1000);

        TurnRight(1);
        Thread.sleep(700);

        DriveForward(1);
        Thread.sleep(2000);

        TurnRight(1);
        Thread.sleep(700);

        DriveForward(0.5);
        Thread.sleep(700);

        Stop();

    }

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void TurnLeft(double power)
    {
     motorLeft.setPower(-power);
     motorRight.setPower(power);
    }

    public void TurnRight(double power)
    {
    motorLeft.setPower(power);
    motorRight.setPower(-power);
    }

    public void Stop()
    {
    motorLeft.setPower(0);
    motorRight.setPower(0);
    }
}

