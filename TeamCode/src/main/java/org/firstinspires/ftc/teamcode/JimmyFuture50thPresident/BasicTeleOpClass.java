package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "BasicTeleOpClass")
public class BasicTeleOpClass extends LinearOpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    Servo armservo;

    static final double ARM_REDACTED_POSITION = 0.2;
    static final double ARM_EXTENDED_POSITION = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {

        motorRight = hardwareMap.dcMotor.get("FrontRight");
        motorLeft = hardwareMap.dcMotor.get("FrontLeft");
        armservo = hardwareMap.servo.get("armservo");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        motorLeft.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);

        if(gamepad2.a) {
            armservo.setPosition(ARM_EXTENDED_POSITION);
        }

        if(gamepad2.b) {
            armservo.setPosition(ARM_REDACTED_POSITION);
        }

        while (opModeIsActive()) ;
        {
            idle();
        }
    }
}
