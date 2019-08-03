package org.firstinspires.ftc.teamcode.NateDoesntDebate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp Tutorial", group = "Tutorials")
public class Workshop extends LinearOpMode
{
    private DcMotor motorleft;

    private DcMotor motorRight;

    private Servo armServo;

    private static final double ARM_RETRACTED_POSITION = 0.2;

    private static final double ARM_EXTENDED_POSITION = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        motorleft = hardwareMap.dcMotor.get("MotorLeft");
        motorRight = hardwareMap.dcMotor.get("MotorRight");

        motorleft.setDirection(DcMotor.Direction.REVERSE);

        armServo = hardwareMap.servo.get("armServo");

        armServo.setPosition(ARM_RETRACTED_POSITION);

        waitForStart ();

        while(opModeIsActive())
        {
            motorleft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

             if(gamepad2.a)
             {
                armServo.setPosition(ARM_EXTENDED_POSITION);
            }
            if(gamepad2.b)
            {
                armServo.setPosition(ARM_RETRACTED_POSITION);
            }

            idle();
        }
    }
}
