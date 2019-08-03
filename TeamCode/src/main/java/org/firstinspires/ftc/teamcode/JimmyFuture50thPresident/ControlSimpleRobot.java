package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Simple Robot")
public class ControlSimpleRobot extends LinearOpMode {

        DcMotor motorRight;
        DcMotor motorLeft;
        Servo armservo;
        boolean a;
        boolean b;

        static final double ARM_REDACTED_POSITION = 0.2;
        static final double ARM_EXTENDED_POSITION = 0.8;


        @Override
        public void runOpMode() throws InterruptedException {

            motorRight = hardwareMap.dcMotor.get("FrontRight");
            motorLeft = hardwareMap.dcMotor.get("FrontLeft");
            armservo = hardwareMap.servo.get("armservo");

            motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();
            while (opModeIsActive()){
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                if (gamepad2.a) {
                    motorLeft.setPower(-1);
                    motorRight.setPower(1);
                }

                if (gamepad2.b) {
                    motorLeft.setPower(1);
                    motorRight.setPower(-1);
                }
            }
        }
    }
