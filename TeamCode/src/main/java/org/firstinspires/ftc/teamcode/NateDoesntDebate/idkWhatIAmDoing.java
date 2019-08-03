package org.firstinspires.ftc.teamcode.NateDoesntDebate;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "wacky controles")
public class idkWhatIAmDoing extends LinearOpMode
    {
        private DcMotor FrontLeft;

        private DcMotor FrontRight;

        boolean a, b;

        int buttonPressed;

        public void runOpMode() throws InterruptedException
        {
            FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
            FrontRight = hardwareMap.dcMotor.get("FrontRight");
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);

            buttonPressed = 0;

            waitForStart ();

            while(opModeIsActive())
            {
                a = gamepad1.a;
                b = gamepad1.b;

                if (a)
                {
                    buttonPressed = 0;
                }else if (b){
                    buttonPressed = 1;
                }
                if (buttonPressed == 1){
                    FrontLeft.setPower(gamepad1.left_stick_y);
                    FrontRight.setPower(-gamepad1.right_stick_y);
                }else {
                    FrontLeft.setPower(gamepad1.left_stick_y);
                    FrontRight.setPower(gamepad1.right_stick_y);
                }
                idle();
            }
        }
    }
