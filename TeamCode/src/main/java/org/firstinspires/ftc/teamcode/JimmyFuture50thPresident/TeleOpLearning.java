package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp(name="Jimmy")
public class TeleOpLearning extends OpMode {

    //Defining

    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor BackRight;
    DcMotor BackLeft;

    //Sensor Defining

    DistanceSensor distanceL, distanceR, distanceC;

    double distance = 18.0832517;


    public void init() {

        //Setup

        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");

        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
    }

    public void loop() {

        //Setup

        float rsx = gamepad1.right_stick_x;
        float rsy = gamepad1.right_stick_y;
        float lsx = gamepad1.left_stick_x;
        float lsy = gamepad1.left_stick_y;
        boolean b =gamepad1.b;
        boolean x =gamepad1.x;
        boolean lb =gamepad1.left_bumper;
        boolean rb =gamepad1.right_bumper;

        float rt = gamepad1.right_trigger;

        telemetry.addData("RightTrigger", rt);
        telemetry.addData("RightStickX", rsx);
        telemetry.addData("RightStickY", rsy);
        telemetry.addData("LeftStickX", lsx);
        telemetry.addData("LeftStickY", lsy);
        telemetry.addData("B", b);
        telemetry.addData("X", x);
        telemetry.addData("Left Bumper", lb);
        telemetry.addData("Right Bumper", rb);

        //Original Movement

        if (rsx > 0.4) {
            FrontRight.setPower(0.5);
            FrontLeft.setPower(0.5);
            BackRight.setPower(0.5);
            BackLeft.setPower(0.5);
        } else if (rsx < -0.4) {
            FrontRight.setPower(-0.5);
            FrontLeft.setPower(-0.5);
            BackRight.setPower(-0.5);
            BackLeft.setPower(-0.5);

            //Secondary Movement

        } else if (rt > 0.2) {
            FrontRight.setPower(0.8);
            FrontLeft.setPower(0.8);
            BackRight.setPower(0.8);
            BackLeft.setPower(0.8);

            //Voluntary Movement

        } else if (lsx > 0.4) {
            FrontRight.setPower(0.3);
            FrontLeft.setPower(0.3);
            BackRight.setPower(0.3);
            BackLeft.setPower(0.3);
        } else if (lsx < -0.4) {
            FrontRight.setPower(-0.8);
            FrontLeft.setPower(-0.8);
            BackRight.setPower(-0.8);
            BackLeft.setPower(-0.8);
        }

        //Secondary Voluntary Movement

        else if (b) {
            FrontRight.setPower(0.9);
        }

        else if (x) {
            FrontLeft.setPower(0.9);
        }

        else if (lb){
            BackLeft.setPower(0.9);
        }

        else if (rb) {
            BackRight.setPower(0.9);
        }

        //Final Stopper

        else {
            FrontRight.setPower(0);
            FrontLeft.setPower(0);
            BackRight.setPower(0);
            BackLeft.setPower(0);
        }
    }
}
