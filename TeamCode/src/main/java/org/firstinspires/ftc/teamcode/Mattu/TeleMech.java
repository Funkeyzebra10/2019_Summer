package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleMech")
public class TeleMech extends OpMode {
    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;

    double leftX, leftY, rightX, rightY;

    boolean leftBumper, rightBumper;

    double distance = 18.0832517;

    //Calculate the power needed to move in a specific direction.
    double fL() {
        return leftY + leftX;
    }

    double fR() {
        return rightY - rightX;
    }

    double bL() {
        return leftY - leftX;
    }

    double bR() {
        return rightY + rightX;
    }

    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);

        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
    }

    public void loop() {
        //Get inputs from left and right sticks.
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;
        leftBumper = gamepad1.left_bumper;
        rightBumper = gamepad1.right_bumper;

        telemetry.addData("Left Distance", distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Distance", distanceC.getDistance(DistanceUnit.CM));
        telemetry.addData("Left X: ", leftX);
        telemetry.addData("Left Y: ", leftY);
        telemetry.addData("Right X: ", rightX);
        telemetry.addData("Right Y", rightY);
        telemetry.addData("Front Left Power: ", fL());
        telemetry.addData("Front Right Power: ", fR());
        telemetry.addData("Back Left Power: ", bL());
        telemetry.addData("Back Right Power: ", bR());

        if (leftBumper) {
            if (distanceL.getDistance(DistanceUnit.CM) >= distance) {
                fLeft.setPower(-0.8);
                fRight.setPower(0.8);
                bLeft.setPower(0.8);
                bRight.setPower(-0.8);
            }
        }

        if (rightBumper) {
            if (distanceR.getDistance(DistanceUnit.CM) >= distance) {
                fLeft.setPower(0.8);
                fRight.setPower(-0.8);
                bLeft.setPower(-0.8);
                bRight.setPower(0.8);
            }
        }

        setPower();
    }

    public void setPower() {
        if (leftY > 0 && rightY > 0) {
            if (distanceC.getDistance(DistanceUnit.CM) >= distance) {
                fLeft.setPower(fL());
                fRight.setPower(fR());
                bLeft.setPower(bL());
                bRight.setPower(bR());
            }
        }
        else {
            fLeft.setPower(fL());
            fRight.setPower(fR());
            bLeft.setPower(bL());
            bRight.setPower(bR());
        }
    }
}