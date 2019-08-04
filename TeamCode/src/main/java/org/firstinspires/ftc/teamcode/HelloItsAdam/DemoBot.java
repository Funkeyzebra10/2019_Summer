package org.firstinspires.ftc.teamcode.HelloItsAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//test
@TeleOp(name="DemoBot")
public class DemoBot extends OpMode {
    DcMotor L, R;
    float left,right;

    public void init(){
        L = hardwareMap.dcMotor.get("L");
        R = hardwareMap.dcMotor.get("R");


        L.setDirection(DcMotorSimple.Direction.REVERSE);
        R.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void loop(){

        left=gamepad1.left_stick_y;
        right=gamepad1.right_stick_y;

        L.setPower(left);
        R.setPower(right);


    }
    public void stop(){

    }
}
