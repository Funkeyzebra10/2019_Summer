package org.firstinspires.ftc.teamcode.Reference;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by emory on 8/15/2018.
 */
//@TeleOp(name = "Laser Test")

public class laserTest extends OpMode {
    //Defining motors
    DcMotor bRight;
    DcMotor bLeft;
    DcMotor fRight;
    DcMotor fLeft;

    //Defining functions
    float lift;

    //Defining magSwitch
    DistanceSensor laser;




    public void init() {

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        laser = hardwareMap.get(DistanceSensor.class, "laser");


        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);



    }

    public void loop(){
        telemetry.addData("range", String.format("%.01f mm", laser.getDistance(DistanceUnit.MM)));
        telemetry.addData("range", String.format("%.01f cm", laser.getDistance(DistanceUnit.CM)));
        telemetry.addData("range", String.format("%.01f in", laser.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }

    public void stop(){
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


}
