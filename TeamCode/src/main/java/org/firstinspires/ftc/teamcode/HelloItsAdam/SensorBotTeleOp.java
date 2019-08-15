package org.firstinspires.ftc.teamcode.HelloItsAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//test
@TeleOp(name="SensorBot")
public class SensorBotTeleOp extends OpMode {
    DcMotor fLeft, fRight, bLeft, bRight;
    //DistanceSensor distanceL, distanceCL, distanceCR, distanceR;
    float leftDrive,rightDrive,leftStrafe,rightStrafe;
    public void init(){
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        /*distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceCL = hardwareMap.get(DistanceSensor.class, "distanceCL");
        distanceCR = hardwareMap.get(DistanceSensor.class, "distanceCR");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");*/

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void loop(){
        /*
        telemetry.addData("Left Distance(cm)",distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Left Distance(cm)",distanceCL.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Right Distance(cm)",distanceCR.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance(cm)",distanceR.getDistance(DistanceUnit.CM));*/

        leftDrive=gamepad1.left_stick_y;
        rightDrive=gamepad1.right_stick_y;

        leftStrafe=gamepad1.left_trigger;
        rightDrive=gamepad1.right_trigger;

        if(leftStrafe>0.1) {
            bLeft.setPower((leftStrafe*100)/128);
            fLeft.setPower(-(leftStrafe*100)/128);
            bRight.setPower(-(leftStrafe*100)/128);
            fRight.setPower((leftStrafe*100)/128);
        }
        else if(rightStrafe>0.1){
            bLeft.setPower(-(rightStrafe*100)/128);
            fLeft.setPower((rightStrafe*100)/128);
            bRight.setPower((rightStrafe*100)/128);
            fRight.setPower(-(rightStrafe*100)/128);
        }
        else{
            bLeft.setPower((leftDrive*100)/128);
            fLeft.setPower((leftDrive*100)/128);
            bRight.setPower((rightDrive*100)/128);
            fRight.setPower((rightDrive*100)/128);
        }


    }
    public void stop(){

    }
}
