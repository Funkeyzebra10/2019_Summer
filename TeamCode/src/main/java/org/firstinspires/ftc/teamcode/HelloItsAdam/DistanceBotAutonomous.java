package org.firstinspires.ftc.teamcode.HelloItsAdam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Roomba Bot")
public class DistanceBotAutonomous extends OpMode {
    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceCL, distanceCR, distanceR;
    float leftDrive,rightDrive;
    double left,leftF,rightF,right;
    public void init(){
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceCL = hardwareMap.get(DistanceSensor.class, "distanceCL");
        distanceCR = hardwareMap.get(DistanceSensor.class, "distanceCR");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop(){
        telemetry.addData("Left Distance(cm)",distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Left Distance(cm)",distanceCL.getDistance(DistanceUnit.CM));
        telemetry.addData("Center Right Distance(cm)",distanceCR.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance(cm)",distanceR.getDistance(DistanceUnit.CM));

        left = distanceL.getDistance(DistanceUnit.CM);
        leftF = distanceCL.getDistance(DistanceUnit.CM);
        rightF = distanceCR.getDistance(DistanceUnit.CM);
        right = distanceR.getDistance(DistanceUnit.CM);

        if(leftF < 30 && rightF < 30)
        {
            bLeft.setPower(0.25);
            fLeft.setPower(0.25);
            bRight.setPower(-0.25);
            fRight.setPower(-0.25);
        }
        else if(leftF < 30 && rightF < 30 && right < 30){
            bLeft.setPower(-0.25);
            fLeft.setPower(-0.25);
            bRight.setPower(-0.15);
            fRight.setPower(-0.15);
        }
        else if(leftF < 30 && rightF < 30 && left < 30){
            bLeft.setPower(-0.15);
            fLeft.setPower(-0.15);
            bRight.setPower(-0.25);
            fRight.setPower(-0.25);
        }
        else if(leftF < 20)
        {
            bLeft.setPower(0.25);
            fLeft.setPower(0.25);
            bRight.setPower(-0.25);
            fRight.setPower(-0.25);
        }
        else if(rightF < 25)
        {
            bLeft.setPower(0.25);
            fLeft.setPower(0.25);
            bRight.setPower(-0.25);
            fRight.setPower(-0.25);
        }
        else if(right < 15 || rightF < 30){
            bLeft.setPower(-0.25);
            fLeft.setPower(-0.25);
            bRight.setPower(0.25);
            fRight.setPower(0.25);
        }
        else if(left < 15 || leftF < 30){
            bLeft.setPower(0.25);
            fLeft.setPower(0.25);
            bRight.setPower(-0.25);
            fRight.setPower(-0.25);
        }
        else{
            bLeft.setPower(0.25);
            fLeft.setPower(0.25);
            bRight.setPower(0.25);
            fRight.setPower(0.25);
        }


    }
    public void stop(){

    }
}
