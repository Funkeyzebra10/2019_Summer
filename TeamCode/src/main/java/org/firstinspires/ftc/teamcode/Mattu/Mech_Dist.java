package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "multi-distance", group = "Sensor")
public class Mech_Dist extends OpMode {

    DcMotor bLeft, bRight, fLeft, fRight;
    private DistanceSensor distanceL, distanceCL, distanceCR, distanceR;
    double left, leftF, rightF, right;
    String[] position= new String[3];
    int distance = 12 ;

    public void init () {
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

    public void loop () {
        left = distanceL.getDistance(DistanceUnit.CM);
        leftF = distanceCL.getDistance(DistanceUnit.CM);
        rightF = distanceCR.getDistance(DistanceUnit.CM);
        right = distanceR.getDistance(DistanceUnit.CM);

        //Center
        if (leftF < distance && rightF < distance) {
            position[1] = "Center";

        }
        else
        {
            position[1] = "";

        }

        //Front Left
        if (leftF < distance || (leftF < distance && left < distance + 6))
        {
            position[0] = "Front Left";

        }
        //Left
        else if (left < distance + 6) {
            position[0] = "Left";
        }
        else{
            position[0] = "";

        }

        //Front Right
        if (rightF < distance || (rightF < distance && right < distance + 6))
        {
            position[2] = "Front Right";

        }
        //Right
        else if (right < distance + 6) {
            position[2] = "Right";

        }
        else{
            position[2] = "";

        }

        if( (position[1].equals("Center") || position[2].equals( "Front Right") || position[0].equals("Front Left"))||(position[2].equals("Right") && position[0].equals("Left")))
        {
            fLeft.setPower(-0.2);
            fRight.setPower(-0.2);
            bLeft.setPower(-0.2);
            bRight.setPower(-0.2);
        }
        else if(position[2].equals("Right"))
        {
            fLeft.setPower(0.8);
            fRight.setPower(-0.8);
            bLeft.setPower(-0.8);
            bRight.setPower(0.8);
        }
        else if(position[0].equals("Left"))
        {
            fLeft.setPower(-0.8);
            fRight.setPower(0.8);
            bLeft.setPower(0.8);
            bRight.setPower(-0.8);
        }
        else
        {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
        }
        telemetry.addData("Obstacles: ", Arrays.toString(position));
        telemetry.addData("left:",left);
        telemetry.addData("left front:",leftF);
        telemetry.addData("right front:",rightF);
        telemetry.addData("right:",right);

    }

    public void stop () {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

}
