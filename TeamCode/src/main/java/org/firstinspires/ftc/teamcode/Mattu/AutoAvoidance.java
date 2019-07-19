package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "AutoAvoidance")
public class AutoAvoidance extends OpMode {
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

    public void loop() {

    }

    public void stop() {

    }
}
