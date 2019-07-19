package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "sweep test duh")
public class colorDumpV3 extends OpMode {


    //Declaring motors and sensors
    DcMotor sweep;

    ColorSensor lSweep, rSweep;
    DistanceSensor lSweepD, rSweepD;

    //RGB Values and Storage
    int r, g, b, rStore, gStore, bStore, r2, g2, b2, r2Store, g2Store, b2Store;

    //Distance Values
    double d, d2;

    boolean notBlueL = false; boolean notBlueR = false;

    boolean lifted = false;

    boolean twoIn = false;

    boolean lessThanL = false; boolean lessThanR = false;

    float sweepIn, sweepOut;

    public void init() {
        //Hardware map- motors
        sweep = hardwareMap.dcMotor.get("sweep");

        //Hardware map- CD Sensors
        lSweep = hardwareMap.colorSensor.get("lSweep");
        lSweepD = hardwareMap.get(DistanceSensor.class, "lSweep");
        rSweep = hardwareMap.colorSensor.get("rSweep");
        rSweepD = hardwareMap.get(DistanceSensor.class, "rSweep");
    }

    public void loop() {

        //Telemetry values
        r = lSweep.red();
        g = lSweep.green();
        b = lSweep.blue();
        d = lSweepD.getDistance(DistanceUnit.CM);
        r2 = rSweep.red();
        g2 = rSweep.green();
        b2 = rSweep.blue();
        d2 = rSweepD.getDistance(DistanceUnit.CM);

        if(d < 9.7) {
            lessThanL = true;
        }
        else {
            lessThanL = false;
        }
        if(d2 < 9.7) {
            lessThanR = true;
        }
        else {
            lessThanR = false;
        }


        if (!lifted && (lessThanR && lessThanL)) {
            twoIn = true;
        }

        telemetry.addData("distance 1: ", d);
        telemetry.addData("distance 2: ", d2);

        if(twoIn) {
            telemetry.addData("lifted?: ", twoIn);
        }

    }

    public void stop() {
        telemetry.addLine("done");
    }

}
