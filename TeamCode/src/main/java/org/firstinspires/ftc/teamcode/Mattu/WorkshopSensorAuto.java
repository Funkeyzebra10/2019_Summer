package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor Autonomous")
public class WorkshopSensorAuto extends LinearOpMode {
    //Motors
    DcMotor FrontRight, FrontLeft;

    //Sensors
    CustomDistance distance;
    ColorSensor color;
    TouchSensor touch;

    //Variables
    double limit;
    int time;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize sensors
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");

        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        distance = new CustomDistance("distance", hardwareMap);
        color = hardwareMap.get(ColorSensor.class, "color");
        touch = hardwareMap.touchSensor.get("touch");

        limit = 10;

        waitForStart();

        //Move to wall
        while (distance.getDistance(DistanceUnit.CM) >= limit) {
            move(0.25, 0.25);
        }

        //Turn away from wall
        time = Math.round(System.currentTimeMillis());
        while (distance.getDistance(DistanceUnit.CM) <= limit + 2) {
            move(0.6, -0.8);
        }
        time = Math.round((System.currentTimeMillis() - time) * 10^5) / 10^5;

        move(0.25, -0.25);
        Thread.sleep(time / 8);

        //Move into opening
        color.enableLed(true);
        while (color.alpha() < 200) {
            move(0.25, 0.25);
        }
        color.enableLed(false);

        //Turn towards opening
        move(-0.6, 0.8);
        Thread.sleep(time);

        move(-0.25, 0.25);
        Thread.sleep(time / 8);

        //Move through opening
        move(0.25, 0.25);
        Thread.sleep(1500);

        //Turn towards end
        move(-0.25, 0.25);
        Thread.sleep(2000);

        //Move towards end
        move(0.25, 0.25);
        Thread.sleep(1000);

        //Stop
        move(0, 0);
    }

    public void move(double lP, double rP) {
        FrontRight.setPower(rP);
        FrontLeft.setPower(lP);
    }
}