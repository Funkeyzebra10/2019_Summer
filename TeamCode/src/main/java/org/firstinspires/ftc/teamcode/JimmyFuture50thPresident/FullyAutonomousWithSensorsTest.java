package org.firstinspires.ftc.teamcode.JimmyFuture50thPresident;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Jimmy Auto With Sensors")
public class FullyAutonomousWithSensorsTest extends LinearOpMode {
    DcMotor motorRight = null;
    DcMotor motorLeft = null;
    Servo armservo = null;

    TouchSensor touch;
    DistanceSensor distance;

    static final double ARM_REDACTED_POSITION = 0.2;
    static final double ARM_EXTENDED_POSITION = 0.8;

    public void runOpMode() throws InterruptedException {

        motorRight = hardwareMap.dcMotor.get("FrontRight");
        motorLeft = hardwareMap.dcMotor.get("FrontLeft");
        armservo = hardwareMap.servo.get("armservo");
        touch = hardwareMap.touchSensor.get("touch");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        telemetry.addData("ServoPosition", armservo.getPosition());
        telemetry.addData("TargetPower", 1);
        telemetry.addData("MotorPower", motorLeft.getPower());
        telemetry.addData("Distance (cm)", distance.getDistance(DistanceUnit.CM));
        telemetry.addData("Status", "Running");
        telemetry.update();

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            //Robot is On
            TurnLeft(1);

            if (distance.getDistance(DistanceUnit.CM) < 8) {
                DriveForward(1);
            }
        }

    }

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void TurnLeft(double power)
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }

    public void TurnRight(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(-power);
    }

    public void Stop()
    {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
