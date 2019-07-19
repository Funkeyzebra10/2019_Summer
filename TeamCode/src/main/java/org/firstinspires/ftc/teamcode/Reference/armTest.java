package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Arm Test")

/*
this program is SUPPOSED to make the lift move up or down.
if up or down button is pressed, you SHOULD be able to just press the up button, and it go to the desired position
this has not been tested, it may go more than it should be ready
 */
public class armTest extends OpMode {

    //declaring devices
    DcMotor arm;
    AnalogInput potentiometer;

    //used to see if the buttons lift up and lift down are pressed, isUp and isDown to check if the lift is either up or down
    boolean armIn,armOut;
    double voltReading, servoValue;

    /*this checks if the lift is either going up or currently down or going down or currently up
    1 is currently down or going up
    2 is curently up or going down */
    int state = 1;
    public void init()
    {
        //sets devices in code to devices in config files
        arm = hardwareMap.dcMotor.get("arm");
        potentiometer = hardwareMap.analogInput.get("potentiometer");

        //sets direction of the lift motor
        arm.setDirection(DcMotorSimple.Direction.FORWARD);

    }//end of init

    public void loop()
    {
        //sets gamepad buttons
        armIn = gamepad1.b;
        armOut = gamepad1.x;

        voltReading = (float) potentiometer.getVoltage();
        telemetry.addData("voltage:", voltReading);
        servoValue = voltReading/3.25;
        telemetry.addData("servoValue:", servoValue);

        if(armIn)
        {
            arm.setPower(1);
        }
        else if(armOut)
        {
            arm.setPower(-1);
        }
        else
        {
            arm.setPower(0);
        }

    }//end of loop

    public void stop()
    {
        //puts motor at rest
        arm.setPower(0);
    }//end of stop
}
