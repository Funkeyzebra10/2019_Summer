package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Lift Test")

/*
this program is SUPPOSED to make the lift move up or down.
if up or down button is pressed, you SHOULD be able to just press the up button, and it go to the desired position
this has not been tested, it may go more than it should be ready
 */
public class liftTest extends OpMode {

    //declaring devices
    DigitalChannel magSwitchUp, magSwitchDown;
    //DcMotor  left, right;
    CRServo liftMotor;

    //used to see if the buttons lift up and lift down are pressed, isUp and isDown to check if the lift is either up or down
    boolean liftUp, liftDown, isUp, isDown;
    //float rStick, lStick;

    /*this checks if the lift is either going up or currently down or going down or currently up
    1 is currently down or going up
    2 is curently up or going down */
    int state = 1;
    public void init()
    {
        //sets devices in code to devices in config files
        liftMotor = hardwareMap.crservo.get("liftMotor");
        magSwitchUp = hardwareMap.digitalChannel.get("magSwitchUp");
        magSwitchDown = hardwareMap.digitalChannel.get("magSwitchDown");
        //left = hardwareMap.dcMotor.get("left");
        //right = hardwareMap.dcMotor.get("right");

        //sets direction of the lift motor


        //makes sure the boolean varibles start as its original position, the lift should be down
        isDown = true;
        isUp = false;

    }//end of init

    public void loop()
    {
        //sets gamepad buttons
        liftUp = gamepad1.y;
        liftDown = gamepad1.a;
        //lStick = gamepad1.left_stick_y;
        //rStick = gamepad1.right_stick_y;



        /*
        //sets the variables depending on the state of the sensors
        if(magSwitchDown.getState())
        {
            isDown = true;
            state = 1;
        }
        else if(magSwitchUp.getState())
        {
            isUp = true;
            state = 2;
        }

        //this should run only if the sensor is read, or if the lift is moving, but has not been tested
        if((liftUp && isDown) || (liftMotor.isBusy() && state == 1))
        {
            liftMotor.setPower(0.25);
        }
        else if((robotReset && isUp) || (liftMotor.isBusy() && state == 2))
        {
            liftMotor.setPower(-0.25);
        }
        else
        {
            liftMotor.setPower(0);
        }
        */

        if(liftUp&&magSwitchUp.getState())
        {
            liftMotor.setPower(0.75);
        }
        else if(liftDown&&magSwitchDown.getState())
        {
            liftMotor.setPower(-0.75);
        }
        else
        {
            liftMotor.setPower(0);
        }
    }//end of loop

    public void stop()
    {
        //puts motor at rest
        liftMotor.setPower(0);
    }//end of stop
}
