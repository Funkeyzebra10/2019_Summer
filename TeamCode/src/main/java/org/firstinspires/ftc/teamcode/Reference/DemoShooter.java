package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Shoot")

public class DemoShooter extends OpMode {
    DcMotor bRight, bLeft, sweeper, lLaunch, rLaunch;
    CRServo convey;
    double speed;
    int pLevel;
    public DemoShooter(){}

    public void init()
    {
        bRight = hardwareMap.dcMotor.get("motor_3");
        bLeft = hardwareMap.dcMotor.get("motor_4");
        convey = hardwareMap.crservo.get("convey");
        sweeper = hardwareMap.dcMotor.get("sweep");
        lLaunch = hardwareMap.dcMotor.get("lLaunch");
        rLaunch = hardwareMap.dcMotor.get("rLaunch");
        bRight.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        convey.setDirection(DcMotor.Direction.FORWARD);
        sweeper.setDirection(DcMotor.Direction.FORWARD);
        rLaunch.setDirection(DcMotor.Direction.FORWARD);
        lLaunch.setDirection(DcMotorSimple.Direction.REVERSE);




    }

    public void loop()
    {   /*sets names to gamepad 1 and gamepad 2 controlls
    gamepad 1 has functions and drive control, gamepad 2 has just functions.*/
        double lStick = gamepad1.left_stick_y, rStick = gamepad1.right_stick_y;
        boolean sweepF = gamepad1.y, sweepB = gamepad1.a;
        boolean conveyF = gamepad1.b, conveyB = gamepad1.x;
        boolean launchF = gamepad1.right_bumper, launchB = gamepad2.left_bumper;







        //for regular driving
        pLevel=256;//controls the speed of drive. 100% power is ((rStick * 100) / 128) 75% power is ((rStick * 100) / 192) 50% power is ((rStick * 100) / 256) 
        bLeft.setPower((lStick * 100) / pLevel);
        bRight.setPower((rStick * 100) / pLevel);


        //Sweep Forward and Backwards for gamepad 1&2
        if((sweepF && !sweepB))
        {
            sweeper.setPower(1);
        }
        else if((sweepB && !sweepF))
        {
            sweeper.setPower(-0.2);
        }
        else
        {
            sweeper.setPower(0);
        }



        //Conveyor Belt Forwards and Backwards for gamepad 1&2

        if((conveyF && !conveyB))
        {
            convey.setPower(0.75);
        }
        else if((conveyB && !conveyF))
        {
            convey.setPower(-0.75);
        }
        else
        {
            convey.setPower(0);
        }

        //launcher for gamepad 1&2

        if((launchF && !launchB))
        {
            rLaunch.setPower(1);
            lLaunch.setPower(1);
            convey.setPower(-0.75);
            sweeper.setPower(1);
        }
        else if((launchB && !launchF))
        {
            rLaunch.setPower(-1);
            lLaunch.setPower(-1);
            convey.setPower(0.75);
            sweeper.setPower(-1);
        }
        else
        {
            rLaunch.setPower(0);
            lLaunch.setPower(0);
            convey.setPower(0);
            sweeper.setPower(0);
        }
        telemetry.addData("left y:", gamepad1.left_stick_y);
        telemetry.addData("left y:", gamepad1.right_stick_y);

    }
        //makes sure evrything is at reast once program stops
    public void stop()
    {
        bLeft.setPower(0);
        bRight.setPower(0);
        convey.setPower(0);
        sweeper.setPower(0);
        rLaunch.setPower(0);
        lLaunch.setPower(0);
    }


}
