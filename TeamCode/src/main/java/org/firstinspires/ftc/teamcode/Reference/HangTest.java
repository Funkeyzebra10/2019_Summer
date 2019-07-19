/*
package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "HangTest")

public class HangTest extends TeleOpV1 {

    CRServo smartServo;

    public void init()
    {
        smartServo = hardwareMap.crservo.get("smartServo");
        smartServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop()
    {

        if (gamepad1.x)
        {
            smartServo.setPower(1);
        }
        else
        {
            smartServo.setPower(0);
        }
        if (gamepad1.b)
        {
            smartServo.setPower(-1);
        }
        else
        {
            smartServo.setPower(0);
        }
    }

    public void stop()
    {
        smartServo.setPower(0.5);
    }

}
*/
