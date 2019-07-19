package org.firstinspires.ftc.teamcode.Reference;

//Imports (duh)
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//@TeleOp(name = "MotorTest")
public class MotorTest extends OpMode {
   DcMotor motor;
   DigitalChannel magSwitch, magSwitch2;
   double power = 1;
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        magSwitch = hardwareMap.digitalChannel.get("magSwitch");
        magSwitch2 = hardwareMap.digitalChannel.get("magSwitch2");
    }

    @Override
    public void loop() {
        boolean magnet = magSwitch.getState();
        boolean magnet2 = magSwitch2.getState();
        if (gamepad1.a && magnet2)
        {
            motor.setPower(power);
        }
        else
        {
        motor.setPower(0);
        }
        if (gamepad1.y && magnet)
        {
        motor.setPower(-power);
        }
        else
        {
        motor.setPower(0);
        }
    if (gamepad1.b)
    {
        stop();
    }

    telemetry.addData("MagSwitch: ", magnet);
    telemetry.addData("MagSwitch2: ", magnet2);
    }
}
