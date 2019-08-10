package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CustomDistance {
    DistanceSensor object;

    public CustomDistance(String id, HardwareMap hardwareMap) {
        object = hardwareMap.get(DistanceSensor.class, id);
    }

    public double getDistance(DistanceUnit unit) {
        if (Double.isNaN(object.getDistance(DistanceUnit.CM))) {
            return unit.fromUnit(DistanceUnit.INCH, 5);
        }
        else {
            return object.getDistance(unit);
        }
    }
}