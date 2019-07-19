package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by emory on 8/19/2017.
 */
/*Hi!, it's Emory. This autonomous uses an object (that I scanned) using Vuforia and drive the robot according to it.*/

//@Autonomous(name = "VuforiaObjectTest")

public class VuforiaObjectTest extends OpMode {
    VuforiaTrackables testing;

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables Actual;


    public void init() {

        telemetry.addLine("here");
        //shows camera - or not to save power
        //VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //camera direction - back has better resolution
        //param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //license key - get this from vuforia site
        //param.vuforiaLicenseKey = "AbbjdQb/////AAABmRRAhmtd30mMq0AwAw9r0DZliL1sYh3xSveL4qZ+dFMO5DZUoOAbWApkduegT7NPvv++REpW3zljlQ1CPubxwaPaSbdGZBLffMxO8nsiledzIVnslT5Uf5Vp+bEvbJQ/3hUcAHRv1VS58XGOQkl8ryHgEJx3hwbeY46V6PeuV/Q7VK8749ipxUjnvnd1dsaYsc15xGzGBaeg/RjRS+89BUNRFHr6gxy6cj4KFj/qpxtUiWHxAa7XqENogqyGEEB46u2bjk9x27LMH+yC+PHOpVpEVrxnQC0N8ZQJrhYJfNrH+qqE4OM9jtd9nFP6yKMbjPFckp/hxDsj+CBKeIcGHtB0kB892U0q1KXsTsAwf1GB";
                //"AbbjdQb/////AAABmRRAhmtd30mMq0AwAw9r0DZliL1sYh3xSveL4qZ+dFMO5DZUoOAbWApkduegT7NPvv++REpW3zljlQ1CPubxwaPaSbdGZBLffMxO8nsiledzIVnslT5Uf5Vp+bEvbJQ/3hUcAHRv1VS58XGOQkl8ryHgEJx3hwbeY46V6PeuV/Q7VK8749ipxUjnvnd1dsaYsc15xGzGBaeg/RjRS+89BUNRFHr6gxy6cj4KFj/qpxtUiWHxAa7XqENogqyGEEB46u2bjk9x27LMH+yC+PHOpVpEVrxnQC0N8ZQJrhYJfNrH+qqE4OM9jtd9nFP6yKMbjPFckp/hxDsj+CBKeIcGHtB0kB892U0q1KXsTsAwf1GB";

        //shows axises on object
        //param.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        //VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(param);
        //this.vuforia = ClassFactory.createVuforiaLocalizer(param);

        //can track more than one object simultaneously
        //Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 2);

        //extracting file and object/s
        //boxTest = vuforia.loadTrackablesFromAsset("FTC_2016-17.xml"); //noooo .xml needed
        /*VuforiaTrackableDefaultListener box = (VuforiaTrackableDefaultListener) boxTest.get(0).getListener();
        boxTest.get(0).setName("LEDLight");
        boxTest.activate();*/
        testing = this.vuforia.loadTrackablesFromAsset("Actual_OT");
        VuforiaTrackable Tractor = testing.get(0);
        Tractor.setName("Tractor");

        telemetry.addLine(":)");
        telemetry.update();

    }

    public void loop() {

        telemetry.addLine("heyyy");

       for (VuforiaTrackable trackable : Actual) {

           telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //


           //gets all the angles
           OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

           OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
           if (robotLocationTransform != null) {
               lastLocation = robotLocationTransform;
           }

            if (pose != null) {
               telemetry.addLine("sees it");
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                //later on for seeing which object is seen
                //telemetry.addData("Object Seen:", boxTest.getName());

                //supposed to tell degrees to turn
                double degreesTurning = Math.toDegrees(Math.atan2(trans.get(0), trans.get(2))); //1,2 if horizontal and 0,2 if vertical
                telemetry.addData("degrees to turn", degreesTurning);


                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }

       }
        telemetry.update();
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}//end of OpMode

