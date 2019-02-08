/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
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


@Autonomous(name="autored_left", group ="Concept")

public class autored_left extends LinearOpMode {

    ColorSensor sensorColor;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftPump = null;
    private DcMotor rightPump = null;
    private Servo stabilizerLeft = null;
    private Servo stabilizerRight = null;
    private DcMotor newElevetor = null;
    private DcMotor relicMotor = null;
    private Servo upDown = null;
    private Servo catchrelic = null;
    DistanceSensor sensorDistance;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        auto_class autoDrive = new auto_class();

        leftFront = hardwareMap.dcMotor.get("Left_Front");
        leftBack = hardwareMap.dcMotor.get("Left_Back");
        rightBack = hardwareMap.dcMotor.get("Right_Back");
        rightFront = hardwareMap.dcMotor.get("Right_Front");
        leftPump = hardwareMap.dcMotor.get("leftPump");
        rightPump = hardwareMap.dcMotor.get("rightPump");
        stabilizerLeft = hardwareMap.servo.get("stabilizerLeft");
        stabilizerRight = hardwareMap.servo.get("stabilizerRight");
        newElevetor = hardwareMap.dcMotor.get("newElevetor");
        relicMotor = hardwareMap.dcMotor.get("relicMotor");
        catchrelic = hardwareMap.servo.get("catcherrelic");
        upDown = hardwareMap.servo.get("upDown");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");


        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AfD73YH/////AAAAmcrp0Qe3BEG7vf2FA9kwSshWl6m6N27rFAobRjcysMhgDJjv2BiM5KnvFY4dAf+bYnHnigVXqKtxrUYYUM1l59iF/rgqgAyVptCmCRxxNCEtRoUTRa2OHjzLSykLiL4KGD0KJ/DT3FNqvx70oMBW3cF7IAkHgbIeONC2uCVStcQHiM3oCG+uh4qGTeS7c7fpkNa2mJQOvmzIG62xrdQb9x6/vGAJ9zcJwNYwE9fPd7YtyHBiT+adn4J1JTsidgrqbXXdHJIdIfnoPVek1V0q4HwJAHhjKi9S7lfk8Bs2tyuVbGtNHehE5TQDJgUJspjMUmA9KlWkD6oTAEWvA5i1w1NzWHB86BGNEfPW7NWpRkDm";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();



        telemetry.update();
//start

        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(1990);
        //for 2 sec drive forward

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);
        //for 2 sec don't drive

        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(-0.5);
        rightFront.setPower(-0.5);
        sleep(1300);
        //for 1 sec drive right  *left

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);
        //for 2 sec don't drive

        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(400);
        //for 0.4 sec drive back

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(200);
        //for 0.2 sec stop

        // Liron:

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(-1);
        rightFront.setPower(-1);
        sleep(2000);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(1);
        rightFront.setPower(1);
        sleep(2000);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(-1);
        rightFront.setPower(-1);
        sleep(2000);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);

        leftBack.setPower(-1);
        leftFront.setPower(-1);
        rightBack.setPower(-1);
        rightFront.setPower(-1);
        sleep(2000);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(2000);

        leftBack.setPower(1);
        leftFront.setPower(1);
        rightBack.setPower(-1);
        rightFront.setPower(-1);
        sleep(2000);
//end

        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData("position: ", vuMark);
            telemetry.addData("position: ", vuMark.toString());

            if(vuMark.toString() == "LEFT")
            {
                leftBack.setPower(0.5);
                leftFront.setPower(0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1980);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1300);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(0.5);
                rightFront.setPower(0.5);
                sleep(400);
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(200);

            }
            else if(vuMark.toString() == "RIGHT")
            {
                leftBack.setPower(0.5);
                leftFront.setPower(0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1150);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1320);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(0.5);
                rightFront.setPower(0.5);
                sleep(400);
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(200);

            }
            else if(vuMark.toString() == "CENTER")
            {
                leftBack.setPower(0.5);
                leftFront.setPower(0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1550);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(-0.5);
                rightFront.setPower(-0.5);
                sleep(1300);

                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(2000);

                leftBack.setPower(-0.5);
                leftFront.setPower(-0.5);
                rightBack.setPower(0.5);
                rightFront.setPower(0.5);
                sleep(400);
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                sleep(200);

            }

            sleep(1000);


                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

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
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
