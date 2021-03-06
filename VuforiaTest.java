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

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="VuforiaTest", group ="Concept")

public class VuforiaTest extends LinearOpMode {

    ColorSensor sensorColor;
    private Servo balls = null;
    private Servo underBalls = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor eleMotor = null;
    private Servo upLeft = null;
    private Servo downLeft = null;
    private Servo upRight = null;
    private Servo downRight = null;
    private Servo grow1 = null;
    private Servo grow2 = null;
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

        leftFront = hardwareMap.dcMotor.get("left_front");
        leftBack = hardwareMap.dcMotor.get("left_back");
        rightBack = hardwareMap.dcMotor.get("right_back");
        rightFront = hardwareMap.dcMotor.get("right_front");
        upLeft = hardwareMap.servo.get("upLeft");
        downLeft = hardwareMap.servo.get("downLeft");
        upRight = hardwareMap.servo.get("upRight");
        downRight = hardwareMap.servo.get("downRight");

        eleMotor =hardwareMap.dcMotor.get("eleMotor");
        grow1 = hardwareMap.servo.get("grow1");
        grow2 = hardwareMap.servo.get("grow2");
        balls = hardwareMap.servo.get("ball");
        underBalls = hardwareMap.servo.get("underBalls");

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

        relicTrackables.activate();
            grow1.setPosition(0);
            grow2.setPosition(1);
            sleep(500);

            balls.setPosition(0);
            sleep(3000);
            if (sensorColor.red() < sensorColor.blue())
            {
                underBalls.setPosition(0.3);
                telemetry.addData("if: ", "blue");
            }
            else if(sensorColor.blue()<sensorColor.red())
            {
                underBalls.setPosition(0.7);
                telemetry.addData("else if: ", "red");
            }
            sleep(2000);
            telemetry.update();

            underBalls.setPosition(0.55);
            sleep(500);
            balls.setPosition(1);
            sleep(2000);

            /*balls.setPosition(0.65);
            sleep(2000);
            underBalls.setPosition(0.89375);
            sleep(200);
            underBalls.setPosition(0.89375);
            sleep(200);
            balls.setPosition(1);*/

            //catch the box
            downLeft.setPosition(MIN_POS);
            downRight.setPosition(MAX_POS);
            sleep(200);

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
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);
                    rightBack.setPower(0.5);
                    rightFront.setPower(0.5);
                    sleep(1990);

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
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);
                    rightBack.setPower(0.5);
                    rightFront.setPower(0.5);
                    sleep(1170);

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
                    leftBack.setPower(-0.5);
                    leftFront.setPower(-0.5);
                    rightBack.setPower(0.5);
                    rightFront.setPower(0.5);
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

                downLeft.setPosition(MAX_POS/2);
                downRight.setPosition(MAX_POS/2);
                sleep(450);


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
