package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import static com.sun.tools.javac.util.Constants.format;

@Autonomous(name="test", group="Linear OpMode")
@Disabled
public class test extends LinearOpMode {

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

    private ElapsedTime runtime = new ElapsedTime();

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //private DcMotor eleMotor = null;
    //private Servo catcher_left = null;
    //private Servo catcher_right = null;

    auto_class autoDrive = new auto_class();

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "AfD73YH/////AAAAmcrp0Qe3BEG7vf2FA9kwSshWl6m6N27rFAobRjcysMhgDJjv2BiM5KnvFY4dAf+bYnHnigVXqKtxrUYYUM1l59iF/rgqgAyVptCmCRxxNCEtRoUTRa2OHjzLSykLiL4KGD0KJ/DT3FNqvx70oMBW3cF7IAkHgbIeONC2uCVStcQHiM3oCG+uh4qGTeS7c7fpkNa2mJQOvmzIG62xrdQb9x6/vGAJ9zcJwNYwE9fPd7YtyHBiT+adn4J1JTsidgrqbXXdHJIdIfnoPVek1V0q4HwJAHhjKi9S7lfk8Bs2tyuVbGtNHehE5TQDJgUJspjMUmA9KlWkD6oTAEWvA5i1w1NzWHB86BGNEfPW7NWpRkDm";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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


        waitForStart();
        runtime.reset();


        /*
        * code between lines - 109, 137
        * sensor color
        * */

        grow1.setPosition(0);
        grow2.setPosition(1);
        sleep(500);

        balls.setPosition(0);
        sleep(2500);
        if (sensorColor.red() < sensorColor.blue())
        {
            underBalls.setPosition(0.0);
            telemetry.addData("if: ", "blue");
        }
        else if(sensorColor.blue()<sensorColor.red())
        {
            underBalls.setPosition(0.9);
            telemetry.addData("else if: ", "red");
        }
        sleep(2000);
        telemetry.update();
        balls.setPosition(0.5);
        sleep(2000);
        underBalls.setPosition(0.55);
        sleep(500);
        balls.setPosition(1);
        sleep(2000);

        /*underBalls.setPosition(0.55);
        sleep(400);
        underBalls.setPosition(0.89375);
        sleep(400);
        balls.setPosition(1);
        sleep(300);
        balls.setPosition(1);
        sleep(300);*/

        //catch the box
        downLeft.setPosition(MIN_POS);
        downRight.setPosition(MAX_POS);
        sleep(200);

        /*
        use the camera
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData("position: ", vuMark);


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

            /*
            * drive forward
            * turn 90 deg' left
            * drive forward
            * leave the box
            * drive backward
            */
        //drive forward
        sleep(3000);

        //drive forward
        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(1150);

        //stop robot
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(500);

        //turn right
        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(-0.5);
        rightFront.setPower(-0.5);
        sleep(1100);

        //stop robot
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(500);

        //drive forward
        leftBack.setPower(-0.5);
        leftFront.setPower(-0.5);
        rightBack.setPower(0.5);
        rightFront.setPower(0.5);
        sleep(400);

        //stop robot
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(500);



        sleep(3000);
        //turn left

        //leave the box
        downLeft.setPosition(MAX_POS/2);
        downRight.setPosition(MAX_POS/2);
        sleep(450);

        //drive backward
        autoDrive.autoDriveBackward(leftBack, leftFront, rightBack, rightFront, 1, 20);
    }
}