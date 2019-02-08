package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="sensor color test", group="Linear OpMode")
//@Disabled
public class sensorColorTest extends LinearOpMode {

    ColorSensor sensorColor;

    private Servo balls = null;
    private Servo underBalls = null;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private Servo upLeft = null;
    private Servo downLeft = null;
    private Servo upRight = null;
    private Servo downRight = null;

    private Servo grow1 = null;
    private Servo grow2 = null;

    private DcMotor eleMotor = null;
    private DcMotor relicMotor1 = null;
    private DcMotor relicMotor2 = null;
    private DcMotor hand = null;
    private Servo relicCatcher = null;
    DistanceSensor sensorDistance;

    private ElapsedTime runtime = new ElapsedTime();

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

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

        relicMotor1 = hardwareMap.dcMotor.get("relicMotor1");
        relicMotor2 = hardwareMap.dcMotor.get("relicMotor2");
        hand = hardwareMap.dcMotor.get("hand");
        relicCatcher = hardwareMap.servo.get("relicCatcher");

        balls = hardwareMap.servo.get("ball");
        underBalls = hardwareMap.servo.get("underBalls");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");


        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        telemetry.addData(">", "Press Play to start");
        telemetry.addData("Status - servo", underBalls.getPosition());
        telemetry.update();
        waitForStart();
        runtime.reset();


        /*
        * code between lines - 109, 137
        * sensor color
        * */
        //balls.setPosition(0.7);
        //sleep(1000);
        //underBalls.setPosition(0.6);
        //sleep(1000);
        balls.setPosition(0);
        sleep(3000);
        if (sensorColor.red() < sensorColor.blue())
        {
            underBalls.setPosition(0.3);
            telemetry.addData("if: ", "blue");
            underBalls.setPosition(0.45);
        }
        else
        {
            underBalls.setPosition(1);
            telemetry.addData("else: ", "red");
            underBalls.setPosition(0.75);
        }
        telemetry.update();
        sleep(1000);
        balls.setPosition(0.6);
        sleep(1000);
        underBalls.setPosition(0.45);
        sleep(500);
        balls.setPosition(1);
        //underBalls.setPosition(-0.7);
        //sleep(1000);
        //balls.setPosition(0.9);
        //sleep(2000);
    }
}