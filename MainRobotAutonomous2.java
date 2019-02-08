/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "MainRobotAutonomousOpposite", group = "Concept")
public class MainRobotAutonomous2 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AfD73YH/////AAAAmcrp0Qe3BEG7vf2FA9kwSshWl6m6N27rFAobRjcysMhgDJjv2BiM5KnvFY4dAf+bYnHnigVXqKtxrUYYUM1l59iF/rgqgAyVptCmCRxxNCEtRoUTRa2OHjzLSykLiL4KGD0KJ/DT3FNqvx70oMBW3cF7IAkHgbIeONC2uCVStcQHiM3oCG+uh4qGTeS7c7fpkNa2mJQOvmzIG62xrdQb9x6/vGAJ9zcJwNYwE9fPd7YtyHBiT+adn4J1JTsidgrqbXXdHJIdIfnoPVek1V0q4HwJAHhjKi9S7lfk8Bs2tyuVbGtNHehE5TQDJgUJspjMUmA9KlWkD6oTAEWvA5i1w1NzWHB86BGNEfPW7NWpRkDm";


    private VuforiaLocalizer vuforia;

    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor elevator;
    private DcMotor Pumper;
    DistanceSensor sensorDistance;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();

        Pumper = hardwareMap.dcMotor.get("Pumper"); // port: 3
        LeftBack = hardwareMap.dcMotor.get("Left_Back");
        RightBack = hardwareMap.dcMotor.get("Right_Back");
        elevator = hardwareMap.dcMotor.get("elevator");
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive())
            {
                elevator.setPower(1);
                sleep(5000);

                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -
                                    1)
                            {

                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                                { // left (Not Tasted)
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    RightBack.setPower(-0.5);
                                    sleep(500);
                                    LeftBack.setPower(0.8);
                                    RightBack.setPower(-0.8);
                                    sleep(500);
                                    LeftBack.setPower(0.5);
                                    RightBack.setPower(0);
                                    sleep(500);
                                    LeftBack.setPower(0.2);
                                    RightBack.setPower(-0.2);
                                    sleep(500);
                                    LeftBack.setPower(-0.2);
                                    RightBack.setPower(-0.2);
                                    sleep(500);
                                }

                                else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                                { // right (Not Tasted)
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    LeftBack.setPower(0.5);
                                    sleep(500);
                                    LeftBack.setPower(0.8);
                                    RightBack.setPower(-0.8);
                                    sleep(1000);
                                    LeftBack.setPower(0);
                                    RightBack.setPower(-0.5);
                                    sleep(1000);
                                    LeftBack.setPower(0.2);
                                    RightBack.setPower(-0.2);
                                    sleep(1000);
                                }

                                else if (goldMineralX > silverMineral1X && goldMineralX < silverMineral2X){ // center (Not Tested)
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    LeftBack.setPower(0.8);
                                    RightBack.setPower(-0.8);
                                    sleep(3000);
                                    LeftBack.setPower(0);
                                    RightBack.setPower(-0.5);
                                    sleep(1000);
                                }
                                sleep(2000);

                                LeftBack.setPower(0);
                                RightBack.setPower(0);



                                Pumper.setPower(1);
                                sleep(1500);
                                Pumper.setPower(0);
                                telemetry.addLine("Mascot Dropped");
                                telemetry.addLine("See Ramp");
                                RightBack.setPower(0);
                                LeftBack.setPower(0);
                                sleep(250);
                                RightBack.setPower(1);
                                sleep(1000);
                                RightBack.setPower(0);
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
