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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TestAutonomous", group = "Concept")
//@Disabled
public class TestAutonomous extends LinearOpMode {

    private DcMotor LeftBack;
    private DcMotor RightBack;
    private DcMotor LeftFront;
    private DcMotor RightFront;


    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        LeftBack = hardwareMap.dcMotor.get("left_back"); // port: 0
        LeftFront = hardwareMap.dcMotor.get("left_front"); // port: 0
        RightBack = hardwareMap.dcMotor.get("right_back"); // port: 1
        RightFront = hardwareMap.dcMotor.get("right_front"); // port: 1

        auto_class AutoDrive = new auto_class();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {//right
            if(gamepad1.b) {
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 300);
                AutoDrive.autoTurnRight(LeftBack, LeftFront, RightBack, RightFront, 0.5, 600);
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 1000);

                sleep(500);

                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, -0.4, 800);
                sleep(500);
                AutoDrive.autoTurnRight(LeftBack, LeftFront, RightBack, RightFront, -0.6, 600);
                sleep(500);
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, -0.4, 300);
            }

            if(gamepad1.x)//left
            {
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 300);
                AutoDrive.autoTurnLeft(LeftBack, LeftFront, RightBack, RightFront, 0.5, 600);
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 1000);

                sleep(500);

                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, -0.4, 800);
                sleep(500);
                AutoDrive.autoTurnLeft(LeftBack, LeftFront, RightBack, RightFront, -0.6, 600);
                sleep(500);
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, -0.4, 300);

            }

            if(gamepad1.a) {//center
                AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 1200);
                AutoDrive.autoDriveBackward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 975);
            }
            /*AutoDrive.autoTurnLeft(LeftBack, LeftFront, RightBack, RightFront, 0.6, 975);
            AutoDrive.autoDriveForward(LeftBack, LeftFront, RightBack, RightFront, 0.4, 1600);*/
            //AutoDrive.autoDriveForward();
            //AutoDrive.driveLeft();
            //AutoDrive.autoDriveForward();
        }
    }
}