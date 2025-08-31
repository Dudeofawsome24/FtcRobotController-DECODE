package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class VisionSubsystem extends SubsystemBase {

    //Using limelight for vision
    private Limelight3A limelight;
    private MecanumDrive drive;

    private Telemetry telemetry;

    private Pose3D pose_MT2;
    private double[] target_distance;

    public VisionSubsystem(HardwareMap hMap, Telemetry telemetry, MecanumDrive drive) {

        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);

        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(1);

        this.drive = drive;

        //Start getting data
        limelight.start();
    }

    public void getVisionResults() {

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            target_distance = new double[] {tx, ty, ta};
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();

    }

    public Pose3D getAprilTagPose() {

        limelight.updateRobotOrientation(drive.localizer.getPose().heading.toDouble());
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            pose_MT2 = result.getBotpose_MT2();
            if (pose_MT2 != null) {
                double x = pose_MT2.getPosition().x;
                double y = pose_MT2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

        return pose_MT2;
    }

    //Moves robot to target
    public void moveRobot(double distanceX, double distanceY, double yaw) {

    }

}
