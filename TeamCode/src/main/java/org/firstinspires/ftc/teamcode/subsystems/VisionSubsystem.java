package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    //Using limelight for vision
    private Limelight3A limelight;

    private Telemetry telemetry;

    public VisionSubsystem(HardwareMap hMap, Telemetry telemetry) {

        limelight = hMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);

        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(1);

        //Start getting data
        limelight.start();
    }

    public LLResult getTargetResult() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("Target Detected", true);
            telemetry.addData("tx", result.getTx()); // Horizontal offset
            telemetry.addData("ty", result.getTy()); // Vertical offset
            telemetry.addData("ta", result.getTa()); // Target area
        } else {
            telemetry.addData("Target Detected", false);
        }

        telemetry.update();
        return result;
    }

    public Pose3D getAprilTagPose() {
        LLResult result = limelight.getLatestResult();

        //Make sure result is valid
        if (result == null || !result.isValid()) {
            telemetry.addData("AprilTag Detected", false);
            telemetry.update();
            return null;
        }

        //Get list of fiducial detections (list detailing tags)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        // Make sure at least one tag was detected
        if (fiducials == null || fiducials.isEmpty()) {
            telemetry.addData("AprilTag Detected", false);
            telemetry.update();
            return null;
        }

        //Take the first tag
        LLResultTypes.FiducialResult tag = fiducials.get(0);

        //Position of robot relative to tag
        Pose3D pose = tag.getRobotPoseTargetSpace();

        if (result != null && result.isValid() && tag != null) {

            telemetry.addData("AprilTag Detected", true);
            telemetry.addData("X", pose.getPosition().x);
            telemetry.addData("Y", pose.getPosition().y);
            telemetry.addData("Z", pose.getPosition().z);
            telemetry.addData("Yaw", pose.getOrientation().getYaw());
            telemetry.addData("Pitch", pose.getOrientation().getPitch());
            telemetry.addData("Roll", pose.getOrientation().getRoll());
            telemetry.update();

            return pose;
        }

        telemetry.addData("AprilTag Detected", false);
        telemetry.update();
        return null;
    }

}
