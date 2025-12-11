package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveLocations {

    private static final double 
        intakeAwayOffset = Units.inchesToMeters(13+4), //13 for robot, 4 for coral
        intakeLeftOffset = Units.inchesToMeters(-20), 
        intakeRightOffset = Units.inchesToMeters(20),
        
        branchAwayOffset = Units.inchesToMeters(13+4), //13 for robot, 4 for coral
        branchLeftOffset = Units.inchesToMeters(-5.5), 
        branchRightOffset = Units.inchesToMeters(5.5),
        
        approachOffset = Units.inchesToMeters(20); 

    public static Alliance allianceColor = Alliance.Blue;

    public static final AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public enum IntakeLocations {
        LEFT_FRONT,
        LEFT_BACK,
        RIGHT_FRONT,
        RIGHT_BACK
    }

    public enum BranchLocations {
        BACK,
        BACK_LEFT,
        BACK_RIGHT,
        FRONT,
        FRONT_LEFT,
        FRONT_RIGHT
    }

    public enum BranchModifiers {
        LEFT,
        RIGHT,
        LEVEL_ONE
    }

    @SuppressWarnings("unused")
    public Command autoDriveCommand(DriveSubsystem drive) {
        DriveToPoseObject[] poses;
        Pose2d finalPose, approachPose, sideIntermediatePose = null, backIntermediatePose = null;
        boolean goingToIntake = false, goingToFarSide =  true, goingToBackSide = true, sidePoseFirst = false;
        IntakeLocations targetIntakeLocation = IntakeLocations.LEFT_BACK;
        BranchLocations targetBranchLocation = BranchLocations.BACK;
        BranchModifiers targetBranchModifiers = BranchModifiers.LEFT;

        //Intake Command
        if (goingToIntake) {
            switch (targetIntakeLocation) {
                case LEFT_BACK:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(13).get().toPose2d();
                    else finalPose = tags.getTagPose(1).get().toPose2d();

                    finalPose = finalPose.transformBy(new Transform2d(intakeLeftOffset, intakeAwayOffset, Rotation2d.kZero));

                case LEFT_FRONT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(13).get().toPose2d();
                    else finalPose = tags.getTagPose(1).get().toPose2d();

                    finalPose = finalPose.transformBy(new Transform2d(intakeRightOffset, intakeAwayOffset, Rotation2d.kZero));

                case RIGHT_BACK:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(12).get().toPose2d();
                    else finalPose = tags.getTagPose(2).get().toPose2d();

                    finalPose = finalPose.transformBy(new Transform2d(intakeRightOffset, intakeAwayOffset, Rotation2d.kZero));

                case RIGHT_FRONT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(12).get().toPose2d();
                    else finalPose = tags.getTagPose(2).get().toPose2d();

                    finalPose = finalPose.transformBy(new Transform2d(intakeLeftOffset, intakeAwayOffset, Rotation2d.kZero));
                default:
                    finalPose = new Pose2d(0, 0, Rotation2d.kZero);
            }
        } else {
            switch (targetBranchLocation) {
                case BACK:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(21).get().toPose2d();
                    else finalPose = tags.getTagPose(10).get().toPose2d();
                case BACK_LEFT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(20).get().toPose2d();
                    else finalPose = tags.getTagPose(11).get().toPose2d();
                case BACK_RIGHT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(22).get().toPose2d();
                    else finalPose = tags.getTagPose(9).get().toPose2d();
                case FRONT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(18).get().toPose2d();
                    else finalPose = tags.getTagPose(7).get().toPose2d();
                case FRONT_LEFT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(19).get().toPose2d();
                    else finalPose = tags.getTagPose(6).get().toPose2d();
                case FRONT_RIGHT:
                    if  (allianceColor == Alliance.Blue) finalPose = tags.getTagPose(17).get().toPose2d();
                    else finalPose = tags.getTagPose(8).get().toPose2d();
                default:
                    finalPose = new Pose2d(0, 0, Rotation2d.kZero);
            }

            switch (targetBranchModifiers) {
                case LEFT:
                    finalPose = finalPose.transformBy(new Transform2d(branchLeftOffset, branchAwayOffset, Rotation2d.kZero));
                case RIGHT:
                    finalPose = finalPose.transformBy(new Transform2d(branchRightOffset, branchAwayOffset, Rotation2d.kZero));
                case LEVEL_ONE:
                    finalPose = finalPose.transformBy(new Transform2d(0, 0, Rotation2d.kZero));
            }
        }


        
        approachPose = finalPose.transformBy(new Transform2d(0, approachOffset, Rotation2d.kZero));

        if (backIntermediatePose == null && sideIntermediatePose == null) {
            return new DriveToPose(drive, 
                new DriveToPoseObject(approachPose), 
                new DriveToPoseObject(finalPose)
            );
        } else if (backIntermediatePose == null){
            return new DriveToPose(drive, 
                new DriveToPoseObject(sideIntermediatePose, 0.5), 
                new DriveToPoseObject(approachPose), 
                new DriveToPoseObject(finalPose)
            );
        } else if (sideIntermediatePose == null) {
            return new DriveToPose(drive, 
                new DriveToPoseObject(backIntermediatePose, 0.5), 
                new DriveToPoseObject(approachPose), 
                new DriveToPoseObject(finalPose)
            );
        } else {
            if (sidePoseFirst) {
                return new DriveToPose(drive, 
                    new DriveToPoseObject(backIntermediatePose, 0.5), 
                    new DriveToPoseObject(sideIntermediatePose, 0.5), 
                    new DriveToPoseObject(approachPose), 
                    new DriveToPoseObject(finalPose)
                );
            } else {
                return new DriveToPose(drive, 
                    new DriveToPoseObject(backIntermediatePose, 0.5), 
                    new DriveToPoseObject(sideIntermediatePose, 0.5), 
                    new DriveToPoseObject(approachPose), 
                    new DriveToPoseObject(finalPose)
                );
            }
        }
    }
}
