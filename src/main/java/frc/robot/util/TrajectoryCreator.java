package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

/**
 * Abstracted Trajectory generation for for ease of use in
 * Robot class
 * 
 * <p>
 * <p>
 */
public class TrajectoryCreator {
    /**
     * Create Trajectories to be run using Ramsete Command.(Use during robot
     * initialization)
     * 
     * @param filename       Input a file name Ex: "ExampleTrajectory.wpilib.json"
     * @param TrajectoryName Input a name for the Trajectory (Can be different than
     *                       Pathname)
     * @return A usable pathweaver path in RamseteCommand
     * @throws IOExecption to DriverStation Console
     */
    public Trajectory generateTrajectory(String filename, String TrajectoryName) {
        Trajectory trajectory = new Trajectory();
        String trajectoryJSON = "paths/" + filename;
        try {
            Path newTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(newTrajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open this trajectory: " + TrajectoryName + " (" + trajectoryJSON + ")",
                    ex.getStackTrace());
        }

        System.out.println(TrajectoryName + " opened and loaded succesfully.");

        return trajectory;

    }

    /**
     * Create Trajectory config
     * 
     * @param startVelocity       Input starting velocity in MPS
     * @param endVelocity         Input ending velocity in MPS
     * @param reversed            Reverse Trajectory?
     * @param kMaxMPS             Input this Constant
     * @param kMaxAccelMPSsquared Input this Constant
     * @param kinematics          Input DiffDrive Kinematics Constant
     * @param voltageConstraint   Input Voltage Constraint
     * @return A trajectoryConfig for manual trajectory generation.
     */

    public TrajectoryConfig configTrajectory(
            double startVelocity, double endVelocity, boolean reversed, double kMaxMPS, double kMaxAccelMPSsquared,
            DifferentialDriveKinematics kinematics, DifferentialDriveVoltageConstraint voltageConstraint) {
        TrajectoryConfig config = new TrajectoryConfig(
                kMaxMPS, kMaxAccelMPSsquared)
                        .setKinematics(kinematics)
                        .setReversed(reversed)
                        .setStartVelocity(startVelocity)
                        .setEndVelocity(endVelocity)
                        .addConstraint(voltageConstraint);

        return config;
    }

}
