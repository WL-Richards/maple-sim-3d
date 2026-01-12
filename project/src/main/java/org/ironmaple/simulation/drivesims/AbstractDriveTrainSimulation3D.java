package org.ironmaple.simulation.drivesims;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.debug.PhysicsWorldVisualizer;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.physics3d.OdePhysicsWorld3D;
import org.ironmaple.simulation.physics3d.OdePhysicsWorld3D.RobotHandle;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;

/**
 * 3D counterpart of {@link AbstractDriveTrainSimulation} backed by ode4j.
 *
 * <p>This base class keeps the drivetrain API identical to the 2D implementation while delegating all pose, velocity,
 * and force operations to an {@link OdePhysicsWorld3D} instance.
 */
public abstract class AbstractDriveTrainSimulation3D implements DriveTrainSimulation {
    protected static final double DEFAULT_CHASSIS_HEIGHT_METERS = 0.35;

    protected final DriveTrainSimulationConfig config;
    private Pose3d desiredPose;
    private RobotHandle robotHandle;
    private final double planarMomentOfInertia;
    private final String visualizerId;
    private double pendingLinearDamping = 0.1;
    private double pendingAngularDamping = 0.01;

    protected AbstractDriveTrainSimulation3D(DriveTrainSimulationConfig config, Pose3d initialPoseOnField) {
        this.config = config;
        this.desiredPose = initialPoseOnField == null ? new Pose3d() : initialPoseOnField;
        final double length = config.bumperLengthX.in(Meters);
        final double width = config.bumperWidthY.in(Meters);
        final double mass = config.robotMass.in(Kilograms);
        this.planarMomentOfInertia = (mass * (length * length + width * width)) / 12.0;
        this.visualizerId = getClass().getSimpleName() + "@" + Integer.toHexString(System.identityHashCode(this));
    }

    /** Internal-use method invoked by {@link OdePhysicsWorld3D} when attaching the robot to the 3D world. */
    public final void bindToWorld(OdePhysicsWorld3D odeWorld) {
        if (this.robotHandle != null) {
            throw new IllegalStateException("This drivetrain simulation is already registered to a 3D world.");
        }
        this.robotHandle = odeWorld.createRobotHandle(
                config, getChassisHeightMeters(), config.moduleTranslations, getDriveWheelRadiusMeters());
        robotHandle.setLinearDamping(pendingLinearDamping);
        robotHandle.setAngularDamping(pendingAngularDamping);
        setSimulationWorldPose(desiredPose);
        desiredPose = null;
    }

    protected double getChassisHeightMeters() {
        return DEFAULT_CHASSIS_HEIGHT_METERS;
    }

    protected double getDriveWheelRadiusMeters() {
        return 0.0508;
    }

    public void setSimulationWorldPose(Pose2d pose2d) {
        setSimulationWorldPose(new Pose3d(pose2d));
    }

    public void setSimulationWorldPose(Pose3d pose3d) {
        if (robotHandle == null) {
            desiredPose = pose3d;
            return;
        }
        robotHandle.setPose(pose3d);
        robotHandle.resetVelocities();
        publishVisualizerWireframe();
    }

    public Pose3d getSimulatedDriveTrainPose3d() {
        if (robotHandle == null) {
            return desiredPose == null ? new Pose3d() : desiredPose;
        }
        return robotHandle.getPose();
    }

    public Pose2d getSimulatedDriveTrainPose() {
        return getSimulatedDriveTrainPose3d().toPose2d();
    }

    public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
        if (robotHandle == null) {
            return;
        }
        robotHandle.setLinearVelocity(givenSpeeds.vxMetersPerSecond, givenSpeeds.vyMetersPerSecond, 0);
        robotHandle.setAngularVelocity(0, 0, givenSpeeds.omegaRadiansPerSecond);
        publishVisualizerWireframe();
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsFieldRelative() {
        if (robotHandle == null) {
            return new ChassisSpeeds();
        }
        final DVector3C linearVelocity = robotHandle.getLinearVelocity();
        final DVector3C angularVelocity = robotHandle.getAngularVelocity();
        return new ChassisSpeeds(linearVelocity.get0(), linearVelocity.get1(), angularVelocity.get2());
    }

    public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
        final ChassisSpeeds fieldRelative = getDriveTrainSimulatedChassisSpeedsFieldRelative();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelative, getSimulatedDriveTrainPose().getRotation());
    }

    protected final void applyForce(Vector2 planarForce) {
        applyForce(planarForce.x, planarForce.y, 0);
    }

    protected final void applyForce(double fx, double fy, double fz) {
        if (robotHandle == null) return;
        robotHandle.addForce(fx, fy, fz);
    }

    protected final void applyTorqueZ(double torque) {
        if (robotHandle == null) return;
        robotHandle.addTorque(0, 0, torque);
    }

    protected final void applyForce(Vector2 planarForce, Vector2 worldPoint) {
        if (robotHandle == null) return;
        final double z = getSimulatedDriveTrainPose3d().getZ();
        robotHandle.addForceAtPosition(planarForce.x, planarForce.y, 0, worldPoint.x, worldPoint.y, z);
    }

    protected final Vector2 getLinearVelocityVector2() {
        if (robotHandle == null) {
            return new Vector2();
        }
        final DVector3C velocities = robotHandle.getLinearVelocity();
        return new Vector2(velocities.get0(), velocities.get1());
    }

    protected final double getAngularVelocity() {
        if (robotHandle == null) {
            return 0;
        }
        return robotHandle.getAngularVelocity().get2();
    }

    protected final void setAngularVelocity(double omegaRadPerSec) {
        if (robotHandle == null) return;
        robotHandle.setAngularVelocity(0, 0, omegaRadPerSec);
    }

    protected final Vector2 getWorldPoint(Vector2 localPoint) {
        final Pose2d pose = getSimulatedDriveTrainPose();
        final Translation2d translation = new Translation2d(localPoint.x, localPoint.y).rotateBy(pose.getRotation());
        return new Vector2(pose.getX() + translation.getX(), pose.getY() + translation.getY());
    }

    protected final Vector2 getLinearVelocity(Vector2 worldPoint) {
        if (robotHandle == null) {
            return new Vector2();
        }

        final DVector3C linear = robotHandle.getLinearVelocity();
        final double omega = robotHandle.getAngularVelocity().get2();
        final Pose2d pose = getSimulatedDriveTrainPose();
        final double rx = worldPoint.x - pose.getX();
        final double ry = worldPoint.y - pose.getY();
        final double vx = linear.get0() - omega * ry;
        final double vy = linear.get1() + omega * rx;
        return new Vector2(vx, vy);
    }

    protected final void setLinearDamping(double damping) {
        pendingLinearDamping = damping;
        if (robotHandle != null) {
            robotHandle.setLinearDamping(damping);
        }
    }

    protected final void setAngularDamping(double damping) {
        pendingAngularDamping = damping;
        if (robotHandle != null) {
            robotHandle.setAngularDamping(damping);
        }
    }

    protected final double getPlanarMomentOfInertia() {
        return planarMomentOfInertia;
    }

    public final void publishVisualizerWireframe() {
        if (robotHandle == null) return;
        PhysicsWorldVisualizer.publishRobotWireframe(
                visualizerId,
                getSimulatedDriveTrainPose(),
                config.bumperLengthX.in(Meters),
                config.bumperWidthY.in(Meters),
                config.moduleTranslations);
    }

    protected final void applyYawDamping(double coefficient) {
        if (robotHandle == null) return;
        final double yawVelocity = robotHandle.getAngularVelocity().get2();
        applyTorqueZ(-coefficient * yawVelocity);
    }

    protected final void applyPitchDamping(double coefficient) {
        if (robotHandle == null) return;
        final DMatrix3C rotation = robotHandle.getRotationMatrix();
        final DVector3C omega = robotHandle.getAngularVelocity();

        final double localPitchVelocity =
                rotation.get01() * omega.get0() + rotation.get11() * omega.get1() + rotation.get21() * omega.get2();
        final double torqueMag = -coefficient * localPitchVelocity;

        final double axisX = rotation.get01();
        final double axisY = rotation.get11();
        final double axisZ = rotation.get21();

        robotHandle.addTorque(axisX * torqueMag, axisY * torqueMag, axisZ * torqueMag);
    }

    protected final void applyRollDamping(double coefficient) {
        if (robotHandle == null) return;
        final DMatrix3C rotation = robotHandle.getRotationMatrix();
        final DVector3C omega = robotHandle.getAngularVelocity();

        final double localRollVelocity =
                rotation.get00() * omega.get0() + rotation.get10() * omega.get1() + rotation.get20() * omega.get2();
        final double torqueMag = -coefficient * localRollVelocity;

        final double axisX = rotation.get00();
        final double axisY = rotation.get10();
        final double axisZ = rotation.get20();

        robotHandle.addTorque(axisX * torqueMag, axisY * torqueMag, axisZ * torqueMag);
    }

    protected final void applyPitchLeveling(double coefficient) {
        if (robotHandle == null) return;
        final DMatrix3C rotation = robotHandle.getRotationMatrix();
        final double denom = Math.sqrt(rotation.get00() * rotation.get00() + rotation.get10() * rotation.get10());
        final double pitchAngle = Math.atan2(-rotation.get20(), denom);
        final double torqueMag = -coefficient * pitchAngle;
        final double axisX = rotation.get01();
        final double axisY = rotation.get11();
        final double axisZ = rotation.get21();
        robotHandle.addTorque(axisX * torqueMag, axisY * torqueMag, axisZ * torqueMag);
    }

    protected final void applyRollLeveling(double coefficient) {
        if (robotHandle == null) return;
        final DMatrix3C rotation = robotHandle.getRotationMatrix();
        final double rollAngle = Math.atan2(rotation.get21(), rotation.get22());
        final double torqueMag = -coefficient * rollAngle;
        final double axisX = rotation.get00();
        final double axisY = rotation.get10();
        final double axisZ = rotation.get20();
        robotHandle.addTorque(axisX * torqueMag, axisY * torqueMag, axisZ * torqueMag);
    }
}
