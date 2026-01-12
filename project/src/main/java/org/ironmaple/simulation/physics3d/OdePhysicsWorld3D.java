package org.ironmaple.simulation.physics3d;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena.FieldMap.DoubleRampExtrusion;
import org.ironmaple.simulation.SimulatedArena.FieldMap.ObstacleExtrusion;
import org.ironmaple.simulation.SimulatedArena.FieldMap.RampExtrusion;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ode4j.math.DMatrix3;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.DBody;
import org.ode4j.ode.DContact;
import org.ode4j.ode.DContactBuffer;
import org.ode4j.ode.DConvex;
import org.ode4j.ode.DGeom;
import org.ode4j.ode.DJoint;
import org.ode4j.ode.DJointGroup;
import org.ode4j.ode.DMass;
import org.ode4j.ode.DSpace;
import org.ode4j.ode.DWorld;
import org.ode4j.ode.OdeConstants;
import org.ode4j.ode.OdeHelper;

/**
 * A minimal ode4j world configured for drivetrain simulations.
 *
 * <p>This helper owns the {@link DWorld}, collision space, and contact handling required for the drivetrain body and
 * the field obstacles.
 */
public final class OdePhysicsWorld3D {
    private static final int MAX_CONTACTS = 8;
    private static final double OBSTACLE_HEIGHT_METERS = 16.0;
    private static final double DEFAULT_SURFACE_FRICTION = 0.8;
    private static final double RAMP_DEPTH_BIAS = 0.01;
    private static volatile boolean odeInitialized = false;

    private final DWorld world;
    private final DSpace space;
    private final DJointGroup contactGroup;
    private final DGeom.DNearCallback nearCallback;

    public OdePhysicsWorld3D(
            List<ObstacleExtrusion> obstacleExtrusions,
            List<RampExtrusion> rampExtrusions,
            List<DoubleRampExtrusion> doubleRampExtrusions) {
        ensureOdeInitialized();
        world = OdeHelper.createWorld();
        world.setGravity(0, 0, -9.81);
        world.setERP(0.2);
        world.setCFM(1e-6);

        space = OdeHelper.createHashSpace(null);
        contactGroup = OdeHelper.createJointGroup();
        nearCallback = createNearCallback();

        // Ground plane at z = 0
        OdeHelper.createPlane(space, 0, 0, 1, 0);

        for (ObstacleExtrusion extrusion : obstacleExtrusions) {
            createObstacleGeom(extrusion);
        }
        for (RampExtrusion ramp : rampExtrusions) createRampGeom(ramp);
        for (DoubleRampExtrusion prism : doubleRampExtrusions) createDoubleRampGeom(prism);
    }

    private void createObstacleGeom(ObstacleExtrusion extrusion) {
        final Pose2d pose = extrusion.getPose();
        final DGeom geom =
                OdeHelper.createBox(space, extrusion.getSizeX(), extrusion.getSizeY(), OBSTACLE_HEIGHT_METERS);
        geom.setPosition(pose.getX(), pose.getY(), OBSTACLE_HEIGHT_METERS / 2.0);
        geom.setRotation(yawMatrix(pose.getRotation().getRadians()));
    }

    private void createRampGeom(RampExtrusion ramp) {
        final double length = ramp.getLength();
        final double width = ramp.getWidth();
        final double height = ramp.getHeight();

        final double[] points = new double[] {
            0,
            -width / 2.0,
            0, // 0
            0,
            width / 2.0,
            0, // 1
            length,
            -width / 2.0,
            height, // 2
            length,
            width / 2.0,
            height, // 3
            length,
            -width / 2.0,
            0, // 4
            length,
            width / 2.0,
            0 // 5
        };

        final int[] polygons = new int[] {
            4,
            0,
            1,
            5,
            4, // bottom
            3,
            0,
            4,
            2, // left
            3,
            1,
            3,
            5, // right
            4,
            2,
            3,
            5,
            4, // back
            4,
            0,
            2,
            3,
            1 // slope
        };

        final DConvex convex = createConvex(points, polygons, new double[] {length / 2.0, 0, height / 2.0});

        final Translation2d baseTranslation = ramp.getPose().getTranslation();
        final Rotation2d heading = ramp.getPose().getRotation();
        convex.setRotation(yawMatrix(heading.getRadians()));
        convex.setPosition(baseTranslation.getX(), baseTranslation.getY(), 0);
        convex.setData(new RampMarker());
    }

    private void createDoubleRampGeom(DoubleRampExtrusion ramp) {
        final double halfLength = ramp.getLength() / 2.0;
        final double halfWidth = ramp.getWidth() / 2.0;
        final double height = ramp.getHeight();

        final double[] points = new double[] {
            -halfLength,
            -halfWidth,
            0, // 0
            -halfLength,
            halfWidth,
            0, // 1
            halfLength,
            -halfWidth,
            0, // 2
            halfLength,
            halfWidth,
            0, // 3
            0,
            -halfWidth,
            height, // 4
            0,
            halfWidth,
            height // 5
        };

        final int[] polygons = new int[] {
            4,
            0,
            1,
            5,
            4, // slope left
            4,
            4,
            5,
            3,
            2, // slope right
            3,
            0,
            2,
            4, // side -Y
            3,
            1,
            5,
            3 // side +Y
        };

        final DConvex convex = createConvex(points, polygons, new double[] {0, 0, height / 2.0});
        final Pose2d pose = ramp.getPose();
        convex.setRotation(yawMatrix(pose.getRotation().getRadians()));
        convex.setPosition(pose.getX(), pose.getY(), 0);
        convex.setData(new RampMarker());
    }

    private DConvex createConvex(double[] points, int[] polygons, double[] interiorPoint) {
        final double[] planes = buildPlanes(points, polygons, interiorPoint);
        return OdeHelper.createConvex(space, planes, planes.length / 4, points, points.length / 3, polygons);
    }

    private static double[] buildPlanes(double[] points, int[] polygons, double[] interiorPoint) {
        int faceCount = 0;
        for (int i = 0; i < polygons.length; ) {
            int count = polygons[i];
            faceCount++;
            i += count + 1;
        }
        final double[] planes = new double[faceCount * 4];
        int planeIdx = 0;
        for (int i = 0; i < polygons.length; ) {
            final int count = polygons[i++];
            if (count < 3) {
                throw new IllegalArgumentException("Polygon must have at least 3 vertices.");
            }
            final int i0 = polygons[i];
            final int i1 = polygons[i + 1];
            final int i2 = polygons[i + 2];
            final double[] plane = planeFromPoints(points, i0, i1, i2, interiorPoint);
            System.arraycopy(plane, 0, planes, planeIdx, 4);
            planeIdx += 4;
            i += count;
        }
        return planes;
    }

    private static double[] planeFromPoints(double[] points, int i0, int i1, int i2, double[] interiorPoint) {
        final double[] p0 = getPoint(points, i0);
        final double[] p1 = getPoint(points, i1);
        final double[] p2 = getPoint(points, i2);

        final double[] v1 = subtract(p1, p0);
        final double[] v2 = subtract(p2, p0);
        double[] normal = cross(v1, v2);
        final double length = Math.sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        if (length == 0) {
            normal = new double[] {0, 0, 1};
        } else {
            normal[0] /= length;
            normal[1] /= length;
            normal[2] /= length;
        }
        double d = -(normal[0] * p0[0] + normal[1] * p0[1] + normal[2] * p0[2]);
        final double interiorDistance =
                normal[0] * interiorPoint[0] + normal[1] * interiorPoint[1] + normal[2] * interiorPoint[2] + d;
        if (interiorDistance > 0) {
            normal[0] = -normal[0];
            normal[1] = -normal[1];
            normal[2] = -normal[2];
            d = -d;
        }
        return new double[] {normal[0], normal[1], normal[2], d};
    }

    private static double[] getPoint(double[] points, int index) {
        final int base = index * 3;
        return new double[] {points[base], points[base + 1], points[base + 2]};
    }

    private static double[] subtract(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    public RobotHandle createRobotHandle(
            DriveTrainSimulationConfig config,
            double chassisHeightMeters,
            Translation2d[] wheelOffsets,
            double wheelRadiusMeters) {
        final double length = config.bumperLengthX.in(Meters);
        final double width = config.bumperWidthY.in(Meters);
        final double height = Math.max(chassisHeightMeters, 0.1);

        final DBody body = OdeHelper.createBody(world);
        body.setAutoDisableFlag(false);
        body.setLinearDamping(0.2);
        body.setAngularDamping(0.2);

        final DMass mass = OdeHelper.createMass();
        mass.setBoxTotal(config.robotMass.in(Kilograms), length, width, height);
        body.setMass(mass);

        final DGeom geom = OdeHelper.createBox(space, length, width, height);
        geom.setBody(body);

        attachWheelNubs(body, wheelOffsets, wheelRadiusMeters, height);

        return new RobotHandle(body, height);
    }

    public void step(double dtSeconds) {
        space.collide(null, nearCallback);
        world.quickStep(dtSeconds);
        contactGroup.empty();
    }

    public void dispose() {
        contactGroup.empty();
        contactGroup.destroy();
        space.destroy();
        world.destroy();
    }

    private DGeom.DNearCallback createNearCallback() {
        return (data, geom1, geom2) -> {
            final DContactBuffer contacts = new DContactBuffer(MAX_CONTACTS);
            final int count = OdeHelper.collide(geom1, geom2, MAX_CONTACTS, contacts.getGeomBuffer());
            for (int i = 0; i < count; i++) {
                final DContact contact = contacts.get(i);
                contact.surface.mode = OdeConstants.dContactBounce | OdeConstants.dContactSoftCFM;
                contact.surface.mu = DEFAULT_SURFACE_FRICTION;
                contact.surface.bounce = 0.0;
                contact.surface.bounce_vel = 0.0;
                contact.surface.soft_cfm = 1e-3;
                contact.surface.soft_erp = 0.3;
                final boolean rampContact = isRamp(geom1) || isRamp(geom2);
                if (rampContact) {
                    contact.surface.soft_erp = 0.45;
                    contact.surface.soft_cfm = 5e-4;
                    contact.geom.depth += RAMP_DEPTH_BIAS;
                }

                final DJoint joint = OdeHelper.createContactJoint(world, contactGroup, contact);
                joint.attach(geom1.getBody(), geom2.getBody());
                if (rampContact) {
                    applyNormalLift(geom1, geom2, contact);
                }
            }
        };
    }

    private boolean isRamp(DGeom geom) {
        return geom != null && geom.getData() instanceof RampMarker;
    }

    private static final class RampMarker {}

    private void applyNormalLift(DGeom geom1, DGeom geom2, DContact contact) {
        final DGeom nonRampGeom = isRamp(geom1) ? geom2 : geom1;
        if (nonRampGeom == null) return;
        final DBody body = nonRampGeom.getBody();
        if (body == null) return;
        final double mass = body.getMass().getMass();
        final double liftMagnitude = mass * 9.81 * 0.35;
        final double nx = contact.geom.normal.get0();
        final double ny = contact.geom.normal.get1();
        final double nz = Math.max(0.0, contact.geom.normal.get2());
        body.addForce(nx * liftMagnitude, ny * liftMagnitude, nz * liftMagnitude);
    }

    private static void ensureOdeInitialized() {
        if (odeInitialized) return;
        synchronized (OdePhysicsWorld3D.class) {
            if (!odeInitialized) {
                OdeHelper.initODE2(0);
                odeInitialized = true;
            }
        }
    }

    private static DMatrix3 yawMatrix(double yaw) {
        final double c = Math.cos(yaw);
        final double s = Math.sin(yaw);
        final DMatrix3 matrix = new DMatrix3();
        matrix.set00(c);
        matrix.set01(-s);
        matrix.set02(0);
        matrix.set10(s);
        matrix.set11(c);
        matrix.set12(0);
        matrix.set20(0);
        matrix.set21(0);
        matrix.set22(1);
        return matrix;
    }

    private static Rotation3d toRotation3d(DMatrix3C rotation) {
        final double sy = Math.sqrt(rotation.get00() * rotation.get00() + rotation.get10() * rotation.get10());
        final boolean singular = sy < 1e-6;

        final double roll;
        final double pitch;
        final double yaw;

        if (!singular) {
            roll = Math.atan2(rotation.get21(), rotation.get22());
            pitch = Math.atan2(-rotation.get20(), sy);
            yaw = Math.atan2(rotation.get10(), rotation.get00());
        } else {
            roll = Math.atan2(-rotation.get12(), rotation.get11());
            pitch = Math.atan2(-rotation.get20(), sy);
            yaw = 0;
        }

        return new Rotation3d(roll, pitch, yaw);
    }

    public static final class RobotHandle {
        private final DBody body;
        private final double chassisHeight;

        RobotHandle(DBody body, double chassisHeight) {
            this.body = body;
            this.chassisHeight = chassisHeight;
        }

        public void setPose(Pose3d pose) {
            final double bodyCenterZ = pose.getZ() + chassisHeight / 2.0;
            body.setPosition(pose.getX(), pose.getY(), bodyCenterZ);
            body.setRotation(yawMatrix(pose.getRotation().getZ()));
        }

        public Pose3d getPose() {
            final DVector3C position = body.getPosition();
            final DMatrix3C rotation = body.getRotation();
            final Rotation3d orientation = toRotation3d(rotation);
            final double groundZ = position.get2() - chassisHeight / 2.0;
            return new Pose3d(position.get0(), position.get1(), groundZ, orientation);
        }

        public void setLinearVelocity(double vx, double vy, double vz) {
            body.setLinearVel(vx, vy, vz);
        }

        public void setAngularVelocity(double wx, double wy, double wz) {
            body.setAngularVel(wx, wy, wz);
        }

        public void setLinearDamping(double damping) {
            body.setLinearDamping(damping);
        }

        public void setAngularDamping(double damping) {
            body.setAngularDamping(damping);
        }

        public DVector3C getLinearVelocity() {
            return body.getLinearVel();
        }

        public DVector3C getAngularVelocity() {
            return body.getAngularVel();
        }

        public void addForce(double fx, double fy, double fz) {
            body.addForce(fx, fy, fz);
        }

        public void addTorque(double tx, double ty, double tz) {
            body.addTorque(tx, ty, tz);
        }

        public void resetVelocities() {
            body.setLinearVel(0, 0, 0);
            body.setAngularVel(0, 0, 0);
        }

        public void addForceAtPosition(double fx, double fy, double fz, double px, double py, double pz) {
            body.addForceAtPos(fx, fy, fz, px, py, pz);
        }

        public DMatrix3C getRotationMatrix() {
            return body.getRotation();
        }
    }

    private void attachWheelNubs(
            DBody body, Translation2d[] wheelOffsets, double wheelRadiusMeters, double chassisHeightMeters) {
        if (wheelOffsets == null || wheelOffsets.length == 0) return;
        final double nubHeight = wheelRadiusMeters * 0.8;
        final double zOffset = -chassisHeightMeters / 2.0 + nubHeight / 2.0;
        for (Translation2d offset : wheelOffsets) {
            final DGeom nub = OdeHelper.createCylinder(space, wheelRadiusMeters * 2.0, nubHeight);
            nub.setBody(body);
            nub.setOffsetPosition(offset.getX(), offset.getY(), zOffset);
        }
    }
}
