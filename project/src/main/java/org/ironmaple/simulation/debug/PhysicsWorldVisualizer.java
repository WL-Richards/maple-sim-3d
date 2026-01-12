package org.ironmaple.simulation.debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.ironmaple.simulation.SimulatedArena.FieldMap;
import org.ironmaple.simulation.SimulatedArena.FieldMap.DoubleRampExtrusion;
import org.ironmaple.simulation.SimulatedArena.FieldMap.ObstacleExtrusion;
import org.ironmaple.simulation.SimulatedArena.FieldMap.RampExtrusion;

/**
 * Publishes a wireframe view of the physics world so it can be visualized on a Field2d widget.
 *
 * <p>The wireframe lives under {@code SmartDashboard/MapleSim/PhysicsWireframe}. Load a Field2d widget there to view
 * the static collision geometry that the physics engine uses.
 */
public final class PhysicsWorldVisualizer {
    private static final Field2d FIELD = new Field2d();
    private static final String WIREFRAME_TABLE_KEY = "MapleSim/PhysicsWireframe";
    private static final String OBSTACLE_PREFIX = "Obstacle-";
    private static final String RAMP_PREFIX = "Ramp-";
    private static final String DOUBLE_RAMP_PREFIX = "DoubleRamp-";
    private static final String ROBOT_PREFIX = "Robot-";
    private static final String ROBOT_MODULE_PREFIX = "RobotModule-";

    private static boolean published = false;
    private static int previousObstacleCount = 0;
    private static int previousRampCount = 0;
    private static int previousDoubleRampCount = 0;
    private static final Map<String, Integer> robotModuleCounts = new HashMap<>();

    private PhysicsWorldVisualizer() {}

    public static synchronized void publishFieldMap(FieldMap fieldMap) {
        ensureFieldRegistered();

        final List<ObstacleExtrusion> obstacles = fieldMap.getObstacleExtrusions();
        int index = 0;
        for (ObstacleExtrusion obstacle : obstacles) {
            final FieldObject2d object = FIELD.getObject(OBSTACLE_PREFIX + index++);
            object.setPoses(buildRectangleWireframe(obstacle.getPose(), obstacle.getSizeX(), obstacle.getSizeY()));
        }
        clearUnusedObjects(OBSTACLE_PREFIX, index, previousObstacleCount);
        previousObstacleCount = index;

        final List<RampExtrusion> ramps = fieldMap.getRampExtrusions();
        index = 0;
        for (RampExtrusion ramp : ramps) {
            final FieldObject2d object = FIELD.getObject(RAMP_PREFIX + index++);
            object.setPoses(buildRectangleWireframe(ramp.getPose(), ramp.getLength(), ramp.getWidth()));
        }
        clearUnusedObjects(RAMP_PREFIX, index, previousRampCount);
        previousRampCount = index;

        final List<DoubleRampExtrusion> doubleRamps = fieldMap.getDoubleRampExtrusions();
        index = 0;
        for (DoubleRampExtrusion ramp : doubleRamps) {
            final FieldObject2d object = FIELD.getObject(DOUBLE_RAMP_PREFIX + index++);
            object.setPoses(buildRectangleWireframe(ramp.getPose(), ramp.getLength(), ramp.getWidth()));
        }
        clearUnusedObjects(DOUBLE_RAMP_PREFIX, index, previousDoubleRampCount);
        previousDoubleRampCount = index;
    }

    public static synchronized void publishRobotWireframe(
            String robotName, Pose2d pose, double length, double width, Translation2d[] moduleOffsets) {
        ensureFieldRegistered();

        final FieldObject2d chassis = FIELD.getObject(ROBOT_PREFIX);
        chassis.setPoses(buildRectangleWireframe(pose, length, width));

        final int moduleCount = moduleOffsets == null ? 0 : moduleOffsets.length;
        for (int i = 0; i < moduleCount; i++) {
            Translation2d offset = moduleOffsets[i].rotateBy(pose.getRotation());
            Pose2d modulePose = new Pose2d(pose.getTranslation().plus(offset), Rotation2d.kZero);
            FIELD.getObject(ROBOT_MODULE_PREFIX + "-" + i).setPoses(buildRectangleWireframe(modulePose, 0.15, 0.15));
        }

        final int previousCount = robotModuleCounts.getOrDefault(robotName, 0);
        for (int i = moduleCount; i < previousCount; i++) {
            FIELD.getObject(ROBOT_MODULE_PREFIX + "-" + i).setPoses();
        }
        robotModuleCounts.put(robotName, moduleCount);
    }

    private static void ensureFieldRegistered() {
        if (!published) {
            SmartDashboard.putData(WIREFRAME_TABLE_KEY, FIELD);
            published = true;
        }
    }

    private static void clearUnusedObjects(String prefix, int startIndex, int previousCount) {
        for (int i = startIndex; i < previousCount; i++) {
            FIELD.getObject(prefix + i).setPoses();
        }
    }

    private static Pose2d[] buildRectangleWireframe(Pose2d pose, double sizeX, double sizeY) {
        final double halfX = sizeX / 2.0;
        final double halfY = sizeY / 2.0;

        final Translation2d[] localCorners = new Translation2d[] {
            new Translation2d(-halfX, -halfY),
            new Translation2d(-halfX, halfY),
            new Translation2d(halfX, halfY),
            new Translation2d(halfX, -halfY)
        };

        final Pose2d[] outline = new Pose2d[localCorners.length + 1];
        for (int i = 0; i < localCorners.length; i++) {
            final Translation2d rotatedCorner = localCorners[i].rotateBy(pose.getRotation());
            final Translation2d fieldTranslation = pose.getTranslation().plus(rotatedCorner);
            outline[i] = new Pose2d(fieldTranslation, Rotation2d.kZero);
        }
        outline[outline.length - 1] = outline[0];
        return outline;
    }
}
