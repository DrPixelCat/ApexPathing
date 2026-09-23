package org.codeblooded.ftcodesim.ascope;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/** Adds Apex's active path to FTCodeSim's dedicated AdvantageScope field tabs. */
public final class ApexAdvantageScopeLayout {
    private static final String TWO_D_TITLE = "2D FTCodeSim";
    private static final String THREE_D_TITLE = "3D FTCodeSim";
    private static final String PATH_LOG_KEY =
            "RealOutputs/Apex/CurrentPath";
    private static final String ROBOT_LOG_KEY =
            "RealOutputs/Drivetrain/position ftc coords (m)";
    private static final String MECANUM_ROBOT_LOG_KEY =
            "RealOutputs/Mecanum/position ftc coords (m)";
    private static final String LEGACY_TANK_ROBOT_LOG_KEY =
            "RealOutputs/Tank/position ftc coords (m)";

    private ApexAdvantageScopeLayout() { }

    /** Updates the layout created by FTCodeSim. Has no effect when AdvantageScope is unavailable. */
    public static void install(boolean tank) {
        AdvantageScopeRunner editor = AdvantageScopeRunner.INSTANCE;
        if (editor == null) { return; }
        if (configure(editor.root, editor.mapper, tank)) {
            editor.saveConfig();
        }
    }

    static boolean configure(ObjectNode root, ObjectMapper mapper) {
        return configure(root, mapper, false);
    }

    static boolean configure(ObjectNode root, ObjectMapper mapper, boolean tank) {
        String robotKey = tank ? ROBOT_LOG_KEY : MECANUM_ROBOT_LOG_KEY;
        JsonNode tabsNode = root.path("hubs").path(0).path("state").path("tabs").path("tabs");
        if (!(tabsNode instanceof ArrayNode)) { return false; }

        ArrayNode tabs = (ArrayNode) tabsNode;
        ObjectNode threeDimensionalTab = findTab(tabs, 3, THREE_D_TITLE);
        if (threeDimensionalTab == null) { return false; }

        boolean changed = repairRobotSources(threeDimensionalTab, mapper, robotKey);
        changed |= putPathBehindRobot(threeDimensionalTab, mapper);
        String game = threeDimensionalTab.path("controller").path("game").asText(
                "FTC:2025-2026 Field");

        ObjectNode twoDimensionalTab = findTab(tabs, 2, TWO_D_TITLE);
        if (twoDimensionalTab == null) {
            twoDimensionalTab = createTwoDimensionalTab(mapper, game);
            tabs.add(twoDimensionalTab);
            changed = true;
        }
        changed |= repairRobotSources(twoDimensionalTab, mapper, robotKey);
        changed |= putPathBehindRobot(twoDimensionalTab, mapper);

        // Match the active implementation: alpha.1 mecanum overrides logging; tank inherits it.
        for (JsonNode tab : tabs) {
            if (tab instanceof ObjectNode && (tab.path("type").asInt(-1) == 2 ||
                    tab.path("type").asInt(-1) == 3)) {
                ObjectNode fieldTab = (ObjectNode) tab;
                ArrayNode fieldSources = sources(fieldTab, mapper);
                if (sourceIndexByLogKey(fieldSources, ROBOT_LOG_KEY) >= 0 ||
                        sourceIndexByLogKey(fieldSources, MECANUM_ROBOT_LOG_KEY) >= 0 ||
                        sourceIndexByLogKey(fieldSources, LEGACY_TANK_ROBOT_LOG_KEY) >= 0) {
                    changed |= repairRobotSources(fieldTab, mapper, robotKey);
                }
                if (sourceIndexByLogKey(sources(fieldTab, mapper), PATH_LOG_KEY) >= 0) {
                    changed |= putPathBehindRobot(fieldTab, mapper);
                }
            }
        }
        return changed;
    }

    private static ObjectNode findTab(ArrayNode tabs, int type, String title) {
        for (JsonNode tab : tabs) {
            if (tab instanceof ObjectNode &&
                    tab.path("type").asInt(-1) == type &&
                    title.equals(tab.path("title").asText())) {
                return (ObjectNode) tab;
            }
        }
        return null;
    }

    private static ObjectNode createTwoDimensionalTab(ObjectMapper mapper, String game) {
        ObjectNode controller = mapper.createObjectNode();
        controller.set("sources", mapper.createArrayNode());
        controller.put("field", game);
        controller.put("orientation", 0);
        controller.put("size", "large");

        ObjectNode tab = mapper.createObjectNode();
        tab.put("type", 2);
        tab.put("title", TWO_D_TITLE);
        tab.set("controller", controller);
        tab.put("controllerUUID", "apexpathing2dftcodesim0000000000");
        tab.putNull("renderer");
        tab.put("controlsHeight", 200);
        return tab;
    }

    private static boolean repairRobotSources(ObjectNode tab, ObjectMapper mapper, String robotKey) {
        ArrayNode sources = sources(tab, mapper);
        boolean changed = false;
        boolean hasRobot = false;
        for (int i = 0; i < sources.size(); i++) {
            JsonNode source = sources.get(i);
            String key = source.path("logKey").asText();
            if (!ROBOT_LOG_KEY.equals(key) && !MECANUM_ROBOT_LOG_KEY.equals(key)
                    && !LEGACY_TANK_ROBOT_LOG_KEY.equals(key)) { continue; }
            ObjectNode object = (ObjectNode) source;
            if (!robotKey.equals(key)) {
                object.put("logKey", robotKey);
                changed = true;
            }
            if ("robot".equals(source.path("type").asText())) {
                if (hasRobot) {
                    sources.remove(i--);
                    changed = true;
                    continue;
                }
                hasRobot = true;
                if (!source.path("visible").asBoolean()) {
                    object.put("visible", true);
                    changed = true;
                }
            }
        }
        if (hasRobot) { return changed; }
        ObjectNode options = mapper.createObjectNode();
        if (tab.path("type").asInt() == 3) {
            options.put("model", "CodeBloodedDecode");
        } else {
            options.put("bumpers", "");
        }
        sources.add(source(mapper, "robot", robotKey, "Pose2d", options));
        return true;
    }
    /** Earlier sources draw over later ones, so keep sampled robot boxes below the live robot. */
    private static boolean putPathBehindRobot(ObjectNode tab, ObjectMapper mapper) {
        ArrayNode sources = sources(tab, mapper);
        int existingIndex = sourceIndexByLogKey(sources, PATH_LOG_KEY);
        ObjectNode pathSource;
        boolean changed = false;
        if (existingIndex >= 0) {
            pathSource = (ObjectNode) sources.remove(existingIndex);
            changed = existingIndex != sources.size();
        } else {
            ObjectNode options = mapper.createObjectNode();
            options.put("color", "#00ff00");
            options.put("model", "CodeBloodedDecode");
            pathSource = source(mapper, "ghost", PATH_LOG_KEY, "Pose2d[]", options);
            changed = true;
        }
        if (!"ghost".equals(pathSource.path("type").asText())) {
            pathSource.put("type", "ghost");
            changed = true;
        }
        if (!"Pose2d[]".equals(pathSource.path("logType").asText())) {
            pathSource.put("logType", "Pose2d[]");
            changed = true;
        }
        ObjectNode options = pathSource.with("options");
        if (!"#00ff00".equals(options.path("color").asText())) {
            options.put("color", "#00ff00");
            changed = true;
        }
        if (!"CodeBloodedDecode".equals(options.path("model").asText())) {
            options.put("model", "CodeBloodedDecode");
            changed = true;
        }
        if (options.remove("size") != null) { changed = true; }
        sources.add(pathSource);
        return changed;
    }

    private static int sourceIndexByLogKey(ArrayNode sources, String logKey) {
        for (int i = 0; i < sources.size(); i++) {
            if (logKey.equals(sources.get(i).path("logKey").asText())) { return i; }
        }
        return -1;
    }

    private static int sourceIndex(ArrayNode sources, String type, String logKey) {
        for (int i = 0; i < sources.size(); i++) {
            JsonNode source = sources.get(i);
            if (type.equals(source.path("type").asText()) &&
                    logKey.equals(source.path("logKey").asText())) {
                return i;
            }
        }
        return -1;
    }

    private static ArrayNode sources(ObjectNode tab, ObjectMapper mapper) {
        JsonNode controllerNode = tab.path("controller");
        ObjectNode controller;
        if (controllerNode instanceof ObjectNode) {
            controller = (ObjectNode) controllerNode;
        } else {
            controller = mapper.createObjectNode();
            tab.set("controller", controller);
        }

        JsonNode sourcesNode = controller.path("sources");
        if (sourcesNode instanceof ArrayNode) { return (ArrayNode) sourcesNode; }

        ArrayNode sources = mapper.createArrayNode();
        controller.set("sources", sources);
        return sources;
    }

    private static boolean containsSource(ArrayNode sources, String type, String logKey) {
        return sourceIndex(sources, type, logKey) >= 0;
    }

    private static ObjectNode source(ObjectMapper mapper, String type, String logKey,
                                     String logType, ObjectNode options) {
        ObjectNode source = mapper.createObjectNode();
        source.put("type", type);
        source.put("logKey", logKey);
        source.put("logType", logType);
        source.put("visible", true);
        source.set("options", options);
        return source;
    }
}
