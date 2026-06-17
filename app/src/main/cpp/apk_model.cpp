#include "apk_model.h"

#include <algorithm>
#include <cmath>

namespace apk_model {
namespace {

constexpr double Pi = 3.14159265358979323846;

double lerp(double a, double b, double t)
{
    return (b * t) + ((1.0 - t) * a);
}

bool valid(double value)
{
    return std::isfinite(value);
}

void scale(Vec3& value, double factor)
{
    value.x *= factor;
    value.y *= factor;
    value.z *= factor;
}

void add(Vec3& value, double x, double y, double z)
{
    value.x += x;
    value.y += y;
    value.z += z;
}

void setNeutralFeet(double radius,
                    double z,
                    double corner_angle_deg,
                    double elongation,
                    std::array<Vec3, 6>& feet,
                    const std::vector<int>& active)
{
    double radians = Pi * corner_angle_deg / 180.0;
    double cos_v = std::cos(radians);
    double sin_v = std::sin(radians);
    for (int leg : ApkLegOrder) {
        double x = 1.0;
        double y = 0.0;
        if (leg == 0) {
            x = -cos_v;
            y = sin_v;
        } else if (leg == 1) {
            x = -1.0;
        } else if (leg == 2) {
            x = -cos_v;
            y = -sin_v;
        } else if (leg == 3) {
            x = cos_v;
            y = sin_v;
        } else if (leg == 4) {
            x = 1.0;
        } else if (leg == 5) {
            x = cos_v;
            y = -sin_v;
        }
        feet[leg] = {x, y, 0.0};
    }

    if (active.size() < 6) {
        std::array<bool, 6> is_active = {};
        for (int leg : active) {
            is_active[leg] = true;
        }
        int inactive_span = static_cast<int>((6 - active.size()) * 3);
        std::array<int, 6> offsets = {};
        constexpr std::array<int, 6> default_active_order = {5, 2, 1, 0, 3, 4};
        for (int index = 0; index < 6; ++index) {
            int leg = default_active_order[index];
            if (!is_active[leg]) continue;
            for (int step = 1; step < 6; ++step) {
                if (!is_active[default_active_order[(index + step) % 6]]) {
                    offsets[leg] += step;
                }
            }
            offsets[leg] -= inactive_span;
        }
        Vec3 rotated;
        for (int leg : active) {
            Vec3 original = feet[leg];
            rotated = original;
            rotateDegrees(rotated, static_cast<double>(offsets[leg]) * 15.0, 0.0, 0.0);
            double mix = std::min(0.5 / std::sqrt(
                                      (rotated.x - original.x) * (rotated.x - original.x)
                                    + (rotated.y - original.y) * (rotated.y - original.y)
                                    + (rotated.z - original.z) * (rotated.z - original.z)),
                                  1.0);
            scale(original, 1.0 - mix);
            scale(rotated, mix);
            add(original, rotated.x, rotated.y, rotated.z);
            feet[leg] = original;
        }
    }

    for (int leg : ApkLegOrder) {
        scale(feet[leg], radius);
        if (leg != 1 && leg != 4) {
            scale(feet[leg], elongation);
        }
        feet[leg].z = z;
    }
}

double commandNorm(const WalkCommand& command)
{
    return std::sqrt((command.forward * command.forward)
                   + (command.left * command.left)
                   + (command.turn * command.turn));
}

WalkCommand cappedCommand(WalkCommand command)
{
    double magnitude = commandNorm(command);
    if (magnitude > 1.0) {
        command.forward /= magnitude;
        command.left /= magnitude;
        command.turn /= magnitude;
    }
    return command;
}

Pose poseFromMotion(const BodyState& state, const Vec3& motion)
{
    Vec3 translation = {-motion.y, motion.x, 0.0};
    rotateDegrees(translation, state.body.uvw.x, 0.0, 0.0);
    return {{translation.x, translation.y, 0.0}, {motion.z, 0.0, 0.0}};
}

void addPoseInPlace(Pose& target, const Pose& delta)
{
    target.xyz.x += delta.xyz.x;
    target.xyz.y += delta.xyz.y;
    target.xyz.z += delta.xyz.z;
    target.uvw.x += delta.uvw.x;
    target.uvw.y += delta.uvw.y;
    target.uvw.z += delta.uvw.z;
}

Pose animationPose(const BodyState& state, double phase, int animation_id, double scale)
{
    double angle = Pi * phase * 2.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;

    switch (animation_id) {
        case 1:
            z = (-std::cos(angle) * 60.0) + 30.0;
            w = std::sin(angle) * 15.0;
            break;
        case 2:
            x = -std::cos(angle) * 40.0;
            v = std::sin(angle) * 15.0;
            break;
        case 3:
            x = -std::cos(angle) * 40.0;
            u = -std::sin(angle) * 15.0;
            break;
        case 4:
            y = -std::cos(angle) * 60.0;
            w = -std::sin(angle) * 15.0;
            break;
        case 5:
            x = -std::cos(angle) * 40.0;
            v = std::cos(angle) * 15.0;
            w = -7.0;
            break;
        case 6:
            x = -std::cos(angle) * 40.0;
            y = std::sin(angle) * 50.0;
            break;
        case 7:
            x = -std::cos(angle) * 40.0;
            y = std::sin(angle) * 50.0;
            v = std::cos(angle) * 12.0;
            w = std::sin(angle) * 12.0;
            break;
        default: {
            double min_z = 1.7976931348623157E308;
            double max_z = -1.7976931348623157E308;
            for (int leg : ApkLegOrder) {
                min_z = std::min(min_z, state.feet[leg].z);
                max_z = std::max(max_z, state.feet[leg].z);
            }
            z = (max_z - min_z) / 2.0;
            break;
        }
    }

    return {{x * scale, y * scale, z * scale}, {u * scale, v * scale, w * scale}};
}

void blendPose(Pose& target, const Pose& source, double t)
{
    target.xyz.x = lerp(target.xyz.x, source.xyz.x, t);
    target.xyz.y = lerp(target.xyz.y, source.xyz.y, t);
    target.xyz.z = lerp(target.xyz.z, source.xyz.z, t);
    target.uvw.x = lerp(target.uvw.x, source.uvw.x, t);
    target.uvw.y = lerp(target.uvw.y, source.uvw.y, t);
    target.uvw.z = lerp(target.uvw.z, source.uvw.z, t);
}

} // namespace

RobotConfig makeDefaultConfig()
{
    RobotConfig config;
    double half_front_width = config.l1_to_r1 / 2.0;
    double half_length = config.l1_to_l3 / 2.0;
    double half_middle_width = config.l2_to_r2 / 2.0;
    double z = config.leg_connection_z;

    config.mounts[0] = {-half_front_width, half_length, z};
    config.mounts[1] = {-half_middle_width, 0.0, z};
    config.mounts[2] = {-half_front_width, -half_length, z};
    config.mounts[3] = {half_front_width, half_length, z};
    config.mounts[4] = {half_middle_width, 0.0, z};
    config.mounts[5] = {half_front_width, -half_length, z};

    config.neutral_feet = makeNeutralFeet(config.leg_radius,
                                          config.leg_sitting_z,
                                          config.corner_leg_angle_deg,
                                          config.elongation,
                                          std::vector<int>(ApkLegOrder.begin(), ApkLegOrder.end()));
    return config;
}

std::array<Vec3, 6> makeNeutralFeet(double radius,
                                    double z,
                                    double corner_angle_deg,
                                    double elongation,
                                    const std::vector<int>& active)
{
    std::array<Vec3, 6> feet = {};
    setNeutralFeet(radius, z, corner_angle_deg, elongation, feet, active);
    return feet;
}

void rotateDegrees(Vec3& value, double x_deg, double y_deg, double z_deg)
{
    double x_rad = x_deg * Pi / 180.0;
    double y_rad = y_deg * Pi / 180.0;
    double z_rad = z_deg * Pi / 180.0;
    double sx = std::sin(x_rad);
    double cx = std::cos(x_rad);
    double sy = std::sin(y_rad);
    double cy = std::cos(y_rad);
    double sz = std::sin(z_rad);
    double cz = std::cos(z_rad);

    double x = value.x;
    double y = value.y;
    double z = value.z;
    double x_term = (cx * cy) * x;
    double y_term = (((cx * sy) * sz) - (sx * cz)) * y;
    double z_term = ((sx * sz) + ((cx * sy) * cz)) * z;
    value.x = (x_term + y_term) + z_term;

    x_term = (sx * cy) * x;
    double sx_sy = sx * sy;
    y_term = ((cx * cz) + (sx_sy * sz)) * y;
    z_term = ((sx_sy * cz) - (cx * sz)) * z;
    value.y = (x_term + y_term) + z_term;

    x_term = x * (-sy);
    y_term = (sz * cy) * y;
    z_term = (cy * cz) * z;
    value.z = z_term + (x_term + y_term);
}

Pose addPose(const Pose& a, const Pose& b)
{
    return {
        {a.xyz.x + b.xyz.x, a.xyz.y + b.xyz.y, a.xyz.z + b.xyz.z},
        {a.uvw.x + b.uvw.x, a.uvw.y + b.uvw.y, a.uvw.z + b.uvw.z},
    };
}

Pose bodyPoseWithLayer(const BodyState& state, const Pose& layer)
{
    Pose transformed = layer;
    rotateDegrees(transformed.xyz, state.body.uvw.x, 0.0, 0.0);
    rotateDegrees(transformed.uvw, 0.0, 0.0, -state.body.uvw.x);
    return addPose(transformed, state.body);
}

double wrapPhase(double phase)
{
    while (phase < 0.0) {
        phase += 1.0;
    }
    while (phase > 1.0) {
        phase -= 1.0;
    }
    return phase;
}

double gaitSwingFraction(int gait_id)
{
    if (gait_id == 20) {
        return 0.18333333333333335;
    }
    switch (gait_id) {
        case 5: return 0.5;
        case 6: return 0.3333333333333333;
        case 7: return 0.16666666666666666;
        case 8: return 0.25;
        case 9: return 0.4166666666666667;
        case 10: return 0.16666666666666666;
        default: return 0.5;
    }
}

std::array<PhaseWindow, 6> gaitPhaseTable(int gait_id)
{
    static constexpr std::array<PhaseWindow, 6> Tripod = {{
        {0.0, 0.5}, {0.5, 1.0}, {0.0, 0.5},
        {0.5, 1.0}, {0.0, 0.5}, {0.5, 1.0},
    }};
    static constexpr std::array<PhaseWindow, 6> Triple = {{
        {0.0, 0.3333}, {0.6667, 1.0}, {0.3333, 0.6667},
        {0.3333, 0.6667}, {0.0, 0.3333}, {0.6667, 1.0},
    }};
    static constexpr std::array<PhaseWindow, 6> Ripple = {{
        {0.0, 0.1667}, {0.6667, 0.8333}, {0.3333, 0.5},
        {0.5, 0.6667}, {0.1667, 0.3333}, {0.8333, 1.0},
    }};
    static constexpr std::array<PhaseWindow, 6> Ripple15 = {{
        {0.0, 0.25}, {0.6667, 0.9167}, {0.3333, 0.5833},
        {0.5, 0.75}, {0.1667, 0.4167}, {0.8333, 0.0833},
    }};
    static constexpr std::array<PhaseWindow, 6> Triple25 = {{
        {0.0, 0.3333}, {0.5833, 0.9167}, {0.1667, 0.5},
        {0.5, 0.8333}, {0.0833, 0.4167}, {0.6667, 1.0},
    }};
    static constexpr std::array<PhaseWindow, 6> Wave = {{
        {0.0, 0.1667}, {0.1667, 0.3333}, {0.3333, 0.5},
        {0.5, 0.6667}, {0.6667, 0.8333}, {0.8333, 1.0},
    }};
    static constexpr std::array<PhaseWindow, 6> Quad = {{
        {0.06, 0.25}, {-0.1, -0.1}, {0.81, 1.0},
        {0.31, 0.5}, {-0.1, -0.1}, {0.56, 0.75},
    }};

    if (gait_id == 20) return Quad;
    switch (gait_id) {
        case 5: return Tripod;
        case 6: return Triple;
        case 7: return Ripple;
        case 8: return Ripple15;
        case 9: return Triple25;
        case 10: return Wave;
        default: return Tripod;
    }
}

double swingProgress(double phase, double start, double end, double margin)
{
    if (start >= 0.0 && end >= 0.0) {
        double adjusted_start = wrapPhase(start + margin);
        double adjusted_end = wrapPhase(end - margin);
        if (adjusted_start < adjusted_end
            && phase >= adjusted_start
            && phase <= adjusted_end) {
            return (phase - adjusted_start) / (adjusted_end - adjusted_start);
        }
        if (adjusted_start > adjusted_end
            && (phase >= adjusted_start || phase <= adjusted_end)) {
            if (phase - adjusted_start >= 0.0) {
                return (phase - adjusted_start) / ((1.0 - adjusted_start) + adjusted_end);
            }
            double first_span = 1.0 - adjusted_start;
            return (phase + first_span) / (first_span + adjusted_end);
        }
    }
    return -1.0;
}

Vec3 swingTrajectory(const Vec3& from, const Vec3& to, double progress, double lift)
{
    double t = std::sin((progress * Pi) / 2.0);

    Vec3 arc = {
        from.x + ((to.x - from.x) * t),
        from.y + ((to.y - from.y) * t),
        (std::sin(Pi * t) * lift) + from.z + ((to.z - from.z) * t),
    };

    double distance = std::sqrt(
        ((from.x - to.x) * (from.x - to.x))
        + ((from.y - to.y) * (from.y - to.y))
        + ((from.z - to.z) * (from.z - to.z)));
    double lift_fraction = lift / ((lift * 2.0) + distance);
    Vec3 plateau;
    if (t < lift_fraction) {
        double u = t / lift_fraction;
        plateau = {from.x, from.y, from.z + (((from.z + lift) - from.z) * u)};
    } else if (t < 1.0 - lift_fraction) {
        double u = (t - lift_fraction) / (1.0 - (lift_fraction * 2.0));
        plateau = {
            from.x + ((to.x - from.x) * u),
            from.y + ((to.y - from.y) * u),
            from.z + ((to.z - from.z) * u) + lift,
        };
    } else {
        double u = ((t - 1.0) + lift_fraction) / lift_fraction;
        plateau = {to.x, to.y, (to.z + lift) + ((to.z - (to.z + lift)) * u)};
    }

    return {
        plateau.x + ((arc.x - plateau.x) * 0.5),
        plateau.y + ((arc.y - plateau.y) * 0.5),
        plateau.z + ((arc.z - plateau.z) * 0.5),
    };
}

Vec3 neutralFootForBody(const RobotConfig& config,
                        const BodyState& state,
                        const Pose& body_delta,
                        int leg)
{
    Vec3 foot = config.neutral_feet[leg];
    Pose body = state.body;
    addPoseInPlace(body, body_delta);
    rotateDegrees(foot, body.uvw.x, 0.0, 0.0);
    add(foot, body.xyz.x, body.xyz.y, 0.0);
    return foot;
}

void initializeWalkState(const RobotConfig& config, WalkState& state)
{
    state = WalkState{};
    state.initialized = true;
    state.body.body.xyz.z = 40.0;
    state.body.feet = config.neutral_feet;
}

WalkStepResult walkStep(const RobotConfig& config,
                        WalkState& state,
                        WalkCommand command,
                        int gait_id,
                        int animation_id,
                        double dt_ms_scaled,
                        bool allow_new_anchors,
                        const std::array<bool, 6>& active,
                        double phase_override,
                        double cycle_override)
{
    if (!state.initialized) {
        initializeWalkState(config, state);
    }

    WalkStepResult result;
    command = cappedCommand(command);
    double magnitude = commandNorm(command);
    double swing_fraction = gaitSwingFraction(gait_id);
    double cycle = lerp(2.0, 0.5, magnitude) / swing_fraction;
    if (cycle_override > 0.0) {
        cycle = cycle_override;
    }
    result.command = command;
    result.command_magnitude = magnitude;
    result.swing_fraction = swing_fraction;
    result.cycle_seconds = cycle;
    BodyState animation_source = state.body;
    if (phase_override >= 0.0) {
        state.phase = wrapPhase(phase_override);
    } else {
        state.phase = wrapPhase(state.phase + (dt_ms_scaled / (cycle * 1000.0)));
    }

    auto table = gaitPhaseTable(gait_id);
    for (int leg : ApkLegOrder) {
        result.swing_progress[leg] = swingProgress(
            state.phase, table[leg].start, table[leg].end, 0.03);
    }

    if (!allow_new_anchors) {
        for (int leg : ApkLegOrder) {
            if (!state.anchor_active[leg]) {
                result.swing_progress[leg] = -1.0;
            }
        }
    }

    Vec3 velocity = {
        command.forward * 120.0 * config.femur_scale,
        command.left * 90.0 * config.femur_scale,
        command.turn * 30.0 * config.femur_scale,
    };
    double gain = (1.5 - swing_fraction) * 4.0 * swing_fraction;
    scale(velocity, gain);
    scale(velocity, 1.0 / cycle);

    Vec3 delta_motion = velocity;
    scale(delta_motion, dt_ms_scaled / 1000.0);
    Pose body_delta = poseFromMotion(state.body, delta_motion);
    result.velocity_per_second = velocity;
    result.delta_motion = delta_motion;
    result.body_delta = body_delta;

    BodyState candidate = state.body;
    addPoseInPlace(candidate.body, body_delta);
    std::array<std::array<double, 3>, 6> scratch_angles = {};
    result.body_ik_ok = inverseKinematics(config, candidate, state.animation_layer, active, scratch_angles);
    if (result.body_ik_ok) {
        state.body.body = candidate.body;
    }

    for (int leg : ApkLegOrder) {
        double progress = result.swing_progress[leg];
        if (progress >= 0.0 && progress < 0.1 && !state.anchor_active[leg]) {
            state.anchor_active[leg] = true;
            state.anchors[leg] = state.body.feet[leg];
        } else if (progress < 0.0 && state.anchor_active[leg]) {
            state.body.feet[leg].z = config.leg_sitting_z;
            state.anchor_active[leg] = false;
        }
    }

    Vec3 lookahead_motion = velocity;
    scale(lookahead_motion, (1.0 - swing_fraction) * cycle * 0.5);
    Pose lookahead_delta = poseFromMotion(animation_source, lookahead_motion);

    for (int leg : ApkLegOrder) {
        double progress = result.swing_progress[leg];
        if (!state.anchor_active[leg]) {
            continue;
        }

        Vec3 target = neutralFootForBody(config, state.body, lookahead_delta, leg);
        Vec3 lifted = swingTrajectory(state.anchors[leg], target, progress, 40.0);
        result.swings[leg].used = true;
        result.swings[leg].leg = leg;
        result.swings[leg].progress = progress;
        result.swings[leg].lift = 40.0;
        result.swings[leg].from = state.anchors[leg];
        result.swings[leg].to = target;
        result.swings[leg].result = lifted;

        bool clear = true;
        for (int other : ApkLegOrder) {
            if (other == leg || !active[other]) {
                continue;
            }
            double dx = lifted.x - state.body.feet[other].x;
            double dy = lifted.y - state.body.feet[other].y;
            if ((dx * dx) + (dy * dy) < 4900.0) {
                clear = false;
                break;
            }
        }

        if (clear) {
            state.body.feet[leg] = lifted;
        } else {
            state.body.feet[leg].z = lifted.z;
        }
        result.swings[leg].clear = clear;
        result.swings[leg].committed = state.body.feet[leg];
    }

    Pose animation_target = animationPose(animation_source, state.phase, animation_id, config.femur_scale);
    result.animation_target = animation_target;
    blendPose(state.animation_layer, animation_target, 1.0 / 12.0);
    result.animation_layer = state.animation_layer;

    result.ik_ok = inverseKinematics(config, state.body, state.animation_layer, active, result.angles_deg);
    result.phase = state.phase;
    return result;
}

void forwardKinematics(const RobotConfig& config,
                       const std::array<std::array<double, 3>, 6>& angles_deg,
                       const Pose& layer,
                       BodyState& state)
{
    for (int leg : ApkLegOrder) {
        Vec3 foot;
        double coxa = angles_deg[leg][0] * (leg > 2 ? -1.0 : 1.0);
        double femur = angles_deg[leg][1];
        double tibia = angles_deg[leg][2];
        const Vec3& mount = config.mounts[leg];
        double mount_radius = std::sqrt(mount.x * mount.x + mount.y * mount.y);

        foot = {-config.tibia_len, 0.0, 0.0};
        rotateDegrees(foot, tibia, 0.0, 0.0);
        add(foot, config.femur_len, 0.0, 0.0);
        rotateDegrees(foot, femur, 0.0, 0.0);
        add(foot, config.coxa_len, 0.0, 0.0);

        double reach = foot.x;
        foot = {
            (mount.x * reach) / mount_radius,
            (reach * mount.y) / mount_radius,
            foot.y,
        };
        rotateDegrees(foot, -coxa, 0.0, 0.0);
        add(foot, mount.x, mount.y, mount.z);

        Pose body = bodyPoseWithLayer(state, layer);
        rotateDegrees(foot, body.uvw.x, 0.0, 0.0);
        rotateDegrees(foot, 0.0, body.uvw.y, 0.0);
        rotateDegrees(foot, 0.0, 0.0, body.uvw.z);
        add(foot, body.xyz.x, body.xyz.y, body.xyz.z);
        state.feet[leg] = foot;
    }
}

bool inverseKinematics(const RobotConfig& config,
                       const BodyState& state,
                       const Pose& layer,
                       const std::array<bool, 6>& active,
                       std::array<std::array<double, 3>, 6>& angles_deg)
{
    for (int leg : ApkLegOrder) {
        if (!active[leg]) {
            continue;
        }

        Vec3 foot = state.feet[leg];
        Pose body = bodyPoseWithLayer(state, layer);
        add(foot, -body.xyz.x, -body.xyz.y, -body.xyz.z);
        rotateDegrees(foot, 0.0, 0.0, -body.uvw.z);
        rotateDegrees(foot, 0.0, -body.uvw.y, 0.0);
        rotateDegrees(foot, -body.uvw.x, 0.0, 0.0);

        const Vec3& mount = config.mounts[leg];
        double mount_x2 = mount.x * mount.x;
        double mount_y2 = mount.y * mount.y;
        double mount_radius = std::sqrt(mount_y2 + mount_x2);
        double dx = foot.x - mount.x;
        double dy = foot.y - mount.y;
        double dx2 = dx * dx;
        double dy2 = dy * dy;
        double horizontal = std::sqrt(dy2 + dx2);
        double dot = (dy * mount.y) + (dx * mount.x);
        double coxa = std::acos(dot / (mount_radius * horizontal));
        double cross = (dx * mount.y) - (mount.x * dy);
        if (cross < 0.0) {
            coxa *= -1.0;
        }
        coxa = (coxa * 180.0) / Pi;
        if (leg > 2) {
            coxa *= -1.0;
        }

        double l = horizontal - config.coxa_len;
        double z = foot.z - mount.z;
        double l2 = l * l;
        double z2 = z * z;
        double lcz2 = z2 + l2;
        double lcz = std::sqrt(lcz2);

        double femur_len2 = config.femur_len * config.femur_len;
        double tibia_len2 = config.tibia_len * config.tibia_len;
        double femur_part_numerator = (lcz2 + femur_len2) - tibia_len2;
        double femur_part_denominator = (config.femur_len * 2.0) * lcz;
        double femur_part = std::acos(femur_part_numerator / femur_part_denominator);
        femur_part = (femur_part * 180.0) / Pi;

        double neg_l = -l;
        double abs_l = std::abs(l);
        double femur_axis_numerator = (z * 0.0) + (neg_l * neg_l);
        double femur_axis_denominator = lcz * abs_l;
        double femur_axis = std::acos(femur_axis_numerator / femur_axis_denominator);
        double femur_cross = (neg_l * 0.0) - (neg_l * z);
        if (femur_cross < 0.0) {
            femur_axis *= -1.0;
        }
        femur_axis = (femur_axis * 180.0) / Pi;
        double femur = femur_axis + femur_part;

        double tibia_numerator = (tibia_len2 + femur_len2) - lcz2;
        double tibia_denominator = (config.femur_len * 2.0) * config.tibia_len;
        double tibia = std::acos(tibia_numerator / tibia_denominator);
        tibia = (tibia * 180.0) / Pi;

        if (!valid(coxa) || !valid(femur) || !valid(tibia)) {
            return false;
        }

        angles_deg[leg][0] = coxa;
        angles_deg[leg][1] = femur;
        angles_deg[leg][2] = tibia;
    }
    return true;
}

} // namespace apk_model
