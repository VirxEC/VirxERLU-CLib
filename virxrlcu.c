#include <math.h>
#include <Python.h>
#include <stdio.h>
#include <stdbool.h>

/*
This is VirxERLU-CLib!

Finding if a shot is viable is extremely expensive, and is unrealistic to do in Python.
Therfore, it's done in C!
Using C drastically improves run times, but it also takes a lot more time to develop.
Hence, I've tried my best to strike a nice balance.
Finding viable shot isn't the only thing VirxERLU-CLib does, although it is the main thing.
It can also find if the car will land on the floor, ceiling, or a wall.
VirxERLU-CLib also runs some small physics simulations.
For example, it can find the time needed in order to jump a certain height.

All of the things that VirxERLU-CLib does is very math-intensive, making C the perfect canidate as a language.
Combine this with the fact that you can call C functions inside your Python script, and you have a VERY speedy function!

GitHub repository: https://github.com/VirxEC/VirxERLU
Questions or comments? Join my Discord server! https://discord.gg/5ARzYRD2Na
Want to be notified of VirxERLU updates? You can get notifications by either Watching the github repo, or subscribe to VirxERLU updates on my Discord server!
*/

// Constants

static const double simulation_dt = 1. / 60.;
static const double physics_dt = 1. / 120.;

static const double min_shot_time = 0;
static const double max_shot_time = 6;
static const double max_shot_slope = 3;
static const double max_shot_0_slope = 10;
static const double ground_shot_min_slope = 0.5;
static const double jump_shot_min_slope = 0.75;
static const double double_jump_shot_min_slope = 0.75;
static const double aerial_shot_min_slope = 1.5;
static const double no_adjust_radians = 0.001;
static const double min_adjust_radians = 0.5;
static const double max_adjust_radians = 2;

static const double jump_max_duration = 0.2;
static const double jump_speed = 291. + (2. / 3.);
static const double jump_acc = 1458. + (1. / 3.);

static const double aerial_throttle_accel = 100. * (2. / 3.);
static const double boost_consumption = 100. * (1. / 3.);

static const double brake_force = 3500.;
static const double max_speed = 2300.;
static const double safe_max_speed = 2100.;
static const double max_speed_no_boost = 1410.;
static const double dodge_offset = 0.12;
static const unsigned char max_boost = 100;
static const unsigned char min_boost = 0;

static const double throttle_accel_division = 1400;
static const double start_throttle_accel_m = -36. / 35.;
static const double start_throttle_accel_b = 1600.;
static const double end_throttle_accel_m = -16.;
static const double end_throttle_accel_b = 160.;

static const double curvature_m_0_500 = 5.84e-6;
static const double curvature_b_0_500 = 0.0069;
static const double curvature_m_500_1000 = 3.26e-6;
static const double curvature_b_500_1000 = 0.00561;
static const double curvature_m_1000_1500 = 1.95e-6;
static const double curvature_b_1000_1500 = 0.0043;
static const double curvature_m_1500_1750 = 1.1e-6;
static const double curvature_b_1500_1750 = 0.003025;
static const double curvature_m_1750_2300 = 4e-7;
static const double curvature_b_1750_2300 = 0.0018;

static const double max_jump_height = 250;
static const double min_double_jump_height = 250;
static const double max_double_jump_height = 500;

static const double PI = 3.14159265358979323846;

static const double side_wall = 4080;
static const double goal_line = 5120;
static const double back_wall = 5110;
static const double ceiling = 2030;
static const double flooring = 20;

static const char direction_forwards = 1;
static const char direction_backwards = -1;

static const unsigned char no_boost = 0;

static const char one = 1;
static const char neg_one = -1;

// see further down for some global vector variables

enum ShotType
{
	GROUND_SHOT,
	JUMP_SHOT,
	DOUBLE_JUMP_SHOT,
	AERIAL_SHOT
};

// simple math stuff

static inline double cap(double value, double min_v, double max_v)
{
    return max(min(value, max_v), min_v);
}

static inline signed char sign(int value)
{
    return (value > 0) - (value < 0);
}

// Vector stuff

typedef struct vector
{
    double x;
    double y;
    double z;
} Vector;

typedef struct vec
{
    double x;
    double y;
} Vec;

static inline Vector fatten(Vec vec, double z)
{
    return (Vector){vec.x, vec.y, z};
}

static inline Vec flatten(Vector vec)
{
    return (Vec){vec.x, vec.y};
}

static inline Vector add(Vector vec1, Vector vec2)
{
    return (Vector){vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z};
}

static inline Vec flat_add(Vec vec1, Vec vec2)
{
    return (Vec){vec1.x + vec2.x, vec1.y + vec2.y};
}

static inline Vector sub(Vector vec1, Vector vec2)
{
    return (Vector){vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z};
}

static inline Vec sub2D(Vector vec1, Vector vec2)
{
    return (Vec){vec1.x - vec2.x, vec1.y - vec2.y};
}

static inline Vec flat_sub(Vec vec1, Vec vec2)
{
    return (Vec){vec1.x - vec2.x, vec1.y - vec2.y};
}

static inline Vector multiply(Vector vec1, Vector vec2)
{
    return (Vector){vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z};
}

static inline Vector multiply_d(Vector vec1, double factor)
{
    return (Vector){vec1.x * factor, vec1.y * factor, vec1.z * factor};
}

static inline Vec flat_multiply_d(Vec vec1, double factor)
{
    return (Vec){vec1.x * factor, vec1.y * factor};
}

static inline Vector divide(Vector vec1, Vector vec2)
{
    return (Vector){vec1.x / vec2.x, vec1.y / vec2.y, vec1.z / vec2.z};
}

static inline double dot(Vector vec1, Vector vec2)
{
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

static inline double dot2D(Vector vec1, Vector vec2)
{
    return vec1.x * vec2.x + vec1.y * vec2.y;
}

static inline double flat_dot(Vec vec1, Vec vec2)
{
    return vec1.x * vec2.x + vec1.y * vec2.y;
}

static inline double magnitude(Vector vec)
{
    return sqrt(dot(vec, vec));
}

static inline double magnitude2D(Vector vec)
{
    return sqrt(dot2D(vec, vec));
}

static inline double flat_magnitude(Vec vec)
{
    return sqrt(flat_dot(vec, vec));
}

static inline Vector cross(Vector vec1, Vector vec2)
{
    return (Vector){(vec1.y * vec2.z) - (vec1.z * vec2.y), (vec1.z * vec2.x) - (vec1.x * vec2.z), (vec1.x * vec2.y) - (vec1.y * vec2.x)};
}

static inline Vector cross_z(Vector vec1, char z)
{
    return (Vector){vec1.y * z, -vec1.x * z, 0};
}

static inline Vector flat_cross(Vec vec1, Vec vec2)
{
    return (Vector){vec1.y - vec2.y, vec2.x - vec1.x, (vec1.x * vec2.y) - (vec1.y * vec2.x)};
}

static inline Vec flat_cross_2d_z(Vec vec1, char z)
{
    return (Vec){vec1.y * z, -vec1.x * z};
}

static inline double dist(Vector vec1, Vector vec2)
{
    return magnitude(sub(vec1, vec2));
}

static inline double dist2D(Vector vec1, Vector vec2)
{
    return flat_magnitude(sub2D(vec1, vec2));
}

static inline double flat_dist(Vec vec1, Vec vec2)
{
    return flat_magnitude(flat_sub(vec1, vec2));
}

static inline Vec flat_rotate(Vec vec, double angle)
{
    return (Vec){cos(angle) * vec.x - sin(angle) * vec.y, sin(angle) * vec.x + cos(angle) * vec.y};
}

Vector normalize(Vector vec)
{
    double mag = magnitude(vec);

    if (mag != 0)
        return (Vector){vec.x / mag, vec.y / mag, vec.z / mag};

    return (Vector){0, 0, 0};
}

Vec normalize2D(Vector vec)
{
    double mag = magnitude2D(vec);

    if (mag != 0)
        return (Vec){vec.x / mag, vec.y / mag};

    return (Vec){0, 0};
}

Vec flat_normalize(Vec vec)
{
    double mag = flat_magnitude(vec);

    if (mag != 0)
        return (Vec){vec.x / mag, vec.y / mag};

    return (Vec){0, 0};
}

static inline double angle(Vector vec1, Vector vec2)
{
    return acos(cap(dot(normalize(vec1), normalize(vec2)), -1, 1));
}

static inline double angle2D(Vector vec1, Vector vec2)
{
    return acos(cap(flat_dot(normalize2D(vec1), normalize2D(vec2)), -1, 1));
}

static inline double flat_angle(Vec vec1, Vec vec2)
{
    return acos(cap(flat_dot(flat_normalize(vec1), flat_normalize(vec2)), -1, 1));
}

static inline Vector scale(Vector vec, double value)
{
    return multiply_d(normalize(vec), value);
}

static inline Vec flat_scale(Vec vec, double value)
{
    return flat_multiply_d(flat_normalize(vec), value);
}

Vector clamp2D(Vector vec, Vector start, Vector end)
{
    Vector s = normalize(vec);
    bool right = dot(s, cross(end, (Vector){0, 0, -1})) < 0;
    bool left = dot(s, cross(start, (Vector){0, 0, -1})) > 0;

    if ((dot(end, cross(start, (Vector){0, 0, -1})) > 0) ? (right && left) : (right || left))
        return vec;

    if (dot(start, s) < dot(end, s))
        return end;

    return start;
}

Vector clamp(Vector vec, Vector start, Vector end)
{
    Vector s = clamp2D(vec, start, end);

    if (s.z < start.z)
    {
        s.z = 0;
        s = scale(s, 1 - start.z);
        s.z = start.z;
    }
    else if (s.z > end.z)
    {
        s.z = 0;
        s = scale(s, 1 - end.z);
        s.z = end.z;
    }

    return s;
}

// orientation

typedef struct orientation
{
    Vector forward;
    Vector left;
    Vector up;
} Orientation;

static inline Vector localize(Orientation or_mat, Vector vec)
{
    return (Vector){dot(or_mat.forward, vec), dot(or_mat.left, vec), dot(or_mat.up, vec)};
}

static inline Vector globalize(Orientation or_mat, Vector vec)
{
    return add(multiply_d(or_mat.forward, vec.x), add(multiply_d(or_mat.left, vec.y), multiply_d(or_mat.up, vec.z)));
}

// hitboxes

typedef struct hitbox
{
    double length;
    double width;
    double height;
    Vector offset;
} HitBox;

// other definitions

typedef struct target_vectors
{
    Vector left;
    Vector right;
} Target;

typedef struct post_correction
{
    Target targets;
    bool fits;
} PostCorrection;

static const PostCorrection default_post_correction = {{{0, 0, 0}, {0, 0, 0}}, 1};

typedef struct generic_shot
{
	bool found;
	bool fast;
	enum ShotType shot_type;
	Target targets;
} Shot;

static const Shot default_generic_shot = {0, 0, 0, {{0, 0, 0}, {0, 0, 0}}};

typedef struct car
{
    Vector location;
    Vector velocity;
    Orientation orientation;
    Vector angular_velocity;
    bool demolished;
    bool airborne;
    bool supersonic;
    bool jumped;
    bool doublejumped;
    unsigned char boost;
    unsigned char index;
    HitBox hitbox;
} Car;

static const Car default_car = {{0, 0, 0}, {0, 0, 0}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, {0, 0, 0}}};

typedef struct ball
{
    Vector location;
    Vector velocity;
} Ball;

static const Ball default_ball = {{0, 0, 0}, {0, 0, 0}};

typedef struct jump_time
{
    double press;
    double time;
} JumpTime;

static const JumpTime default_jump_time = {0, 0};

// extra math functions

PostCorrection correct_for_posts(Vector ball_location, Target *targets)
{
    const int ball_radius = 95;
    const Vector goal_line_perp = cross(sub(targets->right, targets->left), (Vector){0, 0, 1});

    const Vector left_adjusted = add(targets->left, cross(normalize(sub(targets->left, ball_location)), (Vector){0, 0, -ball_radius}));
    const Vector right_adjusted = add(targets->right, cross(normalize(sub(targets->right, ball_location)), (Vector){0, 0, ball_radius}));
    const Vector left_corrected = (dot(sub(left_adjusted, targets->left), goal_line_perp) > 0.0) ? targets->left : left_adjusted;
    const Vector right_corrected = (dot(sub(right_adjusted, targets->right), goal_line_perp) > 0.0) ? targets->right : right_adjusted;

    const Vector left_to_right = sub(right_corrected, left_corrected);
    const Vector new_goal_line = normalize(left_to_right);
    const double new_goal_width = magnitude(left_to_right);

    const Vector new_goal_perp = cross(new_goal_line, (Vector){0, 0, 1});
    const Vector goal_center = add(left_corrected, multiply_d(multiply_d(new_goal_line, new_goal_width), 0.5));
    const Vector ball_to_goal = normalize(sub(goal_center, ball_location));

    return (PostCorrection){
        left_corrected,
        right_corrected,
        new_goal_width * fabs(dot(new_goal_perp, ball_to_goal)) > ball_radius * 2};
}

static inline bool in_2d_quarter_field(double point_x, double point_y, HitBox *car_hitbox)
{
    return point_x <= 4096 - car_hitbox->length / 2. && ((point_x >= 893 - car_hitbox->width && point_y <= 5120 - car_hitbox->length / 2.) || point_x <= 893);
}

static inline bool in_field_2d(Vector point_location, HitBox *car_hitbox)
{
    return in_2d_quarter_field(fabs(point_location.x), fabs(point_location.y), car_hitbox);
}

static inline bool in_flat_field(Vec point_location, HitBox *car_hitbox)
{
    return in_2d_quarter_field(fabs(point_location.x), fabs(point_location.y), car_hitbox);
}

static inline bool in_1d_vertical_field(double point_z, double car_hitbox_height)
{
    return car_hitbox_height / 2. <= point_z && point_z <= 2044 - car_hitbox_height;
}

double find_slope(Vector shot_vector, Vector car_to_target)
{
    const double d = dot(shot_vector, car_to_target);
    const double e = fabs(dot(cross(shot_vector, (Vector){0, 0, 1}), car_to_target));

    if (e == 0)
        return 10 * sign((int)d);

    return max(min(d / e, 3), -3);
}

double flat_find_slope(Vec shot_vector, Vec car_to_target)
{
    const double d = flat_dot(shot_vector, car_to_target);
    const double e = fabs(flat_dot(flat_cross_2d_z(shot_vector, 1), car_to_target));

    if (e == 0)
        return 10 * sign((int)d);

    return max(min(d / e, 3), -3);
}

double throttle_acceleration(double car_velocity_x)
{
    const double x = fabs(car_velocity_x);
    if (x >= max_speed_no_boost)
        return 0;

    // use y = mx + b to find the throttle acceleration
    if (x < throttle_accel_division)
        return start_throttle_accel_m * x + start_throttle_accel_b;

    return end_throttle_accel_m * (x - throttle_accel_division) + end_throttle_accel_b;
}

static inline Vector get_shot_vector_2d(Vector *direction, Ball *ball_slice, Target *targets)
{
    return clamp2D(*direction, normalize(sub(targets->left, ball_slice->location)), normalize(sub(targets->right, ball_slice->location)));
}

static inline Vector get_shot_vector_3d(Vector direction, Ball *ball_slice, Target *targets)
{
    return clamp(direction, normalize(sub(targets->left, ball_slice->location)), normalize(sub(targets->right, ball_slice->location)));
}

static inline Vector get_ball_offset(Ball *ball_slice, Vector shot_vector, double ball_radius)
{
    return sub(ball_slice->location, multiply_d(shot_vector, ball_radius));
}

static inline double min_non_neg(double x, double y)
{
    return ((x < y && x >= 0) || (y < 0 && x >= 0)) ? x : y;
}

int smallest_element_index_no_negative(double work_array[], int work_array_length)
{
    int index = 0;
    for(signed char i = 1; i < work_array_length; i++)
    {
        if((work_array[i] > 0 && work_array[i] < work_array[index]) || (work_array[index] < 0 && work_array[i] > work_array[index]))
            index = i;
    }

    return index;
}

// solve for x
// y = a(x - h)^2 + k
// y - k = a(x - h)^2
// (y - k) / a = (x - h)^2
// sqrt((y - k) / a) = x - h
// sqrt((y - k) / a) + h = x
double vertex_quadratic_solve_for_x_min_non_neg(double a, double h, double k, double y)
{
    const double v_sqrt = sqrt((y - k) / a);
    return min_non_neg(v_sqrt + h, -v_sqrt + h);
}

static inline double get_landing_time(double fall_distance, double falling_time_until_terminal_velocity, double falling_distance_until_terminal_velocity, double terminal_velocity, double k, double h, double g)
{
    return (fall_distance * sign((int)-g) <= falling_distance_until_terminal_velocity * sign((int)-g)) ? vertex_quadratic_solve_for_x_min_non_neg(g, h, k, fall_distance) : falling_time_until_terminal_velocity + ((fall_distance - falling_distance_until_terminal_velocity) / terminal_velocity);
}

int find_landing_plane(Vector l, Vector v, double g)
{
    if (fabs(l.y) >= goal_line || (v.x == 0 && v.y == 0 && g == 0))
        return 5;

    double times[] = { -1, -1, -1, -1, -1, -1 }; // { side_wall_pos, side_wall_neg, back_wall_pos, back_wall_neg, ceiling, floor }

    if (v.x != 0)
    {
        times[0] = (side_wall - l.x) / v.x;
        times[1] = (-side_wall - l.x) / v.x;
    }

    if (v.y != 0)
    {
        times[2] = (back_wall - l.y) / v.y;
        times[3] = (-back_wall - l.y) / v.y;
    }

    if (g != 0)
    {
        // this is the vertex of the equation, which also happens to be the apex of the trajectory
        const double h = v.z / -g; // time to apex
        const double k = v.z * v.z / -g; // vertical height at apex

        // a is the current gravity... because reasons
        // double a = g;

        double climb_dist = -l.z;

        // if the gravity is inverted, the the ceiling becomes the floor and the floor becomes the ceiling...
        if (g < 0)
        {
            climb_dist += ceiling;
            if (k >= climb_dist)
                times[4] = vertex_quadratic_solve_for_x_min_non_neg(g, h, k, climb_dist);
        }
        else if (g > 0)
        {
            climb_dist += flooring;
            if (k <= climb_dist)
                times[5] = vertex_quadratic_solve_for_x_min_non_neg(g, h, k, climb_dist);
        }

        // this is necessary because after we reach our terminal velocity, the equation becomes linear (distance_remaining / terminal_velocity)
        const double terminal_velocity = (max_speed - magnitude2D(v)) * sign((int)g);
        const double falling_time_until_terminal_velocity = (terminal_velocity - v.z) / g;
        const double falling_distance_until_terminal_velocity = v.z * falling_time_until_terminal_velocity + -g * (falling_time_until_terminal_velocity * falling_time_until_terminal_velocity) / 2.;

        const double fall_distance = -l.z;
        if (g < 0)
            times[5] = get_landing_time(fall_distance + flooring, falling_time_until_terminal_velocity, falling_distance_until_terminal_velocity, terminal_velocity, k, h, g);
        else
            times[4] = get_landing_time(fall_distance + ceiling, falling_time_until_terminal_velocity, falling_distance_until_terminal_velocity, terminal_velocity, k, h, g);
    }

    return smallest_element_index_no_negative(times, 6);
}

// v is the magnitude of the velocity in the car's forward direction
double curvature(double v)
{
    if (0 <= v && v < 500)
        return curvature_b_0_500 - curvature_m_0_500 * v;

    if (500 <= v && v < 1000)
        return curvature_b_500_1000 - curvature_m_500_1000 * v;

    if (1000 <= v && v < 1500)
        return curvature_b_1000_1500 - curvature_m_1000_1500 * v;

    if (1500 <= v && v < 1750)
        return curvature_b_1500_1750 - curvature_m_1500_1750 * v;

    if (1750 <= v && v < 2500)
        return curvature_b_1750_2300 - curvature_m_1750_2300 * v;

    return 0;
}

static inline double turn_radius(double v)
{
    return 1. / curvature(v);
}

// use y=mx+b to linearly scale the minimum shot slope
double get_adjusted_min_slope(double *T, double *jump_time, const double *min_shot_slope)
{
    const double m = (*min_shot_slope - (max_shot_slope + *jump_time / 2.)) / (max_shot_time - *jump_time);
    const double b = *min_shot_slope - m * max_shot_time;

    return m * *T + b;
}

// intersection point = origin + direction * t1
double ray_intersects_with_line(Vec origin, Vec direction, Vec p1, Vec p2)
{
    const Vec v1 = flat_sub(origin, p1);
    const Vec v2 = flat_sub(p2, p1);
    const Vec v3 = (Vec){-direction.y, direction.x};
    const double v_dot = flat_dot(v2, v3);

    const double t1 = magnitude(flat_cross(v2, v1)) / v_dot;

    if (t1 < 0)
        return -1;

    const double t2 = flat_dot(v1, v3) / v_dot;

    if (0 <= t1 && t2 <= 1)
        return t1;

    return -1;
}

double get_travel_distance(Vec ball_location, Vec offset_ball_location, Vec car_location, Vec shot_vector, double offset_hitbox_width)
{
    const Vec car_to_ball = flat_sub(ball_location, car_location);
    const char side_of_shot = sign((int)flat_dot(flat_cross_2d_z(shot_vector, one), car_to_ball));
    const Vec ray_direction = flat_rotate(flat_multiply_d(shot_vector, -1), side_of_shot * -min_adjust_radians);
    const Vec car_to_offset_perp = flat_normalize(flat_cross_2d_z(flat_sub(offset_ball_location, car_location), side_of_shot));
    const Vec adjust_target = flat_sub(offset_ball_location, flat_multiply_d(shot_vector, 2560));
    const double distance_from_turn = ray_intersects_with_line(ball_location, ray_direction, car_location, adjust_target);
    const Vec final_target = flat_add(offset_ball_location, flat_multiply_d(car_to_offset_perp, offset_hitbox_width));

    if (distance_from_turn == -1)
        return flat_dist(car_location, final_target);

    const Vec car_turn_point = flat_add(ball_location, flat_multiply_d(ray_direction, distance_from_turn));
    return flat_dist(car_location, car_turn_point) + flat_dist(car_turn_point, final_target);
}

// physics simulations

JumpTime get_jump_time(double car_to_target_z, double car_z_velocity, double gravity_z)
{
    JumpTime r = default_jump_time;
    double l = 0;
    double v = car_z_velocity;

    const double g = gravity_z * simulation_dt;
    const double g2 = 2 * gravity_z;
    const double ja = jump_acc * simulation_dt;

    v += jump_speed;
    v += gravity_z * physics_dt;
    l += v * physics_dt;
    r.time += physics_dt;

    // we can only hold the jump button for max 0.2 seconds
    while (car_to_target_z - l > 0)
    {
        if (r.time <= jump_max_duration && car_to_target_z - l > (v * v) / g2)
            r.press += simulation_dt;
            v += ja;

        v += g;

        if (v <= 0)
            return r;

        l += v * simulation_dt;
        r.time += simulation_dt;
    }

    return r;
}

JumpTime get_double_jump_time(double car_to_target_z, double car_z_velocity, double gravity_z)
{
    JumpTime r = default_jump_time;
    double l = 0;
    double v = car_z_velocity;
    const double g = gravity_z * simulation_dt;
    const double g2 = 2 * gravity_z;
    const double ja = jump_acc * simulation_dt;
    int c = 0;

    bool jump_done = false;

    v += jump_speed;
    v += gravity_z * physics_dt;
    l += v * physics_dt;
    r.time += physics_dt;

    // we can only hold the jump button for max 0.2 seconds
    while (car_to_target_z - l > 0)
    {
        if (c == 1)
            v += jump_speed;

        if (!jump_done)
        {
            if (r.time > jump_max_duration)
                jump_done = 1;
            else
            {
                double v_inc = v + jump_speed;
                if (car_to_target_z - l > (v_inc * v_inc) / g2)
                {
                    r.press += simulation_dt;
                    v += ja;
                }
                else
                    jump_done = 1;
            }
        }
        else if (c < 2)
            c++;

        v += g;

        if (v <= 0)
            return r;

        l += v * simulation_dt;
        r.time += simulation_dt;
    }

    return r;
}


double time_to_speed(double boost_accel, double velocity, unsigned char car_boost, double target_speed)
{
    double T = 0;
    double b = (double)car_boost;
    double v = velocity;
    char ts = 0;

    const double ba_dt = boost_accel * simulation_dt;
    const double bc_dt = boost_consumption * simulation_dt;
    const double bk_dt = brake_force * simulation_dt;
    const bool limited_boost = car_boost <= max_boost;
    const double r = target_speed;

    if (r > ((b < bc_dt && v <= max_speed_no_boost) ? max_speed_no_boost : max_speed) || (b < bc_dt && v > max_speed_no_boost && r > v))
        return -1;

    double t = v - r;
    if (fabs(t) < 90)
        return 0;

    while (!(r > ((b < bc_dt && v <= max_speed_no_boost) ? max_speed_no_boost : max_speed) || (b < bc_dt && v > max_speed_no_boost && r > v)))
    {
        ts = sign((int)(t));

        v += (ts == -sign((int)v)) ? (bk_dt * ts) : (throttle_acceleration(v) * simulation_dt * ts);

        if (b > bc_dt && v >= 0 && t >= ba_dt)
        {
            v += ba_dt;
            if (limited_boost) b -= bc_dt;
        }

        T += simulation_dt;

        t = r - v;
        if (fabs(t) < 90)
            return T;
    }

    return -1;
}

double can_reach_target(double max_time, double distance_remaining, double boost_accel, double velocity, unsigned char car_boost, char direction, bool safe)
{
    double d = distance_remaining;
    double T = max_time;
    double b = (double)car_boost;
    double v = velocity;
    char ts = 0;

    const double ba_dt = boost_accel * simulation_dt;
    const double bc_dt = boost_consumption * simulation_dt;
    const double bk_dt = brake_force * simulation_dt;
    const bool limited_boost = car_boost <= max_boost;
    const bool forwards = direction == 1;
    const bool end_time = simulation_dt * (safe) ? -1 : 10;
    double r = max_time / d * direction;

    if (r > ((b < bc_dt && v <= max_speed_no_boost) ? max_speed_no_boost : max_speed) || (b < bc_dt && v > max_speed_no_boost && r > v))
        return -1;

    int t = (int)round(r - v);
    if (abs(t) < 100)
        return max_time;

    while (T >= -simulation_dt && !(r > ((b < bc_dt && v <= max_speed_no_boost) ? max_speed_no_boost : max_speed) || (b < bc_dt && v > max_speed_no_boost && r > v)))
    {
        ts = sign(t);

        v += (ts == -sign((int)v)) ? (bk_dt * ts) : (throttle_acceleration(v) * simulation_dt * ts);

        if (v >= 0 && b > bc_dt)
        {
            v += ba_dt;
            if (limited_boost) b -= bc_dt;
        }

        d -= v * simulation_dt;
        T -= simulation_dt;

        t = (int)round(r - v);
        if (d <= 0 || abs(t) < 100 || (forwards) ? v > r : v < r)
            return T;

        if (safe)
            r = T / d * direction;
    }

    return -1;
}

// Parsing shot slices

double generic_is_viable(double *T, double *boost_accel, Car *me, Vec direction, double distance_remaining, bool safe)
{
    if (distance_remaining < me->hitbox.length * 0.05)
        return *T;

	if (*T <= 0 || distance_remaining / *T > max_speed)
		return -1;

    const double forward_angle = fabs(flat_angle(direction, flatten(me->orientation.forward)));
    const double backward_angle = PI - forward_angle;

    const double true_car_speed = dot(me->orientation.forward, me->velocity);
    const double car_speed = magnitude2D(me->velocity);

    const double time = *T;
    double forward_time, backward_time;

    if (car_speed > 400)
    {
        const double turn_rad = turn_radius(car_speed);
        forward_time = time - (forward_angle * turn_rad / car_speed);
        backward_time = time - (backward_angle * turn_rad / car_speed);
    }
    else
    {
        forward_time = time - (1.8 * (forward_angle / PI));
        backward_time = time - (1.8 * (backward_angle / PI));
    }

    if (forward_angle < 1.6 || distance_remaining / backward_time > max_speed_no_boost)
    {
        if (forward_time > 0)
        {
            forward_time = can_reach_target(forward_time, distance_remaining, *boost_accel, true_car_speed, me->boost, direction_forwards, safe);

            if (forward_time != -1)
                return forward_time;
        }
    }
    else if (backward_time > 0)
    {
        backward_time = can_reach_target(backward_time, distance_remaining, *boost_accel, true_car_speed, min_boost, direction_backwards, safe);

        if (backward_time != -1)
            return backward_time;
    }

    return -1;
}

static inline bool ground_shot_is_viable(double T, double hitbox_height, bool airborne, double offset_target_z)
{
    return offset_target_z < hitbox_height * 2. && !airborne;
}

bool jump_shot_is_viable(double T, double gravity_z, double velocity_z, double hitbox_height, bool airborne, double offset_target_z, bool safe)
{
    if (safe && (airborne || hitbox_height * 1.5 > offset_target_z || offset_target_z > max_jump_height + (hitbox_height / 2.)))
        return false;

    if (!safe)
        return T >= 0.2;

    return T >= get_jump_time(offset_target_z, velocity_z, gravity_z).time + dodge_offset + 0.05;
}

bool double_jump_shot_is_viable(double T, double gravity_z, double velocity_z, double half_hitbox_height, bool airborne, double offset_target_z, bool safe)
{
    if (safe && (airborne || min_double_jump_height + half_hitbox_height > offset_target_z || offset_target_z > max_double_jump_height + half_hitbox_height))
        return false;

    if (!safe)
        return T >= 0.2;

    return T >= get_double_jump_time(offset_target_z, velocity_z, gravity_z).time + dodge_offset + 0.05;
}

bool *basic_aerial_validation(Vector target, Vector xf, Vector vf, Vector car_forward, unsigned char car_boost, double T, double boost_accel, bool safe)
{
    const Vector delta_x = sub(target, xf);
    const Vector f = normalize(delta_x);

    const double phi = fabs(angle(f, car_forward));
    double turn_time = 2. * sqrt(phi / 9.);
    if (!safe)
    	turn_time *= 0.7;
    const double tau1 = turn_time * cap(1. - 0.3 / phi, 0, 1);
    const double required_acc = (2. * magnitude(delta_x) / ((T - tau1) * (T - tau1)));
    const double ratio = required_acc / boost_accel;
    const double tau2 = T - (T - tau1) * sqrt(1. - cap(ratio, 0, 1));
    const double boost_estimate = floor((tau2 - tau1) * boost_consumption);

    const double velocity_estimate = magnitude(add(vf, multiply_d(f, boost_accel * (tau2 - tau1))));

    static bool valid[3];

    if (safe)
    {
        valid[0] = car_boost > max_boost || boost_estimate < 0.9 * (double)car_boost;
        valid[1] = fabs(ratio) < 0.9;
        valid[2] = velocity_estimate < safe_max_speed;
    }
    else
    {
        valid[0] = car_boost > max_boost || boost_estimate < (double)car_boost;
        valid[1] = fabs(ratio) < 1.;
        valid[2] = velocity_estimate < max_speed;
    }

    return valid;
}

bool aerial_shot_is_viable(double T, double boost_accel, Vector gravity, Car *me, Vector target, bool *fast, bool safe)
{
    boost_accel += aerial_throttle_accel;

    const Vector vf_base = add(me->velocity, multiply_d(gravity, T));
    const Vector xf_base = add(add(me->location, multiply_d(me->velocity, T)), multiply_d(multiply_d(gravity, 0.5), T * T));
    const double total_jump_acc = jump_speed + jump_acc * jump_max_duration;
    const double partial_jump_loc = jump_acc * (T * jump_max_duration - 0.5 * jump_max_duration * jump_max_duration);

    Vector vf, xf;

    if (!me->airborne)
    {
        vf = add(vf_base, multiply_d(me->orientation.up, jump_speed + total_jump_acc));
        xf = add(xf_base, multiply_d(me->orientation.up, jump_speed * (2. * T - jump_max_duration) + partial_jump_loc));
    }
    else
    {
        vf = vf_base;
        xf = xf_base;
    }

    bool *valid;

    valid = basic_aerial_validation(target, xf, vf, me->orientation.forward, me->boost, T, boost_accel, safe);

    bool found = valid[0] && valid[1] && valid[2];

    if (found && !me->airborne)
        *fast = 1;

    if (!me->airborne && (T < 1.45 || !found))
    {
        vf = add(vf_base, multiply_d(me->orientation.up, total_jump_acc));
        xf = add(xf_base, multiply_d(me->orientation.up, jump_speed * T + partial_jump_loc));

        valid = basic_aerial_validation(target, xf, vf, me->orientation.forward, me->boost, T, boost_accel, safe);

        if (valid[0] && valid[1] && valid[2])
        {
            found = 1;
            *fast = 0;
        }
    }

    return found;
}

Shot parse_slice_for_shot_with_target(bool do_ground_shot, bool do_jump_shot, bool do_double_jump_shot, bool do_aerial_shot, double *time_remaining, double *ball_radius, double *boost_accel, Vector *gravity, Ball *ball_slice, Car *me, Target *targets, bool safe, bool can_large_adjust)
{
	Shot r = default_generic_shot;

	const Vector car_to_ball = sub(ball_slice->location, me->location);
	const PostCorrection post_info = correct_for_posts(ball_slice->location, targets);

	if(post_info.fits)
	{
		r.targets = post_info.targets;
        const Vector shot_vector = get_shot_vector_3d(normalize(car_to_ball), ball_slice, &r.targets);
        const Vector ball_offset = get_ball_offset(ball_slice, shot_vector, *ball_radius);
        const double angle_to_target = fabs(angle2D(car_to_ball, shot_vector));

        if (angle_to_target < max_adjust_radians)
        {
            const bool is_min_adjust = angle_to_target < min_adjust_radians;

            if (can_large_adjust || is_min_adjust)
            {
                const double offset_hitbox_width = me->hitbox.width * 0.5;
                const Vec car_location = flatten(me->location);
                Vec final_target = flatten(ball_offset);

                if (angle_to_target > no_adjust_radians)
                {
                    const signed char side_of_shot = sign((int)dot(cross_z(shot_vector, one), car_to_ball));
                    const Vec car_to_offset_target = flat_sub(final_target, car_location);
                    const Vec car_to_offset_perp = flat_normalize(flat_cross_2d_z(car_to_offset_target, side_of_shot));

                    final_target = flat_add(final_target, (is_min_adjust) ? flat_multiply_d(car_to_offset_perp, offset_hitbox_width) : flat_multiply_d(flatten(shot_vector), -(2560 - *ball_radius)));
                }

                if (in_flat_field(final_target, &me->hitbox))
                {
                    if (do_ground_shot || do_jump_shot || do_double_jump_shot)
                    {
                        const double distance_remaining = get_travel_distance(flatten(ball_slice->location), flatten(ball_offset), car_location, flatten(shot_vector), offset_hitbox_width) - (me->hitbox.length * 0.45);
                        const double T = generic_is_viable(time_remaining, boost_accel, me, flat_normalize(flat_sub(final_target, car_location)), distance_remaining, safe);

                        if (T != -1)
                        {
                            if (do_ground_shot)
                            {
                                r.found = ground_shot_is_viable(T, me->hitbox.height, me->airborne, ball_offset.z);
                                r.shot_type = GROUND_SHOT;
                            }

                            if (do_jump_shot && !r.found)
                            {
                                r.found = jump_shot_is_viable(T, gravity->z, me->velocity.z, me->hitbox.height, me->airborne, ball_offset.z, safe);
                                r.shot_type = JUMP_SHOT;
                            }

                            if (do_double_jump_shot && !r.found)
                            {
                                r.found = double_jump_shot_is_viable(T, gravity->z, me->velocity.z, me->hitbox.height / 2., me->airborne, ball_offset.z, safe);
                                r.shot_type = DOUBLE_JUMP_SHOT;
                            }
                        }
                    }

                    if (do_aerial_shot && !r.found && *time_remaining > 0.2 && me->boost > no_boost && angle_to_target < min_adjust_radians && in_1d_vertical_field(ball_offset.z, me->hitbox.height))
                    {
                        r.found = aerial_shot_is_viable(*time_remaining, *boost_accel, *gravity, me, fatten(final_target, ball_offset.z), &r.fast, safe);
                        r.shot_type = AERIAL_SHOT;
                    }
                }
            }
        }
	}

	return r;
}

Shot parse_slice_for_shot(bool do_ground_shot, bool do_jump_shot, bool do_double_jump_shot, bool do_aerial_shot, double *time_remaining, double *ball_radius, double *boost_accel, Vector *gravity, Ball *ball_slice, Car *me, bool safe)
{
	Shot r = default_generic_shot;

	const Vector direction = normalize(sub(ball_slice->location, me->location));
	const Vector ball_offset = get_ball_offset(ball_slice, direction, *ball_radius);

    if (in_field_2d(ball_offset, &me->hitbox))
    {
        if (do_ground_shot || do_jump_shot || do_double_jump_shot)
        {
            const double distance_remaining = dist2D(me->location, ball_offset) - (me->hitbox.length * 0.45);
            const double T = generic_is_viable(time_remaining, boost_accel, me, flat_normalize(sub2D(ball_offset, me->location)), distance_remaining, safe);

            if (T != -1)
            {
                if (do_ground_shot)
                {
                    r.found = ground_shot_is_viable(T, me->hitbox.height, me->airborne, ball_offset.z);
                    r.shot_type = GROUND_SHOT;
                }

                if (do_jump_shot && !r.found)
                {
                    r.found = jump_shot_is_viable(T, gravity->z, me->velocity.z, me->hitbox.height, me->airborne, ball_offset.z, safe);
                    r.shot_type = JUMP_SHOT;
                }

                if (do_double_jump_shot && !r.found)
                {
                    r.found = double_jump_shot_is_viable(T, gravity->z, me->velocity.z, me->hitbox.height / 2., me->airborne, ball_offset.z, safe);
                    r.shot_type = DOUBLE_JUMP_SHOT;
                }
            }
        }

        if (do_aerial_shot && !r.found && in_1d_vertical_field(fabs(ball_offset.z), me->hitbox.height))
        {
            r.found = aerial_shot_is_viable(*time_remaining, *boost_accel, *gravity, me, ball_offset, &r.fast, safe);
            r.shot_type = AERIAL_SHOT;
        }
    }

	return r;
}

// Linking the C functions to Python methods

static PyObject *method_ground_shot_is_viable(PyObject *self, PyObject *args)
{
    double T, boost_accel, offset_target_z, distance_remaining, dir_z;
    Vec direction;
    Car me;

    if (!PyArg_ParseTuple(args, "dd((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))d(ddd)d", &T, &boost_accel, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &offset_target_z, &direction.x, &direction.y, &dir_z, &distance_remaining))
        return NULL;

    const double T_r = generic_is_viable(&T, &boost_accel, &me, direction, distance_remaining, 0);
    const bool shot_viable = T_r != -1 && ground_shot_is_viable(T_r, me.hitbox.height, me.airborne, offset_target_z - 10);

    if (shot_viable)
        Py_RETURN_TRUE;
    else
        Py_RETURN_FALSE;
}

static PyObject *method_jump_shot_is_viable(PyObject *self, PyObject *args)
{
    double T, boost_accel, offset_target_z, distance_remaining, dir_z;
    Vector gravity;
    Vec direction;
    Car me;

    if (!PyArg_ParseTuple(args, "dd(ddd)((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))d(ddd)d", &T, &boost_accel, &gravity.x, &gravity.y, &gravity.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &offset_target_z, &direction.x, &direction.y, &dir_z, &distance_remaining))
        return NULL;

    const double T_r = generic_is_viable(&T, &boost_accel, &me, direction, distance_remaining, 0);
    const bool shot_viable = T_r != -1 && jump_shot_is_viable(T_r, gravity.z, me.velocity.z, me.hitbox.height, me.airborne, offset_target_z, 0);

    if (shot_viable)
        Py_RETURN_TRUE;
    else
        Py_RETURN_FALSE;
}

static PyObject *method_double_jump_shot_is_viable(PyObject *self, PyObject *args)
{
    double T, boost_accel, offset_target_z, distance_remaining, dir_z;
    Vector gravity;
    Vec direction;
    Car me;

    if (!PyArg_ParseTuple(args, "dd(ddd)((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))d(ddd)d", &T, &boost_accel, &gravity.x, &gravity.y, &gravity.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &offset_target_z, &direction.x, &direction.y, &dir_z, &distance_remaining))
        return NULL;

    const double T_r = generic_is_viable(&T, &boost_accel, &me, direction, distance_remaining, 0);
    const bool shot_viable = T_r != -1 && double_jump_shot_is_viable(T_r, gravity.z, me.velocity.z, me.hitbox.height, me.airborne, offset_target_z, 0);

    if (shot_viable)
        Py_RETURN_TRUE;
    else
        Py_RETURN_FALSE;
}

static PyObject *method_aerial_shot_is_viable(PyObject *self, PyObject *args)
{
    bool fast = 0;
    double T, boost_accel;
    Vector gravity, target_;
    Car me;

    if (!PyArg_ParseTuple(args, "dd(ddd)((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))(ddd)", &T, &boost_accel, &gravity.x, &gravity.y, &gravity.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &target_.x, &target_.y, &target_.z))
        return NULL;

    const bool shot_viable = aerial_shot_is_viable(T, boost_accel, gravity, &me, target_, &fast, 0);

    if (shot_viable)
        Py_RETURN_TRUE;
    else
        Py_RETURN_FALSE;
}

static PyObject *method_get_travel_distance(PyObject *self, PyObject *args)
{
    Vec ball_location, offset_ball_location, car_location, shot_vector;
    double offset_hitbox_width;

    if (!PyArg_ParseTuple(args, "(dd)(dd)(dd)(dd)d", &ball_location.x, &ball_location.y, &offset_ball_location.x, &offset_ball_location.y, &car_location.x, &car_location.y, &shot_vector.x, &shot_vector.y, &offset_hitbox_width))
        return NULL;

    return Py_BuildValue("d", get_travel_distance(ball_location, offset_ball_location, car_location, shot_vector, offset_hitbox_width));
}

static PyObject *method_parse_slice_for_shot_with_target(PyObject *self, PyObject *args)
{
	bool do_ground_shot, do_jump_shot, do_double_jump_shot, do_aerial_shot;
    bool safe_shots = 1;
    bool can_large_adjust = 0;
    double T, boost_accel, ball_radius;
    Vector gravity;
    Target targets;
    Ball ball_slice;
    Car me;

    if (!PyArg_ParseTuple(args, "bbbbddd(ddd)((ddd)(ddd))((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))((ddd)(ddd))|bb", &do_ground_shot, &do_jump_shot, &do_double_jump_shot, &do_aerial_shot, &T, &boost_accel, &ball_radius, &gravity.x, &gravity.y, &gravity.z, &ball_slice.location.x, &ball_slice.location.y, &ball_slice.location.z, &ball_slice.velocity.x, &ball_slice.velocity.y, &ball_slice.velocity.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &targets.left.x, &targets.left.y, &targets.left.z, &targets.left.x, &targets.left.y, &targets.left.z, &safe_shots, &can_large_adjust))
        return NULL;

	const Shot shot_viable = parse_slice_for_shot_with_target(do_ground_shot, do_jump_shot, do_double_jump_shot, do_aerial_shot, &T, &ball_radius, &boost_accel, &gravity, &ball_slice, &me, &targets, safe_shots, can_large_adjust);

    return Py_BuildValue("{s:i,s:i,s:i,s:((ddd)(ddd))}", "found", shot_viable.found, "fast", shot_viable.fast, "shot_type", shot_viable.shot_type, "targets", shot_viable.targets.left.x, shot_viable.targets.left.y, shot_viable.targets.left.z, shot_viable.targets.left.x, shot_viable.targets.left.y, shot_viable.targets.left.z);
}

static PyObject *method_parse_slice_for_shot(PyObject *self, PyObject *args)
{
	bool do_ground_shot, do_jump_shot, do_double_jump_shot, do_aerial_shot;
    bool safe_shots = 1;
    double T, boost_accel, ball_radius;
    Vector gravity;
    Ball ball_slice;
    Car me;

    if (!PyArg_ParseTuple(args, "bbbbddd(ddd)((ddd)(ddd))((ddd)(ddd)((ddd)(ddd)(ddd))(ddd)bbbbbbb(ddd)(ddd))|b", &do_ground_shot, &do_jump_shot, &do_double_jump_shot, &do_aerial_shot, &T, &boost_accel, &ball_radius, &gravity.x, &gravity.y, &gravity.z, &ball_slice.location.x, &ball_slice.location.y, &ball_slice.location.z, &ball_slice.velocity.x, &ball_slice.velocity.y, &ball_slice.velocity.z, &me.location.x, &me.location.y, &me.location.z, &me.velocity.x, &me.velocity.y, &me.velocity.z, &me.orientation.forward.x, &me.orientation.forward.y, &me.orientation.forward.z, &me.orientation.left.x, &me.orientation.left.y, &me.orientation.left.z, &me.orientation.up.x, &me.orientation.up.y, &me.orientation.up.z, &me.angular_velocity.x, &me.angular_velocity.y, &me.angular_velocity.z, &me.demolished, &me.airborne, &me.supersonic, &me.jumped, &me.doublejumped, &me.boost, &me.index, &me.hitbox.length, &me.hitbox.width, &me.hitbox.height, &me.hitbox.offset.x, &me.hitbox.offset.y, &me.hitbox.offset.z, &safe_shots))
        return NULL;

	const Shot shot_viable = parse_slice_for_shot(do_ground_shot, do_jump_shot, do_double_jump_shot, do_aerial_shot, &T, &ball_radius, &boost_accel, &gravity, &ball_slice, &me, safe_shots);

    return Py_BuildValue("{s:i,s:i,s:i}", "found", shot_viable.found, "fast", shot_viable.fast, "shot_type", shot_viable.shot_type);
}

static PyObject *method_find_landing_plane(PyObject *self, PyObject *args)
{
    Vector car_location, car_velocity;
    double gravity;

    if (!PyArg_ParseTuple(args, "(ddd)(ddd)d", &car_location.x, &car_location.y, &car_location.z, &car_velocity.x, &car_velocity.y, &car_velocity.z, &gravity))
        return NULL;

    const int landing_plane = find_landing_plane(car_location, car_velocity, gravity);

    return Py_BuildValue("i", landing_plane);
}

static PyObject *method_get_jump_time(PyObject *self, PyObject *args)
{
    double car_to_target_z, car_z_velocity, gravity_z;

    if (!PyArg_ParseTuple(args, "ddd", &car_to_target_z, &car_z_velocity, &gravity_z))
        return NULL;

    const JumpTime time = get_jump_time(car_to_target_z, car_z_velocity, gravity_z);

    return Py_BuildValue("d", time.time);
}

static PyObject *method_get_double_jump_time(PyObject *self, PyObject *args)
{
    double car_to_target_z, car_z_velocity, gravity_z;

    if (!PyArg_ParseTuple(args, "ddd", &car_to_target_z, &car_z_velocity, &gravity_z))
        return NULL;

    const JumpTime time = get_double_jump_time(car_to_target_z, car_z_velocity, gravity_z);

    return Py_BuildValue("d", time.time);
}

static PyObject *method_time_to_speed(PyObject *self, PyObject *args)
{
    double boost_accel, velocity, boost, target_speed;

    if (!PyArg_ParseTuple(args, "dddd", &boost_accel, &velocity, &boost, &target_speed))
        return NULL;

    const double time = time_to_speed(boost_accel, velocity, (unsigned char)boost, target_speed);

    return Py_BuildValue("d", time);
}

static PyMethodDef methods[] = {
    {"ground_shot_is_viable", method_ground_shot_is_viable, METH_VARARGS, "Check if a ground shot is viable"},
    {"jump_shot_is_viable", method_jump_shot_is_viable, METH_VARARGS, "Check if an jump_shot is viable"},
    {"double_jump_shot_is_viable", method_double_jump_shot_is_viable, METH_VARARGS, "Check if an double jump is viable"},
    {"aerial_shot_is_viable", method_aerial_shot_is_viable, METH_VARARGS, "Check if an aerial is viable"},
    {"parse_slice_for_shot_with_target", method_parse_slice_for_shot_with_target, METH_VARARGS, "Parse slice for one of the specified shots with a target"},
    {"parse_slice_for_shot", method_parse_slice_for_shot, METH_VARARGS, "Parse slice for one of the specified shots"},
    {"get_travel_distance", method_get_travel_distance, METH_VARARGS, "Get the travel distance"},
    {"find_landing_plane", method_find_landing_plane, METH_VARARGS, "Find the plane (side wall, back wall, ceiling, or floor) that the car will collide with first"},
    {"get_jump_time", method_get_jump_time, METH_VARARGS, "Get the time required to jump and reach at target height"},
    {"get_double_jump_time", method_get_jump_time, METH_VARARGS, "Get the time required to double jump and reach at target height"},
    {"time_to_speed", method_time_to_speed, METH_VARARGS, "Get the time required to reach some speed"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "virxrlcu",
    "C Library for VirxERLU",
    -1,
    methods};

PyMODINIT_FUNC PyInit_virxrlcu(void)
{
    return PyModule_Create(&module);
}