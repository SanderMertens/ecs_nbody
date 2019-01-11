#include <include/ecs_nbody.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NBODIES (2500)          /* Number of entities */
#define NTHREADS (4)            /* Number of threads used to simulate bodies */
#define CENTRAL_MASS (12000.0)  /* Central mass */
#define INITIAL_C (12000.0)     /* Mass used in calculation of orbital speed */
#define BASE_MASS (0.1)         /* Base mass */
#define VAR_MASS (0.8)          /* Amount of randomness added to mass */
#define STICKY (10000.0)        /* Reduce acceleration in close encounters */
#define ZOOM (0.1)              /* Zoom (used for viewport) */
#define MAX_RADIUS (50)         /* Circle radius. Will be multiplied by ZOOM */
#define SPEED (2)               /* Speed of simulation */

/* Components */
typedef double Mass;

/* Compute color from speed */
static
EcsColor color_from_speed(float f) {
    f = f / 8 * sqrt(SPEED);
    if (f > 1.0) f = 1.0;

    float f_red = f - 0.2;
    if (f_red < 0) f_red = 0;
    f_red /= 0.8;

    float f_green = f - 0.7;
    if (f_green < 0) f_green = 0;
    f_green /= 0.3;

    return (EcsColor){f_red * 255, f_green * 255, f * 155 + 100, 255};
}

/** Initialize entities with random values. */
static void Init(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        EcsPosition2D *position = ecs_column(rows, row, 0);
        EcsVelocity2D *velocity = ecs_column(rows, row, 1);
        Mass *mass = ecs_column(rows, row, 2);
        EcsCircle *circle = ecs_column(rows, row, 3);

        position->x = rand() % 8000 - 4000;
        position->y = rand() % 200 - 100;
        *mass = BASE_MASS + ((float)rand() / (float)RAND_MAX) * VAR_MASS;

        /* Set speed so it stays in constant orbit (at least initially) */
        if (position->x || position->y) {
            double radius = ecs_vec2_magnitude(position);
            EcsVec2 vec_norm, vec_rot;
            ecs_vec2_div(position, radius, &vec_norm);
            ecs_vec2_perpendicular(&vec_norm, &vec_rot);
            double v = sqrt(INITIAL_C / radius / *mass / SPEED);

            velocity->x = vec_rot.x * v;
            velocity->y = vec_rot.y * v;
        }

        circle->radius = MAX_RADIUS * (*mass / (float)(BASE_MASS + VAR_MASS)) + 1;
    }
}

/* Parameter for GravityComputeForce system */
typedef struct GravityParam {
    EcsHandle me;
    EcsPosition2D *position;
    EcsVelocity2D force_vector;
} GravityParam;

/** On-demand system that computes attraction force from a single entity. */
void GravityComputeForce(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        GravityParam *param = rows->param; /* Parameter passed from Gravity */
        EcsHandle entity = ecs_entity(row);

        if (entity != param->me) {
            EcsPosition2D *position = ecs_column(rows, row, 0);
            Mass *mass = ecs_column(rows, row, 1);

            EcsVec2 diff;
            ecs_vec2_sub(param->position, position, &diff);
            double distance = ecs_vec2_dot(&diff, &diff);

            if (distance < STICKY) {
                distance = STICKY;
            }

            double distance_sqr = sqrt(distance);
            double force = *mass / distance;
            ecs_vec2_mult(&diff, force / distance_sqr, &diff);
            ecs_vec2_add(&param->force_vector, &diff, &param->force_vector);
        }
    }
}

/** Periodic system that computes gravitational force acting on entity */
void Gravity(EcsRows *rows)
{
    /* Get handle to GravityComputeForce system */
    EcsHandle GravityComputeForce_h = ecs_handle(rows, 3);

    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        EcsHandle entity = ecs_entity(row);
        EcsPosition2D *position = ecs_column(rows, row, 0);
        EcsVelocity2D *velocity = ecs_column(rows, row, 1);
        Mass *mass = ecs_column(rows, row, 2);

        /* Compute force vector from all other entities */
        GravityParam param = {
            .me = entity,
            .position = position,
            .force_vector = {0, 0}
        };

        ecs_run_system(rows->world, GravityComputeForce_h, 0, 0, &param);

        /* Add force to speed */
        velocity->x += param.force_vector.x / *mass;
        velocity->y += param.force_vector.y / *mass;
    }
}

/** Periodic system that moves entities */
void Move(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        EcsPosition2D *position = ecs_column(rows, row, 0);
        EcsVelocity2D *velocity = ecs_column(rows, row, 1);
        position->x -= SPEED * velocity->x;
        position->y -= SPEED * velocity->y;
    }
}

/** Periodic system that sets color based on speed */
void SetColor(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        EcsVelocity2D *velocity = ecs_column(rows, row, 0);
        EcsColor *color = ecs_column(rows, row, 1);
        *color = color_from_speed (ecs_vec2_magnitude(velocity));
    }
}

int main(int argc, char *argv[]) {
    EcsWorld *world = ecs_init();

    /* Import reflecs modules */
    ECS_IMPORT(world, EcsComponentsTransform, ECS_2D);
    ECS_IMPORT(world, EcsComponentsPhysics, ECS_2D);
    ECS_IMPORT(world, EcsComponentsGeometry, ECS_2D);
    ECS_IMPORT(world, EcsComponentsGraphics, ECS_2D);
    ECS_IMPORT(world, EcsSystemsSdl2, ECS_2D);

    /* Register Mass component (other components come from imported modules) */
    ECS_COMPONENT(world, Mass);

    /* Define components of the Body family */
    ECS_FAMILY(world, Body, EcsPosition2D, EcsVelocity2D, Mass, EcsCircle, EcsColor);

    /* Register systems */
    ECS_SYSTEM(world, Init, EcsOnAdd, EcsPosition2D, EcsVelocity2D, Mass, EcsCircle, EcsColor);
    ECS_SYSTEM(world, GravityComputeForce, EcsOnDemand, EcsPosition2D, Mass);
    ECS_SYSTEM(world, Gravity, EcsOnFrame, EcsPosition2D, EcsVelocity2D, Mass, HANDLE.GravityComputeForce);
    ECS_SYSTEM(world, Move, EcsOnFrame, EcsPosition2D, EcsVelocity2D);
    ECS_SYSTEM(world, SetColor, EcsOnFrame, EcsVelocity2D, EcsColor);

    /* Create the drawing canvas (SDL listens for this to create the window) */
    EcsHandle canvas = ecs_set(world, 0, EcsCanvas2D, {
        .window.width = 1024, .window.height = 768,
        .viewport.width = 1024 / ZOOM, .viewport.height = 768 / ZOOM
    });

    /* Create the entities for the canvas */
    int i;
    for (i = 0; i < NBODIES; i ++) {
        EcsHandle body = ecs_new(world, canvas);

        /* Add Body family, with Position, Velocity, Mass, Circle and Color */
        ecs_add(world, body, Body_h);
        ecs_commit(world, body);

        /* First entity is the central mass */
        if (!i) {
            ecs_set(world, body, EcsPosition2D, {0, 0});
            ecs_set(world, body, Mass, {CENTRAL_MASS});
            ecs_set(world, body, EcsVelocity2D, {0, 0});
        }
    }

    /* Don't run simulation faster than 60FPS */
    ecs_set_target_fps(world, 60);

    /* Use multiple threads to calculate the movement of the entities */
    ecs_set_threads(world, NTHREADS);

    /* Run until ecs_quit is called by SDL (when window is closed) */
    while ( ecs_progress(world, 0));

    /* Cleanup */
    return ecs_fini(world);
}
