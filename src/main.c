#include <ecs_nbody.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NBODIES (3000)          /* Number of entities */
#define NTHREADS (12)            /* Number of threads used to simulate bodies */
#define CENTRAL_MASS (12000.0)  /* Central mass */
#define INITIAL_C (12000.0)     /* Mass used in calculation of orbital speed */
#define BASE_MASS (0.1)         /* Base mass */
#define VAR_MASS (0.8)          /* Amount of randomness added to mass */
#define STICKY (10000.0)        /* Reduce acceleration in close encounters */
#define ZOOM (0.1)              /* Zoom (used for viewport) */
#define MAX_RADIUS (70)         /* Circle radius. Will be multiplied by ZOOM */
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
static void Init(ecs_rows_t *rows)
{
    EcsPosition2D *position = ecs_column(rows, EcsPosition2D, 1);
    EcsVelocity2D *velocity = ecs_column(rows, EcsVelocity2D, 2);
    Mass *mass = ecs_column(rows, Mass, 3);
    EcsCircle *circle = ecs_column(rows, EcsCircle, 4);

    for (int i = 0; i < rows->count; i ++) {
        position[i].x = rand() % 8000 - 4000;
        position[i].y = rand() % 200 - 100;
        mass[i] = BASE_MASS + ((float)rand() / (float)RAND_MAX) * VAR_MASS;

        /* Set speed so it stays in constant orbit (at least initially) */
        if (position[i].x || position[i].y) {
            double radius = ecs_vec2_magnitude(&position[i]);
            EcsVec2 vec_norm, vec_rot;
            ecs_vec2_div(&position[i], radius, &vec_norm);
            ecs_vec2_perpendicular(&vec_norm, &vec_rot);

            double v = sqrt(INITIAL_C / radius / *mass / SPEED);

            velocity[i].x = vec_rot.x * v;
            velocity[i].y = vec_rot.y * v;
        }

        circle[i].radius = MAX_RADIUS * (mass[i] / (float)(BASE_MASS + VAR_MASS)) + 1;
    }
}

/* Parameter for GravityComputeForce system */
typedef struct GravityParam {
    ecs_entity_t me;
    EcsPosition2D *position;
    EcsVelocity2D force_vector;
} GravityParam;

/** On-demand system that computes attraction force from a single entity. */
void GravityComputeForce(ecs_rows_t *rows)
{
    GravityParam *param = rows->param; /* Parameter passed from Gravity */
    ecs_entity_t me = param->me;

    EcsPosition2D *position = ecs_column(rows, EcsPosition2D, 1);
    Mass *mass = ecs_column(rows, Mass, 2);
    
    for (int i = 0; i < rows->count; i ++) {
        ecs_entity_t current = rows->entities[i];

        if (current != me) {
            EcsVec2 diff;
            ecs_vec2_sub(param->position, &position[i], &diff);
            double distance = ecs_vec2_dot(&diff, &diff);

            if (distance < STICKY) {
                distance = STICKY;
            }

            double distance_sqr = sqrt(distance);
            double force = mass[i] / distance;

            /* Multiply direction vector by force, divide by distance (to normalize) */
            ecs_vec2_mult(&diff, force / distance_sqr, &diff);
            ecs_vec2_add(&param->force_vector, &diff, &param->force_vector);
        }
    }
}

/** Periodic system that computes gravitational force acting on entity */
void Gravity(ecs_rows_t *rows)
{
    /* Get handle to GravityComputeForce system */
    EcsPosition2D *position = ecs_column(rows, EcsPosition2D, 1);
    EcsVelocity2D *velocity = ecs_column(rows, EcsVelocity2D, 2);
    Mass *mass = ecs_column(rows, Mass, 3);
    ecs_entity_t EGravityComputeForce = ecs_column_entity(rows, 4);

    for (int i = 0; i < rows->count; i ++) {
        /* Compute force vector from all other entities */
        GravityParam param = {
            .me = rows->entities[i],
            .position = &position[i],
            .force_vector = {0, 0}
        };

        ecs_run(rows->world, EGravityComputeForce, 0, &param);

        /* Add force to speed */
        velocity[i].x += param.force_vector.x / mass[i];
        velocity[i].y += param.force_vector.y / mass[i];
    }
}

/** Periodic system that moves entities */
void Move(ecs_rows_t *rows)
{
    EcsPosition2D *position = ecs_column(rows, EcsPosition2D, 1);
    EcsVelocity2D *velocity = ecs_column(rows, EcsVelocity2D, 2);

    for (int i = 0; i < rows->count; i ++) {
        position[i].x -= SPEED * velocity[i].x;
        position[i].y -= SPEED * velocity[i].y;
    }
}

/** Periodic system that sets color based on speed */
void SetColor(ecs_rows_t *rows)
{
    EcsVelocity2D *velocity = ecs_column(rows, EcsVelocity2D, 1);
    EcsColor *color = ecs_column(rows, EcsColor, 2);
    
    for (int i = 0; i < rows->count; i ++) {
        color[i] = color_from_speed (ecs_vec2_magnitude(&velocity[i]));
    }
}

int main(int argc, char *argv[]) {
    /* Initialize the world object */
    ecs_world_t *world = ecs_init_w_args(argc, argv);


    /* -- Import modules -- */

    ECS_IMPORT(world, FlecsComponentsTransform, ECS_2D);
    ECS_IMPORT(world, FlecsComponentsPhysics, ECS_2D);
    ECS_IMPORT(world, FlecsComponentsGeometry, ECS_2D);
    ECS_IMPORT(world, FlecsComponentsGraphics, ECS_2D);
    ECS_IMPORT(world, FlecsSystemsSdl2, ECS_2D);


    /* -- Components -- */

    /* Register Mass component (other components come from imported modules) */
    ECS_COMPONENT(world, Mass);

    /* Define components of the Body family */
    ECS_TYPE(world, Body, EcsPosition2D, EcsVelocity2D, Mass, EcsCircle, EcsColor);


    /* -- Systems -- */

    /* System that initializes components associated by a body */
    ECS_SYSTEM(world, Init, EcsOnAdd, EcsPosition2D, EcsVelocity2D, Mass, EcsCircle, EcsColor);

    /* System that computes the force for a single entity */
    ECS_SYSTEM(world, GravityComputeForce, EcsManual, EcsPosition2D, Mass);

    /* System that iterates over all entities, and adds force to the velocity */
    ECS_SYSTEM(world, Gravity, EcsOnUpdate, EcsPosition2D, EcsVelocity2D, Mass, .GravityComputeForce);

    /* System that updates the position of the entities */
    ECS_SYSTEM(world, Move, EcsOnUpdate, EcsPosition2D, EcsVelocity2D);

    /* System that computes the color of an entity based on the velocity */
    ECS_SYSTEM(world, SetColor, EcsOnUpdate, EcsVelocity2D, EcsColor);


    /* -- Entity creation -- */

    /* Create the drawing canvas (SDL listens for this to create the window) */
    ecs_set(world, 0, EcsCanvas2D, {
        .window = {.width = 1024, .height = 768},
        .viewport = {.width = 1024 / ZOOM, .height = 768 / ZOOM}
    });

    /* Create NBODIES entities, return the first one, make that central mass */
    ecs_entity_t central_mass = ecs_new_w_count(world, Body, NBODIES);
    ecs_set(world, central_mass, EcsPosition2D, {0, 0});
    ecs_set(world, central_mass, Mass, {CENTRAL_MASS});
    ecs_set(world, central_mass, EcsVelocity2D, {0, 0});


    /* -- Configuration -- */

    /* Don't run simulation faster than 60FPS */
    ecs_set_target_fps(world, 60);
    ecs_set_threads(world, NTHREADS);

    /* Run until ecs_quit is called by SDL (when window is closed) */
    while ( ecs_progress(world, 0)) {  };

    /* Cleanup */
    return ecs_fini(world);
}
