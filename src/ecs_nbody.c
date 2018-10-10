#include <reflecs/reflecs.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NBODIES (50000)
#define NTHREADS (6)

typedef struct Vector2D {
    float x;
    float y;
} Vector2D;

/* World context used to pass handle to gravity system around */
typedef struct Context {
    EcsHandle gravity;
} Context;

/* Components */
typedef Vector2D Position;
typedef Vector2D Velocity;
typedef float Mass;

/* Gravity system parameter */
typedef struct GravityParam {
    EcsHandle me;
    Position *position;
    Velocity force_vector;
} GravityParam;

static
float rnd(int max) {
    return ((float)rand() / (float)RAND_MAX) * max;
}

/** Initialize components with random values. */
static void Init(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        Position *position = ecs_column(rows, row, 0);
        Velocity *velocity = ecs_column(rows, row, 1);
        Mass *mass = ecs_column(rows, row, 2);

        position->x = rnd(20) - 10;
        position->y = rnd(20) - 10;
        velocity->x = rnd(1) - 0.5;
        velocity->y = rnd(1) - 0.5;
        *mass = rnd(10);
    }
}

/** On-demand system that computes force vector from a single entity. */
void GravityComputeForce(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        GravityParam *param = rows->param;
        EcsHandle entity = ecs_entity(row);

        if (entity != param->me) {
            Position *position = ecs_column(rows, row, 0);
            Mass *mass = ecs_column(rows, row, 1);

            float diff_x = param->position->x - position->x;
            float diff_y = param->position->y - position->y;
            float distance = diff_x * diff_x + diff_y * diff_y;
            float distance_sqr = sqrt(distance);
            float force = *mass / distance;
            param->force_vector.x = (diff_x / distance_sqr) * force;
            param->force_vector.y = (diff_y / distance_sqr) * force;
        }
    }
}

/** Periodic system that invokes Gravity to compute force vector per entity. */
void Gravity(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        EcsHandle entity = ecs_entity(row);
        Position *position = ecs_column(rows, row, 0);
        Velocity *velocity = ecs_column(rows, row, 1);
        Mass *mass = ecs_column(rows, row, 2);
        Context *ctx = ecs_get_context(rows->world);

        GravityParam param = {
            .me = entity,
            .position = position,
            .force_vector = {0, 0}
        };

        /* Invoke on-demand system */
        ecs_run_system(rows->world, ctx->gravity, &param);

        /* Add force to speed */
        velocity->x += param.force_vector.x / *mass;
        velocity->y += param.force_vector.y / *mass;

        /* Don't update position yet, we may still need it for other entites */
    }
}

/** Periodic system that progresses position using speed and time increment. */
void Move(EcsRows *rows)
{
    void *row;
    for (row = rows->first; row < rows->last; row = ecs_next(rows, row)) {
        Position *position = ecs_column(rows, row, 0);
        Velocity *velocity = ecs_column(rows, row, 1);
        Mass *mass = ecs_column(rows, row, 2);

        /* Update proportionally to the time passed since the last iteration */
        position->x -= velocity->x / *mass;
        position->y -= velocity->y / *mass;
    }
}

int main(int argc, char *argv[]) {
    EcsWorld *world = ecs_init();
    Context ctx;

    /* Register components */
    ECS_COMPONENT(world, Position);
    ECS_COMPONENT(world, Velocity);
    ECS_COMPONENT(world, Mass);

    /* Register systems */
    ECS_SYSTEM(world, Init, EcsOnInit, Position, Velocity, Mass);
    ECS_SYSTEM(world, Gravity, EcsPeriodic, Position, Velocity, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Velocity, Mass);
    ECS_SYSTEM(world, GravityComputeForce, EcsOnDemand, Position, Mass);

    /* Obtain handle to family */
    ECS_FAMILY(world, Body, Position, Velocity, Mass);

    /* Set world context */
    ctx.gravity = GravityComputeForce_h;
    ecs_set_context(world, &ctx);

    /* Preallocate memory */
    ecs_dim(world, NBODIES);
    ecs_dim_family(world, Body, NBODIES);

    /* Create a bunch of entities */
    int i;
    for (i = 0; i < NBODIES; i ++) {
        ecs_new(world, Body);
    }

    /* Use multiple threads for processing the data */
    ecs_set_threads(world, NTHREADS);

    /* Do a single iteration */
    ecs_progress(world);

    return ecs_fini(world);;
}
