#include <reflecs/reflecs.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NBODIES (26000)

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
static void Init(void *data[], EcsInfo *info)
{
    Position *position = data[0];
    Velocity *velocity = data[1];
    Mass *mass = data[2];

    position->x = rnd(20) - 10;
    position->y = rnd(20) - 10;
    velocity->x = rnd(1) - 0.5;
    velocity->y = rnd(1) - 0.5;
    *mass = rnd(10);
}

/** On-demand system that computes force vector from a single entity. */
void Gravity(void *data[], EcsInfo *info)
{
    GravityParam *param = info->param;

    if (info->entity != param->me) {
        Position *position = data[0];
        Mass *mass = data[2];
        float diff_x = param->position->x - position->x;
        float diff_y = param->position->y - position->y;
        float distance = diff_x * diff_x + diff_y * diff_y;
        float distance_sqr = sqrt(distance);
        float force = *mass / distance;
        param->force_vector.x = (diff_x / distance_sqr) * force;
        param->force_vector.y = (diff_y / distance_sqr) * force;
    }
}

/** Periodic system that invokes Gravity to compute force vector per entity. */
void Visit(void *data[], EcsInfo *info)
{
    Context *ctx = ecs_get_context(info->world);
    Velocity *velocity = data[1];
    Mass *mass = data[2];

    GravityParam param = {
        .me = info->entity,
        .position = data[0],
        .force_vector = {0, 0}
    };

    /* Invoke on-demand system */
    ecs_run_system(info->world, ctx->gravity, &param);

    /* Add force to speed */
    velocity->x += param.force_vector.x / *mass;
    velocity->y += param.force_vector.y / *mass;

    /* Don't update position yet, we may still need it for other entites */
}

/** Periodic system that progresses position using speed and time increment. */
void Move(void *data[], EcsInfo *info)
{
    Position *position = data[0];
    Velocity *velocity = data[1];

    /* Update proportionally to the time passed since the last iteration */
    position->x += velocity->x * info->delta_time;
    position->y += velocity->y * info->delta_time;
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
    ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Velocity, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Velocity);
    ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Velocity, Mass);

    /* Set world context. This lets us use the Gravity system from Visit */
    ctx.gravity = Gravity_h;
    ecs_set_context(world, &ctx);

    /* Create a bunch of entities */
    int i;
    for (i = 0; i < NBODIES; i ++) {
        EcsHandle e = ecs_new(world);
        ecs_stage(world, e, Position_h);
        ecs_stage(world, e, Velocity_h);
        ecs_stage(world, e, Mass_h);
        ecs_commit(world, e);
    }

    /* Use multiple threads for processing the data */
    ecs_set_threads(world, 12);

    /* Do a single iteration */
    ecs_progress(world);

    return ecs_fini(world);;
}
