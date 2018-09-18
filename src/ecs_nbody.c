#include <include/ecs_nbody.h>
#include <reflecs/reflecs.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define NBODIES (24000)

/* Components */

typedef struct Position {
    float x;
    float y;
} Position;

typedef struct Speed {
    float x;
    float y;
} Speed;

typedef float Mass;

/* World context */

typedef struct Context {
    EcsHandle gravity;
} Context;

/* Gravity system parameter */

typedef struct GravityParam {
    EcsHandle me;
    Position *position;
    Speed force_vector;
} GravityParam;

static
float rnd(int max) {
    return ((float)rand() / (float)RAND_MAX) * max;
}

void Init(void *data[], EcsInfo *info)
{
    Position *position = data[0];
    Speed *speed = data[1];
    Mass *mass = data[2];

    position->x = rnd(20) - 10;
    position->y = rnd(20) - 10;
    speed->x = rnd(1) - 0.5;
    speed->y = rnd(1) - 0.5;
    *mass = rnd(10);
}

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

void Visit(void *data[], EcsInfo *info)
{
    Context *ctx = ecs_get_context(info->world);
    Speed *speed = data[1];
    Mass *mass = data[2];

    GravityParam param = {
        .me = info->entity,
        .position = data[0],
        .force_vector = {0, 0}
    };

    ecs_run_system(info->world, ctx->gravity, &param);

    speed->x += param.force_vector.x / *mass;
    speed->y += param.force_vector.y / *mass;
}

void Move(void *data[], EcsInfo *info)
{
    Position *position = data[0];
    Speed *speed = data[1];
    position->x += speed->x;
    position->y += speed->y;
}

int main(int argc, char *argv[]) {
    EcsWorld *world = ecs_init();
    Context ctx;

    ECS_COMPONENT(world, Position);
    ECS_COMPONENT(world, Speed);
    ECS_COMPONENT(world, Mass);

    ECS_SYSTEM(world, Init, EcsOnInit, Position, Speed, Mass);
    ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Speed, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Speed);
    ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Speed, Mass);

    ctx.gravity = Gravity_h;
    ecs_set_context(world, &ctx);

    int i;
    for (i = 0; i < NBODIES; i ++) {
        EcsHandle e = ecs_new(world);
        ecs_stage(world, e, Position_h);
        ecs_stage(world, e, Speed_h);
        ecs_stage(world, e, Mass_h);
        ecs_commit(world, e);
    }

    ecs_set_threads(world, 10);

    ecs_progress(world);

    ecs_fini(world);

    return 0;
}
