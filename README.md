# ecs_nbody
This example shows how to implement an n-body algorithm (simple Newtonian gravity) with reflecs ECS. The challenging part of efficiently implementing n-body is that each entity needs to evaluate every other entity to calculate the attraction force. In reflecs, this can be elegantly solved with `OnDemand` systems. This example processes over 6000 entities per second on a single thread, on a 2018 Macbook Pro. Per iteration, 36.012.000 system invocations are made.

A naive implementation could build a `Gravity` system that walks over all entities with `Position`, `Mass` and `Speed` compoments. Inside the logic for that system (that is ran for every entity) there would be code that queries all entities again with `Position`, `Mass` and `Speed`, and then walks over them (skipping itself) to compute the resulting force. That is clearly not efficient: this set of entities is going to be the same for every entity, and is in fact the same set of entities that `Gravity` is walking over.

## OnDemand systems
To address this elegantly, applications can create an `OnDemand` system which, just like any other system in reflecs, is efficiently matched to the internal tables that contain matching entities. The biggest difference with a normal system is, that an `OnDemand` system is not invoked when calling `ecs_progress`, but has to be called manually. Another difference is that it is possible to pass custom data when invoking an `OnDemand` system, which allows storing the computed result on stack, rather than in a separate component.

With these building blocks, we can now build an elegant and efficient implementation of the n-body algorithm. First, we define our components:

```c
ECS_COMPONENT(world, Position);
ECS_COMPONENT(world, Speed);
ECS_COMPONENT(world, Mass);
```
The purpose of these should speak for itself. Now we define our systems:

```c
ECS_SYSTEM(world, Init, EcsOnInit, Position, Speed, Mass);
ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Speed, Mass);
ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Speed, Mass);
ECS_SYSTEM(world, Move, EcsPeriodic, Position, Speed);
```
This requires a bit more explanation. The first system `Init` is simple: it initializes the values for the `Position`, `Speed` and `Mass` components. The `Visit` system visits all the entities with `Position`, `Speed` and `Mass` when `ecs_progress` is called, since it is an `EcsPeriodic` system. The `Gravity` system calculates the attraction force of all entities it visits. It, however, is an `OnDemand` system and is called for every entity visited by `Visit`. The `Visit` system takes the result of `Gravity`, and adds it to the `Speed` vector. Finally, the `Move` system adds the speed to the position data. This code shows the high-level design of the Gravity, Visit and Move systems:

```c
void Gravity(void *data[], EcsInfo *info) {
    if (info->entity != param->me) {
        /* ... Compute force ... */
    }
}

void Visit(void *data[], EcsInfo *info) {
    /* ... Invoke Gravity system manually, pass information about current entity ... */
    ecs_run_system(info->world, ctx->gravity, &param /* in: current entity, out: resulting force */);
    
    /* ... Add force to speed ... */
}

void Move(void *data[], EcsInfo *info) {
    /* ... Add speed to position ... */
}

int main(int argc, char *argv[]) {
  
    /* ... */
    
    ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Speed, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Speed);
    ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Speed, Mass);

    while (true) {
        ecs_progress(world); /* Runs Visit, followed by Move */
    }
}
```
The reason for a separate `Move` system is that we don't actually want to change the `Position` of an entity until we've finished calculating the attraction force for all entities, as this would produce inaccurate results. Reflecs processes systems sequentially, thus it is ensured that `Move` isn't ran until `Visit` has finished.

## World Context
To pass around data that is "global" to a world, an application can use the `ecs_set_context` and `ecs_get_context` functions. This is typically used to store entity handles, but can also be used to store global configuration parameters. In this example, the context is used to pass the handle to the `Gravity` system to the `Visit` system. This code in the main function sets the context:

```c
Context ctx;
...
ctx.gravity = Gravity_h; /* Gravity_h is the handle to the gravity system, auto-declared by `ECS_SYSTEM` */
ecs_set_context(world, &ctx);
```

This data is then later retrieved and used by the `Visit` system, like this:
```c
Context *ctx = ecs_get_context(info->world);
...
ecs_run_system(info->world, ctx->gravity, &param);
```
