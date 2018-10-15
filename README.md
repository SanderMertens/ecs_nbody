# ecs_nbody
This example shows how to implement n-body (using simple Newtonian gravity) with reflecs ECS. The challenging part of elegantly implementing n-body in ECS is that each entity needs to evaluate every other entity to calculate the attraction force. This example demonstrates an efficient technique for achieving this, while also showing how to easily parallelize the load using the reflecs job scheduler.

## Performance
Some rough benchmarks have been conducted with this code on a 2018 2.6Ghz i7 Macbook Pro. The benchmarks are conducted on a release build (-O3) on MacOS 10.13.6. The time is obtained using the `time` command.

The throughput is computed by dividing the number of operations by the time to completion. The number of system calls is 2N + N ^ 2, where N is the number of entities. 2N represents the Init and Visit systems. N ^ 2 represents calling the Gravity system on all entities (N) for all entities ( * N).

Entities | Threads | Time to completion | FPS   | Operations/sec
---------|---------|--------------------|-------|-----------
5000     | 1       | 0.087              | 11.49 | 287.356.321
5000     | 6       | 0.024s             | 41.57 | 1.041.666.666
10000    | 1       | 0.315s             | 3.17  | 317.460.317
10000    | 6       | 0.067              | 14.93 | 1.492.537.313
50000    | 1       | 7.435s             | 0.13  | 336.247.478
50000    | 6       | 1.32s              | 0.76  | 1.893.939.393

For comparison, artemis-odb (reportedly one of the fastest implementations of Artemis ECS) reports a throughput of around 51.000.000 operations per second on their GitHub page (https://github.com/junkdog/artemis-odb).

## OnDemand systems
To address n-body we have to iterate over all entities, and then for every entity, iterate over all entities again. We thus need to obtain a list of entities to walk over that have the `Position` and `Mass` components. Ideally we reuse how reflecs matches entities agaist components, since this is much more efficient than linearly iterating over all entities.

We can achieve this by using `OnDemand` systems. `OnDemand` systems are efficiently matched to entities just like normal systems, with the one difference that they are only invoked manually with the `ecs_run_system` function, instead of for every `ecs_progress`, as is the case with `EcsPeriodic`.

Additionally, `OnDemand` systems let us pass information to the system that, rather than being another component, is stored on stack. This keeps all data localized, which makes the code efficient when running on multiple threads.

With these building blocks, we define the following systems:

```c
ECS_SYSTEM(world, Init, EcsOnInit, Position, Velocity, Mass);
ECS_SYSTEM(world, Gravity, EcsPeriodic, Position, Velocity, Mass);
ECS_SYSTEM(world, GravityComputeForce, EcsOnDemand, Position, Mass);
ECS_SYSTEM(world, Move, EcsPeriodic, Position, Velocity);
```
The `Init` system initializes the values for all components. The `Gravity` system visits all entities ever time `ecs_progress` is called. The `GravityComputeForce` system calculates the attraction force of all entities it visits. Because it is an `OnDemand` system it needs to be manually invoked with `ecs_run_system` This is done by the `Visit` system, which achieves the N * N behavior we need. The result of the `GravityComputeForce` system is added to the `Velocity` component. Finally, the `Move` system adds the `Velocity` component to the `Position` component. In code:

```c
void GravityComputeForce(EcsRows *rows) {
    /* ... Compute force ... */
}

void Gravity(EcsRows *rows) {
    /* ... plumbing to get context & setup parameter for Gravity ... */

    ecs_run_system(info->world, ctx->gravity, &param /* in: current entity, out: resulting force */);

    /* ... Add resulting force to speed ... */
}

void Move(EcsRows *rows) {
    /* ... Add Velocity to Position ... */
}

int main(int argc, char *argv[]) {

    /* plumbing to create world, register components etc. */

    ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Velocity, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Velocity);
    ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Mass);

    /* Set Gravity handle in world context, so it can be accessed from Visit system */
    Context ctx = { .gravity = Gravity_h };
    ecs_set_context(world, &ctx);

    while (true) {
        ecs_progress(world); /* Runs Gravity, followed by Move */
    }
}
```
The reason for a separate `Move` system is that we don't actually want to change the `Position` of an entity until we've finished calculating the attraction force for all entities, as this would produce inaccurate results. Reflecs processes systems sequentially, thus it is ensured that `Move` isn't ran until `Gravity` has finished.
