# ecs_nbody
This example shows how to implement n-body (using simple Newtonian gravity) with reflecs ECS. The challenging part of efficiently implementing n-body is that each entity needs to evaluate every other entity to calculate the attraction force. This example shows how this can be achieved in ECS, while still being quite efficient.

## Performance
Some rough benchmarks have been conducted with this code on a 2018 2.6Ghz i7 Macbook Pro. The benchmarks are conducted on a release build (-O3) on MacOS 10.13.6. The time is obtained using the `time` command.

The throughput is computed by dividing the total number of system calls by the time to completion. The number of system calls is 2N + N ^ 2, where N is the number of entities. 2N represents the Init and Visit systems. N ^ 2 represents calling the Gravity system on all entities (N) for all entities ( * N).

Entities | Threads | Time to completion | Throughput
---------|---------|--------------------|--------------
5000     | 1       | 0.263s             | 95.057.034
5000     | 6       | 0.062s             | 403.225.806
5000     | 12      | 0.051s             | 490.196.078
10000    | 1       | 0.995s             | 100.502.512
10000    | 6       | 0.195s             | 512.820.512
10000    | 12      | 0.167s             | 598.802.395
27000    | 1       | 7.114s             | 102.473.994
27000    | 6       | 1.27s              | 574.015.748
27000    | 12      | 1.055s             | 690.995.260

## OnDemand systems
To address n-body we have to iterate over all entities, and then for every entity, iterate over all entities again. We thus need to obtain a list of entities to walk over that have the `Position` and `Mass` components. Ideally we reuse how reflecs matches entities agaist components, since this is much more efficient than linearly iterating over all entities.

We can achieve this by using `OnDemand` systems. `OnDemand` systems are efficiently matched to entities just like normal systems, with the one difference that they are only invoked manually with the `ecs_run_system` function, instead of for every `ecs_progress`, as is the case with `EcsPeriodic`.

Additionally, `OnDemand` systems let us pass information to the system that, rather than being another component, is stored on stack. This keeps all data localized, which makes the code efficient when running on multiple threads.

With these building blocks, we define the following systems:

```c
ECS_SYSTEM(world, Init, EcsOnInit, Position, Velocity, Mass);
ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Velocity, Mass);
ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Mass);
ECS_SYSTEM(world, Move, EcsPeriodic, Position, Velocity);
```
The `Init` system initializes the values for all components. The `Visit` system visits all entities ever time `ecs_progress` is called. The `Gravity` system calculates the attraction force of all entities it visits. Because it is an `OnDemand` system it needs to be manually invoked with `ecs_run_system` This is done by the `Visit` system, which achieves the N * N behavior we need. The result of the `Gravity` system is added to the `Velocity` component. Finally, the `Move` system adds the `Velocity` component to the `Position` component. In code:

```c
void Gravity(void *data[], EcsInfo *info) {
    if (info->entity != param->me) {
        /* ... Compute force ... */
    }
}

void Visit(void *data[], EcsInfo *info) {
    /* ... plumbing to get context & setup parameter for Gravity ... */

    ecs_run_system(info->world, ctx->gravity, &param /* in: current entity, out: resulting force */);

    /* ... Add resulting force to speed ... */
}

void Move(void *data[], EcsInfo *info) {
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
        ecs_progress(world); /* Runs Visit, followed by Move */
    }
}
```
The reason for a separate `Move` system is that we don't actually want to change the `Position` of an entity until we've finished calculating the attraction force for all entities, as this would produce inaccurate results. Reflecs processes systems sequentially, thus it is ensured that `Move` isn't ran until `Visit` has finished.
