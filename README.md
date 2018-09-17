# ecs_nbody
This example shows how to implement an n-body algorithm (simple Newtonian gravity) with reflecs ECS. The challenging part of efficiently implementing n-body is that each entity needs to evaluate every other entity to calculate the attraction force. In reflecs, this can be elegantly solved with `OnDemand` systems. This example processes over 6000 entities per second on a single thread, on a 2018 Macbook Pro. Per iteration, 36.012.000 system invocations are made.

A naive implementation could build a `Gravity` system that walks over all entities with `Position`, `Mass` and `Speed` compoments. Inside the logic for that system (that is ran for every entity) there would be code that queries all entities again with `Position`, `Mass` and `Speed`, and then walks over them (skipping itself) to compute the resulting force. That is clearly not efficient: this set of entities is the same for every entity, and is actually the same set of entities that the `Gravity` system is walking over.

## OnDemand systems
To address n-body elegantly, we'd ideally reuse how reflecs matches entities against systems. Instead of matching individual entities, reflecs matches systems with groups of entities that share the same components ("tables"). Reflecs lets us do just that with `OnDemand` systems. `OnDemand` systems are matched to entities just like normal systems, except that they are invoked manually with the `ecs_run_system` function, instead of for every `ecs_progress`, as is the case with `EcsPeriodic`.

Another nice feature of OnDemand systems is that we can pass information to the system that we store on stack. This keeps all data localized, which makes it likely that the code runs efficiently on multiple threads.

With these building blocks, we define the following systems:

```c
ECS_SYSTEM(world, Init, EcsOnInit, Position, Speed, Mass);
ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Speed, Mass);
ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Speed, Mass);
ECS_SYSTEM(world, Move, EcsPeriodic, Position, Speed);
```
The `Init` system initializes the values for all components. The `Visit` system visits all entities ever time `ecs_progress` is called. The `Gravity` system calculates the attraction force of all entities it visits. Because it is an `OnDemand` system it needs to be manually invoked with `ecs_run_system` This is done by the `Visit` system, which achieves the N * N behavior we need. The result of the `Gravity` system is added to the `Speed` component. Finally, the `Move` system adds the `Speed` component to the `Position` component. In code:

```c
void Gravity(void *data[], EcsInfo *info) {
    if (info->entity != param->me) {
        /* ... Compute force ... */
    }
}

void Visit(void *data[], EcsInfo *info) {
    /* ... plumbing to get context & setup parameter for Gravity ... */
    
    ecs_run_system(info->world, ctx->gravity, &param /* in: current entity, out: resulting force */);
    
    /* ... Add force to speed ... */
}

void Move(void *data[], EcsInfo *info) {
    /* ... Add Speed to Position ... */
}

int main(int argc, char *argv[]) {
  
    /* plumbing to create world, register components etc. */
    
    ECS_SYSTEM(world, Visit, EcsPeriodic, Position, Speed, Mass);
    ECS_SYSTEM(world, Move, EcsPeriodic, Position, Speed);
    ECS_SYSTEM(world, Gravity, EcsOnDemand, Position, Speed, Mass);
    
    /* Set Gravity handle in world context, so it can be accessed from Visit system */
    Context ctx = { .gravity = Gravity_h }; 
    ecs_set_context(world, &ctx);

    while (true) {
        ecs_progress(world); /* Runs Visit, followed by Move */
    }
}
```
The reason for a separate `Move` system is that we don't actually want to change the `Position` of an entity until we've finished calculating the attraction force for all entities, as this would produce inaccurate results. Reflecs processes systems sequentially, thus it is ensured that `Move` isn't ran until `Visit` has finished.

