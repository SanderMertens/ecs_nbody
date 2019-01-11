# ecs_nbody
This is an example that shows how a brute force version of the nbody problem can be implemented in Reflecs ECS (https://github.com/SanderMertens/reflecs). The example demonstrates various reflecs features, like importing modules, on demand systems, multithreading and different ways to initialize components.

## Getting Started
Reflecs uses the bake build system (https://github.com/SanderMertens/bake). To install bake on Linux and MacOS, do:

```
git clone https://github.com/SanderMertens/bake
make -C bake/build-$(uname)
bake/bake setup
```

Before you can run the demo, make sure to have SDL2 installed.

On MacOS:

```
brew install sdl2
```

On Debian/Ubuntu:

```
sudo apt-get install sdl2
```

To install and run the demo, do:

```
bake clone SanderMertens/ecs_nbody
bake run ecs_nbody
```

## Overview
The n-body algorithm computes how n objects will move under the effect of a physical force, like gravity. The resulting acceleration of the forces acting upon the objects causes the objects to move. This project contains a simple implementation of the nbody algorithm, where for every object, the sum of the forces acting upon that object is computed. 

This approach has a time complexity of N^2, as calculating the sum involves iterating over all other objects, and adding up their masses divided by the square root of the distance. More efficient approaches to the nbody problem exist, though they are more complex and beyond the scope of this project.

The project instantiates a configurable number of randomly placed objects (`NBODIES`) of a randomized mass (`BASE_MASS`, `VAR_MASS`). In addition, it is possible to configure a center mass (`CENTRAL_MASS`) which typically is much larger than that of the other objects. During initialization, objects can optionally receive an initial velocity that brings it in orbit around the central mass (`INITIAL_C`).

## Implementation
The objects are instances of the `Body` family, which is composed out of the `EcsPosition2D`, `EcsVelocity2D`, `Mass`, `EcsCircle` and `EcsColor`components. All but the `Mass` component come from imported modules (`Transform`, `Physics`, `Geometry`, `Graphics`). In addition, objects specify the `Canvas` entity as a component, which binds the objects to the drawing surface. When the components from the `Body` family are added, the `Init` system is automatically invoked to initialize the values of the objects.

Once the canvas and entities are initialized, the application will set the target FPS and the number of threads that will be used. These are both optional features that an application can choose to enable.

The `ecs_progress` function runs the main loop of the application. In here, all `EcsOnFrame` systems are executed, the most notable one being the `Gravity` system. This system iterates over all the objects, computes the force for the object, and updates the velocity with the force. To compute the force, the `Gravity` system needs to iterate over all objects again. 

In Reflecs this can be efficiently achieved with an OnDemand system. On demand systems need to be invoked manually by the application, as opposed to being invoked automatically by `ecs_progress`. The advantage of on demand systems is that they are prematched with the entity tables, which makes invoking them a relatively cheap operation. As an additional bonus, on demand systems allow for the passing of parameters. In the project, the `Gravity` system invokes the on demand system `GravityComputeForce`, which stores the resulting force in the `force_vector` member of the passed in parameter. 

The difference between automatic invokcation and manual invocation can be found in the system declaration, where one specifiecs `EcsOnFrame` whereas the other specifies `EcsOnDemand`.

Another notable implementation detail is found in how the `Gravity` system invokes the `GravityComputeForce` system. To do so, it needs the handle to the system. To pass the handle of the `GravityComputeForce` system to the `Gravity` system, a special `HANDLE` argument is provided to the system signature.
