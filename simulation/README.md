# Skateboard Simulation
Skateboard Simulation for Robot Design Studio 2020

## Overview
Application entry point is `main.m`, which calls (through wrapper functions) dynamics-related functions that are automatically generated by running `derive_equations.m`.
That means you have to run `derive_equations.m` before you run `main.m` for the first time.
The workflow is depicted below.

![architecture](https://github.com/michaeldoody/skateboard-squad/blob/master/simulation/graphics/svg/template_architecture.svg)

`derive_equations.m` uses symbolic computation to generate the state-space dynamics of the skateboard robot, which are then exported as MATLAB functions (e.g., `autogen_drift_vector_field.m` and `autogen_control_vector_field.m`).

## Autogenerated functions and wrappers
`derive_equations.m` uses `matlabFunction(variable,'File','filename')` extensively, to convert symbolic expressions to MATLAB functions that you can evaluate numerically.
However, these functions often require many input arguments.
For example, the autogenerated function that computes the conservative forces looks like this:

```Matlab
function G_q = autogen_conservative_forces(boardMass,boardTheta,boardHeight,bottomLinkRCoM,bottomLinkMass,bottomLinkTheta,bottomLinkHeight,g,topLinkRCoM,topLinkMass,topLinkTheta)
...
```
You don't want to have to supply all those input arguments individually!
Instead, recognize that they can be grouped into two categories:
* robot state `x = [boardX; boardY; boardTheta; bottomLinkTheta; topLinkTheta; boardDX; boardDY; boardDTheta; bottomLinkDTheta; topLinkDTheta]`
* parameters (literally everything else in the example above)

### MATLAB structs and parsers
In addition to physical parameters (mass, length, etc) that anchor the dynamics in reality, there are many other parameters relevant to simulation, such as timestep size, appearance of the skateboard robot, etc.
All the parameters are grouped into a struct called `params`, which is generated by calling `init_params.m`.
This struct gets passed around throughout the simulation, and saves you from having to worry about the order in which you supply input arguments.

Another way to avoid input argument order-dependency (a trademark of brittle code) is to use Name-Value pairs, which you have probably experienced if you've used MATLAB before.
Name-Value pairs rely on an "input parser"; check out `plot_robot.m` and `animate_robot.m` to see how this works.