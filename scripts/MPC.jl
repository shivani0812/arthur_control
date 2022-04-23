import Pkg; Pkg.activate(@__DIR__); 
Pkg.instantiate()

using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using Altro
using TrajectoryOptimization
using Parameters
using Rotations
using LinearAlgebra
using ForwardDiff
# using mujoco_sim

# activate_mujoco(joinpath(@__DIR__, "../mjkey.txt"))

# export
#     Arthur,
#     rk4

# export
#     MPC_Params,
#     ArthurProblem,
#     ArthurHorizonProblem
    # mpc_update,
    # update_guess_trajectory!,
    # update_trajectory_fill!,
    # update_constraints!

include("Arthur.jl")
include("MPCUtil.jl")