# import Pkg; Pkg.activate(@__DIR__); 
# Pkg.instantiate()
using RigidBodyDynamics
using StaticArrays
using RobotDynamics
using BenchmarkTools

# Defining Arthur model using RigidBodyDynamics
struct Arthur{C} <: AbstractModel
    mechanism::Mechanism{Float64}
    statecache::C
    dyncache::DynamicsResultCache{Float64}
    function Arthur(mechanism::Mechanism)
        statecache = StateCache(mechanism)
        rescache = DynamicsResultCache(mechanism)
        new{typeof(statecache)}(mechanism, statecache, rescache)
    end
end

#TODO: Change Path
Arthur(; mechanism=RigidBodyDynamics.URDF.parse_urdf(joinpath(@__DIR__,"../../../kortex_description/arms/gen3/7dof/urdf/Gen3_URDF_v13.urdf"), remove_fixed_tree_joints = false)) = Arthur(mechanism)

# State, s, is [q q̇ x ẋ F]
# x will be input from the camera
# q, q̇, ẋ will be taken or derived from the arm
# F will be input from the F/T Sensor
# Input, u, is Torque (τ)
function RobotDynamics.dynamics(model::Arthur, x::AbstractVector{T1}, u::AbstractVector{T2}) where {T1,T2} 
    # Create a state of the mechanism model and a result struct for the dynamics
    T = promote_type(T1,T2)
    state = model.statecache[T]
    res = model.dyncache[T]
    
    # Get states and constants of system not dependent on model state
    # num_q = RigidBodyDynamics.num_positions(model.mechanism)
    # q = x[SA[1, 2, 3, 4, 5, 6, 7]]
    # q̇ = x[SA[8, 9, 10, 11, 12, 13, 14]]
    # p = x[2*num_q + 1:2*num_q + 6]
    # ṗ = x[2*num_q + 7:2*num_q + 12]
    # F = x[2*num_q + 13:2*num_q + 18]
    # Be = zeros(T, 6, 6)
    
    # if (norm(ṗ) > 1e-5)
    #     for k = 1:6
    #         Be[k,k] = norm(F) / norm(ṗ)
    #     end
    # end

    # Set mechanism state to current state
    # copyto!(parent(state.q), 1, x, 1, num_positions(state))
    # copyto!(parent(state.v), 1, x, num_positions(state) + 1, num_velocities(state))
    # setdirty!(state)
    copyto!(state, x[SA[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]])
    
    # w = Wrench{T}(default_frame(bodies(model.mechanism)[end-1]), F[4:6], F[1:3])
    # wrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, root_frame(model.mechanism)) for b in bodies(model.mechanism))
    # wrenches[bodies(model.mechanism)[end-1].id] = transform(w, transform_to_root(mechanismState, bodies(model.mechanism)[end-1]))    dynamics!(dynamicsResult, mechanismState, u, wrenches)
    # dynamics!(res, state, u, wrenches)
    dynamics!(res, state, u)

    # p̈ = [res.accelerations[bodies(model.mechanism)[end].id].linear; res.accelerations[bodies(model.mechanism)[end].id].angular]
    # Ḟ = Be*p̈
    # return SVector{32}([q̇; q̈; ṗ; p̈; Ḟ])
    # return SVector{14}([x[SA[8, 9, 10, 11, 12, 13, 14]]; res.v̇])
    return [x[SA[8, 9, 10, 11, 12, 13, 14]]; res.v̇]
end

RobotDynamics.state_dim(::Arthur) = 14
RobotDynamics.control_dim(::Arthur) = 7

# cd(joinpath(@__DIR__,"../../../kortex_description/arms/gen3/7dof/urdf/GEN3_URDF_V12 copy.urdf"))
# readdir(joinpath(@__DIR__,"../../../kortex_description/arms/gen3/7dof/urdf/"))
# Arthur()

"""
Integrates the dynamics ODE 1 dt forward, x_{k+1} = rk4(x_k,u_k,dt).

returns x_{k+1}
"""
function rk4(model::Arthur,x::AbstractVector,u::AbstractVector,dt::Float64)
    # rk4 for integration
    k1 = dt*dynamics(model,x,u)
    k2 = dt*dynamics(model,x + k1/2,u)
    k3 = dt*dynamics(model,x + k2/2,u)
    k4 = dt*dynamics(model,x + k3,u)
    return x + (1/6)*(k1 + 2*k2 + 2*k3 + k4)
end

# model = Arthur()
# n,m = size(model)
# x = rand(n)
# u = rand(m)
# T1 = Float64
# T2 = Float64
# T = promote_type(T1,T2)
# state = model.statecache[T]
# res = model.dyncache[T]
# # copyto!(state, x)
# copyto!(parent(state.q), 1, x, 1, num_positions(state))
# copyto!(parent(state.v), 1, x, num_positions(state) + 1, num_velocities(state))
# @btime begin
#     setdirty!(state)
#     dynamics_bias!($res, $state)
#     mass_matrix!($res, $state)
#     # [x[SA[8, 9, 10, 11, 12, 13, 14]]; res.v̇]
# end
# @btime dynamics($model, $x, $u);
# suite = BenchmarkGroup()
# suite["test"] = @benchmarkable(begin
    
# end, 
# # setup = begin
    
# # end, 
# evals=1)
# overhead = BenchmarkTools.estimate_overhead()
# results = run(suite, verbose=true, overhead=overhead, gctrial=false)
# for result in results
#     println("$(first(result)):")
#     display(last(result))
#     println()
# end
# """
#     simulate(model, x0, ctrl; [kwargs...])

# Simulate `model` starting from `x0` using the `get_control(ctrl, x, t)` method to get the 
# closed-loop feedback command.

# # Keyword Arguments
# * `tf`: final time
# * `dt`: simulation time step
# """
# function simulate(model::Arthur, x0, ctrl; tf=ctrl.times[end], dt=1e-2)
#     n,m = size(model)
#     times = range(0, tf, step=dt)
#     N = length(times)
#     X = [@SVector zeros(n) for k = 1:N] 
#     U = [@SVector zeros(m) for k = 1:N-1]
#     X[1] = x0

#     tstart = time_ns()
#     for k = 1:N-1
#         U[k] = get_control(ctrl, X[k], times[k])
# #         u = clamp(U[k], umin, umax)
#         X[k+1] = discrete_dynamics(RK4, model, X[k], U[k], times[k], dt)
#     end
#     tend = time_ns()
#     rate = N / (tend - tstart) * 1e9
#     println("Controller ran at $rate Hz")
#     return X,U,times
# end