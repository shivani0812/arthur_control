# import Pkg; Pkg.activate(@__DIR__); 
# Pkg.instantiate()

# using RigidBodyDynamics
# using StaticArrays
# using RobotDynamics
# using Altro
# using TrajectoryOptimization

# function mpc_update(altro, prob_mpc, Z_track, t0, k_mpc)
#     TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

#     # Propagate the system forward w/ noise
#     # x0 = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc),
#     #                             prob_mpc.model, prob_mpc.Z[1])      
#     x0 = rk4(prob_mpc.model, state(prob_mpc.Z[1]), control(prob_mpc.Z[1]), prob_mpc.Z[1].dt)

#     TrajectoryOptimization.set_initial_state!(prob_mpc, x0)

#     # Update tracking cost
#     TrajectoryOptimization.update_trajectory!(prob_mpc.obj, Z_track, k_mpc)

#     # Shift the initial trajectory
#     RobotDynamics.shift_fill!(prob_mpc.Z)

#     # Shift the multipliers and penalties
#     Altro.shift_fill!(TrajectoryOptimization.get_constraints(altro.solver_al))
# end

# function update_constraints!(conSet::Altro.ALConstraintSet, cons::Altro.ALConstraintSet, start=1)
#     for c = 1:length(conSet)
#         prob_cons = conSet.convals[c]
#         inds = (start-1) .+ (1:length(conSet.convals[c].λ))
#         if (start + length(conSet.convals[c]) - 1) <= length(cons)
#             for (i,k) in enumerate(inds)
#                 conSet.convals[c].λ[i] = cons.convals[k].λ
#                 # TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[k]), control(Z[k]))
#             end
#         else
#             for (i,k) in enumerate(inds)
#                 conSet.convals[c].λ[i] = cons[min(k,length(cons))]
#                 # TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[min(k,length(Z))]), control(Z[min(k,length(Z))]))
#             end
#         end
#     end
# end

# function update_trajectory_fill!(obj::Objective{<:TrajectoryOptimization.QuadraticCostFunction}, Z::TrajectoryOptimization.AbstractTrajectory, start=1)
#     inds = (start-1) .+ (1:length(obj))
#     if (start + length(obj) - 1) <= length(Z)
#         for (i,k) in enumerate(inds)
#             TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[k]), control(Z[k]))
#         end
#     else
#         for (i,k) in enumerate(inds)
#             TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[min(k,length(Z))]), control(Z[min(k,length(Z))]))
#         end
#     end
# end

# function update_guess_trajectory!(prob_Z::Traj, Z::TrajectoryOptimization.AbstractTrajectory, start=1)
#     inds = (start-1) .+ (1:length(prob_Z))
#     if (start + length(prob_Z) - 1) <= length(Z)
#         for (i,k) in enumerate(inds)
#             prob_Z[i] = Z[k]
#             # TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[k]), control(Z[k]))
#         end
#     else
#         for (i,k) in enumerate(inds)
#             prob_Z[i] = Z[min(k,length(Z))]
#             # TrajectoryOptimization.set_LQR_goal!(obj[i], state(Z[min(k,length(Z))]), control(Z[min(k,length(Z))]))
#         end
#     end
# end

# function mpc_update(altro, prob_mpc, Z_track, cons, x0, t0)
#     TrajectoryOptimization.set_initial_time!(prob_mpc, t0)

#     # Propagate the system forward w/ noise
#     # x0 = discrete_dynamics(TrajectoryOptimization.integration(prob_mpc),
#     #                             prob_mpc.model, prob_mpc.Z[1])      
#     # x0 = rk4(prob_mpc.model, state(prob_mpc.Z[1]), control(prob_mpc.Z[1]), prob_mpc.Z[1].dt)
#     k_mpc = argmin(norm.([(states(Z_track)[k] - x0) for k=1:length(Z_track)]))
#     println(k_mpc)

#     TrajectoryOptimization.set_initial_state!(prob_mpc, x0)

#     # Update tracking cost
#     update_trajectory_fill!(prob_mpc.obj, Z_track, k_mpc)
    
#     # Shift the initial trajectory
#     # update_guess_trajectory!(prob_mpc.Z, Z_track, k_mpc)
#     RobotDynamics.shift_fill!(prob_mpc.Z)
    
#     # Shift the multipliers and penalties
#     # update_constraints!(TrajectoryOptimization.get_constraints(altro.solver_al), cons, k_mpc)
#     Altro.shift_fill!(TrajectoryOptimization.get_constraints(altro.solver_al), 0)
# end

function ArthurProblem(Xref; params=MPC_Params())
    # Create initial guess of U trajectory
    Qref = Vector{Float64}[]
    Uref = Vector{Float64}[]
    N = length(Xref)
    n = params.n
    m = params.m
    tf = round(params.dt*(N-1), digits = 3)
    Qref = [Xref[1] for k=1:length(Xref)]
    set_configuration!(params.state, Xref[1][1:7])
    Uref = [inverse_dynamics(params.state, params.v̇) for k=1:(length(Xref)-1)]

    # push!(Qref, copy(Xref[1][1:14]))
    # for k = 1:N-1
    #     set_configuration!(params.state, Qref[k][1:7])
    #     set_velocity!(params.state, Qref[k][8:14])
    #     push!(Uref, inverse_dynamics(params.state, params.v̇))
    #     push!(Qref, rk4(params.model, Qref[k], Uref[k], params.dt))
    # end

    # Gather trajectory information
    # Create Empty ConstraintList
    conSet = ConstraintList(n,m,N)

    # Control Bounds based on Robot Specs (Joint torque limits)
    u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
    control_bnd = BoundConstraint(n,m, u_min=-u_bnd, u_max=u_bnd)
    add_constraint!(conSet, control_bnd, 1:N-1)

    # State Bounds based on Robot Specs (Joint velocity and speed limits)
    x_bnd = zeros(n)
    x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
    x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
    # x_bnd[8:14] = [0.25, 0.05, 0.05, 0.05, 0.25, 0.15, 0.15] # rad/sec
    # x_bnd[15:end] = [Inf for k=1:(n-14)] # Constraints on force elsewhere
    state_bnd = BoundConstraint(n,m, x_min=-x_bnd, x_max=x_bnd)
    add_constraint!(conSet, state_bnd, 1:N-1)

    # # Cartesian Velocity Bound
    # ẋ_max = 0.0005 # m/s
    # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
    # add_constraint!(conSet, vel_bnd, 1:N)

    # # Force Bound (Fx Fy Fz)
    # F_max = 20 # Newtons
    # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
    # add_constraint!(conSet, F_bnd, 1:N)

    dtref = [params.dt for k=1:N]
    traj = RobotDynamics.Traj(Xref, Uref, dtref, cumsum(dtref) .- dtref[1])
    obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, traj, Qf=params.Qf)
    prob = Problem(params.model, obj, Xref[end], tf, x0=Xref[1], constraints=conSet, X0=Qref, U0=Uref, integration=RK3)
    return prob
end

# function ArthurHorizonProblem(prob::TrajectoryOptimization.Problem, x0, N; start=1, params=MPC_Params())
#     # H = N
#     while (start+N-1) > length(prob.Z)
#         N -= 1
#     end

#     n,m = size(prob)
#     dt = prob.Z[1].dt
#     tf = (N-1)*dt
#     # Get sub-trajectory
#     if N > 1  
#     # if N == H
#         Z = Traj(prob.Z[start:start+N-1])
#     # else
#         # Z = Traj([prob.Z[start:start+N-1]; [prob.Z[end] for k = 1:(H-N+1)]])
#     # end

#     # Generate a cost that tracks the trajectory
#         obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, Z, Qf=params.Qf)

#     # Use the same constraints, except the Goal constraint
#         cons = ConstraintList(n,m,N)
#         for (inds, con) in zip(prob.constraints)
#             if !(con isa GoalConstraint)
#                 if inds.stop > N
#                     inds = inds.start:N-(prob.N - inds.stop)
#                 end
#                 length(inds) > 0 && TrajectoryOptimization.add_constraint!(cons, con, inds)
#             end
#         end

#         prob = TrajectoryOptimization.Problem(prob.model, obj, state(Z[end]), tf, x0=x0, constraints=cons,
#             integration=TrajectoryOptimization.integration(prob)
#         )
#         initial_trajectory!(prob, Z)
#     else
#         obj = TrajectoryOptimization.LQRObjective(params.Q, params.R, params.Qf, state(prob.Z[end]), params.LQRH)
#          # # Gather trajectory information
#         # Create Empty ConstraintList
#         cons = ConstraintList(params.n,params.m,params.LQRH)

#         # Control Bounds based on Robot Specs (Joint torque limits)
#         u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
#         control_bnd = BoundConstraint(params.n,params.m, u_min=-u_bnd, u_max=u_bnd)
#         add_constraint!(cons, control_bnd, 1:params.LQRH-1)

#         # State Bounds based on Robot Specs (Joint velocity and speed limits)
#         x_bnd = zeros(params.n)
#         x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
#         x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
#         # x_bnd[8:14] = [0.25, 0.05, 0.05, 0.05, 0.25, 0.15, 0.15] # rad/sec
#         # x_bnd[15:end] = [Inf for k=1:(n-14)] # Constraints on force elsewhere
#         state_bnd = BoundConstraint(params.n,params.m, x_min=-x_bnd, x_max=x_bnd)
#         add_constraint!(cons, state_bnd, 1:params.LQRH-1)

#         # # # Cartesian Velocity Bound
#         # # ẋ_max = 0.0005 # m/s
#         # # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
#         # # add_constraint!(cons, vel_bnd, 1:N)

#         # # # Force Bound (Fx Fy Fz)
#         # # F_max = 20 # Newtons
#         # # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
#         # # add_constraint!(cons, F_bnd, 1:N)
#         Z = Traj([prob.Z[end] for k = 1:params.LQRH])
#         tf = params.LQRH*dt
#         prob = TrajectoryOptimization.Problem(prob.model, obj, state(prob.Z[end]), tf, x0=x0, constraints=cons,
#             integration=TrajectoryOptimization.integration(prob)
#         )
#         initial_trajectory!(prob, Z)
#     end
    
#     return prob
# end

# function ArthurHorizonProblem(prob::TrajectoryOptimization.Problem, x0, N; start=1, params=MPC_Params())
#     # H = N
#     # while (start+N-1) > length(prob.Z)
#     #     N -= 1
#     # end

#     n,m = size(prob)
#     dt = prob.Z[1].dt
#     tf = params.LQRH*dt
#     # tf = (N-1)*dt
#     if start >= length(prob.Z)
#         N = length(prob.Z)
#     else
#         N = start + 1
#     end
    
#     x_current = zeros(n)
#     x_current[1:7] = x0[1:7]
#     xf = zeros(n)
#     xf[1:7] .= state(prob.Z[N])[1:7]

#     obj = TrajectoryOptimization.LQRObjective(params.Q, params.R, params.Qf, xf, params.LQRH)

#     # Create Empty ConstraintList
#     cons = ConstraintList(params.n,params.m,params.LQRH)

#     # Control Bounds based on Robot Specs (Joint torque limits)
#     u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
#     control_bnd = BoundConstraint(params.n,params.m, u_min=-u_bnd, u_max=u_bnd)
#     add_constraint!(cons, control_bnd, 1:params.LQRH-1)

#     # State Bounds based on Robot Specs (Joint velocity and speed limits)
#     x_bnd = zeros(params.n)
#     x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
#     x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
#     # x_bnd[15:end] = [Inf for k=1:(n-14)] # Constraints on force elsewhere
#     state_bnd = BoundConstraint(params.n,params.m, x_min=-x_bnd, x_max=x_bnd)
#     add_constraint!(cons, state_bnd, 1:params.LQRH-1)

#     # # # Cartesian Velocity Bound
#     # # ẋ_max = 0.0005 # m/s
#     # # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
#     # # add_constraint!(cons, vel_bnd, 1:N)

#     # # # Force Bound (Fx Fy Fz)
#     # # F_max = 20 # Newtons
#     # # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
#     # # add_constraint!(cons, F_bnd, 1:N)
    
#     Qref = [x_current for k=1:params.LQRH]
#     set_configuration!(params.state, x_current[1:7])
#     Uref = [inverse_dynamics(params.state, params.v̇) for k=1:params.LQRH-1]
#     dtref = [params.dt for k=1:params.LQRH]
#     Z = Traj(Qref, Uref, dtref, cumsum(dtref) .- dtref[1])
#     # Z = Traj([prob.Z[end] for k = 1:params.LQRH])
#     tf = params.LQRH*dt
#     prob = TrajectoryOptimization.Problem(prob.model, obj, xf, tf, x0=x0, constraints=cons,
#         integration=TrajectoryOptimization.integration(prob)
#     )
#     initial_trajectory!(prob, Z)
    
#     return prob
# end

function ArthurHorizonProblem(Xref, x0, N; start=1, params=MPC_Params())
    # H = N
    # while (start+N-1) > length(prob.Z)
    #     N -= 1
    # end

    n = params.n
    m = params.m
    dt = params.dt
    tf = params.LQRH*dt
    # tf = (N-1)*dt
    if start >= length(Xref)
        N = length(Xref)
    else
        N = start + 1
    end
    
    x_current = zeros(n)
    x_current[1:7] = x0[1:7]
    xf = zeros(n)
    xf[1:7] .= Xref[N][1:7]
    # xf[1:7] .= x0[1:7]

    obj = TrajectoryOptimization.LQRObjective(params.Q, params.R, params.Qf, xf, params.LQRH)

    # Create Empty ConstraintList
    cons = ConstraintList(params.n,params.m,params.LQRH)

    # Control Bounds based on Robot Specs (Joint torque limits)
    u_bnd = [39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0]
    control_bnd = BoundConstraint(params.n,params.m, u_min=-u_bnd, u_max=u_bnd)
    add_constraint!(cons, control_bnd, 1:params.LQRH-1)

    # State Bounds based on Robot Specs (Joint velocity and speed limits)
    x_bnd = zeros(params.n)
    x_bnd[1:7] = [Inf, deg2rad(128.9), Inf, deg2rad(147.8), Inf, deg2rad(120.3), Inf] # rad
    x_bnd[8:14] = [1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22] # rad/sec
    # x_bnd[15:end] = [Inf for k=1:(n-14)] # Constraints on force elsewhere
    state_bnd = BoundConstraint(params.n,params.m, x_min=-x_bnd, x_max=x_bnd)
    add_constraint!(cons, state_bnd, 1:params.LQRH-1)

    # # # Cartesian Velocity Bound
    # # ẋ_max = 0.0005 # m/s
    # # vel_bnd = NormConstraint(n, m, ẋ_max, Inequality(), 21:23)
    # # add_constraint!(cons, vel_bnd, 1:N)

    # # # Force Bound (Fx Fy Fz)
    # # F_max = 20 # Newtons
    # # F_bnd = NormConstraint(n, m, F_max, Inequality(), 27:29)
    # # add_constraint!(cons, F_bnd, 1:N)
    
    Qref = [x_current for k=1:params.LQRH]
    set_configuration!(params.state, x_current[1:7])
    Uref = [inverse_dynamics(params.state, params.v̇) for k=1:params.LQRH-1]
    dtref = [params.dt for k=1:params.LQRH]
    Z = Traj(Qref, Uref, dtref, cumsum(dtref) .- dtref[1])
    # Z = Traj([prob.Z[end] for k = 1:params.LQRH])
    tf = params.LQRH*dt
    prob = TrajectoryOptimization.Problem(params.model, obj, xf, tf, x0=x0, constraints=cons,
        integration=RK3
    )
    initial_trajectory!(prob, Z)
    
    return prob
end

struct MPC_Params
    model::Arthur
    n::Int64
    m::Int64
    H::Int64
    LQRH::Int64
    tf::Float64
    dt::Float64
    state::MechanismState
    v̇::SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}
    opts::SolverOptions{Float64}
    Q::Diagonal{Float64, SVector{14, Float64}}
    Qf::Diagonal{Float64, SVector{14, Float64}}
    R::Diagonal{Float64, SVector{7, Float64}}

    function MPC_Params()
        model = Arthur()
        n,m = size(model)

        tf = 0.25 # Time Horizon (seconds)
        dt = 0.05 # Time step (seconds)
        H = Int(round(tf/dt) + 1) # Time Horizon (discrete steps)
        LQRH = 5
        state = MechanismState(model.mechanism)
        zero!(state)
        v̇ = similar(velocity(state))
        for joint in joints(state.mechanism)
            if joint == joints(state.mechanism)[2] || joint == joints(state.mechanism)[3] || joint == joints(state.mechanism)[4] || joint == joints(state.mechanism)[5] || joint == joints(state.mechanism)[6] || joint == joints(state.mechanism)[7] || joint == joints(state.mechanism)[8]
                v̇[joint][1] = 0.0
            end
        end

        opts = SolverOptions(
            cost_tolerance_intermediate=1e-2,
            penalty_scaling=10.,
            penalty_initial=1.0,
            show_summary=true,
            projected_newton=true,
        )

        Q = 100.0*Diagonal(@SVector ones(n))
        Qf = 1000.0*Diagonal(@SVector ones(n))
        R = 1.0e-1*Diagonal(@SVector ones(m))
        new(model,n,m,H,LQRH,tf,dt,state,v̇,opts,Q,Qf,R)
    end
end


# function initialize_solver(params::MPC_Params)
#     traj = RobotDynamics.Traj(params.n, params.m, params.dt, params.H)
#     obj = TrajectoryOptimization.TrackingObjective(params.Q, params.R, traj, Qf=params.Qf)
#     prob = Problem(params.model, obj, Xref[end], tf, x0=Xref[1], constraints=params.conSet, X0=Xref, U0=Uref)
#     altro = ALTROSolver(prob, params.opts)
# end