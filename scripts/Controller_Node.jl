#!/usr/bin/env julia

include("MPC.jl")

# Black magic to make Julia and Python happy
using Distributed

using Pkg
Distributed.@everywhere using Pkg

Distributed.@everywhere begin
  ENV["PYTHON"] = "/usr/bin/python3"
  Pkg.build("PyCall")
end

using PyCall
Distributed.@everywhere using PyCall


#Import Statements for RobotOS
using RobotOS

@rosimport trajectory_msgs.msg: JointTrajectory, JointTrajectoryPoint
@rosimport arthur_planning.msg: arthur_traj
@rosimport sensor_msgs.msg: JointState
rostypegen()
using .trajectory_msgs.msg
using .arthur_planning.msg
using .sensor_msgs.msg

function callback(msg::JointTrajectory, X, U, initialized, N, idx, lastTime)
    if msg.header.stamp != lastTime[1]
        lastTime[1] = msg.header.stamp
        N .= length(msg.points)
        for k = 1:N[1]
            X[k][1:7] .= msg.points[k].positions
            X[k][8:14] .= msg.points[k].velocities
            U[k][1:end] .= msg.points[k].effort
        end
        if !initialized[1] && N[1] > 0
            initialized .= true
        end
        idx .= 1
    end
end

function traj_callback(msg::arthur_traj, Xref)
    Xref[1] = typeof(zeros(14))[]
    for k=1:length(msg.traj.points)
        q = msg.traj.points[k].positions
        q̇ = msg.traj.points[k].velocities
        push!(Xref[1], [q; q̇])
    end
    # RobotOS.loginfo("In Callback")
    # println(Xref[1][1])
end

function callback_x0(msg::JointState, x0)
    x0_new = zeros(14)
    x0_new[1:7] .= msg.position
    x0_new[8:14] .= msg.velocity
    copy!(x0, x0_new)
end

function loop(sub_obj1, sub_obj2, sub_obj3, pub_obj, x0, X, U, params, initialized, N, Xref, mpc_controller, idx, starting)
    loop_rate = Rate(20)
    while ! is_shutdown()
        RobotOS._run_callbacks(sub_obj1)
        RobotOS._run_callbacks(sub_obj2)
        RobotOS._run_callbacks(sub_obj3)
        if initialized[1] && mpc_controller[1]
            i = argmin(norm.([(X[k] - x0) for k=1:N[1]]))
            if norm(x0[1:7] - Xref[1][end][1:7]) < 0.05
                mpc_controller .= false
            end
            # if !isnan(x0[1])
            #     RobotOS.loginfo("MPC Control")
            #     println(U[i])
            #     println(x0)
            #     println(norm(x0 - Xref[1][end]))
            # end
            JointTrajectoryOutput = JointTrajectoryPoint()
            JointTrajectoryOutput.effort = U[i]
            RobotOS.loginfo("MPC Control")
            println(i)
            println(U[i])
            println(x0)
            # JointTrajectoryOutput.positions = x0[1:7]
            # JointTrajectoryOutput.velocities = x0[8:14]
            publish(pub_obj, JointTrajectoryOutput)
            # idx[1] = min(N[1], idx[1] + 1)
        # elseif norm(x0[8:14]) < 1e-6
        #     set_configuration!(params.state, x0[1:7])
        #     set_velocity!(params.state, zeros(7))
        #     params.v̇ .= zeros(7)
        #     U_grav = inverse_dynamics(params.state, params.v̇)
        #     RobotOS.loginfo("Gravity Compensation Control")
        #     println(U_grav)
        #     println(x0)
        #     # println(RobotDynamics.dynamics(params.model, x0, U_grav))
        #     JointTrajectoryOutput = JointTrajectoryPoint()
        #     JointTrajectoryOutput.effort = U_grav
        #     publish(pub_obj, JointTrajectoryOutput)
        elseif starting[1]
            if norm(x0 - Xref[1][1]) < 1e-2
                starting .= false
            end
            set_configuration!(params.state, x0[1:7])
            set_velocity!(params.state, x0[8:14])
            i = 1
            ∆x = Xref[1][i][1:7] - x0[1:7]
            # println(∆x)
            Kp = [30, 20, 20, 20, 20, 50, 20]
            Kd = [20, 20, 20, 20, 20, 20, 20]
            params.v̇ .= (Kp .* ∆x) .- (Kd .* x0[8:14])
            U_PD = inverse_dynamics(params.state, params.v̇)
            RobotOS.loginfo("PD Control")
            println(U_PD)
            println(x0)
            println(norm(x0 - Xref[1][1]))
            JointTrajectoryOutput = JointTrajectoryPoint()
            JointTrajectoryOutput.effort = U_PD
            publish(pub_obj, JointTrajectoryOutput)
        else
            set_configuration!(params.state, x0[1:7])
            set_velocity!(params.state, x0[8:14])

            # if initialized[1]
            #     i = argmin(norm.([(Xref[1][k][1:7] - x0[1:7]) for k=1:length(Xref[1])]))
            # else
            #     i = 1
            #     # println(Xref[1][i][1:7])
            #     # println(x0[1:7])
            # end

            i = min(argmin(norm.([(Xref[1][k][1:7] - x0[1:7]) for k=1:length(Xref[1])])) + 1, length(Xref[1]))
            
            ∆x = Xref[1][i][1:7] - x0[1:7]
            # println(∆x)
            Kp = [100, 100, 100, 100, 100, 100, 100]
            Kd = [20, 20, 20, 20, 20, 20, 20]
            params.v̇ .= (Kp .* ∆x) .- (Kd .* x0[8:14])
            U_PD = inverse_dynamics(params.state, params.v̇)
            # x0 .= rk4(params.model, x0, U_PD, params.dt)
            # if !isnan(x0[1])
            #     RobotOS.loginfo("PD Control")
            #     println(params.v̇[1:7])
            #     println(U_PD[1:7])
            #     println(x0[1:7])
            #     println(norm(x0 - Xref[end]))
            # end
            RobotOS.loginfo("PD Tracking Control")
            println(U_PD)
            println(x0)
            println(i)
            JointTrajectoryOutput = JointTrajectoryPoint()
            JointTrajectoryOutput.effort = U_PD
            publish(pub_obj, JointTrajectoryOutput)
        end
        rossleep(loop_rate)
    end
end

function main()
    init_node("Controller_Node")
    params = MPC_Params()
    
    # Xref = get_trajectory(params)
    # x0 = copy(Xref[1])
    Xref = [typeof(zeros(14))[]]
    x0 = zeros(14)
    # N = [params.H-1]
    N = [params.LQRH]
    X = [zeros(14) for k = 1:params.H-1]
    U = [zeros(7) for k = 1:params.H-1]
    initialized = [false]
    mpc_controller = [false]
    starting = [true]
    idx = [1]
    lastTime = [RobotOS.now()]

    sub_x0 = Subscriber{JointState}("/my_gen3/joint_states", callback_x0, (x0,), queue_size=1)
    sub = Subscriber{JointTrajectory}("joint_torques",callback,(X, U, initialized, N, idx, lastTime),queue_size=1)
    sub_traj = Subscriber{arthur_traj}("/my_gen3/arthur_traj",traj_callback,(Xref,),queue_size=1)
    
    pub = Publisher{JointTrajectoryPoint}("mpc_torques",queue_size=1)

    while length(Xref[1]) == 0
        RobotOS._run_callbacks(sub_traj)
    end
    while norm(x0 - zeros(14)) == 0
        RobotOS._run_callbacks(sub_x0)
    end

    loop(sub, sub_traj, sub_x0, pub, x0, X, U, params, initialized, N, Xref, mpc_controller, idx, starting)
end

if !isinteractive()
    main()
end

