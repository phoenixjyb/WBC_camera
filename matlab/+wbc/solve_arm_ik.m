function [qMatrix, jointNames, infos] = solve_arm_ik(robot, eeName, poseTforms, opts)
%SOLVE_ARM_IK Unified entry point for classic IK and generalized IK solves.
%   [Q, NAMES, INFOS] = WBC.SOLVE_ARM_IK(ROBOT, EENAME, POSETFORMS, OPTS)
%   wraps the existing rt_compute_arm_ik/rt_compute_arm_gik helpers while
%   enforcing consistent option handling for MATLAB Coder and future C++ ports.
%
%   Required inputs:
%     ROBOT      - rigidBodyTree robot model
%     EENAME     - name of the end-effector link
%     POSETFORMS - cell array of 4x4 transforms or N-by-16 numeric array
%
%   Optional opts fields:
%     UseGeneralizedIK (logical)     : invoke rt_compute_arm_gik instead of IK
%     GIKOptions (struct)            : options passed to rt_compute_arm_gik
%     IKWeights (1x6 double)         : weights vector for rt_compute_arm_ik
%     InitialGuess (struct array)    : configuration array used as warm start
%
%   Outputs mirror the legacy helpers: joint matrix, joint name cell-array,
%   and per-sample solver info.

arguments
    robot (1,1) rigidBodyTree
    eeName (1,:) char
    poseTforms
    opts.UseGeneralizedIK logical = false
    opts.GIKOptions = struct()
    opts.IKWeights (1,6) double = [0.5 0.5 0.5 1 1 1]
    opts.InitialGuess = []
end

if opts.UseGeneralizedIK
    gikOpts = opts.GIKOptions;
    if isempty(opts.InitialGuess)
        gikOpts.InitialGuess = homeConfiguration(robot);
    else
        gikOpts.InitialGuess = opts.InitialGuess;
    end
    [qMatrix, jointNames, infos] = rt_compute_arm_gik(robot, eeName, poseTforms, gikOpts);
else
    if isempty(opts.InitialGuess)
        initialGuess = homeConfiguration(robot);
    else
        initialGuess = opts.InitialGuess;
    end
    [qMatrix, jointNames, infos] = rt_compute_arm_ik(robot, eeName, poseTforms, opts.IKWeights, initialGuess);
end
end
