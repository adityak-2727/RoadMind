classdef RealtimeRenderer < handle
    % RealtimeRenderer - decoupled, smooth (~60 FPS target) visualization
    % layer for the closed-loop demo, completely downstream of the real
    % simulation. Replaces main.m's old per-tick cla()+fresh-plot()+
    % pause(0.3) block, which recreated every graphics object from
    % scratch every 10 Hz tick and blocked the simulation loop on a fixed
    % sleep - the textbook anti-pattern for choppy, slow MATLAB
    % animation.
    %
    % ARCHITECTURE (matches the requested decoupling exactly):
    %   real simulation tick (10 Hz, unchanged, unpaused)
    %       -> this class's update() is called ONCE per real tick, with
    %          the REAL, already-computed ego/agent/prediction/path state
    %       -> update() renders ~6 interpolated sub-frames between the
    %          PREVIOUS real tick's state and this one (round(dt/(1/60)))
    %          using ONLY linear interpolation of already-computed,
    %          real values - it never invents new dynamics and never
    %          writes anything back to the caller. The renderer only
    %          ever READS state passed into update(); nothing it computes
    %          is returned to or reused by the simulation.
    %       -> each sub-frame updates persistent graphics handles
    %          in-place (XData/YData/text position, never plot() again)
    %          and paces itself to a 1/60s target using measured elapsed
    %          time - NOT a fixed pause() count, so it gracefully renders
    %          slower than 60 FPS if the machine can't keep up, and never
    %          waits longer than necessary if a frame renders fast.
    %
    % Interpolation is visual-only by construction: update() is called
    % from main.m's visualization section, strictly AFTER perception,
    % prediction (K1), decision, planning (K2), collision checking,
    % control, and the vehicle model have all already run using the real,
    % non-interpolated state - this class has no way to feed anything
    % back into that computation even in principle, since it holds no
    % reference to any of main.m's simulation variables and is never
    % called from within the computation section.
    %
    % Predicted trajectories and the planned path are per-tick decisions,
    % not continuously-varying quantities, so they are held at the
    % previous tick's content through the interpolation sub-frames (via
    % the 'HOLD' sentinel, unambiguous since a real path/prediction value
    % is always numeric/cell, never char) and snap to the new tick's
    % actual content only on the final sub-frame - never blended between
    % two different predictions/paths, which would show something the
    % planner never actually computed.

    properties (Access = private)
        Fig
        Ax
        TitleHandle
        FPSTextHandle
        EgoMarkerHandle
        EgoHeadingHandle
        PlannedPathHandle
        AgentMarkerHandles  = matlab.graphics.chart.primitive.Line.empty
        AgentLabelHandles   = matlab.graphics.Graphics.empty
        PredTrajHandles     = matlab.graphics.chart.primitive.Line.empty

        HasPrevState = false
        PrevEgoX = 0
        PrevEgoY = 0
        PrevEgoYaw = 0
        PrevAgentPositions  % containers.Map: id -> [x,y]

        ViewportHalfSize
        PlaybackRate
        TargetFrameDt = 1/60

        MeasuredFPS = 0
        LastTitleTick = -1
        FrameStartTic
    end

    methods
        function obj = RealtimeRenderer(playbackRate, viewportHalfSize)
            % playbackRate - real-time multiplier for the render layer
            %                only (1.0 = play back at real-time-equivalent
            %                speed; 0.5 = twice as slow, for a more
            %                deliberate walkthrough). Does NOT touch the
            %                simulation timestep or iteration rate.
            % viewportHalfSize - [m] fixed half-width/height of the
            %                ego-centered camera window, for a smooth,
            %                non-jumpy top-down follow view instead of
            %                MATLAB's default autoscale-to-content
            %                (which jumps/rescales as content enters or
            %                leaves the plotted extent).
            if nargin < 1 || isempty(playbackRate)
                playbackRate = 1.0;
            end
            if nargin < 2 || isempty(viewportHalfSize)
                viewportHalfSize = 50; % [m] a bit past camera range (40m) for context
            end
            obj.PlaybackRate = playbackRate;
            obj.ViewportHalfSize = viewportHalfSize;
            obj.PrevAgentPositions = containers.Map('KeyType', 'double', 'ValueType', 'any');

            obj.Fig = figure('Name', 'Closed-loop demo', 'Color', 'w');
            obj.Ax = axes(obj.Fig);
            hold(obj.Ax, 'on');
            axis(obj.Ax, 'equal');
            grid(obj.Ax, 'on');
            xlabel(obj.Ax, 'x [m]');
            ylabel(obj.Ax, 'y [m]');

            % Persistent handles, created ONCE - every subsequent frame
            % only updates their data, never recreates them.
            obj.PlannedPathHandle = plot(obj.Ax, NaN, NaN, 'b--', 'LineWidth', 1);
            obj.EgoMarkerHandle = plot(obj.Ax, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
            obj.EgoHeadingHandle = quiver(obj.Ax, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);
            obj.TitleHandle = title(obj.Ax, '', 'Interpreter', 'none');
            obj.FPSTextHandle = text(obj.Ax, 0.02, 0.96, '', 'Units', 'normalized', ...
                'FontSize', 9, 'Color', [0.2, 0.2, 0.2], 'VerticalAlignment', 'top');
        end

        function update(obj, egoState, trackedAgents, predictedTrajectories, plannedPath, decisionState, targetSpeed, minTTC, scenarioName)
            % Called once per REAL simulation tick with the real,
            % already-computed state. Internally renders several
            % interpolated sub-frames to reach ~60 FPS without altering
            % anything the caller passed in.

            if ~ishghandle(obj.Fig)
                return; % figure closed by the user - render layer degrades silently, sim is unaffected
            end

            numSubFrames = max(1, round(0.1 / obj.TargetFrameDt)); % dt=0.1s (10Hz) is the sim's own rate, read only for sub-frame count

            if ~obj.HasPrevState
                % First call: nothing to interpolate from yet - draw the
                % initial state directly and establish the baseline.
                obj.drawSubFrame(egoState.x, egoState.y, egoState.yaw, obj.PrevAgentPositions, ...
                    predictedTrajectories, plannedPath, decisionState, targetSpeed, minTTC, scenarioName, egoState.timestamp);
                obj.storePrevState(egoState, trackedAgents);
                obj.HasPrevState = true;
                return;
            end

            prevPositions = obj.PrevAgentPositions;

            for k = 1:numSubFrames
                alpha = k / numSubFrames;
                x = obj.lerp(obj.PrevEgoX, egoState.x, alpha);
                y = obj.lerp(obj.PrevEgoY, egoState.y, alpha);
                yaw = obj.lerpAngle(obj.PrevEgoYaw, egoState.yaw, alpha);

                if k < numSubFrames
                    predToShow = 'HOLD';
                    pathToShow = 'HOLD';
                else
                    predToShow = predictedTrajectories;
                    pathToShow = plannedPath;
                end

                % Timing wraps ALL of this sub-frame's real work (agent
                % interpolation update included) - it must, otherwise the
                % pacing budget and the measured FPS would both silently
                % exclude a real, sometimes-dominant cost and understate
                % how long each frame actually takes.
                obj.beginFrameTiming();
                obj.updateInterpolatedAgents(trackedAgents, prevPositions, alpha);
                obj.drawSubFrame(x, y, yaw, [], predToShow, pathToShow, decisionState, targetSpeed, minTTC, scenarioName, egoState.timestamp);
                obj.paceFrame();
            end

            obj.storePrevState(egoState, trackedAgents);
        end

        function drawFinalTrace(obj, egoHistory)
            % One-time, end-of-run overlay of the full driven path -
            % matches the old main.m's post-loop trace plot, called once
            % (not per-frame), so no special handle-reuse concerns apply.
            if ~ishghandle(obj.Fig) || isempty(egoHistory)
                return;
            end
            plot(obj.Ax, egoHistory(:, 1), egoHistory(:, 2), 'g-', 'LineWidth', 1.5);
        end

        function fps = getMeasuredFPS(obj)
            fps = obj.MeasuredFPS;
        end
    end

    methods (Access = private)
        function storePrevState(obj, egoState, trackedAgents)
            obj.PrevEgoX = egoState.x;
            obj.PrevEgoY = egoState.y;
            obj.PrevEgoYaw = egoState.yaw;
            obj.PrevAgentPositions = containers.Map('KeyType', 'double', 'ValueType', 'any');
            for i = 1:numel(trackedAgents)
                obj.PrevAgentPositions(double(trackedAgents(i).id)) = trackedAgents(i).position;
            end
        end

        function drawSubFrame(obj, egoX, egoY, egoYaw, ~, predictedTrajectories, plannedPath, decisionState, targetSpeed, minTTC, scenarioName, simTime)
            % --- Ego marker + heading: updated in place, no new objects ---
            set(obj.EgoMarkerHandle, 'XData', egoX, 'YData', egoY);
            arrowLen = 2;
            set(obj.EgoHeadingHandle, 'XData', egoX, 'YData', egoY, ...
                'UData', arrowLen * cos(egoYaw), 'VData', arrowLen * sin(egoYaw));

            % --- Ego-centered viewport: smooth camera-follow instead of autoscale jumps ---
            h = obj.ViewportHalfSize;
            xlim(obj.Ax, [egoX - h, egoX + h]);
            ylim(obj.Ax, [egoY - h, egoY + h]);

            % --- Planned path: 'HOLD' sentinel means "leave the existing handle data as-is" ---
            if ~(ischar(plannedPath) || isstring(plannedPath))
                if isempty(plannedPath)
                    set(obj.PlannedPathHandle, 'XData', NaN, 'YData', NaN);
                else
                    set(obj.PlannedPathHandle, 'XData', plannedPath(:, 1), 'YData', plannedPath(:, 2));
                end
            end

            % --- Predicted trajectories: same hold-then-snap rule ---
            if ~(ischar(predictedTrajectories) || isstring(predictedTrajectories))
                obj.updatePredictedTrajectoryPool(predictedTrajectories);
            end

            % --- Title: only refresh text content once per real tick (not every sub-frame) ---
            if simTime ~= obj.LastTitleTick
                set(obj.TitleHandle, 'String', sprintf('%s | t=%.1fs | speed target %.1f m/s | state=%s | minTTC=%.1fs', ...
                    scenarioName, simTime, targetSpeed, decisionState, minTTC));
                obj.LastTitleTick = simTime;
            end

            set(obj.FPSTextHandle, 'String', sprintf('Render: %.0f FPS', obj.MeasuredFPS));

            drawnow limitrate;
        end

        function updateInterpolatedAgents(obj, trackedAgents, prevPositions, alpha)
            n = numel(trackedAgents);
            while numel(obj.AgentMarkerHandles) < n
                obj.AgentMarkerHandles(end + 1) = plot(obj.Ax, NaN, NaN, 's', ...
                    'MarkerFaceColor', [0.9, 0.5, 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
                obj.AgentLabelHandles(end + 1) = text(obj.Ax, NaN, NaN, '', 'FontSize', 8, 'Color', [0.4, 0.2, 0]);
            end

            for i = 1:n
                agent = trackedAgents(i);
                id = double(agent.id);
                if isKey(prevPositions, id)
                    prevPos = prevPositions(id);
                    px = obj.lerp(prevPos(1), agent.position(1), alpha);
                    py = obj.lerp(prevPos(2), agent.position(2), alpha);
                else
                    % New id this tick (no prior sample) - appears at its real position, no interpolation possible.
                    px = agent.position(1);
                    py = agent.position(2);
                end
                set(obj.AgentMarkerHandles(i), 'XData', px, 'YData', py, 'Visible', 'on');
                set(obj.AgentLabelHandles(i), 'Position', [px + 0.5, py + 0.5, 0], 'String', char(agent.class), 'Visible', 'on');
            end
            for i = (n + 1):numel(obj.AgentMarkerHandles)
                set(obj.AgentMarkerHandles(i), 'Visible', 'off');
                set(obj.AgentLabelHandles(i), 'Visible', 'off');
            end
        end

        function updatePredictedTrajectoryPool(obj, predictedTrajectories)
            n = numel(predictedTrajectories);
            while numel(obj.PredTrajHandles) < n
                obj.PredTrajHandles(end + 1) = plot(obj.Ax, NaN, NaN, ':', 'Color', [0.9, 0.4, 0.1], 'LineWidth', 1);
            end
            for i = 1:n
                pred = predictedTrajectories{i};
                if isempty(pred)
                    set(obj.PredTrajHandles(i), 'XData', NaN, 'YData', NaN, 'Visible', 'off');
                else
                    set(obj.PredTrajHandles(i), 'XData', pred(:, 1), 'YData', pred(:, 2), 'Visible', 'on');
                end
            end
            for i = (n + 1):numel(obj.PredTrajHandles)
                set(obj.PredTrajHandles(i), 'Visible', 'off');
            end
        end

        function beginFrameTiming(obj)
            obj.FrameStartTic = tic;
        end

        function paceFrame(obj)
            elapsed = toc(obj.FrameStartTic);
            target = obj.TargetFrameDt / max(obj.PlaybackRate, 1e-6);
            remaining = target - elapsed;
            if remaining > 0
                pause(remaining); % render-layer pacing only - never inside the autonomous computation loop
                elapsed = target;
            end
            if elapsed > 0
                instFps = 1 / elapsed;
                if obj.MeasuredFPS == 0
                    obj.MeasuredFPS = instFps;
                else
                    obj.MeasuredFPS = 0.9 * obj.MeasuredFPS + 0.1 * instFps; % smoothed, but always from real measured intervals
                end
            end
        end
    end

    methods (Static, Access = private)
        function v = lerp(a, b, alpha)
            v = a + (b - a) * alpha;
        end

        function v = lerpAngle(a, b, alpha)
            d = atan2(sin(b - a), cos(b - a)); % shortest-path angular difference, avoids +/-pi wraparound artifacts
            v = a + d * alpha;
        end
    end
end
