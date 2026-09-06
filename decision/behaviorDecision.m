function behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, plannerConfig, decisionState) %#ok<INUSD>
% behaviorDecision - maps current perception/prediction context into a
% high-level behavior intent, evaluating the project brief's priority
% hierarchy (Hour 20-22) in strict order - the first tier that fires wins,
% so safety always dominates route efficiency:
%
%   1. Emergency collision avoidance  -> "emergency_stop"
%   2. Pedestrian / animal safety     -> "brake" (or "wait")
%   3. Dynamic obstacle avoidance     -> "avoid"
%   4. Merge / interaction            -> "merge" / "follow"
%   5. Normal navigation              -> "cruise"
%
% Tier 2 exists specifically so vulnerable road users get a wider margin
% than vehicles do: a pedestrian or animal triggers caution at 1.5x the
% TTC threshold and a wider proximity radius than a car at the same
% distance would. That is the whole point of ranking their safety above
% generic obstacle avoidance.
%
% "unknown"-class agents are deliberately NOT treated as vulnerable.
% Unclassified is not the same as vulnerable, and the known duplicate-track
% bug (see docs/architecture.md) currently mints phantom "unknown" tracks -
% treating those as vulnerable would amplify a perception bug into
% corridor-wide braking.
%
% Hysteresis: an escalation (or same-severity) classification is returned
% immediately, but a relaxation is re-tested with thresholds widened by
% RECOVERY_MARGIN, so a value hovering at a threshold cannot drop the
% command the instant it wobbles under it. That widened re-test's only
% valid answers are "relax to this" or "hold the current state" - it must
% never be read as authorizing a step UP from the state it was testing
% whether to leave (see the inline comment at the call site for the
% avoid<->brake oscillation this caused when it could).
%
% Note: plannerConfig was added versus the original Phase 0 draft signature
% - the TTC classification needs plannerConfig.ttcThresholds and there was
% no other way to receive it. See docs/architecture.md's interface notes.
%
% Inputs:
%   egoState               - current ego state struct
%   trackedAgents          - struct array of tracked agents
%   predictedTrajectories  - cell array of predicted agent trajectories
%                            (unused for now; current-state TTC plus
%                            classifyBehavior is enough to pick a tier)
%   plannerConfig          - struct from config/plannerConfig.m
%   decisionState          - current state from decisionStateMachine, used
%                            here for the relaxation hysteresis
% Output:
%   behaviorCommand - one of: "cruise", "follow", "merge", "avoid",
%                     "wait", "brake", "emergency_stop"

VULNERABLE_CLASSES = ["pedestrian", "animal", "bicycle"];
VRU_TTC_FACTOR = 1.5;   % vulnerable road users trigger at 1.5x the normal TTC threshold
VRU_PROXIMITY = 3.0;    % [m] wider than the general proximity trigger below
GENERAL_PROXIMITY = 2.0; % [m] near the actual graze distance - these roads are only ~5m wide
FOLLOW_DIST = 15.0;     % [m] range within which a slower lead agent means "follow"
MERGE_DIST = 20.0;      % [m] range within which a merging agent means "merge"
FORWARD_CONE = deg2rad(75); % agents outside this cone are beside/behind, not a forward hazard
NEARLY_STOPPED_SPEED = 0.5; % [m/s]
RECOVERY_MARGIN = 1.5;

if isempty(trackedAgents)
    behaviorCommand = "cruise";
    return;
end

numAgents = numel(trackedAgents);
ttc = zeros(1, numAgents);
dist = zeros(1, numAgents);
isAhead = false(1, numAgents);
isVulnerable = false(1, numAgents);
category = strings(1, numAgents);
isSlowerLead = false(1, numAgents);

egoPos = [egoState.x, egoState.y];
egoHeadingVec = [cos(egoState.yaw), sin(egoState.yaw)];

for i = 1:numAgents
    agent = trackedAgents(i);
    relVec = agent.position - egoPos;

    ttc(i) = calculateTTC(egoState, agent);
    dist(i) = norm(relVec);

    bearing = atan2(relVec(2), relVec(1)) - egoState.yaw;
    bearing = atan2(sin(bearing), cos(bearing));
    isAhead(i) = abs(bearing) <= FORWARD_CONE;

    isVulnerable(i) = any(strcmp(agent.class, VULNERABLE_CLASSES));
    category(i) = classifyBehavior(agent, egoState.yaw);

    agentSpeedAlongRoad = dot(agent.velocity, egoHeadingVec);
    isSlowerLead(i) = isAhead(i) && category(i) == "normal" && agentSpeedAlongRoad < egoState.velocity;
end

rawCommand = classifyTier(1.0);
if behaviorSeverity(rawCommand) >= behaviorSeverity(decisionState)
    behaviorCommand = rawCommand; % escalating or holding: apply immediately, unchanged by the fix below
else
    % Relaxing: re-test with widened thresholds so a value hovering at a
    % boundary can't drop the command the instant it wobbles under it.
    % BUT the widened thresholds compound (e.g. the VRU tier multiplies
    % RECOVERY_MARGIN and VRU_TTC_FACTOR together), so this check can
    % itself return something MORE severe than decisionState - found live
    % in villageRoad: raw said "cruise", but the widened recovery check
    % alone returned "brake" while decisionState was "avoid", and because
    % decisionStateMachine can't tell a recovery check from a raw one, it
    % read severity(brake) >= severity(avoid) as a fresh escalation and
    % applied it immediately, bypassing the dwell time relaxation is
    % supposed to have - producing a sustained avoid<->brake oscillation.
    % A recovery check's only valid answers are "relax to this" or "hold
    % where you are" - it must never be read as authorizing a step UP from
    % the state it was testing whether to leave.
    recoveryCommand = classifyTier(RECOVERY_MARGIN);
    if behaviorSeverity(recoveryCommand) >= behaviorSeverity(decisionState)
        behaviorCommand = decisionState; % hold; do not let a relaxation test escalate
    else
        behaviorCommand = recoveryCommand;
    end
end

    function command = classifyTier(marginFactor)
        criticalTTC = plannerConfig.ttcThresholds.critical * marginFactor;
        warningTTC = plannerConfig.ttcThresholds.warning * marginFactor;
        vruTTC = warningTTC * VRU_TTC_FACTOR;
        vruProximity = VRU_PROXIMITY * marginFactor;
        generalProximity = GENERAL_PROXIMITY * marginFactor;

        % Tier 1 - emergency collision avoidance
        if any(ttc < criticalTTC)
            command = "emergency_stop";
            return;
        end

        % Tier 2 - pedestrian / animal safety
        vruAtRisk = isVulnerable & isAhead & (ttc < vruTTC | dist < vruProximity);
        if any(vruAtRisk)
            % Hold stopped only for a VRU actually moving through the path -
            % a static one would never clear, and setting speed to zero on a
            % condition that cannot change while stopped is exactly the
            % deadlock that froze villageRoad/cattleCrossing previously.
            % Static VRUs get "brake" instead, so the ego creeps past them.
            movingThrough = vruAtRisk & (category == "crossing" | category == "merging");
            if egoState.velocity < NEARLY_STOPPED_SPEED && any(movingThrough)
                command = "wait";
            else
                command = "brake";
            end
            return;
        end

        % Tier 3 - dynamic obstacle avoidance
        if any(ttc < warningTTC | (isAhead & dist < generalProximity))
            command = "avoid";
            return;
        end

        % Tier 4 - merge / interaction
        if any(isAhead & category == "merging" & dist < MERGE_DIST)
            command = "merge";
            return;
        end
        if any(isSlowerLead & dist < FOLLOW_DIST)
            command = "follow";
            return;
        end

        % Tier 5 - normal navigation
        command = "cruise";
    end

end
