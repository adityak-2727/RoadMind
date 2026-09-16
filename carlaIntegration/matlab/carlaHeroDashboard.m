function fig = carlaHeroDashboard(fig, r, globalPath, driven, collisions)
% Display only; initialize graphics BEFORE starting the asynchronous drive.
if isempty(fig) || ~isgraphics(fig)
    fig = figure('Name','SIH 26037 | Live CARLA autonomy','Color','w', ...
        'Position',[40 40 1350 850]);
    h.cameraAxes = subplot(2,2,1,'Parent',fig);
    h.camera = image(h.cameraAxes,zeros(480,640,3,'uint8'));
    axis(h.cameraAxes,'image'); axis(h.cameraAxes,'off');
    title(h.cameraAxes,'Live RGB | classes/poses from simulator metadata');
    h.mapAxes = subplot(2,2,2,'Parent',fig); hold(h.mapAxes,'on');
    plot(h.mapAxes,globalPath(:,1),globalPath(:,2),'k:');
    h.driven = plot(h.mapAxes,NaN,NaN,'b-','LineWidth',2);
    h.candidates = gobjects(1,15);
    for i = 1:15; h.candidates(i) = plot(h.mapAxes,NaN,NaN,'Color',[.75 .75 .75]); end
    h.selected = plot(h.mapAxes,NaN,NaN,'g-','LineWidth',2);
    h.tracks = scatter(h.mapAxes,NaN,NaN,25,'k','filled');
    h.predictions = gobjects(1,12); h.labels = gobjects(1,12);
    for i = 1:12
        h.predictions(i) = plot(h.mapAxes,NaN,NaN,'m--');
        h.labels(i) = text(h.mapAxes,NaN,NaN,'','FontSize',7);
    end
    h.ego = scatter(h.mapAxes,NaN,NaN,70,'b','filled');
    axis(h.mapAxes,'equal'); grid(h.mapAxes,'on');
    xlabel(h.mapAxes,'Project world x (m)'); ylabel(h.mapAxes,'Project world y (m)');
    title(h.mapAxes,'Candidates | green selected | magenta predictions (4 s)');
    h.textAxes = subplot(2,2,3,'Parent',fig); axis(h.textAxes,'off');
    h.telemetry = text(h.textAxes,0,1,'Starting sensors...', ...
        'VerticalAlignment','top','FontSize',10,'Interpreter','none');
    h.lidarAxes = subplot(2,2,4,'Parent',fig);
    h.lidar = scatter(h.lidarAxes,NaN,NaN,2,NaN);
    axis(h.lidarAxes,'equal'); grid(h.lidarAxes,'on');
    xlabel(h.lidarAxes,'LiDAR local x (m)'); ylabel(h.lidarAxes,'LiDAR local y (m)');
    fig.UserData = h; drawnow;
end
if isempty(r); return; end
h = fig.UserData;
set(h.camera,'CData',r.obs.cameraFrame.image);
set(h.driven,'XData',driven(:,1),'YData',driven(:,2));
for i = 1:numel(h.candidates)
    p = [NaN NaN]; color = [.75 .75 .75];
    if i<=numel(r.candidateTrajectories)
        p = r.candidateTrajectories{i};
        if r.candidateColliding(i); color = [.9 .5 .5]; end
    end
    set(h.candidates(i),'XData',p(:,1),'YData',p(:,2),'Color',color);
end
p = r.selectedTrajectory; set(h.selected,'XData',p(:,1),'YData',p(:,2));
positions = reshape([r.trackedAgents.position],2,[])';
set(h.tracks,'XData',positions(:,1),'YData',positions(:,2));
for i = 1:12
    p = [NaN NaN]; label = ''; position = [NaN NaN];
    if i<=numel(r.trackedAgents)
        a = r.trackedAgents(i); p = r.predictedTrajectories{i}; position = a.position;
        label = sprintf(' %d %s %.1fm/s %s',a.id,a.class,norm(a.velocity),r.behaviorInfo(i).label);
    end
    set(h.predictions(i),'XData',p(:,1),'YData',p(:,2));
    set(h.labels(i),'Position',[position 0],'String',label);
end
set(h.ego,'XData',r.egoState.x,'YData',r.egoState.y);
e = r.egoState; c = r.controlCommand;
proximity = Inf;
if ~isempty(positions); proximity = min(vecnorm(positions-[e.x e.y],2,2)); end
lines = {sprintf('Decision: %s | TTC: %g s | centre proximity: %.1f m',upper(r.decisionState),r.minTTC,proximity), ...
    sprintf('Speed %.2f m/s | steer %.1f deg | heading %.1f deg',e.velocity,rad2deg(c.steeringAngle),rad2deg(e.yaw)), ...
    sprintf('Throttle %.2f | brake %.2f | position (%.2f, %.2f)',c.throttle,c.brake,e.x,e.y), ...
    sprintf('Goal %.1f m | physical collision events %d',r.goalDistance,collisions), ...
    sprintf('Fused %d | tracked %d | predicted %d',numel(r.fusedAgents),numel(r.trackedAgents),numel(r.predictedTrajectories)), ...
    sprintf('Candidates %d | feasible %d | geometric flag %d',r.totalCandidates,r.feasibleCandidateCount,r.isColliding), ...
    sprintf('Loop interval %.3f s | planner %.3f s',r.dt,r.planningElapsedS), ...
    'Signals: infrastructure only. Async timing; no real-time claim.', ...
    'Phase 14 contact robustness remains unresolved.'};
set(h.telemetry,'String',strjoin(lines,newline));
xyz = r.obs.lidarPoints.xyzi;
set(h.lidar,'XData',xyz(:,1),'YData',xyz(:,2),'CData',xyz(:,3));
s = r.obs.status;
title(h.lidarAxes,sprintf('RGB %s | LiDAR %s | radar %s (%d)', ...
    s.camera.state,s.lidar.state,s.radar.state,r.obs.radarDetections.numDetections));
drawnow limitrate;
end
