function cfg = carlaIndianSceneConfig()
% carlaIndianSceneConfig - Phase 11.5: deterministic scene definition for
% the Indian urban hero environment. Same struct-config style as
% carlaConfig.m/carlaPerceptionConfig.m/carlaTrackingConfig.m - every
% coordinate below was LIVE-RESOLVED against the real map's road network
% (not hand-guessed), then hardcoded here for reproducibility, matching
% carlaConfig.m's own "verified live, then hardcoded" convention.
%
% ---------------------------------------------------------------------
% MAP / INTERSECTION SELECTION (why this map, why this junction)
% ---------------------------------------------------------------------
% CARLA 0.9.16's stock map list (confirmed live via
% client.get_available_maps()): Town01/02/03/04/05 (+ _Opt variants) and
% Town10HD(_Opt). None are authored as Indian roads - there is no stock
% CARLA asset for that. Town03 was selected because it has the largest,
% most substantial junctions of any stock town (confirmed live via
% carla.Map.get_topology()/get_junction().bounding_box - Town03's
% junction id=103 has a ~40m bounding extent vs Town01's largest at
% ~23m), which is what "sufficiently large for genuine driving" and real
% turning radii actually requires - geometry, not paint style, was the
% deciding factor, since the Indian visual character is built entirely
% through the actor/prop manifest below regardless of which town it
% sits on.
%
% Junction id=103, center (1.10, 133.72) in Town03's world frame, was
% confirmed to have 4 genuinely separated approach directions (bearings
% from center: -124.8 deg (S), 178.1 deg (W), 86.3 deg (N), -8.3 deg (E)
% - not a roundabout, not a 3-way skew). Each approach's road waypoint
% was walked backward along the REAL road spline via CARLA's
% waypoint.previous(distance) API (follows actual curvature, not a
% straight-line approximation) to place every actor below.
%
% ---------------------------------------------------------------------
% NO LANE MARKINGS - HONEST LIMITATION STATEMENT (do not skip this)
% ---------------------------------------------------------------------
% CARLA 0.9.16's road lane markings are baked into the road mesh/material
% of every stock town - confirmed live that carla.MapLayer (the only
% runtime layer-toggle mechanism CARLA 0.9.16 exposes) has NO
% "RoadMarkings" entry (its members are exactly: NONE, Buildings, Decals,
% Foliage, Ground, ParkedVehicles, Particles, Props, StreetLights, Walls,
% All). There is therefore NO CARLA 0.9.16 Python API call that removes
% painted lane markings from an existing town's road surface - this
% would require a custom OpenDRIVE/RoadRunner map, out of scope here.
% This requirement's actual intent - that the autonomy stack must not
% depend on lane markings - IS fully satisfied: no file anywhere in
% perception/, prediction/, planning/, decision/, or control/ reads,
% assumes, or requires lane geometry of any kind (confirmed across
% Phases 1-12's entire architecture). The visual texture itself cannot
% be scrubbed from Town03's stock road mesh; this is stated plainly
% rather than claimed away.
%
% ---------------------------------------------------------------------
% TRAFFIC LIGHTS - unsignalized by construction
% ---------------------------------------------------------------------
% carlaFreezeTrafficLights.m freezes every real CARLA traffic light near
% the junction to a fixed, non-cycling amber state (visible
% infrastructure, never an active red/green cycle). This is redundant
% with, not a substitute for, the deeper reason the intersection is
% unsignalized: decision/behaviorDecision.m and decisionStateMachine.m
% (frozen) have no code path anywhere that queries CARLA traffic-light
% state - right-of-way in this project has only ever been decided by
% perception + prediction + TTC + collision risk, for every phase since
% Phase 1. Freezing the lights only prevents a jury from seeing a
% visually misleading cycling signal; it changes nothing about how any
% decision is made.
%
% Schema:
%   cfg.mapName                 CARLA map to load for this scene
%   cfg.junctionCenter           [x,y] world coords of the intersection
%   cfg.egoBlueprint, cfg.egoApproach  ego spawn (Phase 11.5 stages the
%                                ego at its approach point only - the
%                                actual turn maneuver is Phase 14's job)
%   cfg.trafficActors            struct array, one entry per moving
%                                traffic actor: .blueprint, .x/.y/.z/
%                                .yawDeg, .approach, .intent (free-text
%                                label, not consumed by any algorithm -
%                                for carlaIndianSceneTrafficStep.m's
%                                scripted, non-lane-based motion)
%   cfg.parkedVehicles            struct array: .blueprint, .x/.y/.z/.yawDeg
%   cfg.pedestrians                struct array: .x/.y/.z/.yawDeg,
%                                 .behavior ("roadside_walk" |
%                                 "crossing"), .velocityCarla [vx,vy]
%   cfg.roadDefectProps            struct array: .blueprint, .x/.y/.z,
%                                 pothole/road-defect visual markers -
%                                 see header note above on why these are
%                                 visual props, not deformed terrain
%   cfg.clutterProps                struct array: .blueprint, .x/.y/.z,
%                                 .yawDeg - roadside/Indian-flavor static
%                                 props (see honesty note: generic CARLA
%                                 assets used as the closest available
%                                 approximation, not fabricated as
%                                 India-specific)
%   cfg.trafficLightFreezeRangeM   passed to carlaFreezeTrafficLights.m

cfg = struct();
cfg.mapName = 'Town03';
cfg.junctionCenter = [1.10, 133.72];
cfg.trafficLightFreezeRangeM = 80.0;

% ---------------------------------------------------------------------
% EGO - staged at the West approach, 45m back, facing east into the
% junction (yaw=-1.30 deg, live-resolved). Turning itself is Phase 14.
% ---------------------------------------------------------------------
cfg.egoBlueprint = 'vehicle.tesla.model3';
cfg.egoApproach = struct('x', -63.90, 'y', 135.42, 'z', 0.30, 'yawDeg', -1.30, 'approach', "W");

% ---------------------------------------------------------------------
% MOVING TRAFFIC ACTORS (9 - within the 8-10 target), heterogeneous,
% approaching from all 4 directions. "intent" is a free-text label read
% only by carlaIndianSceneTrafficStep.m's scripted per-tick nudging
% (never CARLA autopilot/Traffic Manager) - it has no effect on, and is
% never read by, any frozen algorithm.
% ---------------------------------------------------------------------
cfg.trafficActors = struct( ...
    'blueprint', {}, 'x', {}, 'y', {}, 'z', {}, 'yawDeg', {}, 'approach', {}, 'intent', {});

% South approach (entering north-bound toward the junction, yaw=89.64)
cfg.trafficActors(end+1) = struct('blueprint','vehicle.tesla.model3',              'x', -9.69,'y',103.35,'z',0.30,'yawDeg', 89.64,'approach',"S",'intent',"straight_through");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.harley-davidson.low_rider', 'x', -9.78,'y', 88.35,'z',0.30,'yawDeg', 89.64,'approach',"S",'intent',"turn_left");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.diamondback.century',       'x', -9.64,'y',110.35,'z',0.30,'yawDeg', 89.64,'approach',"S",'intent',"roadside_edge");

% North approach (entering south-bound, yaw=269.64/-90.36)
cfg.trafficActors(end+1) = struct('blueprint','vehicle.carlamotors.carlacola',     'x',  2.20,'y',164.11,'z',0.30,'yawDeg',-90.36,'approach',"N",'intent',"slow_through");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.bh.crossbike',              'x',  2.29,'y',179.11,'z',0.30,'yawDeg',-90.36,'approach',"N",'intent',"roadside_edge");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.audi.tt',                   'x',  2.15,'y',157.11,'z',0.30,'yawDeg',-90.36,'approach',"N",'intent',"turn_right");

% East approach (entering west-bound, yaw=179.18)
cfg.trafficActors(end+1) = struct('blueprint','vehicle.mitsubishi.fusorosa',       'x', 36.19,'y',130.58,'z',0.30,'yawDeg',179.18,'approach',"E",'intent',"straight_through");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.vespa.zx125',               'x', 51.19,'y',130.36,'z',0.30,'yawDeg',179.18,'approach',"E",'intent',"informal_merge");
cfg.trafficActors(end+1) = struct('blueprint','vehicle.nissan.micra',              'x', 29.19,'y',130.68,'z',0.30,'yawDeg',179.18,'approach',"E",'intent',"turn_right");

% West approach (same side as ego, staged closer in - following/
% overtaking interaction near the ego)
cfg.trafficActors(end+1) = struct('blueprint','vehicle.yamaha.yzf',                'x', -48.91,'y',135.08,'z',0.30,'yawDeg', -1.30,'approach',"W",'intent',"following_ego_lane");

% ---------------------------------------------------------------------
% PARKED VEHICLES (5) - stationary roadside clutter, offset ~4.5m
% perpendicular from each approach's lane centerline (live-resolved
% perpendicular direction, not a hand-guessed offset).
% ---------------------------------------------------------------------
cfg.parkedVehicles = struct('blueprint', {}, 'x', {}, 'y', {}, 'z', {}, 'yawDeg', {});
% Phase 14.5 scene-authoring correction (measured, not cosmetic): this car
% was at x=-5.19, which put its CENTRE 1.88m from the hero turn route's
% centreline. Its half-width is 0.93m and the ego's is ~1.0m, so 1.93m is
% the minimum non-contact separation - the parked car was ~5cm INSIDE the
% ego's swept corridor, and forensics recorded the ego scraping it (actor
% id 120, 50 collision events, t=95.5-96.2s). Moved 2.2m further onto the
% roadside; it remains a realistic kerbside parked vehicle on the same
% road with the same heading, it is simply no longer parked inside the
% driving lane.
cfg.parkedVehicles(end+1) = struct('blueprint','vehicle.citroen.c3',           'x', -2.99,'y',103.32,'z',0.30,'yawDeg', 89.64);
cfg.parkedVehicles(end+1) = struct('blueprint','vehicle.mini.cooper_s_2021',   'x',-14.25,'y', 93.38,'z',0.30,'yawDeg', 89.64);
cfg.parkedVehicles(end+1) = struct('blueprint','vehicle.seat.leon',            'x',-34.01,'y',130.24,'z',0.30,'yawDeg', -1.30);
cfg.parkedVehicles(end+1) = struct('blueprint','vehicle.jeep.wrangler_rubicon','x', 36.13,'y',126.08,'z',0.30,'yawDeg',179.18);
cfg.parkedVehicles(end+1) = struct('blueprint','vehicle.chevrolet.impala',     'x', -2.24,'y',174.14,'z',0.30,'yawDeg',269.64);

% ---------------------------------------------------------------------
% PEDESTRIANS (2) - one roadside walker, one crossing near the ego's
% approach. velocityCarla is applied every tick by
% carlaIndianSceneTrafficStep.m via carlaSetActorTargetVelocity.m's
% WalkerControl path (Phase 12 fix - set_target_velocity does not move a
% CARLA walker).
% ---------------------------------------------------------------------
cfg.pedestrians = struct('x', {}, 'y', {}, 'z', {}, 'yawDeg', {}, 'behavior', {}, 'velocityCarla', {});
cfg.pedestrians(end+1) = struct('x', -13.19,'y',105.38,'z',0.50,'yawDeg',  0.0,'behavior',"roadside_walk", 'velocityCarla',[0.0, 0.8]); % nudged 2m from the original (-14.19,103.38) - that exact point collided with nearby road/prop geometry on spawn (confirmed live)
cfg.pedestrians(end+1) = struct('x', -33.81,'y',139.24,'z',0.30,'yawDeg',-90.0,'behavior',"crossing",      'velocityCarla',[0.0,-1.2]);

% ---------------------------------------------------------------------
% ROAD DEFECT ("pothole") PROPS - visual approximation only, per the
% explicit instruction that a reproducible visual approximation is
% acceptable and vehicle physics must not be compromised. Built from
% static.prop.brokentile0{1-4} clusters (broken road-surface debris),
% each ringed by a single static.prop.trafficcone01 for real-world-style
% visual read-ability (a common real way road defects get marked).
% ---------------------------------------------------------------------
cfg.roadDefectProps = struct('blueprint', {}, 'x', {}, 'y', {}, 'z', {}, 'yawDeg', {});
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.brokentile01', 'x', -20.0,'y',132.5,'z',0.05,'yawDeg',  0.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.brokentile02', 'x', -20.6,'y',133.3,'z',0.05,'yawDeg', 40.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.trafficcone01','x', -19.2,'y',131.8,'z',0.05,'yawDeg',  0.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.brokentile03', 'x',  40.0,'y',131.0,'z',0.05,'yawDeg', 15.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.brokentile04', 'x',  40.6,'y',130.4,'z',0.05,'yawDeg',-25.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.trafficcone02','x',  39.4,'y',131.6,'z',0.05,'yawDeg',  0.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.brokentile01', 'x',  -9.9,'y', 98.0,'z',0.05,'yawDeg', 60.0);
cfg.roadDefectProps(end+1) = struct('blueprint','static.prop.dirtdebris02', 'x',  -9.5,'y', 97.3,'z',0.05,'yawDeg',  0.0);

% ---------------------------------------------------------------------
% ROADSIDE CLUTTER / INDIAN-FLAVOR PROPS - generic CARLA static props
% used as the closest available approximation (coconut palms are a
% genuine visual match for South Asian streetscapes; food carts/plastic
% chairs/tables approximate an informal roadside stall; the rest is
% ordinary urban clutter that reads as "informal roadside", not
% presented as India-specific where it plainly isn't).
% ---------------------------------------------------------------------
cfg.clutterProps = struct('blueprint', {}, 'x', {}, 'y', {}, 'z', {}, 'yawDeg', {});
cfg.clutterProps(end+1) = struct('blueprint','static.prop.foodcart',      'x',-16.5,'y',140.0,'z',0.10,'yawDeg', 90.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.plasticchair',  'x',-15.5,'y',139.5,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.plastictable',  'x',-16.0,'y',138.5,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.coconutpalm',   'x',-11.0,'y',137.0,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.coconutpalm',   'x', 24.0,'y',126.0,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.coconutpalm',   'x',  6.0,'y',167.0,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.trashcan01',    'x', -6.0,'y',106.0,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.garbage02',     'x',  8.5,'y',159.0,'z',0.10,'yawDeg',  0.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.streetsign',    'x', 40.5,'y',134.5,'z',0.10,'yawDeg',180.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.advertisement', 'x',-40.0,'y',140.0,'z',0.10,'yawDeg', 90.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.bench01',       'x',-30.0,'y',126.0,'z',0.10,'yawDeg', 90.0);
cfg.clutterProps(end+1) = struct('blueprint','static.prop.chainbarrier',  'x', 25.0,'y',135.5,'z',0.10,'yawDeg', 90.0);

end
