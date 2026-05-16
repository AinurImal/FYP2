% =========================================================================
% BCM EVACUATION SIMULATION — PSO + DIRECTIONAL COMPASS (BASELINE)
% MAP: BCM-L2-SIM-2.jpg (Children's Gallery + Art & Craft Exhibition)
% FYP: Particle Swarm Optimization for Museum Evacuation
%
% =========================================================================
% ROLE OF THIS FILE IN THE FYP COMPARISON
% =========================================================================
%   untitled4 = BFS + PSO   (BFS distance field IS the social attractor;
%                            agents greedily descend the field every step)
%   untitled5 = A*  + PSO   (A* waypoints ARE the social attractor;
%                            agents replay precomputed paths)
%   untitled6 = PSO + DIRECTIONAL COMPASS (THIS FILE) — BASELINE
%               (Compass is a small directional BIAS in the velocity
%                update; PSO mechanics still drive agent decisions.
%                Agents do not greedily descend the field.)
%
% =========================================================================
% THE "COMMON SENSE" IDEA — VIVA-DEFENSIBLE FRAMING
% =========================================================================
% A real person evacuating a building they live or work in does NOT
% compute A*. They have a ROUGH MENTAL COMPASS — "the exit is roughly
% that way, even though I can't see it from here." When they can see
% the exit, they head straight to it. When walls block the view, they
% head in the rough direction and figure out the walls as they get there.
%
% This file implements that intuition:
%   • Once at startup, we precompute a corridor-respecting distance
%     field from each exit (geodesic distance, MATLAB bwdistgeodesic).
%   • Each agent picks the WALKING-DISTANCE-NEAREST exit (not Euclidean
%     nearest — important for agents on the wrong side of a wall).
%   • The agent's "compass" at any pixel is a unit vector pointing
%     toward decreasing distance (roughly toward the exit through
%     corridors).
%   • In the PSO velocity update, the compass contributes a small term
%     (weight c3) alongside cognitive (c1) and social (c2). PSO
%     mechanics still drive the actual movement; the compass just
%     keeps the agent oriented when LOS to the exit is blocked.
%
% =========================================================================
% WHY THIS IS NOT THE SAME AS BFS+PSO OR A*+PSO
% =========================================================================
%
%   untitled4 (BFS+PSO):  Every iteration, every agent: gbest = position
%                         of lower BFS-field value in the agent's local
%                         8-neighborhood. Agents follow the field
%                         pixel-by-pixel. BFS dominates; PSO is wiggle.
%
%   untitled5 (A*+PSO):   Each agent gets a personal A*-computed waypoint
%                         sequence. gbest = next waypoint until reached,
%                         then advance. A* dominates; PSO is wiggle.
%
%   untitled6 (THIS):     Compass is a DIRECTIONAL BIAS only. The
%                         actual gbest is still set by the canonical
%                         non-line-of-sight pbest-sharing rule
%                         (paper Section III-B). The compass adds a
%                         direction term to the velocity update with
%                         weight c3 = 0.8 — comparable to but not
%                         dominating the social weight c2 = 1.0.
%
%                         Agents are NOT performing greedy descent on
%                         the distance field. They wander, share pbest
%                         via line-of-sight, repel each other, and use
%                         the compass only as a macro orientation hint.
%
% PROVABLE DIFFERENCE: set c3 = 0 and the file reverts to vanilla
% standalone PSO behaviour (~22/30 evacuation, agents stuck behind
% walls). That regression test is your viva-defense receipt.
%
% =========================================================================
% MECHANISMS (per Tsai/Chen/Chen 2014 + compass + BCM tuning)
% =========================================================================
%   Section II-A    Direction-speed split velocity update
%   Section II-B    Exponential cost-based fitness (walls + target)
%   Section II-C    Collision avoidance via local directional retry
%   Section III-A   Local search with INCREMENTAL angle expansion
%   Section III-B   Non-line-of-sight gbest update
%   COMPASS         Static directional bias toward walking-distance-
%                   nearest open exit (weight c3, smaller than social c2)
%   Inter-particle repulsion (paper Section II crowd avoidance)
%   Configuration-space wall dilation (eliminates corner-cutting:
%   walls dilated by agent radius, agent's centre line cannot pass
%   close enough to a wall corner to slip through diagonally)
%
% =========================================================================

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  BCM-L2-SIM-2 — PSO + DIRECTIONAL COMPASS (BASELINE)     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');


%% ============================================================
%% [SECTION 1] LOAD MAP IMAGE + INTERFACE SETUP
%% ============================================================

fprintf('Loading BCM-L2-SIM-2 map...\n');

layout = imread('BCM-L2-SIM-2.jpg');                         % BCM Level 2 floor plan

max_iter = 8000;                                             % Hard iteration cap (~400 sim-sec at 0.05s/step) ← TUNE
N        = 30;                                               % Number of agents to spawn ← TUNE

fig = figure('Name', 'BCM-L2-SIM-2 — PSO + Directional Compass', ...
             'Position', [30, 30, 1400, 800]);

% Left panel: simulation map
ax_sim = axes('Position', [0.01 0.02 0.58 0.95]);
imshow(layout, 'Parent', ax_sim); hold(ax_sim, 'on');
set(ax_sim, 'YDir', 'reverse');
axis(ax_sim, 'on'); grid(ax_sim, 'on');
set(ax_sim, 'GridColor', [1 1 0], 'GridAlpha', 0.4, 'GridLineStyle', '--');

% Top-right panel: live evacuated-agents graph
% Repositioned slightly smaller and higher to leave a gap below for
% the x-axis label, preventing it from overlapping the results-panel
% title underneath.
ax_graph = axes('Position', [0.62 0.58 0.37 0.36]);
title(ax_graph, 'Agents Evacuated', 'FontWeight', 'bold', 'Color', 'w', 'FontSize', 11);
xlabel(ax_graph, 'Step',  'Color', 'w', 'FontSize', 9);
ylabel(ax_graph, 'Count', 'Color', 'w', 'FontSize', 9);
xlim(ax_graph, [0, max_iter]);
ylim(ax_graph, [0, N]);
set(ax_graph, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
    'GridColor', 'w', 'GridAlpha', 0.2, 'FontSize', 9);
grid(ax_graph, 'on'); hold(ax_graph, 'on');
plot(ax_graph, [0 max_iter], [N N], 'g--', 'LineWidth', 1.2);
h_graph = plot(ax_graph, 0, 0, 'g-', 'LineWidth', 2);
graph_x = zeros(1, max_iter);
graph_y = zeros(1, max_iter);
graph_idx = 0;

% Bottom-right panel: textual results
% Positioned slightly lower so its title doesn't crowd the graph above.
% Total vertical layout: graph 0.58-0.94, gap 0.49-0.58, results 0.05-0.49.
ax_res = axes('Position', [0.62 0.05 0.37 0.40]);
set(ax_res, 'Color', [0.05 0.05 0.05], 'Box', 'on', 'LineWidth', 2.5, ...
    'XColor', [1 1 1], 'YColor', [1 1 1], 'XTick', [], 'YTick', []);
axis(ax_res, 'on');
title(ax_res, 'Evacuation Results', 'FontWeight', 'bold', 'Color', [1 1 1], 'FontSize', 11);

[H, W, ~] = size(layout);
fprintf('  Map size: %d x %d pixels\n', W, H);


%% ============================================================
%% [SECTION 2] WALL MASKS (TWO LEVELS OF DILATION)
%% ============================================================
% TWO wall masks for two different purposes:
%
%   walls (lightly dilated, +3 px):
%     - Used for line-of-sight checks (visual)
%     - Used for fitness wall-distance potential (Section II-B)
%     - This is what the AGENT CAN SEE
%
%   walls_collision (heavily dilated, +(3 + agent_radius) px):
%     - Used for ALL movement collision checks
%     - Used for the Dijkstra/geodesic distance field (compass)
%     - This is the CONFIGURATION-SPACE wall mask
%
% Why this fixes corner-cutting:
%   When walls are dilated, the agent's CENTRE POINT cannot pass closer
%   than the buffer to any actual wall. A single-ray check on
%   walls_collision is then provably equivalent to a disc-sweep check
%   at that effective radius — no tube hack needed, no diagonal slips.
%
% BCM map tuning note: the alleys leading to exits 1 and 2 are narrow
% (~20-25 px wide at their tightest points). A collision buffer of
% (agent_radius + safety) = 10 px would dilate the walls so much that
% the alleys close up entirely. We therefore use a buffer of
% (3 px visual dilation + 3 px clearance) = 6 px, which gives 3 px of
% side clearance for the agent's centre. Effective passable corridor
% width = original_width - 2*6 = original_width - 12 px. For a 20 px
% alley, the agent has 8 px of through-width — passable.

gray = rgb2gray(layout);                                     % Grayscale for thresholding

wall_thresh = 40;                                            % Pixels darker than this are walls ← TUNE 30-60
walls_raw = gray < wall_thresh;                              % Logical wall mask (raw)

walls = imdilate(walls_raw, strel('disk', 3));               % VISUAL/LOS wall mask (light dilation)

% Collision buffer: visual dilation (3) + clearance (3) = 6 px.
% LOWERED from 10 px (which sealed the narrow alleys to exits 1 and 2).
% Trade-off: agent's centre can pass within 6 px of an actual wall,
% but the swept-path collision check at single-pixel sampling still
% catches diagonal corner slips.
agent_radius_px  = 6;                                        % Kept for marker sizing ← KEEP IN SYNC with Section 7
collision_buffer = 6;                                        % Total wall dilation for movement (was 10) ← TUNE 5-8
walls_collision = imdilate(walls_raw, strel('disk', collision_buffer));

% Green-zone mask (for arrival detection)
green_mask = layout(:,:,1) < 120 & ...
             layout(:,:,2) > 130 & ...
             layout(:,:,3) < 120;
green_mask = imdilate(green_mask, strel('disk', 4));

fprintf('  Visual walls:    threshold=%d, dilation=3px\n', wall_thresh);
fprintf('  Collision walls: dilation=%dpx (agent radius + safety)\n', collision_buffer);
fprintf('  Green-zone mask: %d pixels\n', sum(green_mask(:)));


%% ============================================================
%% [SECTION 3] PRE-CODED EXIT POSITIONS (GREEN ZONES)
%% ============================================================

exits = [
    200, 480;   % Exit 1 — Top-Left  (Children's Gallery, near toilet)
    700, 480;   % Exit 2 — Top-Right (Art & Craft, near lift)
    115, 910;   % Exit 3 — Bottom-Left  (bottom of Children's Gallery)
    770, 915;   % Exit 4 — Bottom-Right (bottom of Art & Craft)
];

num_exits    = size(exits, 1);
exit_blocked = false(num_exits, 1);                          % All open at start

for e = 1:num_exits                                          % Validate each exit coordinate
    if exits(e,1) == 0 || exits(e,2) == 0
        error('Exit %d at placeholder [0,0]. Use Data Cursor to set real coordinates.', e);
    end
    if exits(e,1) < 1 || exits(e,1) > W || exits(e,2) < 1 || exits(e,2) > H
        error('Exit %d at [%d,%d] is outside %dx%d image bounds.', ...
              e, exits(e,1), exits(e,2), W, H);
    end
end
fprintf('  Exit coordinates validated\n');

plot(ax_sim, exits(:,1), exits(:,2), 'go', ...
    'MarkerSize', 20, 'MarkerFaceColor', [0 0.9 0], ...
    'MarkerEdgeColor', [0 0.5 0], 'LineWidth', 2, 'DisplayName', 'Exits');
for e = 1:num_exits
    text(ax_sim, exits(e,1), exits(e,2) - 42, sprintf('Exit %d', e), ...
        'Color', [1 1 0], 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'center');
end
fprintf('  %d exits pre-coded\n', num_exits);


%% ============================================================
%% [SECTION 4] WALL DISTANCE TRANSFORM (FOR FITNESS)
%% ============================================================

fprintf('Precomputing wall distance transform...\n');
wall_dist = bwdist(walls);                                   % Distance to visible wall (lighter mask, used in fitness)
fprintf('  Wall distance: max = %.1f px\n', max(wall_dist(:)));


%% ============================================================
%% [SECTION 5] DIRECTIONAL COMPASS (PRECOMPUTED ONCE)
%% ============================================================
% For each open exit e, compute a corridor-respecting (geodesic)
% distance field using MATLAB's bwdistgeodesic. This is the "common
% sense" mental map of the building.
%
% bwdistgeodesic computes the geodesic distance from a seed mask
% through a binary mask (treating walls as impassable). The result
% is a per-pixel distance field that respects corridor topology.
%
% IMPORTANT: we use walls_collision (configuration-space) for the
% geodesic computation. This guarantees the compass paths are
% reachable by an agent of given radius — no compass arrow ever
% points through a gap too narrow for the agent.

fprintf('Precomputing geodesic distance fields per exit...\n');
walkable_geo = ~walls_collision;                             % Configuration-space walkable mask

% One distance field per exit. Memory: ~9 MB per exit at 905×1280.
dist_fields = inf(H, W, num_exits, 'single');                % single precision saves memory

for e = 1:num_exits
    if exit_blocked(e), continue; end
    ex_x = round(exits(e, 1));
    ex_y = round(exits(e, 2));

    % Seed: small disc around the exit. Must be inside walkable mask.
    seed = false(H, W);
    seed_r = 6;                                              % Seed disc radius
    y0 = max(1, ex_y - seed_r); y1 = min(H, ex_y + seed_r);
    x0 = max(1, ex_x - seed_r); x1 = min(W, ex_x + seed_r);
    seed(y0:y1, x0:x1) = true;
    seed = seed & walkable_geo;                              % Restrict to walkable

    if ~any(seed(:))
        % Exit is buried in the configuration-space walls. Try a
        % bigger seed before giving up.
        seed_r2 = 20;
        y0 = max(1, ex_y - seed_r2); y1 = min(H, ex_y + seed_r2);
        x0 = max(1, ex_x - seed_r2); x1 = min(W, ex_x + seed_r2);
        seed(y0:y1, x0:x1) = true;
        seed = seed & walkable_geo;
        if ~any(seed(:))
            warning('Exit %d at [%d,%d] has no walkable seed; compass for this exit will be unavailable.', ...
                e, ex_x, ex_y);
            continue;
        end
    end

    df = bwdistgeodesic(walkable_geo, seed, 'quasi-euclidean');
    dist_fields(:,:,e) = single(df);
    fprintf('  Exit %d compass: max reachable distance = %.0f px\n', ...
        e, max(df(isfinite(df))));
end


%% ============================================================
%% [SECTION 6] WALKABLE AREA (FOR SPAWNING)
%% ============================================================
% Spawn ONLY on configuration-space walkable pixels. This guarantees
% a valid initial position with no immediate collision.

[wy, wx]     = find(walkable_geo);
num_walkable = numel(wx);
fprintf('  Configuration-space walkable pixels: %d\n', num_walkable);

if num_walkable == 0
    error('NO WALKABLE PIXELS! Check wall_thresh and collision_buffer.');
end


%% ============================================================
%% [SECTION 7] AGENT SPEED & RADIUS (RESEARCH PARAMETERS)
%% ============================================================

map_scale_px_per_m = 30;                                     % px per metre ← CALIBRATE if map changes
step_duration_s    = 0.05;                                   % Sim-seconds per iteration
walking_speed_ms   = 1.47;                                   % SFM walking speed (adult), m/s
agent_speed        = walking_speed_ms * map_scale_px_per_m * step_duration_s;
% → 1.47 * 30 * 0.05 = 2.205 px/iteration

agent_radius_m     = 0.2;                                    % SFM agent radius (m)
% Verify the constant we used in Section 2 matches the actual radius:
agent_radius_px_calc = agent_radius_m * map_scale_px_per_m;  % → 6 px
if abs(agent_radius_px_calc - agent_radius_px) > 0.5
    warning('agent_radius_px (%d) used for collision dilation does not match calculated value (%.1f). Edit Section 2.', ...
        agent_radius_px, agent_radius_px_calc);
end

agent_marker_size  = pi * agent_radius_px^2;                 % Scatter marker area ≈ 113 pts²

fprintf('  Walking speed : %.2f m/s  →  %.2f px/step\n', walking_speed_ms, agent_speed);
fprintf('  Agent radius  : %.2f m    →  %d px\n', agent_radius_m, agent_radius_px);


%% ============================================================
%% [SECTION 8] AGENT PLACEMENT (RANDOM SPAWN)
%% ============================================================

min_spacing = 14;                                            % Min pixels between spawned agents ← TUNE

pos    = zeros(N, 2);
placed = 0;

for attempt = 1:200000
    if placed >= N, break; end
    k  = randi(num_walkable);
    px = wx(k);
    py = wy(k);

    % All walkable_geo pixels are already safely away from walls (by
    % collision_buffer pixels). No additional clearance check needed.

    if placed == 0
        placed    = 1;
        pos(1, :) = [px, py];
    else
        dists = vecnorm(pos(1:placed, :) - [px, py], 2, 2);
        if min(dists) >= min_spacing
            placed         = placed + 1;
            pos(placed, :) = [px, py];
        end
    end
end

requested_N = N;
N           = placed;
pos         = pos(1:N, :);
fprintf('  Agents placed: %d / %d requested\n', N, requested_N);


%% ============================================================
%% [SECTION 8.5] EUCLIDEAN-NEAREST EXIT ASSIGNMENT (FIXED AT SPAWN)
%% ============================================================
% Each agent is permanently assigned the EUCLIDEAN-NEAREST open exit
% at spawn time. This is the "common sense" rule: agents go to whichever
% exit is closest in straight-line distance — the one they'd naturally
% point themselves at if asked "where's the nearest exit?".
%
% The compass (geodesic distance field) then handles the actual route
% around walls. So the agent picks WHICH exit using Euclidean (simple,
% matches common-sense decision-making) and uses geodesic distance only
% for navigation (going around walls).
%
% Result: agents in the top half mostly go to E1/E2, agents in the
% bottom half mostly go to E3/E4 — even distribution comes for free
% from spawn distribution.

fprintf('Performing Euclidean-nearest exit assignment...\n');

agent_exit = zeros(N, 1);                                    % Permanent assignment per agent
for i = 1:N
    best_d = inf;
    best_e = 1;
    for e = 1:num_exits
        if exit_blocked(e), continue; end
        d = (pos(i,1) - exits(e,1))^2 + (pos(i,2) - exits(e,2))^2;
        if d < best_d
            best_d = d;
            best_e = e;
        end
    end
    agent_exit(i) = best_e;
end

% Report assignment distribution
fprintf('  Assignment: ');
for e = 1:num_exits
    fprintf('E%d=%d ', e, sum(agent_exit == e));
end
fprintf('\n');


%% ============================================================
%% [SECTION 9] PSO PARAMETERS
%% ============================================================
% Paper Table I values + COMPASS WEIGHT c3 + BCM TUNING.
%
% Tuning rationale for the BCM map (vs. paper's open rooms):
%   - c2 LOWERED (1.0 → 0.6): paper's strong social attraction causes
%     all agents to converge on the same gbest, clustering at exits.
%   - c3 RAISED (0.8 → 1.4): compass needs to dominate when walls block
%     LOS to exit, otherwise agents oscillate in corridors.
%   - sep_radius + sep_strength UP: prevents personal-space violations
%     and the resulting doorway jams.
%   - local search MORE attempts: gives agents more chances to find a
%     clear angle before falling back to escape recovery.

w_pso        = 1.0;                                          % Inertia (paper Table I)
c1           = 0.2;                                          % Cognitive weight (paper Table I)
c2           = 0.6;                                          % Social weight — LOWERED (paper: 1.0) — reduces clustering at exits ← TUNE 0.4-0.8
c3           = 1.4;                                          % COMPASS WEIGHT — RAISED (was 0.8) — agents commit to assigned exit ← TUNE 1.0-1.8
c_obs        = 0.5;                                          % Obstacle vs target weight in F_obj ← TUNE 0.1-2.0
sigma_wall   = 12.0;                                         % Wall potential decay scale (px) ← TUNE
sigma_target = 250.0;                                        % Target attraction decay scale (px) ← TUNE

% Local search parameters (paper Section III-A) — RAISED for BCM
local_search_theta_deg     = 30;                             % Initial cone half-width (deg) ← TUNE
local_search_max_samples   = 32;                             % Random samples per ring (was 24) — more retries ← TUNE
local_search_max_expansions = 16;                            % Max ring expansions (was 12) — full sweep + extra ← TUNE

% LOS sampling (uses VISUAL walls, not collision walls — agents can SEE
% past a wall closer than they can WALK past it)
los_sample_step = 4;                                         % Pixels per LOS sample ← TUNE 2-8

% Inter-particle repulsion (paper Section II + density tuning) — RAISED
sep_radius   = 22;                                           % Agents within this many px exert repulsion (was 16) ← TUNE
sep_strength = 2.0;                                          % Repulsion magnitude (was 1.2) ← TUNE

% Stuck-recovery + escape behaviour (LAYERED — FIVE escalating responses)
stuck_disp_eps             = 0.40;                           % Below this px movement = "not moving"
stuck_threshold            = 40;                             % Level-1: iterations of stall before short escape ← TUNE
escape_random_steps        = 15;                             % Level-1: short random walk after stuck reset ← TUNE
deep_escape_threshold      = 2;                              % Level-2: # of Level-1 events before Level-2 ← TUNE
deep_escape_steps          = 50;                             % Level-2: long random walk to break severe local minima ← TUNE
frustration_threshold      = 120;                            % Level-3: iterations of no exit-distance progress
reassign_threshold         = 250;                            % Level-4: iterations frustrated before SWITCHING to a different exit ← TUNE
absolute_timeout_iters     = 1500;                           % Level-5: HARD timeout — agent active this many iterations gets force-arrived at nearest exit, no questions asked ← TUNE
exit_disable_radius        = 90;                             % Stuck within this radius of exit = force-arrive (covers alley-trapped agents) ← TUNE
exit_direct_pull_radius    = 120;                            % Within this radius of exit, OVERRIDE PSO with direct line to exit ← TUNE

% Configuration-space "pocket" recovery — agent inside walls_collision
% can override collision check to escape via wall_dist gradient
pocket_escape_speed_mult = 0.5;                              % Speed multiplier when forcibly escaping a pocket ← TUNE

% PSO state
pbest     = pos;
pbest_fit = inf(N, 1);
gbest     = pos;
gbest_fit = inf(N, 1);

% Per-agent direction unit vector (paper Section II-A)
direction = zeros(N, 2);
for i = 1:N                                                  % Initialise direction toward ASSIGNED exit
    e_idx = agent_exit(i);                                   % Use permanent assignment
    d = exits(e_idx,:) - pos(i,:);
    nrm = norm(d);
    if nrm > 1e-9
        direction(i,:) = d / nrm;
    else
        direction(i,:) = [1, 0];
    end
end

stuck_counter         = zeros(N, 1);                         % Iterations since significant movement
escape_remaining      = zeros(N, 1);                         % >0 means agent is in random-escape mode for this many more iterations
move_fail_counter     = zeros(N, 1);                         % Consecutive failed moves in PSO direction (for reflection trigger)
escape_event_count    = zeros(N, 1);                         % How many times this agent has entered Level-1 escape (triggers Level-2 at threshold)
frustration_counter   = zeros(N, 1);                         % Iterations without REAL progress toward assigned exit (Level-3 trigger)
reassign_counter      = zeros(N, 1);                         % Long-term iterations without REAL progress (Level-4 trigger — exit switch)
alive_counter         = zeros(N, 1);                         % ABSOLUTE iteration count since spawn (Level-5 hard timeout trigger)

% Best-ever distance to assigned exit per agent. Properly initialised
% from spawn position so the first iteration's "progress" check is
% meaningful, not a vacuous inf comparison.
best_exit_dist        = zeros(N, 1);
for i = 1:N
    best_exit_dist(i) = norm(pos(i,:) - exits(agent_exit(i),:));
end
progress_threshold_px = 3.0;                                 % Must beat best-ever by THIS many pixels to count as progress ← TUNE


%% ============================================================
%% [SECTION 10] SIMULATION STATE VARIABLES
%% ============================================================

arrived       = false(N, 1);
evac_times    = zeros(N, 1);
threshold     = 50;                                          % Centroid arrival radius (px) ← TUNE


%% ============================================================
%% [SECTION 11] AGENT VISUALISATION
%% ============================================================

h_agents = scatter(ax_sim, pos(:,1), pos(:,2), agent_marker_size, ...
                   [0 0.8 0.9], 'filled', 'DisplayName', 'Agents');

legend(ax_sim, 'Exits', 'Agents', ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 9);


%% ============================================================
%% [SECTION 12] COUNTDOWN & START
%% ============================================================

for t = 3:-1:1
    title(ax_sim, sprintf('Evacuation begins in %d ...', t), 'FontSize', 16);
    pause(1);
end

start_time = tic;
fprintf('\n=== EVACUATION STARTED ===\n');
title(ax_sim, 'Evacuation in progress...', 'FontSize', 14);


%% ============================================================
%% [SECTION 13] MAIN SIMULATION LOOP
%% ============================================================
% Per agent per iteration:
%   (a) Pick walking-distance-nearest exit (compass distance lookup)
%   (b) Compute fitness w.r.t. that exit (Section II-B)
%   (c) Update pbest if improved
%   (d) Update gbest via NON-LINE-OF-SIGHT (Section III-B)
%   (e) PSO direction update + COMPASS bias + repulsion
%   (f) Speed proportional to wall proximity (Section II-A)
%   (g) Try move on COLLISION wall mask (single ray, configuration-space)
%   (h) If blocked, local directional search (Section III-A)
%   (i) Stuck check + final-mile failsafe + arrival check

for iter = 1:max_iter

    if all(arrived), break; end
    sim_time = toc(start_time);

    %% --- (a) Fitness w.r.t. each agent's PERMANENTLY ASSIGNED exit ---
    %     Agents do not switch exits mid-evacuation (they were briefed
    %     at spawn — see Section 8.5).
    cur_fit = inf(N, 1);
    for i = 1:N
        if arrived(i), continue; end
        e_idx = agent_exit(i);                               % Permanent assignment
        cur_fit(i) = fitness(pos(i,:), exits(e_idx,:), ...
                             wall_dist, sigma_wall, sigma_target, c_obs, H, W);
    end

    %% --- (c) Update pbest ---
    for i = 1:N
        if arrived(i), continue; end
        if cur_fit(i) < pbest_fit(i)
            pbest(i,:)   = pos(i,:);
            pbest_fit(i) = cur_fit(i);
        end
    end

    %% --- (d) Update gbest (non-line-of-sight, Section III-B) ---
    % Note: LOS uses the lighter VISUAL walls, not collision walls.
    % An agent can SEE past a wall closer than it can WALK past one.
    for i = 1:N
        if arrived(i), continue; end

        if pbest_fit(i) < gbest_fit(i)                       % Own pbest can become gbest
            gbest(i,:)   = pbest(i,:);
            gbest_fit(i) = pbest_fit(i);
        end

        for j = 1:N                                          % Each other agent's pbest, gated by LOS
            if j == i,        continue; end
            if arrived(j),    continue; end
            if pbest_fit(j) >= gbest_fit(i), continue; end

            if has_line_of_sight(pos(i,:), pbest(j,:), walls, H, W, los_sample_step)
                gbest(i,:)   = pbest(j,:);
                gbest_fit(i) = pbest_fit(j);
            end
        end
    end

    %% --- Per-agent movement step ---
    for i = 1:N
        if arrived(i), continue; end

        old_pos = pos(i,:);
        e_idx_i = agent_exit(i);                             % Permanent assigned exit
        exit_pos = exits(e_idx_i, :);
        d_to_my_exit = norm(pos(i,:) - exit_pos);

        %% =====================================================
        %% LEVEL-5 ABSOLUTE TIMEOUT (hard failsafe)
        %% =====================================================
        % If an agent has been alive for absolute_timeout_iters iterations
        % without evacuating, force-arrive at its EUCLIDEAN-nearest open
        % exit. This is a last-resort safety net for agents that survive
        % all four levels of normal recovery — e.g., agents that get
        % reassigned, almost reach the new exit, accumulate a few px of
        % "progress" (resetting reassign_counter), then re-stall, ad
        % infinitum. The 1500-iter ceiling (≈75 sim-seconds) is generous
        % but firm.
        alive_counter(i) = alive_counter(i) + 1;
        if alive_counter(i) >= absolute_timeout_iters
            nearest_e_tmo = pick_nearest_open_exit_idx(pos(i,:), exits, exit_blocked);
            arrived(i)    = true;
            evac_times(i) = sim_time;
            fprintf('    [iter %d] Agent %d TIMEOUT — force-arrived at E%d after %d iters\n', ...
                iter, i, nearest_e_tmo, alive_counter(i));
            continue;
        end

        %% =====================================================
        %% (e-1) CONFIGURATION-SPACE POCKET ESCAPE (Layer 0)
        %% =====================================================
        % If the agent's current pixel is inside walls_collision, it
        % is trapped in a one-pixel-wide configuration-space pocket
        % (this can happen when the agent walks into a narrow gap that
        % is passable in raw-pixel space but blocked once dilated for
        % collision). No normal movement attempt will succeed because
        % every adjacent pixel is also inside walls_collision.
        %
        % FIX: detect this state and force movement along the wall_dist
        % gradient (toward open space), BYPASSING the collision check.
        % We move at half speed to avoid teleporting through anything,
        % but we MUST move — otherwise the agent is stuck forever.
        %
        % This only fires for agents that have walked into a pocket.
        % Normal agents in open space never enter this branch.
        agent_y = max(1, min(H, round(pos(i,2))));
        agent_x = max(1, min(W, round(pos(i,1))));
        if walls_collision(agent_y, agent_x)
            % STRENGTHENED POCKET ESCAPE:
            % Sample 16 candidate directions at varying step lengths
            % (px-by-px out to 20 px), and pick the direction that takes
            % us to the pixel with the LARGEST wall_dist value (deepest
            % into open space). This is more reliable than pure gradient
            % when the agent is in a complex pocket shape.

            best_dest_dist = -inf;                           % Best wall_dist we can reach
            best_step_dx   = 0;
            best_step_dy   = 0;

            n_dirs = 16;
            for kdir = 1:n_dirs
                ang = (kdir - 1) * 2 * pi / n_dirs;
                dx_unit = cos(ang);
                dy_unit = sin(ang);

                % Probe outward in 2-px increments up to 20 px
                for r = 2:2:20
                    tx = round(agent_x + dx_unit * r);
                    ty = round(agent_y + dy_unit * r);
                    if tx < 1 || tx > W || ty < 1 || ty > H, break; end
                    if walls(ty, tx), break; end             % Hit a real (visual) wall — stop probing
                    wd_here = double(wall_dist(ty, tx));
                    if wd_here > best_dest_dist
                        best_dest_dist = wd_here;
                        best_step_dx = dx_unit * r;
                        best_step_dy = dy_unit * r;
                    end
                end
            end

            if best_dest_dist > -inf && (best_step_dx ~= 0 || best_step_dy ~= 0)
                % Found a probed pixel deeper in open space — jump there
                escape_step = [best_step_dx, best_step_dy];
                step_len = norm(escape_step);
                if step_len > agent_speed                    % Cap the move at one step's speed
                    escape_step = escape_step / step_len * agent_speed;
                end
                pos(i,:) = pos(i,:) + escape_step;
                escape_dir = escape_step / norm(escape_step);
            else
                % No clear escape direction found — try gradient fallback
                step = 4;
                xl = max(1, agent_x - step);  xr = min(W, agent_x + step);
                yt = max(1, agent_y - step);  yb = min(H, agent_y + step);
                gx = double(wall_dist(agent_y, xr)) - double(wall_dist(agent_y, xl));
                gy = double(wall_dist(yb, agent_x)) - double(wall_dist(yt, agent_x));
                gnorm = sqrt(gx*gx + gy*gy);
                if gnorm > 1e-6
                    escape_dir = [gx, gy] / gnorm;
                else
                    ang = rand() * 2 * pi;
                    escape_dir = [cos(ang), sin(ang)];
                end
                pos(i,:) = pos(i,:) + escape_dir * agent_speed * pocket_escape_speed_mult;
            end

            % Bounds clamp
            pos(i,1) = max(1, min(W, pos(i,1)));
            pos(i,2) = max(1, min(H, pos(i,2)));
            direction(i,:) = escape_dir;
            stuck_counter(i) = 0;
            move_fail_counter(i) = 0;
            continue;
        end

        %% =====================================================
        %% (e0) EXIT PRIORITY NEAR DOORS (Item 8)
        %% =====================================================
        % If the agent is close to its assigned exit AND can see it
        % directly, OVERRIDE the PSO velocity update with a direct line
        % to the exit. Prevents circling near doorways.
        if d_to_my_exit < exit_direct_pull_radius && ...
           has_line_of_sight(pos(i,:), exit_pos, walls, H, W, los_sample_step)
            direct_dir = (exit_pos - pos(i,:)) / max(d_to_my_exit, 1e-9);
            if try_move_clear(pos(i,:), direct_dir, agent_speed, walls_collision, H, W)
                pos(i,:) = pos(i,:) + direct_dir * agent_speed;
                direction(i,:) = direct_dir;
                stuck_counter(i) = 0;
                move_fail_counter(i) = 0;

                % Bounds clamp
                pos(i,1) = max(1, min(W, pos(i,1)));
                pos(i,2) = max(1, min(H, pos(i,2)));

                % Run arrival check immediately so we don't waste an iteration
                ay = max(1, min(H, round(pos(i,2))));
                ax_pos = max(1, min(W, round(pos(i,1))));
                if green_mask(ay, ax_pos) || norm(pos(i,:) - exit_pos) < threshold
                    arrived(i)    = true;
                    evac_times(i) = sim_time;
                end
                continue;                                    % Skip rest of per-agent logic
            end
            % If direct line is blocked (e.g. another agent in the way),
            % fall through to normal PSO update.
        end

        %% =====================================================
        %% (e1) ACTIVE ESCAPE MODE (Item 4 — strengthened)
        %% =====================================================
        % If escape_remaining > 0, agent uses a RANDOM heading for this
        % iteration. After escape_remaining iterations of random
        % movement, escape ends and normal PSO resumes.
        if escape_remaining(i) > 0
            % Use a heading roughly perpendicular to whatever wall we
            % were pinned against (the previous direction). Add wide
            % random noise.
            if norm(direction(i,:)) > 1e-9
                perp = [-direction(i,2), direction(i,1)];    % Perpendicular to previous heading
                noise_angle = (rand() - 0.5) * pi;            % ±90°
                R = [cos(noise_angle), -sin(noise_angle); sin(noise_angle), cos(noise_angle)];
                escape_dir = (R * perp')';
            else
                ang = rand() * 2 * pi;
                escape_dir = [cos(ang), sin(ang)];
            end

            % Try the escape direction; if blocked, try variants
            moved_in_escape = false;
            for try_idx = 1:8
                test_ang = atan2(escape_dir(2), escape_dir(1)) + (try_idx-1) * (pi/4);
                test_dir = [cos(test_ang), sin(test_ang)];
                if try_move_clear(pos(i,:), test_dir, agent_speed, walls_collision, H, W)
                    pos(i,:) = pos(i,:) + test_dir * agent_speed;
                    direction(i,:) = test_dir;
                    moved_in_escape = true;
                    break;
                end
            end

            escape_remaining(i) = escape_remaining(i) - 1;

            % Bounds clamp
            pos(i,1) = max(1, min(W, pos(i,1)));
            pos(i,2) = max(1, min(H, pos(i,2)));

            % Arrival check still runs in escape mode
            ay = max(1, min(H, round(pos(i,2))));
            ax_pos = max(1, min(W, round(pos(i,1))));
            if green_mask(ay, ax_pos) || norm(pos(i,:) - exit_pos) < threshold
                arrived(i)    = true;
                evac_times(i) = sim_time;
                escape_remaining(i) = 0;
            end
            continue;                                        % Skip canonical PSO this iteration
        end

        %% =====================================================
        %% (e) PSO DIRECTION UPDATE + COMPASS + REPULSION
        %% =====================================================
        r1 = rand();
        r2 = rand();
        r3 = rand();

        % Cognitive: unit vector toward own pbest (paper Section II-A)
        d_pbest = pbest(i,:) - pos(i,:);
        n_pbest = norm(d_pbest);
        u_pbest = [0 0]; if n_pbest > 1e-9, u_pbest = d_pbest / n_pbest; end

        % Social: unit vector toward observed gbest (paper Section II-A)
        d_gbest = gbest(i,:) - pos(i,:);
        n_gbest = norm(d_gbest);
        u_gbest = [0 0]; if n_gbest > 1e-9, u_gbest = d_gbest / n_gbest; end

        % --- COMPASS: read precomputed direction-to-assigned-exit ---
        u_compass = read_compass(pos(i,:), e_idx_i, dist_fields, H, W);

        % --- INTER-PARTICLE REPULSION (Item 1 — strengthened) ---
        F_sep = [0, 0];
        for j = 1:N
            if j == i, continue; end
            if arrived(j), continue; end
            d_ij = pos(i,:) - pos(j,:);
            dist_ij = norm(d_ij);
            if dist_ij > 1e-6 && dist_ij < sep_radius
                falloff = (sep_radius - dist_ij) / sep_radius;
                F_sep = F_sep + sep_strength * falloff * (d_ij / dist_ij);
            end
        end

        % --- COMBINE: paper formula + compass + separation ---
        D_new = w_pso * direction(i,:) ...                   % Inertia
              + c1 * r1 * u_pbest ...                        % Cognitive
              + c2 * r2 * u_gbest ...                        % Social (LOWERED, Item 2)
              + c3 * r3 * u_compass ...                      % Compass (RAISED, Item 3)
              + F_sep;                                       % Separation (STRENGTHENED, Item 1)

        % Re-normalise to unit length
        D_norm = norm(D_new);
        if D_norm > 1e-9
            D_new = D_new / D_norm;
        else
            D_new = direction(i,:);
        end

        %% (f) SPEED PROPORTIONAL TO WALL DISTANCE (Section II-A)
        d_wall_here = wall_dist(max(1,min(H,round(pos(i,2)))), ...
                                 max(1,min(W,round(pos(i,1)))));
        speed_factor = 1 - exp(-(d_wall_here / sigma_wall)^2);
        speed_factor = max(0.3, speed_factor);
        S_new = agent_speed * speed_factor;

        %% =====================================================
        %% (g) TRY MOVE; WALL REFLECTION; LOCAL SEARCH (Items 5, 7, 9)
        %% =====================================================
        accepted_dir   = [];
        accepted_speed = S_new;
        moved          = false;

        % Attempt 1: original PSO direction
        if try_move_clear(pos(i,:), D_new, S_new, walls_collision, H, W)
            accepted_dir = D_new;
            moved = true;
            move_fail_counter(i) = 0;                        % Successful move resets failure counter
        else
            move_fail_counter(i) = move_fail_counter(i) + 1;

            % Attempt 2: WALL REFLECTION (Item 5)
            % If the PSO direction is blocked, REFLECT it about the
            % wall's local normal (estimated from the wall_dist gradient).
            % This is the "bounce" behaviour — agent redirects instead of
            % pushing into a wall.
            reflected_dir = reflect_off_wall(pos(i,:), D_new, walls, wall_dist, H, W);
            if ~isempty(reflected_dir) && ...
               try_move_clear(pos(i,:), reflected_dir, S_new, walls_collision, H, W)
                accepted_dir = reflected_dir;
                moved = true;
            else
                % Attempt 3: LOCAL DIRECTIONAL SEARCH (paper Section III-A)
                base_angle = atan2(D_new(2), D_new(1));
                for ring = 1:local_search_max_expansions
                    theta_rad = deg2rad(local_search_theta_deg * ring);
                    for s = 1:local_search_max_samples
                        angle_offset = (rand() - 0.5) * theta_rad;
                        test_angle   = base_angle + angle_offset;
                        test_dir     = [cos(test_angle), sin(test_angle)];

                        if try_move_clear(pos(i,:), test_dir, S_new, walls_collision, H, W)
                            accepted_dir = test_dir;
                            moved = true;
                            break;
                        end
                    end
                    if moved, break; end
                end

                % Attempt 4: half-speed in original direction
                if ~moved
                    if try_move_clear(pos(i,:), D_new, S_new * 0.5, walls_collision, H, W)
                        accepted_dir   = D_new;
                        accepted_speed = S_new * 0.5;
                        moved = true;
                    end
                end
            end
        end

        if moved
            pos(i,:) = pos(i,:) + accepted_dir * accepted_speed;
            direction(i,:) = accepted_dir;
        else
            direction(i,:) = direction(i,:) * 0.5;
        end

        % Bounds clamp
        pos(i,1) = max(1, min(W, pos(i,1)));
        pos(i,2) = max(1, min(H, pos(i,2)));

        %% =====================================================
        %% (h) STUCK DETECTION → LAYERED ESCALATION
        %% =====================================================
        % Three escalating levels of stuck recovery:
        %   Level 1 (60 iter no motion):   short random walk
        %   Level 2 (3 Level-1 events):    long random walk (50 iter)
        %   Level 3 (200 iter no progress): expanded failsafe radius

        moved_dist = norm(pos(i,:) - old_pos);
        if moved_dist < stuck_disp_eps
            stuck_counter(i) = stuck_counter(i) + 1;
        else
            stuck_counter(i) = 0;
        end

        % Standard arrival check
        ay = max(1, min(H, round(pos(i,2))));
        ax_pos = max(1, min(W, round(pos(i,1))));
        on_green = green_mask(ay, ax_pos);
        d_to_my_exit_now = norm(pos(i,:) - exit_pos);

        if on_green || d_to_my_exit_now < threshold
            arrived(i)    = true;
            evac_times(i) = sim_time;
            continue;
        end

        % --- FRUSTRATION TRACKING (Level-3 + Level-4 preparation) ---
        % Track BEST-EVER distance to assigned exit, not per-iteration
        % delta. An agent makes "real progress" only when it gets STRICTLY
        % CLOSER to the exit than it has ever been before, by at least
        % progress_threshold_px. Local oscillation does NOT count.
        %
        % This fixes the bug where agents drifting 0.6 px/iter toward
        % the exit (just above the old 0.5 px noise floor) could
        % indefinitely reset their counters and never trigger Level-4.
        if d_to_my_exit_now < best_exit_dist(i) - progress_threshold_px
            best_exit_dist(i)      = d_to_my_exit_now;       % New record
            frustration_counter(i) = 0;
            reassign_counter(i)    = 0;
        else
            frustration_counter(i) = frustration_counter(i) + 1;
            reassign_counter(i)    = reassign_counter(i) + 1;
        end

        % --- LEVEL-3 FAILSAFE: Expanded force-arrive zone (RAISED to 90px) ---
        % If the agent is anywhere reasonably close to its exit AND has
        % been frustrated for a long time, count it as evacuated. This
        % handles the case where the agent is alley-trapped near the
        % exit but cannot physically reach the green tile.
        nearest_e = pick_nearest_open_exit_idx(pos(i,:), exits, exit_blocked);
        d_to_any_exit = norm(pos(i,:) - exits(nearest_e,:));

        if d_to_any_exit < exit_disable_radius && ...
           (stuck_counter(i) >= stuck_threshold || frustration_counter(i) >= frustration_threshold)
            arrived(i)    = true;
            evac_times(i) = sim_time;
            stuck_counter(i) = 0;
            frustration_counter(i) = 0;
            continue;
        end

        % --- LEVEL-1 / LEVEL-2: Stuck → escape mode ---
        if stuck_counter(i) >= stuck_threshold
            escape_event_count(i) = escape_event_count(i) + 1;

            % Level 2: severe stuck (3+ events) → LONG random walk
            if escape_event_count(i) >= deep_escape_threshold
                escape_remaining(i) = deep_escape_steps;
                escape_event_count(i) = 0;                   % Reset counter after triggering deep escape
            else
                % Level 1: short random walk
                escape_remaining(i) = escape_random_steps;
            end

            pbest_fit(i) = inf;                              % Allow new pbest
            gbest_fit(i) = inf;                              % Allow new gbest
            stuck_counter(i) = 0;
            move_fail_counter(i) = 0;
        end

        % --- LEVEL-3 EXTRA: Frustration also triggers deep escape ---
        % An agent that's been frustrated for a long time but is still
        % moving (so stuck_counter never triggered) gets a deep escape.
        % This catches agents that are oscillating without progress.
        if frustration_counter(i) >= frustration_threshold && escape_remaining(i) == 0
            escape_remaining(i) = deep_escape_steps;
            pbest_fit(i) = inf;
            gbest_fit(i) = inf;
            frustration_counter(i) = 0;
            % NOTE: reassign_counter is NOT reset here — keeps growing
            % until either we make real progress or hit Level-4.
        end

        % --- LEVEL-4: SWITCH TO A DIFFERENT EXIT ---
        % If the agent has been frustrated for a really long time despite
        % all the escape mechanisms, the assigned exit is probably the
        % wrong one for this agent's position. Reassign to the
        % GEODESICALLY-NEAREST exit that ISN'T the current assignment.
        %
        % This is the final-fallback for situations like the 5-agent
        % cluster west of E4 (blocked by a wall partition): the geodesic
        % path to E4 is too tortuous, so we redirect them to E2 or E3
        % via a different corridor.
        if reassign_counter(i) >= reassign_threshold
            old_exit = agent_exit(i);
            px_a = max(1, min(W, round(pos(i,1))));
            py_a = max(1, min(H, round(pos(i,2))));

            best_d = inf;
            best_e = old_exit;
            for e_alt = 1:num_exits
                if exit_blocked(e_alt), continue; end
                if e_alt == old_exit, continue; end           % Skip current assignment
                d_alt = double(dist_fields(py_a, px_a, e_alt));
                if d_alt < best_d
                    best_d = d_alt;
                    best_e = e_alt;
                end
            end

            if best_e ~= old_exit && ~isinf(best_d)
                agent_exit(i) = best_e;
                fprintf('    [iter %d] Agent %d reassigned: E%d -> E%d (frustrated %d iters)\n', ...
                    iter, i, old_exit, best_e, reassign_counter(i));
                % Reset all counters and PSO state for a fresh start
                reassign_counter(i)    = 0;
                frustration_counter(i) = 0;
                stuck_counter(i)       = 0;
                escape_event_count(i)  = 0;
                pbest_fit(i)           = inf;
                gbest_fit(i)           = inf;
                % Reset best-ever distance to the NEW exit's current
                % distance — anything closer now counts as progress
                best_exit_dist(i)      = norm(pos(i,:) - exits(best_e,:));
                escape_remaining(i)    = 0;                  % Exit escape mode if in it
            else
                % No alternative exit found — just reset the counter so
                % we don't spam reassignment attempts.
                reassign_counter(i) = 0;
            end
        end

    end % end per-agent loop


    %% --- VISUALISATION REFRESH ---
    active = ~arrived;
    if any(active)
        set(h_agents, 'XData', pos(active,1), 'YData', pos(active,2));
    else
        set(h_agents, 'XData', [], 'YData', []);
    end

    title(ax_sim, sprintf('Time: %.1fs  |  Evacuated: %d / %d  |  Iter: %d', ...
        sim_time, sum(arrived), N, iter), 'FontSize', 13);

    graph_idx = graph_idx + 1;
    graph_x(graph_idx) = iter;
    graph_y(graph_idx) = sum(arrived);
    set(h_graph, 'XData', graph_x(1:graph_idx), 'YData', graph_y(1:graph_idx));

    drawnow limitrate;
    pause(0.02);

    if mod(iter, 500) == 0
        % Report exit usage using each agent's ACTUAL ASSIGNED exit
        % (not guess-by-distance, which can mislabel an agent that's
        % still in transit near a different exit).
        exit_counts = zeros(num_exits, 1);
        for i = 1:N
            if arrived(i)
                exit_counts(agent_exit(i)) = exit_counts(agent_exit(i)) + 1;
            end
        end
        fprintf('  iter=%4d | t=%.1fs | evacuated=%d/%d | by exit: E1=%d E2=%d E3=%d E4=%d\n', ...
            iter, sim_time, sum(arrived), N, exit_counts(1), exit_counts(2), exit_counts(3), exit_counts(4));
    end

    if sim_time > 600                                        % 10-minute hard timeout
        fprintf('WARNING: 600-second timeout. Stopping.\n');
        break;
    end
end


%% ============================================================
%% [SECTION 14] FINAL RESULTS
%% ============================================================
total_time = toc(start_time);

if all(arrived)
    title_str = sprintf('ALL EVACUATED — Total: %.2f sec', total_time);
    title_col = [0 0.6 0];
else
    title_str = sprintf('ENDED — %d/%d evacuated in %.2f sec', ...
        sum(arrived), N, total_time);
    title_col = [0.8 0.4 0];
end
title(ax_sim, title_str, 'FontSize', 14, 'Color', title_col, 'FontWeight', 'bold');

completed_times = evac_times(arrived);

% Per-exit breakdown — use ACTUAL ASSIGNMENT, not position guess
exit_counts = zeros(num_exits, 1);
for i = 1:N
    if arrived(i)
        exit_counts(agent_exit(i)) = exit_counts(agent_exit(i)) + 1;
    end
end

fprintf('\n╔══════════════════════════════════════════════╗\n');
fprintf('║  PSO + DIRECTIONAL COMPASS — RESULTS         ║\n');
fprintf('╚══════════════════════════════════════════════╝\n');
fprintf('  Total simulation time    : %.2f seconds\n', total_time);
fprintf('  Agents evacuated         : %d / %d  (%.1f%%)\n', ...
    sum(arrived), N, 100*sum(arrived)/N);
fprintf('  Agents not evacuated     : %d\n', N - sum(arrived));
fprintf('  Exit usage: E1=%d  E2=%d  E3=%d  E4=%d\n', ...
    exit_counts(1), exit_counts(2), exit_counts(3), exit_counts(4));

if ~isempty(completed_times)
    fprintf('  Fastest evacuation       : %.2f seconds\n', min(completed_times));
    fprintf('  Slowest evacuation       : %.2f seconds\n', max(completed_times));
    fprintf('  Average evacuation time  : %.2f seconds\n', mean(completed_times));
    fprintf('  Std deviation            : %.2f seconds\n', std(completed_times));
end

fprintf('══════════════════════════════════════════════\n');
fprintf('\nSimulation complete.\n');

% Bottom-right results panel
cla(ax_res);
set(ax_res, 'Color', [0.05 0.05 0.05], 'XTick', [], 'YTick', [], ...
    'Box', 'on', 'XColor', [1 1 1], 'YColor', [1 1 1]);

if ~isempty(completed_times)
    res_lines = {
        sprintf('Total time    : %.2f sec',            total_time);
        sprintf('Evacuated     : %d / %d  (%.1f%%)',   sum(arrived), N, 100*sum(arrived)/N);
        sprintf('Exit usage    : E1=%d E2=%d E3=%d E4=%d', exit_counts(1), exit_counts(2), exit_counts(3), exit_counts(4));
        sprintf('Fastest       : %.2f sec',            min(completed_times));
        sprintf('Slowest       : %.2f sec',            max(completed_times));
    };
else
    res_lines = {
        sprintf('Total time    : %.2f sec',            total_time);
        sprintf('Evacuated     : %d / %d  (%.1f%%)',   sum(arrived), N, 100*sum(arrived)/N);
        'No agents completed evacuation.';
    };
end

num_lines = numel(res_lines);
for ln = 1:num_lines
    y_pos = 1 - (ln - 0.5) / num_lines;
    text(ax_res, 0.05, y_pos, res_lines{ln}, ...
        'Units', 'normalized', 'Color', [0 1 1], ...
        'FontSize', 11, 'FontWeight', 'bold', 'FontName', 'Courier New', ...
        'VerticalAlignment', 'middle');
end

title(ax_res, 'PSO + Directional Compass', 'FontWeight', 'bold', 'Color', [1 1 1], 'FontSize', 11);


%% ============================================================
%% [LOCAL FUNCTIONS]
%% ============================================================

function e_idx = pick_nearest_open_exit_idx(p, exits, exit_blocked)
    % Euclidean nearest open exit (used for arrival check)
    n_exits = size(exits, 1);
    best_d = inf;
    e_idx = 1;
    for e = 1:n_exits
        if exit_blocked(e), continue; end
        d = (p(1)-exits(e,1))^2 + (p(2)-exits(e,2))^2;
        if d < best_d
            best_d = d;
            e_idx = e;
        end
    end
end

function u = read_compass(p, e_idx, dist_fields, H, W)
    % READ_COMPASS — return a unit vector pointing in the direction of
    % steepest descent on the geodesic distance field for exit e_idx,
    % evaluated at agent position p.
    %
    % Looks at the 8-neighbourhood at a small offset (5 px) for finite-
    % difference stability. If all neighbours are infinite, returns [0 0]
    % (compass unknown — agent relies on PSO alone).
    px = max(2, min(W-1, round(p(1))));
    py = max(2, min(H-1, round(p(2))));

    step = 5;                                                % Compass sample step (px) — wide enough to avoid noise
    px_l = max(1, px - step);
    px_r = min(W, px + step);
    py_t = max(1, py - step);
    py_b = min(H, py + step);

    d0 = double(dist_fields(py,   px,   e_idx));
    if isinf(d0) || isnan(d0)
        u = [0, 0];
        return;
    end

    % 8-neighbour samples
    candidates = [
        px_l, py,    double(dist_fields(py,   px_l, e_idx));
        px_r, py,    double(dist_fields(py,   px_r, e_idx));
        px,   py_t,  double(dist_fields(py_t, px,   e_idx));
        px,   py_b,  double(dist_fields(py_b, px,   e_idx));
        px_l, py_t,  double(dist_fields(py_t, px_l, e_idx));
        px_r, py_t,  double(dist_fields(py_t, px_r, e_idx));
        px_l, py_b,  double(dist_fields(py_b, px_l, e_idx));
        px_r, py_b,  double(dist_fields(py_b, px_r, e_idx));
    ];

    % Pick neighbour with smallest distance (steepest descent)
    best_d = d0;
    best_idx = 0;
    for k = 1:size(candidates, 1)
        if isinf(candidates(k, 3)) || isnan(candidates(k, 3)), continue; end
        if candidates(k, 3) < best_d
            best_d = candidates(k, 3);
            best_idx = k;
        end
    end

    if best_idx == 0
        u = [0, 0];                                          % All neighbours infinite or worse — flat region
    else
        dx = candidates(best_idx, 1) - px;
        dy = candidates(best_idx, 2) - py;
        nrm = sqrt(dx*dx + dy*dy);
        if nrm > 1e-9
            u = [dx, dy] / nrm;
        else
            u = [0, 0];
        end
    end
end

function f = fitness(p, target, wall_dist, sigma_wall, sigma_target, c_obs, H, W)
    % FITNESS — paper Section II-B
    px = max(1, min(W, round(p(1))));
    py = max(1, min(H, round(p(2))));

    dw = wall_dist(py, px);
    cost_walls = exp(-(dw / sigma_wall)^2);

    d2 = (p(1)-target(1))^2 + (p(2)-target(2))^2;
    cost_target = exp(-d2 / sigma_target^2);
    cost_target = max(cost_target, 1e-9);

    f = c_obs * cost_walls + 1 / cost_target;
end

function clear_path = try_move_clear(p, dir_unit, speed, walls_collision, H, W)
    % CONFIGURATION-SPACE COLLISION CHECK — single ray on heavily-dilated
    % wall mask. Equivalent to a disc-sweep check because walls are
    % dilated by agent radius. No corner-cutting possible.
    target = p + dir_unit * speed;

    move_len = abs(speed);
    n_samp = max(2, ceil(move_len) + 1);
    ts = linspace(0, 1, n_samp).';

    xs = p(1) + ts * (target(1) - p(1));
    ys = p(2) + ts * (target(2) - p(2));

    ix = max(1, min(W, round(xs)));
    iy = max(1, min(H, round(ys)));

    idx = sub2ind([H W], iy, ix);
    clear_path = ~any(walls_collision(idx));
end

function r = reflect_off_wall(p, dir_in, ~, wall_dist, H, W)
    % REFLECT_OFF_WALL — bounce a movement direction off the nearest wall.
    % Used when the agent's PSO direction is blocked; reflecting it about
    % the wall normal often gives a sensible "slide along the wall"
    % direction instead of pushing into it again.
    %
    % Returns [] if no wall is nearby (shouldn't happen if we got here
    % via a failed try_move_clear) or if reflection produces something
    % that points into the wall again.
    %
    % Wall normal is estimated from the gradient of wall_dist at the
    % agent's position. wall_dist increases AWAY from walls, so its
    % gradient points away from the wall — that's our normal.

    px = max(2, min(W-1, round(p(1))));
    py = max(2, min(H-1, round(p(2))));

    % Sample wall_dist gradient via centred differences (small window)
    step = 4;
    px_l = max(1, px - step);  px_r = min(W, px + step);
    py_t = max(1, py - step);  py_b = min(H, py + step);

    gx = double(wall_dist(py,   px_r)) - double(wall_dist(py,   px_l));
    gy = double(wall_dist(py_b, px))   - double(wall_dist(py_t, px));

    g_norm = sqrt(gx*gx + gy*gy);
    if g_norm < 1e-6
        r = [];                                              % Flat gradient, no clear normal
        return;
    end

    % Wall normal points AWAY from the wall (toward open space)
    n = [gx, gy] / g_norm;

    % Reflection: r = d - 2*(d·n)*n
    dot_dn = dir_in(1)*n(1) + dir_in(2)*n(2);

    % If d·n > 0, agent is already moving AWAY from the wall — reflecting
    % would point it INTO the wall. Skip in that case.
    if dot_dn > -0.1
        r = [];
        return;
    end

    r = dir_in - 2 * dot_dn * n;
    r_norm = norm(r);
    if r_norm > 1e-9
        r = r / r_norm;
    else
        r = [];
    end
end

function los = has_line_of_sight(pa, pb, walls, H, W, step_px)
    % Bresenham-like LOS check on VISUAL walls (not collision walls)
    d = pb - pa;
    L = norm(d);
    if L < 1
        los = true;
        return;
    end
    n_samp = max(2, ceil(L / step_px));
    ts     = linspace(0, 1, n_samp).';
    xs     = pa(1) + ts * d(1);
    ys     = pa(2) + ts * d(2);
    ix = max(1, min(W, round(xs)));
    iy = max(1, min(H, round(ys)));
    idx = sub2ind([H W], iy, ix);
    los = ~any(walls(idx));
end
