% =========================================================================
% BCM EVACUATION SIMULATION — BFS-GUIDED PSO
% MAP: BCM-L2-SIM-2.jpg (Children's Gallery + Art & Craft Exhibition)
% FYP: Particle Swarm Optimization for Museum Evacuation
%
% ALGORITHM OVERVIEW:
%   1. Detect walls from dark pixels in BCM map image
%   2. BFS (Breadth-First Search) from every exit builds a "distance field"
%      where dist_field(y,x) = minimum walkable steps to reach nearest exit
%   3. PSO velocity = inertia + cognitive (personal best) + social (BFS gradient)
%   4. Axis-separated movement: X then Y independently — NO corner cutting
%   5. Stuck recovery: try 4 cardinal directions using BFS, then random walk
%
% CHANGES FROM SIM-4 VERSION:
%   - Map file changed to BCM-L2-SIM-2.jpg
%   - Exit coordinates updated for SIM-2 green zone positions
%   - wall_thresh lowered to 40 to avoid coloured rooms being flagged as walls
%   - Figure title updated
%
% ALL AGENTS ARE NEUTRAL (same speed, same colour)
% =========================================================================

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║    BCM-L2-SIM-2 EVACUATION — BFS + PSO SIMULATION       ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');


%% ============================================================
%% [SECTION 1] LOAD BCM MAP IMAGE
%% ============================================================

fprintf('Loading BCM-L2-SIM-2 map...\n');

% Map file for this simulation
% ← CHANGE to 'BCM-L2-SIM-4.jpg' to switch back to the previous map
layout = imread('BCM-L2-SIM-2.jpg');

% Define max_iter and N here so the graph axes can use them during figure setup.
% Both are also declared in their proper sections below — all values must match.
max_iter = 5000;   % hard iteration cap (matches Section 9)
N        = 80;     % total number of agents (matches Section 6)

% Wider figure: left = simulation map, right = live graph + results panel
fig = figure('Name', 'BCM-L2-SIM-2 Evacuation — BFS + PSO', 'Position', [30, 30, 1400, 800]);

% Left panel: simulation map (occupies left 60% of figure)
ax_sim = axes('Position', [0.01 0.02 0.58 0.95]);   % [left bottom width height]
imshow(layout, 'Parent', ax_sim); hold(ax_sim, 'on');
set(ax_sim, 'YDir', 'reverse');   % Pixel coords: Y increases downward
axis(ax_sim, 'on');
grid(ax_sim, 'on');                       % Grid lines for coordinate reference
set(ax_sim, 'GridColor', [1 1 0], 'GridAlpha', 0.4, 'GridLineStyle', '--');

% Top-right panel: live evacuated agents graph (occupies right 37%, top half)
ax_graph = axes('Position', [0.62 0.50 0.37 0.46]);
title(ax_graph, 'Agents Evacuated', 'FontWeight', 'bold', 'Color', 'w', 'FontSize', 12);
xlabel(ax_graph, 'Step',  'Color', 'w', 'FontSize', 11);
ylabel(ax_graph, 'Count', 'Color', 'w', 'FontSize', 11);
xlim(ax_graph, [0, max_iter]);
ylim(ax_graph, [0, N]);
set(ax_graph, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
    'GridColor', 'w', 'GridAlpha', 0.2, 'FontSize', 10);
grid(ax_graph, 'on');
hold(ax_graph, 'on');
% Dashed line showing total agent count (target)
plot(ax_graph, [0 max_iter], [N N], 'g--', 'LineWidth', 1.2);
% Live evacuated count line (updated each iteration)
h_graph = plot(ax_graph, 0, 0, 'g-', 'LineWidth', 2);
graph_x = zeros(1, max_iter);   % Pre-allocate iteration x-data
graph_y = zeros(1, max_iter);   % Pre-allocate evacuated count y-data
graph_idx = 0;                  % Current fill index

% Bottom-RIGHT results panel — border via axes Box property (no annotation overlay)
ax_res = axes('Position', [0.62 0.03 0.37 0.43]);
set(ax_res, ...
    'Color',     [0.05 0.05 0.05], ...  % Dark background
    'Box',       'on', ...              % Draw border around axes
    'LineWidth',  2.5, ...              % Border thickness
    'XColor',    [1 1 1], ...           % White border (X axis line = left+right sides)
    'YColor',    [1 1 1], ...           % White border (Y axis line = top+bottom sides)
    'XTick',     [], ...                % No tick marks on X
    'YTick',     []);                   % No tick marks on Y
axis(ax_res, 'on');                     % Keep axis ON so Box border is visible
title(ax_res, 'Evacuation Results', 'FontWeight', 'bold', 'Color', [1 1 1], 'FontSize', 13);

[H, W, ~] = size(layout);     % H = image height (rows), W = image width (cols)
fprintf('  Map size: %d x %d pixels\n', W, H);


%% ============================================================
%% [SECTION 2] WALL DETECTION FROM IMAGE
%% ============================================================

gray = rgb2gray(layout);        % Convert RGB image to grayscale

% SIM-2 has coloured rooms (grey, orange, blue) — these must NOT become walls.
% Only the pure black outlines and the black border outside the building = walls.
% wall_thresh = 40 is strict enough to catch black lines only.
% ← INCREASE toward 60 if thin black lines are being missed
% ← DECREASE toward 30 if coloured rooms are wrongly detected as walls
wall_thresh = 40;
walls = gray < wall_thresh;     % Logical mask: true = wall pixel

% Dilate wall mask to add a safety buffer around wall edges
% ← INCREASE disk radius if agents still clip through thin walls
walls = imdilate(walls, strel('disk', 3));

fprintf('  Walls detected (threshold = %d)\n', wall_thresh);


%% ============================================================
%% [SECTION 3] PRE-CODED EXIT POSITIONS (GREEN ZONES)
%% ============================================================
% Each row = one exit: [x, y] = pixel column and pixel row of green zone CENTER.
%
% SIM-2 has 4 green exit zones:
%   Exit 1 — Top-Left    (near toilet / Children's Gallery top)
%   Exit 2 — Top-Right   (near lift / Art & Craft top)
%   Exit 3 — Bottom-Left (bottom of Children's Gallery)
%   Exit 4 — Bottom-Right(bottom of Art & Craft Exhibition)
%
% ════════════════════════════════════════════════════════════
% HOW TO GET EXACT COORDINATES FOR THIS MAP:
%   Step 1 — In MATLAB Command Window, run:
%               figure; imshow(imread('BCM-L2-SIM-2.jpg'))
%   Step 2 — In the figure toolbar, click the Data Cursor icon (crosshair)
%             OR go to: Tools → Data Cursor
%   Step 3 — Click the EXACT CENTRE of each green box
%   Step 4 — Read the [X: ... Y: ...] values shown in the tooltip
%   Step 5 — Replace the placeholder values below with your readings
% ════════════════════════════════════════════════════════════

exits = [
    200, 480;   % Exit 1 — Top-Left     (confirmed)
    700, 480;   % Exit 2 — Top-Right    (confirmed)
    115, 910;   % Exit 3 — Bottom-Left  (confirmed)
    770, 915;   % Exit 4 — Bottom-Right (confirmed)
];

% ← ADD more rows here if there are extra exits in SIM-2

num_exits    = size(exits, 1);          % Total exit count (auto-updates)
exit_blocked = false(num_exits, 1);     % All exits open at start
% ← To block an exit from the start: exit_blocked(2) = true;

% ── EXIT COORDINATE VALIDATION ───────────────────────────────
% Checks that no exit is still set to the placeholder value [0, 0].
% If any row is [0,0], the simulation cannot run — BFS will find nothing
% and randi will crash with "First input must be a positive scalar integer".
for e = 1:num_exits
    if exits(e,1) == 0 || exits(e,2) == 0
        error(['EXIT COORDINATES NOT SET!\n' ...
               'Exit %d is still at placeholder [0, 0].\n\n' ...
               'HOW TO FIX:\n' ...
               '  1. Run: figure; imshow(imread(''BCM-L2-SIM-2.jpg''))\n' ...
               '  2. Go to Tools → Data Cursor (or press D)\n' ...
               '  3. Click the centre of each green exit box\n' ...
               '  4. Read [X, Y] from the tooltip\n' ...
               '  5. Replace the 000,000 placeholders in Section 3\n'], e);
    end
    if exits(e,1) < 1 || exits(e,1) > W || exits(e,2) < 1 || exits(e,2) > H
        error(['EXIT %d IS OUTSIDE THE IMAGE BOUNDS!\n' ...
               'Given [%d, %d] but image is %d x %d pixels.\n' ...
               'Re-check coordinates using Data Cursor.\n'], ...
               e, exits(e,1), exits(e,2), W, H);
    end
end
fprintf('  Exit coordinates validated OK\n');
% ─────────────────────────────────────────────────────────────

% Draw exit markers explicitly on ax_sim (avoids drawing on wrong axes)
plot(ax_sim, exits(:,1), exits(:,2), 'go', ...
    'MarkerSize', 20, 'MarkerFaceColor', [0 0.9 0], ...
    'MarkerEdgeColor', [0 0.5 0], 'LineWidth', 2, ...
    'DisplayName', 'Exits');

% Label each exit on ax_sim
for e = 1:num_exits
    text(ax_sim, exits(e,1), exits(e,2) - 22, sprintf('Exit %d', e), ...
        'Color', [0 0.6 0], 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'center');
end

fprintf('  %d exits pre-coded\n', num_exits);


%% ============================================================
%% [SECTION 4] BFS DISTANCE FIELD COMPUTATION
%% ============================================================
% BFS floods outward from all open exits through walkable pixels only.
% Result: dist_field(y,x) = wall-free steps from pixel (x,y) to nearest exit.
%   dist = 0   → inside exit zone
%   dist = N   → N steps to nearest exit
%   dist = Inf → unreachable (isolated room or inside wall)
%
% Agents follow the decreasing gradient of dist_field, so every step is
% guaranteed wall-free. Narrow doors and alleys work automatically.

fprintf('Computing BFS distance field (please wait)...\n');

dist_field = inf(H, W, 'double');   % All pixels initialised as unreachable

% Preallocate BFS queue — maximum size = total pixel count
q_data = zeros(H * W, 2, 'int32'); % Each row: [row_y, col_x]
q_head = int32(1);                  % Index of next item to dequeue
q_tail = int32(0);                  % Index of last item enqueued

% Radius of pixels around exit center that count as "at the exit"
% ← INCREASE if agents don't register as arrived at exits
% ← DECREASE if two exits are very close together
exit_radius = 22;   % pixels

% Seed BFS from all open exits
for e = 1:num_exits
    if exit_blocked(e), continue; end   % Skip exits blocked at start

    ex = exits(e, 1);   % Exit center column (x)
    ey = exits(e, 2);   % Exit center row    (y)

    % Mark a circular patch around each exit as distance = 0
    for dy = -exit_radius : exit_radius
        for dx = -exit_radius : exit_radius
            nx = ex + dx;
            ny = ey + dy;

            if nx < 1 || nx > W || ny < 1 || ny > H, continue; end   % Out of bounds
            if sqrt(dx^2 + dy^2) > exit_radius,       continue; end   % Outside circle
            if walls(ny, nx),                          continue; end   % Wall pixel
            if dist_field(ny, nx) == 0,                continue; end   % Already seeded

            dist_field(ny, nx) = 0;
            q_tail = q_tail + 1;
            q_data(q_tail, :) = int32([ny, nx]);
        end
    end
end

% 4-connected neighbour offsets — no diagonals prevents corner cutting
NY4 = int32([0,  0,  1, -1]);   % Row offsets: right, left, down, up
NX4 = int32([1, -1,  0,  0]);   % Col offsets

% BFS propagation
while q_head <= q_tail
    cy = q_data(q_head, 1);     % Current pixel row
    cx = q_data(q_head, 2);     % Current pixel col
    q_head = q_head + 1;

    cd = dist_field(cy, cx);

    for n = 1:4
        ny2 = cy + NY4(n);
        nx2 = cx + NX4(n);

        if ny2 < 1 || ny2 > H || nx2 < 1 || nx2 > W, continue; end
        if walls(ny2, nx2),                            continue; end
        if dist_field(ny2, nx2) <= cd + 1,             continue; end

        dist_field(ny2, nx2) = cd + 1;
        q_tail = q_tail + 1;
        q_data(q_tail, :) = int32([ny2, nx2]);
    end
end

% Report unreachable walkable cells
unreachable_count = sum(~walls(:) & isinf(dist_field(:)));
fprintf('  BFS complete. Unreachable walkable cells: %d\n', unreachable_count);
if unreachable_count > 0
    fprintf('  WARNING: some cells cannot reach any exit.\n');
    fprintf('  Fix: lower wall_thresh or add exits to isolated rooms.\n');
end


%% ============================================================
%% [SECTION 5] WALKABLE AREA
%% ============================================================
% Valid spawn pixel: not a wall AND BFS-reachable from at least one exit.

walkable     = ~walls & ~isinf(dist_field);
[wy, wx]     = find(walkable);
num_walkable = numel(wx);

fprintf('  Walkable + reachable pixels: %d\n', num_walkable);

% Guard: if num_walkable is 0, randi will crash — catch it early with a clear message
if num_walkable == 0
    error(['NO WALKABLE PIXELS FOUND!\n' ...
           'This means BFS could not reach any floor area from the exits.\n\n' ...
           'Most likely causes:\n' ...
           '  1. Exit coordinates land on a wall pixel — re-check with Data Cursor\n' ...
           '  2. wall_thresh is too high — lower it in Section 2 (try 30 or 40)\n' ...
           '  3. The exit zone is completely surrounded by wall pixels\n']);
end


%% ============================================================
%% [SECTION 6] AGENT PLACEMENT
%% ============================================================

N           = 80;    % ← CHANGE total number of agents here
min_spacing = 10;    % ← DECREASE if too few agents fit on the map

pos    = zeros(N, 2);   % Agent positions: [x (col), y (row)]
placed = 0;

for attempt = 1:200000
    if placed >= N, break; end

    k  = randi(num_walkable);
    px = wx(k);
    py = wy(k);

    if placed == 0
        placed    = 1;
        pos(1, :) = [px, py];
    else
        % Only place agent if far enough from all existing agents
        dists = vecnorm(pos(1:placed, :) - [px, py], 2, 2);
        if min(dists) >= min_spacing
            placed         = placed + 1;
            pos(placed, :) = [px, py];
        end
    end
end

N   = placed;
pos = pos(1:N, :);
fprintf('  Agents placed: %d / %d requested\n', N, 80);


%% ============================================================
%% [SECTION 7] AGENT SPEED & RADIUS (FROM RESEARCH PARAMETERS)
%% ============================================================
% Research reference parameters (SFM):
%   Walking speed (adult) = 1.47 m/s
%   Agent radius          = 0.2 m
%
% Pixel-to-metre conversion:
%   map_scale_px_per_m = how many pixels represent 1 metre on this map.
%   ← CALIBRATE: measure a known distance on the map in pixels,
%     divide by the real distance in metres.
%     e.g. if a 10m corridor = 180 pixels → map_scale_px_per_m = 18
map_scale_px_per_m = 30;        % pixels per metre — adjust to match BCM-L2-SIM-2 scale
                                 % SIM-2 image is larger (~900px) so scale is higher than SIM-4

% Simulation time step = pause duration (seconds per iteration)
step_duration_s    = 0.05;      % seconds per iteration (matches pause(0.05) below)

% Agent speed in pixels per iteration step — derived from research value
walking_speed_ms   = 1.47;      % m/s — from research (SFM walking speed, adult)
agent_speed        = walking_speed_ms * map_scale_px_per_m * step_duration_s;
% → agent_speed = 1.47 × 30 × 0.05 = 2.205 px/step

% Agent radius in pixels — derived from research value
agent_radius_m     = 0.2;       % metres — from research (SFM agent radius)
agent_radius_px    = agent_radius_m * map_scale_px_per_m;
% → agent_radius_px = 0.2 × 30 = 6 px
% Scatter marker area (points²) ≈ π × radius²  (1 pt ≈ 1 px at screen resolution)
agent_marker_size  = pi * agent_radius_px^2;
% → agent_marker_size ≈ 113 points²

fprintf('  Walking speed  : %.2f m/s  →  %.2f px/step\n', walking_speed_ms, agent_speed);
fprintf('  Agent radius   : %.2f m    →  %.2f px (marker area ≈ %.0f pts²)\n', ...
    agent_radius_m, agent_radius_px, agent_marker_size);


%% ============================================================
%% [SECTION 8] PSO PARAMETERS
%% ============================================================
% PSO velocity formula each step:
%   vel = w_pso * vel                       (inertia)
%       + c1 * r1 * (pbest - pos)           (cognitive: own best memory)
%       + c2 * r2 * (BFS_direction * speed) (social: exit pull via BFS)

w_pso = 0.4;   % Inertia weight ← TUNE: 0.3 to 0.7
c1    = 1.2;   % Cognitive coefficient ← TUNE: 0.5 to 2.0
c2    = 2.5;   % Social coefficient ← TUNE: 1.5 to 3.0

% How much BFS direction vs direct exit line influences movement
% 1.0 = fully wall-safe BFS, 0.0 = straight line to exit (may clip walls)
bfs_weight = 0.80;   % ← INCREASE toward 1.0 if agents clip walls

vel   = zeros(N, 2);   % All agents start stationary
pbest = pos;           % Personal best starts at spawn position


%% ============================================================
%% [SECTION 9] SIMULATION STATE VARIABLES
%% ============================================================

arrived       = false(N, 1);   % true once agent reaches an exit
stuck_counter = zeros(N, 1);   % consecutive low-movement steps
evac_times    = zeros(N, 1);   % wall-clock seconds at evacuation

threshold = 18;    % pixels — agent counted as arrived within this distance of exit
                   % ← INCREASE if agents stop just short of exit zone

max_stuck = 25;    % stuck steps before recovery triggers
                   % ← DECREASE for faster recovery

max_iter  = 5000;  % hard iteration cap ← INCREASE for large maps


%% ============================================================
%% [SECTION 10] VISUALISATION SETUP
%% ============================================================

% All agents drawn as cyan scatter dots on the simulation axes
% Marker size = agent_marker_size (derived from 0.2m radius in Section 7)
h_agents = scatter(ax_sim, pos(:,1), pos(:,2), agent_marker_size, [0 0.8 0.9], 'filled', ...
    'DisplayName', 'Agents');

legend(ax_sim, 'Exits', 'Agents', ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 9);


%% ============================================================
%% [SECTION 11] COUNTDOWN & START
%% ============================================================

for t = 3:-1:1
    title(ax_sim, sprintf('Evacuation begins in %d ...', t), 'FontSize', 16);
    pause(1);
end

start_time = tic;
fprintf('\n=== EVACUATION STARTED ===\n');
title(ax_sim, 'Evacuation in progress...', 'FontSize', 14);


%% ============================================================
%% [SECTION 12] MAIN SIMULATION LOOP
%% ============================================================

for iter = 1:max_iter

    if all(arrived), break; end      % Stop early if all agents evacuated

    sim_time = toc(start_time);

    %% PER-AGENT UPDATE
    for i = 1:N
        if arrived(i), continue; end

        %% Find nearest open exit (Euclidean — BFS handles wall-safe movement)
        best_e   = -1;
        best_euc = inf;
        for e = 1:num_exits
            if exit_blocked(e), continue; end
            d_euc = norm(exits(e,:) - pos(i,:));
            if d_euc < best_euc
                best_euc = d_euc;
                best_e   = e;
            end
        end

        if best_e < 0, continue; end    % All exits blocked — agent waits
        gbest = exits(best_e, :);       % Target exit [x, y]

        %% Clamp agent position to valid pixel range
        ax = max(1, min(W, round(pos(i,1))));
        ay = max(1, min(H, round(pos(i,2))));

        %% BFS gradient direction
        % Pick the neighbour with the smallest dist_field value.
        % This is the wall-safe step direction toward the exit.
        bfs_dir = [0, 0];
        best_d  = dist_field(ay, ax);

        for n = 1:4
            ny2 = ay + NY4(n);
            nx2 = ax + NX4(n);

            if ny2 < 1 || ny2 > H || nx2 < 1 || nx2 > W, continue; end

            if dist_field(ny2, nx2) < best_d
                best_d  = dist_field(ny2, nx2);
                bfs_dir = [double(NX4(n)), double(NY4(n))];
            end
        end

        if norm(bfs_dir) > 0
            bfs_dir = bfs_dir / norm(bfs_dir);   % Normalise to unit vector
        end

        %% Direct exit vector (straight line — may cross walls at long range)
        exit_vec = gbest - pos(i,:);
        if norm(exit_vec) > 0
            exit_vec = exit_vec / norm(exit_vec);
        end

        %% Blend BFS + direct direction
        % bfs_weight controls how much wall-safe BFS vs straight line is used
        blended = bfs_weight * bfs_dir + (1 - bfs_weight) * exit_vec;
        if norm(blended) > 0
            blended = blended / norm(blended);
        end

        %% PSO velocity update
        r1 = rand();   % Random factor for cognitive term [0,1]
        r2 = rand();   % Random factor for social term    [0,1]

        vel(i,:) = w_pso * vel(i,:) ...                     % Inertia
                 + c1 * r1 * (pbest(i,:) - pos(i,:)) ...    % Cognitive (own best)
                 + c2 * r2 * (blended * agent_speed);        % Social (BFS exit pull)

        % Clamp velocity magnitude to agent_speed
        vmag = norm(vel(i,:));
        if vmag > agent_speed
            vel(i,:) = vel(i,:) / vmag * agent_speed;
        end

        %% Axis-separated collision detection — NO corner cutting
        % X and Y are tested independently.
        % A wall hit on X only cancels X (agent slides along wall in Y).
        % A wall hit on Y only cancels Y (agent slides along wall in X).

        old_pos = pos(i,:);    % Save for stuck detection
        new_pos = pos(i,:);

        % Try X move
        tx = max(1, min(W, round(pos(i,1) + vel(i,1))));   % Proposed column
        ty = max(1, min(H, round(pos(i,2))));               % Current row

        if ~walls(ty, tx)
            new_pos(1) = pos(i,1) + vel(i,1);   % X accepted
        else
            vel(i,1) = -vel(i,1) * 0.4;          % X wall hit — bounce & dampen
        end

        % Try Y move using updated X position
        tx2 = max(1, min(W, round(new_pos(1))));            % Updated column
        ty2 = max(1, min(H, round(pos(i,2) + vel(i,2))));  % Proposed row

        if ~walls(ty2, tx2)
            new_pos(2) = pos(i,2) + vel(i,2);   % Y accepted
        else
            vel(i,2) = -vel(i,2) * 0.4;          % Y wall hit — bounce & dampen
        end

        % Clamp final position within image bounds
        new_pos(1) = max(1, min(W, new_pos(1)));
        new_pos(2) = max(1, min(H, new_pos(2)));

        pos(i,:) = new_pos;

        %% Stuck detection
        % Increment counter if movement was less than 0.25 pixels this step
        if norm(pos(i,:) - old_pos) < 0.25
            stuck_counter(i) = stuck_counter(i) + 1;
        else
            stuck_counter(i) = 0;
        end

        %% Stuck recovery
        if stuck_counter(i) >= max_stuck

            escaped = false;

            % Strategy 1: try 4 cardinal directions at 2.5x step size
            % Only accept if not a wall AND BFS can reach an exit from there
            dirs_try = [1 0; -1 0; 0 1; 0 -1];   % Right, Left, Down, Up

            for d = 1:4
                trial = pos(i,:) + dirs_try(d,:) * agent_speed * 2.5;
                tx3   = max(1, min(W, round(trial(1))));
                ty3   = max(1, min(H, round(trial(2))));

                if ~walls(ty3, tx3) && ~isinf(dist_field(ty3, tx3))
                    pos(i,:)         = trial;
                    vel(i,:)         = dirs_try(d,:) * agent_speed;
                    stuck_counter(i) = 0;
                    escaped          = true;
                    break;
                end
            end

            % Strategy 2: random angle, 40 attempts — BFS-reachable cells only
            if ~escaped
                for rnd_try = 1:40
                    angle = rand() * 2 * pi;
                    nudge = [cos(angle), sin(angle)];
                    trial = pos(i,:) + nudge * agent_speed * 1.5;
                    tx3   = max(1, min(W, round(trial(1))));
                    ty3   = max(1, min(H, round(trial(2))));

                    if ~walls(ty3, tx3) && ~isinf(dist_field(ty3, tx3))
                        pos(i,:)         = trial;
                        vel(i,:)         = nudge * agent_speed;
                        stuck_counter(i) = 0;
                        break;
                    end
                end
            end

        end % end stuck recovery

        %% Update personal best
        % Personal best = position with lowest BFS distance seen so far
        ax2  = max(1, min(W, round(pos(i,1))));
        ay2  = max(1, min(H, round(pos(i,2))));
        axpb = max(1, min(W, round(pbest(i,1))));
        aypb = max(1, min(H, round(pbest(i,2))));

        if dist_field(ay2, ax2) < dist_field(aypb, axpb)
            pbest(i,:) = pos(i,:);
        end

        %% Arrival check
        % Agent counted as evacuated when within threshold pixels of exit center
        if norm(pos(i,:) - gbest) < threshold
            arrived(i)    = true;
            evac_times(i) = sim_time;
        end

    end % end per-agent loop


    %% VISUALISATION REFRESH
    active = ~arrived;   % Mask of agents still inside

    if any(active)
        set(h_agents, 'XData', pos(active,1), 'YData', pos(active,2));
    else
        set(h_agents, 'XData', [], 'YData', []);
    end

    title(ax_sim, sprintf('Time: %.1fs  |  Evacuated: %d / %d  |  Iteration: %d', ...
        sim_time, sum(arrived), N, iter), 'FontSize', 13);

    % Update live evacuated agents graph (top-right panel)
    graph_idx = graph_idx + 1;
    graph_x(graph_idx) = iter;
    graph_y(graph_idx) = sum(arrived);
    set(h_graph, 'XData', graph_x(1:graph_idx), 'YData', graph_y(1:graph_idx));

    drawnow limitrate;
    pause(0.05);   % 50ms pause per iteration — matches PSO_Optimization.m timing
                   % Ensures visual speed and evacuation time are comparable
                   % ← REMOVE this line if you want maximum simulation speed

    % Console progress every 500 iterations
    if mod(iter, 500) == 0
        fprintf('  iter=%4d | t=%.1fs | evacuated=%d/%d (%.0f%%)\n', ...
            iter, sim_time, sum(arrived), N, 100*sum(arrived)/N);
    end

    % Timeout — stops simulation if it runs too long
    % ← INCREASE timeout if map is large or agent count is high
    if sim_time > 300
        fprintf('WARNING: 300-second timeout. Stopping.\n');
        fprintf('  Check: are exits reachable? Is wall_thresh correct?\n');
        break;
    end

end % end main loop


%% ============================================================
%% [SECTION 13] FINAL RESULTS
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

completed_times = evac_times(arrived);   % Times for agents that evacuated

fprintf('\n╔══════════════════════════════════════════════╗\n');
fprintf('║           EVACUATION RESULTS                 ║\n');
fprintf('╚══════════════════════════════════════════════╝\n');
fprintf('  Total simulation time    : %.2f seconds\n', total_time);
fprintf('  Agents evacuated         : %d / %d  (%.1f%%)\n', ...
    sum(arrived), N, 100*sum(arrived)/N);
fprintf('  Agents not evacuated     : %d\n', N - sum(arrived));

if ~isempty(completed_times)
    fprintf('  Fastest evacuation       : %.2f seconds\n', min(completed_times));
    fprintf('  Slowest evacuation       : %.2f seconds\n', max(completed_times));
    fprintf('  Average evacuation time  : %.2f seconds\n', mean(completed_times));
    fprintf('  Std deviation            : %.2f seconds\n', std(completed_times));
end

fprintf('══════════════════════════════════════════════\n');
fprintf('\nSimulation complete.\n');

% Fill bottom-right results panel with simulation statistics
cla(ax_res);                            % Clear old content but keep axes properties
set(ax_res, 'Color', [0.05 0.05 0.05], ...
    'XTick', [], 'YTick', [], ...       % Keep ticks hidden
    'Box', 'on', 'XColor', [1 1 1], 'YColor', [1 1 1]);

if ~isempty(completed_times)
    res_lines = {
        sprintf('Total time    : %.2f sec',            total_time);
        sprintf('Evacuated     : %d / %d  (%.1f%%)',   sum(arrived), N, 100*sum(arrived)/N);
        sprintf('Not evacuated : %d',                  N - sum(arrived));
        sprintf('Fastest       : %.2f sec',            min(completed_times));
        sprintf('Slowest       : %.2f sec',            max(completed_times));
    };
else
    res_lines = {
        sprintf('Total time    : %.2f sec',            total_time);
        sprintf('Evacuated     : %d / %d  (%.1f%%)',   sum(arrived), N, 100*sum(arrived)/N);
        sprintf('Not evacuated : %d',                  N - sum(arrived));
        'No agents completed evacuation.';
    };
end

% Draw each result line — bright cyan, size 13, bold, evenly spaced
num_lines = numel(res_lines);
for ln = 1:num_lines
    y_pos = 1 - (ln - 0.5) / num_lines;
    text(ax_res, 0.05, y_pos, res_lines{ln}, ...
        'Units', 'normalized', 'Color', [0 1 1], ...   % Bright cyan — clearly visible on dark bg
        'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Courier New', ...
        'VerticalAlignment', 'middle');
end

title(ax_res, 'Evacuation Results', 'FontWeight', 'bold', 'Color', [1 1 1], 'FontSize', 14);
