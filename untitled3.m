% =========================================================================
% BCM EVACUATION SIMULATION — BFS-GUIDED PSO
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
% ALL AGENTS ARE NEUTRAL (same speed, same colour)
% =========================================================================

clear; clc; close all;

fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║        BCM EVACUATION — BFS + PSO SIMULATION            ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% ============================================================
%% [SECTION 1] LOAD BCM MAP IMAGE
%% ============================================================

fprintf('Loading BCM map...\n');

% ← CHANGE filename here to switch map variant
%   Options: 'BCM-L2-SIM-3.jpg'  or  'BCM-L2-SIM-4.jpg'
layout = imread('BCM-L2-SIM-4.jpg');

% Open figure window
figure('Name', 'BCM Evacuation — BFS + PSO', 'Position', [50, 50, 950, 800]);
imshow(layout); hold on;
set(gca, 'YDir', 'reverse');   % Pixel coords: Y increases downward
axis on;

[H, W, ~] = size(layout);     % H = image height (rows), W = image width (cols)
fprintf('  Map size: %d x %d pixels\n', W, H);


%% ============================================================
%% [SECTION 2] WALL DETECTION FROM IMAGE
%% ============================================================

gray = rgb2gray(layout);        % Convert RGB image to grayscale

% Pixels darker than wall_thresh are treated as walls (black lines in floorplan)
% ← ADJUST wall_thresh if walls are not detected correctly
%   Lower value = stricter (fewer pixels become walls), range: 40 to 80
wall_thresh = 60;
walls = gray < wall_thresh;     % Logical mask: true = wall pixel

% Dilate wall mask to add a safety buffer around wall edges
% ← INCREASE disk radius if agents still clip through thin walls
walls = imdilate(walls, strel('disk', 3));

fprintf('  Walls detected (threshold = %d)\n', wall_thresh);


%% ============================================================
%% [SECTION 3] PRE-CODED EXIT POSITIONS (GREEN ZONES)
%% ============================================================
% Each row is one exit: [x, y] = pixel column, pixel row of the green zone center.
%
% HOW TO FIND CORRECT COORDINATES:
%   1. Run: imshow(imread('BCM-L2-SIM-4.jpg'))
%   2. Click the Data Cursor icon (crosshair) in the figure toolbar
%   3. Click the center of each green zone → read [X, Y] from the tooltip
%   4. Replace the values below with your readings

exits = [
    148, 145;   % Exit 1 — Top-Left green zone
    505, 140;   % Exit 2 — Top-Right green zone
     72, 462;   % Exit 3 — Bottom-Left green zone   (confirmed correct — DO NOT CHANGE)
    540, 468;   % Exit 4 — Bottom-Right green zone
];
% ─────────────────────────────────────────────────────────────────
% TO FINE-TUNE ANY EXIT THAT IS STILL OFF:
%   Step 1 — In MATLAB Command Window, run:
%               figure; imshow(imread('BCM-L2-SIM-4.jpg'))
%   Step 2 — In the figure, go to: Tools → Data Cursor (or press D)
%   Step 3 — Click the EXACT CENTRE of the green box for that exit
%   Step 4 — Read the [X: ... Y: ...] values from the tooltip
%   Step 5 — Replace ONLY that exit row above with the new [X, Y]
%   (Exit 3 at [72, 462] is correct — only change the others if needed)
% ─────────────────────────────────────────────────────────────────
% ← ADD more rows for extra exits, e.g.:  300, 50;  % Top-Centre exit

num_exits    = size(exits, 1);          % Total exit count (auto-updates)
exit_blocked = false(num_exits, 1);     % All exits open at start
% ← To block an exit from the start: exit_blocked(3) = true;

% Draw exit markers on the figure
plot(exits(:,1), exits(:,2), 'go', ...
    'MarkerSize', 20, 'MarkerFaceColor', [0 0.9 0], ...
    'MarkerEdgeColor', [0 0.5 0], 'LineWidth', 2, ...
    'DisplayName', 'Exits');

% Label each exit
for e = 1:num_exits
    text(exits(e,1), exits(e,2) - 22, sprintf('Exit %d', e), ...
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
% ← INCREASE if agents don't register as arrived
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
NY4 = int32([0,  0,  1, -1]);   % Row offsets
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
%% [SECTION 7] AGENT SPEED (ALL NEUTRAL)
%% ============================================================
% All agents are identical — same speed, no type distinction.

agent_speed = 3;    % Matches max_speed_normal = 3 from PSO_Optimization.m
                    % (PSO_Optimization.m used: normal = 3, elderly = 1.5)
                    % Since all agents here are neutral, we use the normal speed

fprintf('  All %d agents: neutral, speed = %.1f px/step\n', N, agent_speed);


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

threshold = 18;    % pixels — counted as arrived within this distance of exit
                   % ← INCREASE if agents stop just short of exit zone

max_stuck = 25;    % stuck steps before recovery triggers
                   % ← DECREASE for faster recovery

max_iter  = 5000;  % hard iteration cap ← INCREASE for large maps


%% ============================================================
%% [SECTION 10] VISUALISATION SETUP
%% ============================================================

% All agents drawn as a single cyan scatter — neutral, no type distinction
h_agents = scatter(pos(:,1), pos(:,2), 40, [0 0.8 0.9], 'filled', ...
    'DisplayName', 'Agents');

legend('Exits', 'Agents', ...
    'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 9);


%% ============================================================
%% [SECTION 11] COUNTDOWN & START
%% ============================================================

for t = 3:-1:1
    title(sprintf('Evacuation begins in %d ...', t), 'FontSize', 16);
    pause(1);
end

start_time = tic;
fprintf('\n=== EVACUATION STARTED ===\n');
title('Evacuation in progress...', 'FontSize', 14);


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

        if best_e < 0, continue; end    % All exits blocked — wait
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

        %% Direct exit vector (straight line — may cross walls)
        exit_vec = gbest - pos(i,:);
        if norm(exit_vec) > 0
            exit_vec = exit_vec / norm(exit_vec);
        end

        %% Blend BFS + direct direction
        blended = bfs_weight * bfs_dir + (1 - bfs_weight) * exit_vec;
        if norm(blended) > 0
            blended = blended / norm(blended);
        end

        %% PSO velocity update
        r1 = rand();
        r2 = rand();

        vel(i,:) = w_pso * vel(i,:) ...                     % Inertia
                 + c1 * r1 * (pbest(i,:) - pos(i,:)) ...    % Cognitive
                 + c2 * r2 * (blended * agent_speed);        % Social (BFS exit pull)

        % Clamp to max speed
        vmag = norm(vel(i,:));
        if vmag > agent_speed
            vel(i,:) = vel(i,:) / vmag * agent_speed;
        end

        %% Axis-separated collision detection — NO corner cutting
        % X tested first, then Y using the new X position.
        % Each axis bounces independently on a wall hit.

        old_pos = pos(i,:);
        new_pos = pos(i,:);

        % Try X move
        tx = max(1, min(W, round(pos(i,1) + vel(i,1))));
        ty = max(1, min(H, round(pos(i,2))));

        if ~walls(ty, tx)
            new_pos(1) = pos(i,1) + vel(i,1);   % X accepted
        else
            vel(i,1) = -vel(i,1) * 0.4;          % X bounce
        end

        % Try Y move using updated X position
        tx2 = max(1, min(W, round(new_pos(1))));
        ty2 = max(1, min(H, round(pos(i,2) + vel(i,2))));

        if ~walls(ty2, tx2)
            new_pos(2) = pos(i,2) + vel(i,2);   % Y accepted
        else
            vel(i,2) = -vel(i,2) * 0.4;          % Y bounce
        end

        % Clamp final position within image bounds
        new_pos(1) = max(1, min(W, new_pos(1)));
        new_pos(2) = max(1, min(H, new_pos(2)));

        pos(i,:) = new_pos;

        %% Stuck detection
        if norm(pos(i,:) - old_pos) < 0.25
            stuck_counter(i) = stuck_counter(i) + 1;
        else
            stuck_counter(i) = 0;
        end

        %% Stuck recovery
        if stuck_counter(i) >= max_stuck

            escaped = false;

            % Strategy 1: try 4 cardinal directions at 2.5x step size
            dirs_try = [1 0; -1 0; 0 1; 0 -1];

            for d = 1:4
                trial = pos(i,:) + dirs_try(d,:) * agent_speed * 2.5;
                tx3   = max(1, min(W, round(trial(1))));
                ty3   = max(1, min(H, round(trial(2))));

                % Only accept if not a wall AND BFS can reach an exit from there
                if ~walls(ty3, tx3) && ~isinf(dist_field(ty3, tx3))
                    pos(i,:)         = trial;
                    vel(i,:)         = dirs_try(d,:) * agent_speed;
                    stuck_counter(i) = 0;
                    escaped          = true;
                    break;
                end
            end

            % Strategy 2: random angle — 40 attempts, BFS-reachable cells only
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
        ax2  = max(1, min(W, round(pos(i,1))));
        ay2  = max(1, min(H, round(pos(i,2))));
        axpb = max(1, min(W, round(pbest(i,1))));
        aypb = max(1, min(H, round(pbest(i,2))));

        if dist_field(ay2, ax2) < dist_field(aypb, axpb)
            pbest(i,:) = pos(i,:);
        end

        %% Arrival check
        if norm(pos(i,:) - gbest) < threshold
            arrived(i)    = true;
            evac_times(i) = sim_time;
        end

    end % end per-agent loop


    %% VISUALISATION REFRESH
    active = ~arrived;

    if any(active)
        set(h_agents, 'XData', pos(active,1), 'YData', pos(active,2));
    else
        set(h_agents, 'XData', [], 'YData', []);
    end

    title(sprintf('Time: %.1fs  |  Evacuated: %d / %d  |  Iteration: %d', ...
        sim_time, sum(arrived), N, iter), 'FontSize', 13);

    drawnow limitrate;
    pause(0.05);   % 50ms pause per iteration — matches PSO_Optimization.m timing
                   % Ensures visual speed and evacuation time are comparable
                   % ← REMOVE this line if you want maximum simulation speed

    if mod(iter, 500) == 0
        fprintf('  iter=%4d | t=%.1fs | evacuated=%d/%d (%.0f%%)\n', ...
            iter, sim_time, sum(arrived), N, 100*sum(arrived)/N);
    end

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
title(title_str, 'FontSize', 14, 'Color', title_col, 'FontWeight', 'bold');

completed_times = evac_times(arrived);   % Evacuation times for agents that made it

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