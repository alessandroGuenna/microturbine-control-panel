function plotCycle(dp)

fprintf("\nGenerating T-S diagram...........\n")

s = [dp.stations.s];
T = [dp.stations.T];
ids = {dp.stations.id};

% main loop segments -> {from_idx, to_idx, label, color}
segments = {
    1,2, 'Compressor',            [0 0.4 0.7];
    2,3, 'Heat addition (REC+CC)',[0.93 0.69 0.13];
    3,4, 'Turbine',                [0.49 0.18 0.56];
};

figure
hold on; grid on; box on;

legend_handles = gobjects(size(segments,1)+3,1);
legend_labels  = cell(size(segments,1)+3,1);
n_leg = 0;

for k = 1:size(segments,1)
    i1 = segments{k,1};
    i2 = segments{k,2};
    lbl = segments{k,3};
    col = segments{k,4};

    h = plot(s([i1 i2]), T([i1 i2]), '-o', ...
        'Color', col, 'LineWidth', 2, ...
        'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', 'MarkerSize', 6);
    n_leg = n_leg + 1;
    legend_handles(n_leg) = h;
    legend_labels{n_leg} = lbl;
end

% closing segment (4p -> REC exhaust outlet): not a modeled process,
% just visual closure -- stops at the REC exhaust outlet rather than
% running all the way to station 1
frac_eg = (dp.recuperator.T_eg_out - T(4)) / (T(1) - T(4));
s_eg_out = s(4) + frac_eg*(s(1) - s(4));
plot([s(4) s_eg_out], [T(4) dp.recuperator.T_eg_out], ...
    'Color', [0.55 0.55 0.55], 'LineWidth', 1.2, 'HandleVisibility', 'off');

% station labels
offsets = containers.Map();
offsets('1')  = {0.5 , -0.5,'right','top'};
offsets('2p') = { -1.25, 1.25,'left','top'};
offsets('3')  = { 0.5, 1,'left','top'};
offsets('4p') = { 0, -0.5,'left','top'};

xlims_pre = [min(s) max(s)];
ylims_pre = [min(T) max(T)];
dx0 = 0.035*diff(xlims_pre);
dy0 = 0.045*diff(ylims_pre);

for i = 1:numel(ids)
    id = ids{i};
    if isKey(offsets, id)
        o = offsets(id);
    else
        o = {1,1,'left','bottom'};
    end
    text(s(i) + o{1}*dx0, T(i) + o{2}*dy0, id, 'FontSize', 9, ...
        'HorizontalAlignment', o{3}, 'VerticalAlignment', o{4}, ...
        'Interpreter','none');
end

% REC outlet markers: placed at the intersection between the relevant
% segment and the target temperature (linear interpolation along that
% straight segment, since T is monotonic along both).
h1 = plotIntersectionMarker(s, T, 2, 3, dp.recuperator.T_air_out, [0.00 0.45 0.74]);
n_leg = n_leg + 1;
legend_handles(n_leg) = h1;
legend_labels{n_leg} = 'REC air outlet (T2'''')';

h2 = plot(s_eg_out, dp.recuperator.T_eg_out, 'd', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.85 0.33 0.10], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
n_leg = n_leg + 1;
legend_handles(n_leg) = h2;
legend_labels{n_leg} = 'REC exhaust outlet (T5)';

xlabel('s  [J/(kg·K)]');
ylabel('T  [K]');
title('Microturbine cycle -- T-s diagram (design point)');
legend(legend_handles(1:n_leg), legend_labels(1:n_leg), 'Location', 'northwest');
hold off;

end

%% ---------------------------------------------------------------------
function h = plotIntersectionMarker(s, T, i1, i2, T_target, col)
% Finds where T_target falls along the straight segment from station i1
% to station i2 (linear interpolation, since the plotted segment is a
% straight line and T is monotonic along it), and marks that point.
frac = (T_target - T(i1)) / (T(i2) - T(i1));
s_interp = s(i1) + frac*(s(i2) - s(i1));

h = plot(s_interp, T_target, 'd', 'MarkerSize', 9, ...
    'MarkerFaceColor', col, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
end
