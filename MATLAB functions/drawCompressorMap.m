function drawCompressorMap(DataCompMap, speeds, ax)
% drawCompressorMap(DataCompMap, speeds)      -> draws in a new figure
% drawCompressorMap(DataCompMap, speeds, ax)  -> draws into the given axes (e.g. app UIAxes)

    flow = []; beta = []; n = [];
    for k = 1:length(speeds)
        flow_k = DataCompMap.tableData(:, 2*k-1);
        beta_k = DataCompMap.tableData(:, 2*k);
        flow = [flow; flow_k];  beta = [beta; beta_k];
        n = [n; repmat(speeds(k), length(flow_k), 1)];
    end

    [beta, n, flow] = removeDuplicates(beta, n, flow, 1e-6, 1e-4);

    assignin("base","flow",flow);  assignin("base","n",n);  assignin("base","beta",beta);
    m_surge = DataCompMap.tableData(:, 11);
    beta_surge = DataCompMap.tableData(:, 12);
    assignin("base","m_surge",m_surge);  assignin("base","beta_surge",beta_surge);

    m1=flow(n==speeds(1)); beta1=beta(n==speeds(1));
    m2=flow(n==speeds(2)); beta2=beta(n==speeds(2));
    m3=flow(n==speeds(3)); beta3=beta(n==speeds(3));
    m4=flow(n==speeds(4)); beta4=beta(n==speeds(4));
    m5=flow(n==speeds(5)); beta5=beta(n==speeds(5));

    % target: supplied axes, or a new figure if none given
    if nargin < 3 || isempty(ax) || ~isgraphics(ax)
        figure; ax = axes;
    else
        cla(ax);          % reusing the app axes -> clear the old map first
    end

    hold(ax,'on'); grid(ax,'on');
    plot(ax, m1, beta1, 'b','LineWidth',1.5)
    plot(ax, m2, beta2, 'c','LineWidth',1.5)
    plot(ax, m3, beta3, 'g','LineWidth',1.5)
    plot(ax, m4, beta4, 'r','LineWidth',1.5)
    plot(ax, m5, beta5, 'm','LineWidth',1.5)
    plot(ax, m_surge, beta_surge, 'w','LineWidth',1.2,'LineStyle','--')
    legend(ax, {'48000 rpm','54000 rpm','60000 rpm','66000 rpm','72000 rpm','Surge line'}, ...
           'Location','northwest')
    ylim(ax,[0 6]); xlabel(ax,'Mass flow [kg/s]'); ylabel(ax,'Pressure ratio')

    plot(ax, NaN, NaN, 'ro', 'MarkerFaceColor','w','MarkerSize',8, ...
        'Tag','CompressorOpPoint', 'DisplayName', 'OP');
    drawnow
end