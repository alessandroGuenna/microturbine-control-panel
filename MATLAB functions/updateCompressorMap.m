function updateCompressorMap(m, beta)
% updateCompressorMap  Move the operating point marker on the compressor map

    h = findall(groot, 'Tag', 'CompressorOpPoint');

    if ~isempty(h)
        set(h(1), 'XData', m, 'YData', beta);
        drawnow('limitrate');
    end

end