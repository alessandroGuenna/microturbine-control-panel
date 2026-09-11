mdl = 'microturbine_model_v2';
load_system(mdl);
set_param(mdl, 'StopTime', '10');

% Warm-up run — pays the compile cost, discard this timing
set_param(mdl, 'EnablePacing', 'off');
sim(mdl);

% Un-paced
set_param(mdl, 'EnablePacing', 'off');
tic; sim(mdl); t_unpaced = toc

% % Paced at 1x
set_param(mdl, 'EnablePacing', 'on');
set_param(mdl, 'PacingRate', '1');
tic; sim(mdl); t_paced = toc