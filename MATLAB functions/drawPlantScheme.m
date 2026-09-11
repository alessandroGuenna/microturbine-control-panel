function h = drawPlantScheme(ax)
%DRAWPLANTSCHEME  Draw the microturbine plant scheme into an axes.
%
%   h = drawPlantScheme(ax)  draws the plant into UIAxes `ax` and returns a
%   struct of text handles for the live temperature labels:
%
%       h.T1     compressor inlet          h.T4     turbine outlet
%       h.T2     compressor outlet         h.T5     recuperator outlet (exhaust)
%       h.T2pp   recuperator outlet (air)  h.T6     HRB outlet (exhaust)
%       h.T3     CC outlet                 h.Twin   HRB water inlet
%                                          h.Twout  HRB water outlet
%
%   Update them from the app, e.g.:
%       h.T3.String = sprintf('T3 CC out\n%.0f °C', val);
%
%   Fixed 104 x 78 view with a locked data aspect ratio, so the scheme keeps
%   its shape at any window size and stays centred.
%
%   TO MOVE A LABEL: edit its x,y in the LABELS table near the bottom.

% ------------------------------------------------------------------ setup
cla(ax); hold(ax,'on');
ax.XLim = [-3 101];  ax.YLim = [-6 72];   % centred on the drawing (4:3)
ax.DataAspectRatio = [1 1 1];             % never distort
ax.Visible = 'off';                       % hide ticks/box, keep contents
try ax.Toolbar.Visible = 'off'; disableDefaultInteractivity(ax); catch, end

% palette (tuned for a dark app theme; change here to restyle everything)
col.body   = [0.42 0.45 0.50];   % component fill
col.edge   = [0.85 0.88 0.92];   % component outline
col.gas    = [0.95 0.55 0.25];   % hot gas piping
col.air    = [0.35 0.75 0.95];   % air piping
col.water  = [0.30 0.85 0.60];   % water piping
col.shaft  = [0.70 0.72 0.76];   % mechanical shaft
col.txt    = [0.92 0.94 0.97];   % component captions
col.lblBg  = [0.10 0.11 0.14];   % label background
col.lblEd  = [0.55 0.58 0.65];   % label border

% ------------------------------------------------------- major components
% compressor (narrows in flow direction) and turbine (widens)
patch(ax,[34 46 46 34],[60 55 47 42],col.body,'EdgeColor',col.edge,'LineWidth',1.2);
patch(ax,[64 76 76 64],[55 60 42 47],col.body,'EdgeColor',col.edge,'LineWidth',1.2);
text(ax,40,64,'Compressor','Color',col.txt,'HorizontalAlignment','center','FontSize', 12);
text(ax,70,64,'Turbine',   'Color',col.txt,'HorizontalAlignment','center','FontSize', 12);

% shaft + generator
plot(ax,[46 64],[51 51],'-','Color',col.shaft,'LineWidth',4);
plot(ax,[76 85],[51 51],'-','Color',col.shaft,'LineWidth',4);
th = linspace(0,2*pi,100);
plot(ax,91+6*cos(th),51+6*sin(th),'-','Color',col.edge,'LineWidth',1.2);
sx = linspace(-4,4,60);
plot(ax,91+sx,51+2*sin(sx*pi/4),'-','Color',col.edge,'LineWidth',1.2);
text(ax,91,60,'Generator','Color',col.txt,'HorizontalAlignment','center','FontSize',12);
pipe(ax,[91 91],[45 39],col.shaft); arrowhead(ax,91,39,'d',col.shaft);
text(ax,91,36,'Electricity','Color',col.txt,'HorizontalAlignment','center','FontSize',11);

% combustion chamber
rectangle(ax,'Position',[52 30 8 8],'FaceColor',col.body,'EdgeColor',col.edge,'LineWidth',1.2);
text(ax,56,34,'CC','Color',col.txt,'HorizontalAlignment','center', ...
     'VerticalAlignment','middle','FontSize',11,'FontWeight','bold');

% small fuel arrow, kept clear of the blue compressor-outlet pipe
pipe(ax,[49 52],[34 34],col.gas); arrowhead(ax,52,34,'r',col.gas,0.9);
text(ax,48,34,'Fuel','Color',col.txt,'HorizontalAlignment','right', ...
     'VerticalAlignment','middle','FontSize',11);

% recuperator and heat-recovery boiler (wide gap between them for the T5 label)
rectangle(ax,'Position',[38 10 34 12],'FaceColor',col.body,'EdgeColor',col.edge,'LineWidth',1.2);
rectangle(ax,'Position',[ 8 10 16 12],'FaceColor',col.body,'EdgeColor',col.edge,'LineWidth',1.2);
text(ax,55,8,'Recuperator','Color',col.txt,'HorizontalAlignment','center','FontSize',12);
text(ax,16,7,'HRB',        'Color',col.txt,'HorizontalAlignment','center','FontSize',12);
zigzag(ax,42,68,16,col.edge);    % exchanger symbol inside recuperator
zigzag(ax,11,21,16,col.edge);    % exchanger symbol inside HRB

% ------------------------------------------------------------------ piping
% air in -> compressor
pipe(ax,[24 34],[51 51],col.air);  arrowhead(ax,34,51,'r',col.air);
text(ax,24,55,'Combustion air','Color',col.txt,'FontSize',11);

% compressor -> recuperator (cold side), straight down
pipe(ax,[40 40],[44.5 22],col.air); arrowhead(ax,40,22,'d',col.air);

% recuperator (cold side) -> CC
pipe(ax,[56 56],[22 30],col.air);  arrowhead(ax,56,30,'u',col.air);

% CC -> turbine
pipe(ax,[56 56 66 66],[38 41 41 46.2],col.gas); arrowhead(ax,66,46.2,'u',col.gas);
% turbine -> recuperator (hot side), straight down
pipe(ax,[70 70],[44.5 22],col.gas); arrowhead(ax,70,22,'d',col.gas);

% recuperator -> HRB -> stack
pipe(ax,[38 24],[16 16],col.gas);  arrowhead(ax,24,16,'l',col.gas);
pipe(ax,[8 1],  [16 16],col.gas);  arrowhead(ax,1,16,'l',col.gas);
text(ax,-1,20,'Exhaust gas','Color',col.txt,'FontSize',11);

% water side of the HRB
pipe(ax,[12 12],[4 10],col.water);  arrowhead(ax,12,10,'u',col.water);
pipe(ax,[20 20],[10 4],col.water);  arrowhead(ax,20,4,'d',col.water);

% ------------------------------------------------------------ LABELS table
%   name        x     y    caption
L = { 'T1'   , 26 , 47 , 'T1 inlet'      ; ...
      'T2'   , 33 , 34 , 'T2 comp out'   ; ...
      'T2pp' , 49 , 25 , 'T2pp rec out'; ...
      'T3'   , 57 , 45 , 'T3 CC out'     ; ...
      'T4'   , 77 , 36 , 'T4 turb out'   ; ...
      'T5'   , 31 , 20 , 'T5 rec out'    ; ...
      'T6'   ,  8 , 26 , 'T6 HRB out'    ; ...
      'Twin' , 10 ,  1 , 'water in'      ; ...
      'Twout', 21 ,  1 , 'water out'     };

h = struct();
for k = 1:size(L,1)
    h.(L{k,1}) = text(ax, L{k,2}, L{k,3}, sprintf('%s\n--- °C',L{k,4}), ...
        'Color',col.txt,'FontSize',14,'FontName','Consolas', ...
        'HorizontalAlignment','center','VerticalAlignment','middle', ...
        'BackgroundColor',col.lblBg,'EdgeColor',col.lblEd,'Margin',2);
end

hold(ax,'off');
end

% ===================================================== local helpers
function pipe(ax,X,Y,c)
    plot(ax,X,Y,'-','Color',c,'LineWidth',1.8);
end

function arrowhead(ax,x,y,dir,c,s)
    if nargin < 6, s = 1.4; end
    switch dir
        case 'r', X=[x x-s x-s]; Y=[y y+s y-s];
        case 'l', X=[x x+s x+s]; Y=[y y+s y-s];
        case 'u', X=[x x-s x+s]; Y=[y y-s y-s];
        case 'd', X=[x x-s x+s]; Y=[y y+s y+s];
    end
    patch(ax,X,Y,c,'EdgeColor','none');
end

function zigzag(ax,x0,x1,y,c)
    n = 6;  xs = linspace(x0,x1,n+1);
    ys = y + repmat([1.8 -1.8],1,ceil((n+1)/2));
    plot(ax,xs,ys(1:numel(xs)),'-','Color',c,'LineWidth',1);
end
