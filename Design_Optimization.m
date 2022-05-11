% Design_Optimization.m 
% Caitlin Freeman
% Last updated November 21, 2020

% This script imports data from previous intermodular distortion
% calculations and uses it to formulate and solve the objective function
% for all Platonic solids for our design optimization problem.

% How to use:
    % This script needs the following .mat and .m files to run:
        % InverseOrtho.m: applies the inverse orthographic projection to
                    % the input base sketch, thereby creating a spherical
                    % sketch in (lat,long) cooridnates
        % EqAzimuthal.m: applies the equidistant azimuthal projection to
                    % the spherical sketch, thereby creating the planar 
                    % module sketch
        % CompleteSketch.m: appropriately rotates a single limb sketch to
                    % create a full module sketch
        % distort_plot.mat:  data file containing:
                    % A_sweep (1x101 vector of ampltidue values 0:0.01:1)
                    % D_inter (5x101 matrix of intramodular distortion values)
                            % row value: Platonic solids ordered by # faces
                            % col value: corresponds to amplitude
    % What values can I change?
        % infeas_cutoff_icosa = A_sweep index past which the icosahedron
                % sketch becomes infeasible (w.r.t. locomotion)
        % a = edge length of the circumscribed Platonic solid
        % N = number of data points used to build sketches 
                % NOTE: odd numbers result in a missing point at x = 0
        % w = weighting factor for the objective function
        
        
% TO-DO

% need to add more formal procedure to calculate locomotion infeasibility
% by adding in a buffer (defining at what point the limbs become too thin)

% potentially include more amplitudes by changing the normalization formula

% add interpolation to the D_inter, A_sweep data to get smoother plots and
% more precise optimal amplitude values

close all
clear
clc

%% Intermodular Distortion v. Amplitude
% This section loads the intermodular distortion vs amplitude data for all
% 5 Platonic solids. 
load('distort_plot.mat')

% This removes the locomotion infeasible points from the icosahedron
infeas_cutoff_icosa = 80;   % buffer for thinness of icosahedron (ad hoc value)
D_inter(5,infeas_cutoff_icosa+1:end)=NaN;

% plot the data
figure(1)
hold on
for j = 1:5
    plot(A_sweep,D_inter(j,:),'Linewidth',4)
end
xlabel('Module toplogy curve amplitude (A)')
ylabel('Intermodular Distortion (\epsilon_{inter})')
legend('Tetrahedron','Cube','Octahedron','Dodecahedron','Icosahedron')
grid on; grid minor; 
set(gca,'FontSize',36);
%% Locomotion Cost v. Amplitude
a = 2;      % edge length
N = 500;    % number of data points
x = linspace(-a/2,a/2,N);       % base sketch x

% Platonic solid definitions and characteristics 
Solid = {'tetrahedron' 'hexahedron' 'octahedron' 'dodecahedron' 'icosahedron'}';
DihedralAngle = [acos(1/3), pi/2, acos(-1/3), acos(-sqrt(5)/5), acos(-sqrt(5)/3)]';
Circumradius = a*[sqrt(6)/4, sqrt(3)/2, sqrt(2)/2, (sqrt(15)+sqrt(3))/4, (sqrt(10+2*sqrt(5)))/4]';
FaceEdgeNumber = [3, 4, 3, 5, 3]';  
P = table(Solid,DihedralAngle,Circumradius,FaceEdgeNumber);
PlatonicSolid = table2struct(P);

% Imagine a circle centered at the center of the planar module sketch and
% coincident to the sketch point furthest from the center:

    % arm = radius of this circle
    % locom_cost_abs = arm^(-1)
    % locom_cost = normalized locom_cost

% As this radius increases, locomotion is assumed to be easier

for j = 1:5     % for each Platonic solid
    % Define Platonic solid and characteristics
    solid = Solid{j};
    s = find(contains(Solid,solid));
    R = PlatonicSolid(s).Circumradius;
    lat_0 = PlatonicSolid(s).DihedralAngle/2;

    % For each amplitude value, find the planar module sketch and the sketch
    % point further away from the center. Then, find the length of the radius
        for i = 1:length(A_sweep)
            y = A_sweep(i)*a/2*sin(2*pi/a*x); 
            [lat , long] = InverseOrtho(x,y,lat_0,0,R);
            [x1 , y1] = EqAzimuthal(lat,long,pi/2,0,R);
 
            [M,I] = max(- y1);
            % radius value:
            arm(j,i) = sqrt(M^2+x(I)^2)/a;
        end

    % Inverted radius value --> locomotion cost / difficulty
    locom_cost_abs(j,:) = 1./arm(j,:);
    % normalized locomotion cost
    locom_cost(j,:) = locom_cost_abs(j,:)/max(locom_cost_abs(j,:));
end
locom_cost(5,infeas_cutoff_icosa+1:end)=NaN;

% plot the data
figure(2)
hold on
for j = 1:5     % for each Platonic solid
    plot(A_sweep,locom_cost(j,:),'Linewidth',4)
end
grid on; grid minor;
xlabel('Module toplogy curve amplitude (A)')
ylabel('Locomotion Difficulty Score (D_{loco})')
legend('Tetrahedron','Cube','Octahedron','Dodecahedron','Icosahedron')
grid on; grid minor; 
set(gca,'FontSize',36);
%% Final Optimization Problem 

w = 0.56; % weighting parameter

% build plot
 figure(3)
set(gcf,'DefaultAxesFontSize',36)
set(gcf,'DefaultLineLineWidth',4)
 t=tiledlayout(2,5);
%  xlabel(t,'Amplitude, A');
%  ylabel(t, 'Objective Function');
fprintf('Optimal Amplitudes:\n')
for j = 1:5   % for each Platonic solid
    Names = {'Tetrahedron','Cube','Octahedron','Dodecahedron','Icosahedron'};
    nexttile
    hold on
    % plot intermodular distortion vs. amplitude
    plot(A_sweep,D_inter(j,:))

    % plot locom_cost vs. amplitude
    plot(A_sweep,locom_cost(j,:))

    % define objective function with weighting parameter
    obj_fun = w*D_inter(j,:)+(1-w)*locom_cost(j,:);

    % plot objective function
    plot(A_sweep,obj_fun)

    % define optimal amplitude
    A_opt(j) = A_sweep(obj_fun==min(obj_fun));

    % plot vertical line with optimal amplitude
%     xline(A_opt(j),'--k',{"Optimal Amplitude ="+A_opt(j)},'Linewidth',2)
    xline(A_opt(j),'--k','Linewidth',4)

    title(Names{j})

    % report results
    fprintf('%s:    %g \n',Names{j},A_opt(j))
end
title(t,'Optimal module topology curve amplitude for different base platonic solids','FontSize',44)
% add legend
lg = legend({'Intermodular Distortion','Locomotion Cost','Objective Function'},'Orientation','Horizontal');
lg.Layout.Tile = 'North';

%% Plot the optimal amplitude sketches:
% figure(4)
% t2 = tiledlayout(1,5);

for j = 1:5     % for each Platonic solid
    % Define Platonic solid and characteristics
    solid = Solid{j};
    s = find(contains(Solid,solid));
    R = PlatonicSolid(s).Circumradius;
    lat_0 = PlatonicSolid(s).DihedralAngle/2;
    n = PlatonicSolid(s).FaceEdgeNumber;
    
    % find planar module sketch 
    y = A_opt(j)*a/2*sin(2*pi/a*x); 
    [lat , long] = InverseOrtho(x,y,lat_0,0,R);
    [x1 , y1] = EqAzimuthal(lat,long,pi/2,0,R);
    [x_full , y_full] = CompleteSketch(x1 , y1, n);
    
    % plot module sketch
    nexttile
    plot(x_full,y_full)
    axis square
%     title(Names{j}+", A = "+A_opt(j))
    title("A* = "+A_opt(j))
end

