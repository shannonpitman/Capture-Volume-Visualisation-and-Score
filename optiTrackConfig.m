%OptiTrack and/or Single Configuration
%Runs the cost function based on designer choice 
% Main adaptation from main code is the fact that the cameras configuration array is
% computed before the cost functions (i.e is the input as opposed to the camera chromosome)  


%% Design Specifications
specs.Cams = 7; %Number of Cameras

%OptiTrack Parameters Taken from their specs. 
specs.Resolution = [1280 1024]; % Resolution
specs.PrincipalPoint = [specs.Resolution(1)/2, specs.Resolution(2)/2];
specs.Focal = 0.0055; %focal length [m]
specs.FocalWide = 0.0035; % focal length for wide angle camera [m]
specs.Range = 16;%m for passive markers
specs.RangeWide = 9;%m for passive markers

% Target space is an uniformly discretised grid within the flight volume 
% This workspace volume matches the available dimensions of the MS.G flight envelope 
flight_envelope = [-4 4; -4 4; 0 4]; %m
spacing = 0.5;
x_marker = flight_envelope(1,1):spacing:flight_envelope(1,2);
y_marker = flight_envelope(2,1):spacing:flight_envelope(2,2);
z_marker = flight_envelope(3,1):spacing:flight_envelope(3,2);
[X,Y,Z] = meshgrid(x_marker, y_marker, z_marker);

specs.Target = [X(:), Y(:), Z(:)];
specs.NumPoints = size(specs.Target, 1);

numSections = floor(specs.Cams/2);
nx = ceil(sqrt(numSections)); %number of divisions in x direction of grid 
ny = ceil(numSections /nx); %number of divisions in y direction of grid 
x_div = linspace(flight_envelope(1,1), flight_envelope(1,2), nx+1);
y_div = linspace(flight_envelope(2,1), flight_envelope(2,2), ny+1);
z_mid = mean(flight_envelope(3,:)); %mid-height

count = 1;
section_centres = zeros(numSections,3);
for i = 1:length(x_div)-1
    for j = 1:length(y_div)-1
        if count <= numSections
            section_centres(count,:) = [(x_div(i) + x_div(i+1))/2, (y_div(j)+ y_div(j+1))/2, z_mid];
            count = count+1;
        end
    end
end

specs.SectionCentres = section_centres;

% Pre-compute data for resolution uncertainty
specs.PreComputed.adjacentSurfaces = [1 2; 2 3; 3 4; 4 1];
specs.PreComputed.du = 0.5; %pixels
specs.PreComputed.dv = 0.5; %pixels
specs.PreComputed.penaltyUncertainty = 100;

% Pre-compute data for dynamic occlusion
specs.PreComputed.minTriangAngle = 40; % degrees
specs.PreComputed.maxTriangAngle = 140; % degrees
specs.PreComputed.maxCameraRange = specs.Range; %m effective range
specs.PreComputed.maxCameraRangeWide = specs.RangeWide; %m effective range

specs.PreComputed.uncertNorm = 100;  
specs.PreComputed.occlNorm = 440;  


%% Comparison to OpTitrack setup 
% These values come from Motive's calibration file 
% Using the world camera positions and orientations from OptiTrack 
% First convert form OptiTrack's Y-up coord system to Matlab Z-up 
% OptiTrack uses the OpenGL convention
specs.Cams = 7; %Number of Cameras
%T_transform = eye(3);
% R_CV = [1, 0 , 0;
%         0, -1, 0;
%         0, 0, 1];
% R_Mat = [1, 0, 0;
%          0 0, 1;
%          0, 1, 0];
T_transform = [1,  0,  0;
                0, 0, -1;
                0, 1,  0];
% T_transform = R_CV*R_Mat
rotX = [1, 0, 0;
        0, cos(pi), -sin(pi);
        0, sin(pi), cos(pi)];
cam1Pos =  [-4.31837, 3.16485, -1.40049]; %rotate 180 degrees around xy plane
cam1Pos = T_transform*cam1Pos';
cam1Orientation = [-0.0964006, 0.553382,-0.82733;
		           -0.0128461, 0.830441, 0.556959;
                    0.99526, 0.0643192, -0.0729462];
R_new_cam1 = T_transform * cam1Orientation; %* T_transform';
R_new_cam1 = R_new_cam1* rotX;
% det(R_new_cam1);
T1 = se3(R_new_cam1, cam1Pos');
camera1 = CentralCamera(name="cam1", pose=T1, resolution = specs.Resolution, focal = specs.FocalWide, center = specs.PrincipalPoint);

cam2Pos = [-4.3058, 3.34875, -4.15416];
cam2Pos = T_transform*cam2Pos';
cam2Orientation = [-0.463454, 0.348497,-0.814715;
		            0.107535, 0.934741, 0.338667;
		            0.879572, 0.0693455,-0.470685];
R_new_cam2 = T_transform * cam2Orientation;% * T_transform';
R_new_cam2 = R_new_cam2 * rotX; 
T2= se3(R_new_cam2, cam2Pos');
camera2 = CentralCamera(name="cam2", pose=T2, resolution = specs.Resolution, focal = specs.Focal, center = specs.PrincipalPoint);

cam3Pos = [4.41376, 3.32004, -4.03407];
cam3Pos = T_transform* cam3Pos';
cam3Orientation =[-0.383207, -0.381934, 0.840999;
		          -0.0928076, 0.921818, 0.376349;
		          -0.918988, 0.0661683,-0.388693];
R_new_cam3 = T_transform * cam3Orientation;% * T_transform';
R_new_cam3 = R_new_cam3 * rotX; 
T3= se3(R_new_cam3, cam3Pos');
camera3 = CentralCamera(name="cam3", pose=T3, resolution = specs.Resolution, focal = specs.Focal, center = specs.PrincipalPoint);

cam4Pos = [4.38502, 3.27992, -1.30325];
cam4Pos = T_transform*cam4Pos';
cam4Orientation = [-0.0468672, -0.55851, 0.828173;
		           -0.0167187, 0.829406, 0.558395;
		           -0.998761, 0.0123245, -0.0482095];
R_new_cam4 = T_transform * cam4Orientation;% * T_transform';
R_new_cam4 = R_new_cam4 * rotX;
T4= se3(R_new_cam4, cam4Pos');
camera4 = CentralCamera(name="cam4", pose=T4, resolution = specs.Resolution, focal = specs.FocalWide, center = specs.PrincipalPoint);

cam5Pos = [4.36588, 3.21471, 1.41838];
cam5Pos = T_transform*cam5Pos';
cam5Orientation = [0.143307, -0.376961, 0.915076;
		           0.0465306, 0.926163, 0.374242;
                  -0.988584, -0.0110526, 0.150266];
R_new_cam5 = T_transform * cam5Orientation;% * T_transform';
R_new_cam5 = R_new_cam5 * rotX; 
T5= se3(R_new_cam5, cam5Pos');
camera5 = CentralCamera(name="cam5", pose=T5, resolution = specs.Resolution, focal = specs.FocalWide, center = specs.PrincipalPoint);

cam6Pos = [4.33365, 3.33527, 4.16674];
cam6Pos = T_transform*cam6Pos';
cam6Orientation = [0.46651, -0.359219, 0.808289;
		            0.101597, 0.929534, 0.354465;
		           -0.878662, -0.0832416, 0.470132];
R_new_cam6 = T_transform * cam6Orientation;% * T_transform';
R_new_cam6 = R_new_cam6 * rotX; 
T6= se3(R_new_cam6, cam6Pos');
camera6 = CentralCamera(name="cam6", pose=T6, resolution = specs.Resolution, focal = specs.Focal, center = specs.PrincipalPoint);

cam7Pos = [-4.39346, 3.30429, 4.03519];
cam7Pos = T_transform*cam7Pos';
cam7Orientation = [0.770965, 0.395039, -0.499556;
                  -0.0314617, 0.80705, 0.589644;
                   0.6361,-0.438878, 0.634636];
R_new_cam7 = T_transform * cam7Orientation;% * T_transform';
R_new_cam7 = R_new_cam7 * rotX;
T7= se3(R_new_cam7, cam7Pos');
camera7 = CentralCamera(name="cam7", pose=T7, resolution = specs.Resolution, focal = specs.Focal, center = specs.PrincipalPoint);


cameras = {camera1, camera2, camera3, camera4, camera5, camera6, camera7};

%% Plots
currentDateTime = datetime('now');
dateTimeStr = string(currentDateTime, 'yyyyMMdd_HHmmSS');

% Choose cost function:
% 1 = Resolution Uncertainty only
% 2 = Dynamic Occlusion only  
% 3 = Combined (weighted)

costFunctionType = 3; % Change this to select cost function

%Weights for combined cost function (only used if costFunctionType = 3)
%Weights need to sum to 1
specs.WeightUncertainty = 0.5; % Resolution uncertainty weight
specs.WeightOcclusion = 0.5; % Dynamic occlusion weight

% Select cost function
switch costFunctionType
    case 1
        CostFunction = @resUncertainty;
    case 2
        CostFunction = @dynamicOcclusion;
    case 3
        CostFunction = @combinedCostFunction;
end


OptiCost = CostFunction(cameras, specs);

figure;

hold on;
camera1.plot_camera('label', scale = 0.8);
camera2.plot_camera('label', scale = 0.8);
camera3.plot_camera('label', scale = 0.8);
camera4.plot_camera('label', scale = 0.8);
camera5.plot_camera('label', scale = 0.8);
camera6.plot_camera('label', scale = 0.8);
camera7.plot_camera('label', scale = 0.8);

axis('equal')
grid('on')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title(sprintf('OptiTrack Camera Placement - Cost: %.4f',OptiCost))
view(45, 30)
hold off

cameraPlotFilename = sprintf('%dCams_Run_%s_cameras.png', specs.Cams, dateTimeStr);
saveas(gcf, cameraPlotFilename); %saves as png


figTitle = sprintf('Camera Coverage of OptiTrack ad hoc Configuration - %d Cameras (Cost: %.4f)', specs.Cams, OptiCost);
[coverageStats] = visualizeCameraCoverage(cameras, specs, figTitle);
coveragePlotFilename = sprintf('%dCams_Run_%s_coverage.png', specs.Cams, dateTimeStr);
saveas(gcf, coveragePlotFilename);
fprintf('Coverage plot saved to: %s\n', coveragePlotFilename);
% Text summary
txtFilename = sprintf('OptiTrack_%dCams_%s.txt',7,dateTimeStr);
fid = fopen(txtFilename, 'w');
fprintf(fid, 'Camera Coverage Statistics:\n');
fprintf(fid, '---------------------------\n');
fprintf(fid, 'Total target points: %d\n', coverageStats.numPoints);
fprintf(fid, 'Points with 0 cameras: %d (%.1f%%)\n', ...
    coverageStats.zeroCameras, coverageStats.zeroCamerasPercent);
fprintf(fid, 'Points with 1 camera: %d (%.1f%%)\n', ...
    coverageStats.oneCamera, coverageStats.oneCameraPercent);
fprintf(fid, 'Points with 2+ cameras: %d (%.1f%%)\n', ...
    coverageStats.twoPlusCameras, coverageStats.twoPlusCamerasPercent);
fprintf(fid, '\nCoverage Metrics:\n');
fprintf(fid, '  Average coverage: %.2f cameras per point\n', coverageStats.avgCoverage);
fprintf(fid, '  Maximum coverage: %d cameras per point\n', coverageStats.maxCoverage);
fprintf(fid, '  Minimum coverage: %d cameras per point\n', coverageStats.minCoverage);
fprintf(fid, '  Median coverage: %.2f cameras per point\n', coverageStats.medianCoverage);

fclose(fid);
fprintf('Summary saved to: %s\n', txtFilename);