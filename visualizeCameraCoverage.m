function coverageStats = visualizeCameraCoverage(cameras, specs, figTitle)
% Visualizes camera coverage of target points with color coding:
% Red: No cameras see the point
% Cyan: Only one camera sees the point  
% Green: Two or more cameras see the point

    numCams = specs.Cams;
    
    %Camera Parameters
    resolution = specs.Resolution;
    TargetSpace = specs.Target;
    
    numPoints = size(TargetSpace,1);
    cameraCoverage = zeros(numPoints,1);
    
    %Amount of cams seeing each point
    for p = 1:numPoints
    point = TargetSpace(p,:);
    visibleCount = 0;
    
        for i = 1:numCams
            uv = cameras{i}.project(point);
            u = uv(1);
            v = uv(2);
            
            % In camera's field of view
            if (u >= 1 && u <= resolution(1) && v >= 1 && v <= resolution(2))
                % Check range based on focal length
                camCenter = cameras{i}.center();
                camCenter = camCenter(:)';  % Ensure row vector
                distance = norm(point - camCenter);
                
                % Determine effective range based on lens type
                if cameras{i}.f == specs.FocalWide
                    effectiveRange = specs.RangeWide;
                else
                    effectiveRange = specs.Range;
                end
                
                if distance <= effectiveRange && distance > 0
                    visibleCount = visibleCount + 1;
                end
            end
        end
        cameraCoverage(p) = visibleCount;
    end

    % Calculate statistics
    coverageStats.numPoints = numPoints;
    coverageStats.zeroCameras = sum(cameraCoverage == 0);
    coverageStats.oneCamera = sum(cameraCoverage == 1);
    coverageStats.twoPlusCameras = sum(cameraCoverage >= 2);
    coverageStats.zeroCamerasPercent = 100*coverageStats.zeroCameras/numPoints;
    coverageStats.oneCameraPercent = 100*coverageStats.oneCamera/numPoints;
    coverageStats.twoPlusCamerasPercent = 100*coverageStats.twoPlusCameras/numPoints;
    coverageStats.avgCoverage = mean(cameraCoverage);
    coverageStats.maxCoverage = max(cameraCoverage);
    coverageStats.minCoverage = min(cameraCoverage);
    coverageStats.medianCoverage = median(cameraCoverage);

    % Colour map based on coverage
    colors = zeros(numPoints, 3);
    parfor p = 1:numPoints
        if cameraCoverage(p) == 0
            colors(p,:) = [1, 0, 0]; % Red: no cameras
        elseif cameraCoverage(p) == 1
            colors(p,:) = [0, 1, 1]; % Cyan: one camera
        else
            colors(p,:) = [0, 1, 0]; % Green: two or more cameras
        end
    end
    

    figure('Name', figTitle, 'Position', [100, 100, 800, 600]);
    % Subplot 1: Colour coded scatter plot
    subplot(1, 2, 1);
    scatter3(TargetSpace(:,1), TargetSpace(:,2), TargetSpace(:,3), 30, colors, 'filled');
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title(figTitle);
    view(45, 30);

    % Subplot 2: Pie chart
    subplot(1, 2, 2);
    pieData = [coverageStats.zeroCamerasPercent, ...
               coverageStats.oneCameraPercent, ...
               coverageStats.twoPlusCamerasPercent];
    pieLabels = {'0 Cameras', '1 Camera', '2+ Cameras'};
    pieColors = [1, 0, 0; 0, 1, 1; 0, 1, 0]; % Match scatter plot colors
    
    p = pie(pieData);
    
    % Set colors for pie slices
    for i = 1:2:length(p)
        p(i).FaceColor = pieColors((i+1)/2, :);
    end
    
    % Add percentage labels to pie slices
    for i = 2:2:length(p)
        p(i).String = sprintf('%.1f%%', pieData(i/2));
        p(i).FontSize = 12;
        p(i).FontWeight = 'bold';
    end
    
    % Add callout for max coverage in 2+ cameras slice
    % Find the text object for "2+ Cameras" slice
    textObjs = findobj(gca, 'Type', 'text');
    for i = 1:length(textObjs)
        if contains(textObjs(i).String, sprintf('%.1f%%', coverageStats.twoPlusCamerasPercent))
            % Add annotation with max coverage
            pos = textObjs(i).Position;
            text(pos(1)*1.3, pos(2)*1.3, sprintf('Max: %d cameras', coverageStats.maxCoverage), ...
                'FontSize', 10, 'FontWeight', 'bold', ...
                'BackgroundColor', 'w', 'EdgeColor', 'k', 'Margin', 3);
            break;
        end
    end
       
    legend(pieLabels, 'Location', 'southoutside', 'Orientation', 'horizontal');
    title('Coverage Distribution'); 

    % Overall figure title
    sgtitle('Coverage for a Normally Discretised Flightspace', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Add cameras to the plot
    % hold on;
    % for i = 1:numCams
    %     hold on;
    %     cameras{i}.plot_camera('label', 'scale', 0.3);
    % end
    
    
    % Add legend
    % h1 = scatter3(nan, nan, nan, 50, [0, 1, 0], 'filled');
    % h2 = scatter3(nan, nan, nan, 50, [0, 1, 1], 'filled');
    % h3 = scatter3(nan, nan, nan, 50, [1, 0, 0], 'filled');
    % legend([h1, h2, h3], {'No cameras', '1 camera', '2+ cameras'}, ...
    %     'Location', 'best');
    
    % Print statistics
    fprintf('\n Camera Coverage Statistics \n');
    fprintf('Points with 0 cameras: %d (%.1f%%)\n', ...
        sum(cameraCoverage == 0), 100*sum(cameraCoverage == 0)/numPoints);
    fprintf('Points with 1 camera: %d (%.1f%%)\n', ...
        sum(cameraCoverage == 1), 100*sum(cameraCoverage == 1)/numPoints);
    fprintf('Points with 2+ cameras: %d (%.1f%%)\n', ...
        sum(cameraCoverage >= 2), 100*sum(cameraCoverage >= 2)/numPoints);
    
    % Calculate average coverage
    avgCoverage = mean(cameraCoverage);
    fprintf('Average camera coverage: %.2f cameras per point\n', avgCoverage);
    fprintf('Maximum camera coverage: %d cameras per point\n', coverageStats.maxCoverage);
    fprintf('Minimum camera coverage: %d cameras per point\n', coverageStats.minCoverage);
    hold off;

end
