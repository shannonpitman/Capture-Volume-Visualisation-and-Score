function occlusionError = dynamicOcclusion(cameras, specs)
% Based on probabilistic occlusion model from Rahimian & Kearney 2017 paper
% Considers all possible orientations of vertical occluder
% occlusionError= Mean occlusion angle across all target points

    numCams = specs.Cams;
    resolution = specs.Resolution;
    TargetSpace = specs.Target;
    
    % Triangulability constraints (from paper)
    minTriangAngle = specs.PreComputed.minTriangAngle; % degrees
    maxTriangAngle = specs.PreComputed.maxTriangAngle; % degrees
    maxCameraRange = specs.PreComputed.maxCameraRange; %m -> 16m effective range (OptiTrack)
    maxCameraRangeWide = specs.PreComputed.maxCameraRangeWide; %m -> 9m effective range (optiTrack) wide angle cams
    numPoints = size(TargetSpace, 1);
    occlusionAngles = zeros(numPoints, 1);
    
    % Process each target point
    parfor p = 1:numPoints
        point = TargetSpace(p, :);
        
        % Find which cameras can see this point
        [visibleCams, camViewVectors] = findVisibleCameras(point, cameras, numCams, resolution, maxCameraRange, maxCameraRangeWide);
        
        % Calculate occlusion angle for this point
        occlusionAngles(p) = calculatePointOcclusion(visibleCams, camViewVectors, minTriangAngle, maxTriangAngle);
    end
    
    occlusionError = mean(occlusionAngles); 
end





