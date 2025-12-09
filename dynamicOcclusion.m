function occlusionError = dynamicOcclusion(cameras, specs)
% Based on probabilistic occlusion model from Rahimian & Kearney 2017 paper
% Considers all possible orientations of vertical occluder
% occlusionError= Mean occlusion angle across all target points

    numCams = specs.Cams;
    resolution = specs.Resolution;
    TargetSpace = specs.Target;
    
    % Triangulability constraints (from paper)
    minTriangAngle = 40; % degrees
    maxTriangAngle = 140; % degrees
    maxCameraRange = 700; % cm effective range -> update when tested 

    numPoints = size(TargetSpace, 1);
    occlusionAngles = zeros(numPoints, 1);
    
    % Process each target point
    parfor p = 1:numPoints
        point = TargetSpace(p, :);
        
        % Find which cameras can see this point
        [visibleCams, camViewVectors] = findVisibleCameras(point, cameras, numCams, resolution, maxCameraRange);
        
        % Calculate occlusion angle for this point
        occlusionAngles(p) = calculatePointOcclusion(visibleCams, camViewVectors, minTriangAngle, maxTriangAngle);
    end
    
    occlusionError = mean(occlusionAngles)/3.6; %scale to be a point score out of 100
end





