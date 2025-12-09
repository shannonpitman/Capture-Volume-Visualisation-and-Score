function errorVolume = resUncertainty(cameras, specs)
%Computes total uncertainty due to image quantisation over a 3D target
%Space for a given chromosome (camera arrangement) 
    numCams = specs.Cams;
    
    %Camera Parameters
    resolution = specs.Resolution;
    TargetSpace = specs.Target;
    
    
    
    adjacentSurfaces = specs.Precomputed.adjacentSurfaces; % [1 2; 2 3; 3 4; 4 1]
    du = specs.Precomputed.du; %pixels
    dv = specs.Precomputed.dv; %pixels
    penaltyUncertainty = specs.Precomputed.penaltyUncertainty; %high uncertainty for points camera cannot see 
        
    numPoints = specs.NumPoints;
    uncertainties = zeros(numPoints,1);
    
    parfor p =1:numPoints
        point = TargetSpace(p,:);
        uncertainties(p) = computePointUncertainty(point, cameras, numCams, resolution, adjacentSurfaces, du,dv, penaltyUncertainty)
    end

    errorVolume = mean(uncertainties);
end
