% This MATLAB code is made as part of the course project for 
% COMP 765 - Advanced Topics (Mobile Robotics) at McGill University, Canada
% It demonstrates the CAMSHIFT algorithm for object tracking in videos
%
% Author: Srushti Dhope (srushti.dhope@mail.mcgill.ca)
%
% Date: April 29th, 2013

function [ ] = CAMSHIFT( )

clear all;

% Part of video to be used (time in seconds)
tstart = 9;
tend = 15;

% Set constants
T = 1; % Threshold of convergence (in pixels for each deg. of freedom)
P = 5; % Number of pixels to expand search window by
val = 2;

obj = VideoReader('robot.mp4');
fRate = (floor(obj.FrameRate) + 1);
prevframe = read(obj, (tstart-1)*fRate);
imagesc(prevframe,[0,255]); axis off; hold on;
[m,n,~] = size(prevframe);

% Get initial search window around robot
Points = [4,2];
for u=1:4
    [x,y] = ginput(1);
    plot(x,y,'ro')
    Points(u,:) = [x,y];
end
minP = min(Points);
maxP = max(Points);
windowCol = round(minP(1,1));
windowRow = round(minP(1,2));
windowColSize = round(maxP(1,1) - minP(1,1));
windowRowSize = round(maxP(1,2) - minP(1,2));
win = val*ones(m,n);
win(windowRow:windowRow+windowRowSize,windowCol:windowCol+windowColSize) = -val;
contour(win, [0,0], 'r'); hold off; drawnow;
hold off;

for i=tstart:tend
    
    % Read next frame
    currframe = read(obj, i*fRate);
    figure, imagesc(currframe,[0,255]); axis off; hold on;
    image = rgb2hsv(currframe); % Convert to HSV
	image = image(:,:,1); % Extract hue information
    
    % Initialize
    MeanConverge = 1;
    maxIter = 20;
    count = 0;
    
    while MeanConverge && count<maxIter
        
        % Compute centroid of search window
		M00 = 0.0;
        M10 = 0.0;
        M01 = 0.0;
        for u = floor(windowCol-P) : floor(windowCol+windowColSize+P)
            for v = floor(windowRow-P) : floor(windowRow+windowRowSize+P)
                if ~isnan(u) && ~isnan(v)&& u>1 && v>1 && u<n && v<m
                    M00 = M00 + double(image(v,u)); % Zeroth moment
                    M10 = M10 + u * double(image(v,u)); % First moment of x
                    M01 = M01 + v * double(image(v,u)); % First moment of y
                end
            end
        end
        
        % Mean search window location (centroid)
		Xc = round(M10 / M00);
		Yc = round(M01 / M00);

        tempRow = windowRow;
		tempCol = windowCol;
        windowRow = floor(Yc - (windowRowSize/2));
		windowCol = floor(Xc - (windowColSize/2));
       
        % Check threshold
        if abs(tempRow-windowRow) < T || abs(tempCol-windowCol) < T
            MeanConverge = 0;
        end
        
        count = count+1;
        
    end
    
    % Display window
    win = val*ones(m,n);
    win(windowRow:windowRow+windowRowSize,windowCol:windowCol+windowColSize) = -val;
    contour(win, [0,0], 'r'); hold off; drawnow;    
    
    % Use centroid and zeroth moment to adjust the search window size.
    s = round(1.1*sqrt(M00));
    windowColSize = floor(1.2*s);
    windowRowSize = s;
    windowCol = floor(Xc - (windowColSize/2));
    windowRow = floor(Yc - (windowRowSize/2));
    if windowCol<1, windowCol=1; end
    if windowCol+windowColSize>n, windowColSize = n-windowCol; end
    if windowRow<1, windowRow=1; end
    if windowRow+windowRowSize>m, windowRowSize=m-windowRow; end
    
end

end
