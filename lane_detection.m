clc
% close all
clear all
%%
frame_count=0;

%RED thresholds
    channel1MinY = 162;
    channel1MaxY = 196;
    %GREEN thresholds
    channel2MinY = 111;
    channel2MaxY = 130;
    %BLUE thresholds
    channel3MinY = 41;
    channel3MaxY = 79;
    
    % White Color Masking
    
    %RED thresholds
    channel1MinW = 171;
    channel1MaxW = 187;
    %GREEN thresholds
    channel2MinW = 166;
    channel2MaxW = 177;
    %BLUE thresholds
    channel3MinW = 166;
    channel3MaxW = 180;
    
    
%% loading the mask value
load ('roi_Y.mat');
load ('roi_W.mat');

%% Importing the Video File

VideoFile = VideoReader('test_video.mp4');

%% Defining Variables for saving Video File

Output_Video=VideoWriter('Result');
Output_Video.FrameRate= 25;
open(Output_Video);

%% Image processing of frames

sum = 0;                           % sum of the time taken to process all frames in a video file
while hasFrame(VideoFile)
                                % to find the average time per frame
    clc
    frame = readFrame(VideoFile);
    % subplot(1,8,1)
    % figure(1)
%     imshow(frame);
    % title('Sampled Frame')
    
    %% Gaussaian Filter to remove noise
    
    frame = imgaussfilt3(frame);
    % subplot(1,8,2)
%     figure(1)
%     imshow(frame);
%     title('Filtered Frame')
    
    %% Masking the image for White and Yellow Color
    
    % Yellow Color Masking
    tic
    %Masking
    Yellow=((frame(:,:,1)>=channel1MinY)&(frame(:,:,1)<=channel1MaxY))& ...
        (frame(:,:,2)>=channel2MinY)&(frame(:,:,2)<=channel2MaxY)&...
        (frame(:,:,3)>=channel3MinY)&(frame(:,:,3)<=channel3MaxY);
    
    % subplot(1,8,3)
    % figure(1)
%     imshow(Yellow);
    %title('Yellow Color Masked Frame')
    %%
    
    %Masking
    White=((frame(:,:,1)>=channel1MinW)&(frame(:,:,1)<=channel1MaxW))&...
        (frame(:,:,2)>=channel2MinW)&(frame(:,:,2)<=channel2MaxW)& ...
        (frame(:,:,3)>=channel3MinW)&(frame(:,:,3)<=channel3MaxW);
    %
    % subplot(1,8,4)
    % figure(1)
%    imshow(White);
    % title('White Color Masked Frame')
    sum = sum+toc;
    
    %% Canny Edge detection on yellow and white mask
    
    frameW = edge(White, 'canny', 0.4);
    frameY = edge(Yellow, 'canny', 0.6);
    
    % subplot(1,8,5)
    % imshow(frameY);
    % title('Yellow mask edge detection')
    %
    % subplot(1,8,6)
    % imshow(frameW);
    % title('White mask edge detection')
    
    %% Removing small objects
    
    frameY = bwareaopen(frameY,10);
    frameW = bwareaopen(frameW,10);
    
    % figure(1)
    % imshow(frameY);
    %%
    
    % figure(1)
    % imshow(frameW);
    
    
    %% Region of interest (ROI) from white and yellow frames
    
    frameY = (frameY).*(roi_Y);
    frameW = (frameW).*(roi_W);
    
    % figure(1)
%     imshow(frameY);
    % title('ROI of Yellow mask')
    %%
    
    % figure(1)
%     imshow(frameW);
    % title('ROI of White mask')
    
    %% Application of Hough Transform to find lane white and yellow lines
    
    %Applying Hough Transform to White and Yellow Frames
    
    [H_Y,theta_Y,rho_Y] = hough(frameY);
    [H_W,theta_W,rho_W] = hough(frameW);
    
    %Extracting Hough Peaks from Hough Transform of frames
    
    P_Y = houghpeaks(H_Y,2,'threshold',2);
    P_W = houghpeaks(H_W,2,'threshold',2);
    
    lines_Y = houghlines(frameY,theta_Y,rho_Y,P_Y,'FillGap',2000,'MinLength',8);
    
    lines_W = houghlines(frameW,theta_W,rho_W,P_W,'FillGap',1500,'MinLength',8);
    
    
    
    %% Dislaying lines extracted with the help of Hough peaks on Original frame
    
%     figure(2), imshow(frame), hold on
%     
%     for k = 1:length(lines_Y)
%         xy = [lines_Y(k).point1; lines_Y(k).point2];
%         % Plot the lines with the beginnings and ends of Yellow lines
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','yellow');
%     end
%     
%     for k = 1:2
%         xy = [lines_W(k).point1; lines_W(k).point2];
%         % Plot the lines with the beginnings and ends of White lines
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','yellow');
%     end
%     
%     hold off
    
    %% Computation of finalized line coordinates for the trapazium
    
    [r, c]= size(roi_Y);
    
    % Yellow line coordinates
    leftp1 = [lines_Y(1).point1; lines_Y(1).point2];
    leftp2 = [lines_Y(2).point1; lines_Y(2).point2];
    
    % White line coordinates
    rightp1 = [lines_W(1).point1; lines_W(1).point2];
    rightp2 = [lines_W(2).point1; lines_W(2).point2];
    
    %--------------------------------------------------
    % Finding the inward coordinate by comparing the bottom coordinate of
    % the hough lines of the yellow lane edge
    rhp1 = rho_Y(P_Y(1,1));
    thp1 = theta_Y(P_Y(1,2));
    xp1 = (rhp1 - (r*sin(deg2rad(thp1))))/cos(deg2rad(thp1));
    
    rhp2 = rho_Y(P_Y(2,1));
    thp2 = theta_Y(P_Y(2,2));
    xp2 = (rhp2 - (r*sin(deg2rad(thp2))))/cos(deg2rad(thp2));

    if xp1 > xp2
        a=[leftp1(:,2)]';
        pos= find(a==max(a));
        left_plot(1,:) = leftp1(pos,:);
    else
        a=[leftp2(:,2)]';
        pos= find(a==max(a));
        left_plot(1,:) = leftp2(pos,:);
    end
    
    % Finding the furthest coordinate by comparing all the y coordinates of
    % the hough lines on the yellow lane edge
    a= [[leftp1(:,2)]' [leftp2(:,2)]'];
    pos = find (a==min(a));
    if length(pos)>1
        pos=pos(1);
    end
    if pos>2
        if (pos/2)==2
            left_plot(2,:) = leftp2(2,:);
        else
            left_plot(2,:) = leftp2(1,:);
            
        end
    else
        if pos==2
            left_plot(2,:) = leftp1(2,:);
        else
            left_plot(2,:) = leftp1(1,:);
            
        end
    end
    
    % Finding the furthest coordinate by comparing all the y coordinates of
    % the hough lines on the white lane edge
    
    a= [[rightp1(:,2)]' [rightp2(:,2)]'];
    pos = find (a==min(a));
    if length(pos)>1
        pos=pos(1);
    end
    if pos>2
        if (pos/2)==2
            right_plot(2,:) = rightp2(2,:);
        else
            right_plot(2,:) = rightp2(1,:);
            
        end
    else
        if pos==2
            right_plot(2,:) = rightp1(2,:);
        else
            right_plot(2,:) = rightp1(1,:);
            
        end
    end
    
    %--------------------------------------------------
    % Finding the inward coordinate by comparing the bottom coordinate of
    % the hough lines of the yellow lane edge
    rhp1 = rho_Y(P_W(1,1));
    thp1 = theta_Y(P_W(1,2));
    xp1 = floor(rhp1 - (r*sin(deg2rad(thp1))))/cos(deg2rad(thp1));
    
    rhp2 = rho_Y(P_W(2,1));
    thp2 = theta_Y(P_W(2,2));
    xp2 = floor(rhp2 - (r*sin(deg2rad(thp2))))/cos(deg2rad(thp2));   

    if xp1 < xp2
        a=[rightp1(:,2)]';
        pos= find(a==max(a));
        right_plot(1,:) = rightp1(pos,:);
    else
        a=[rightp2(:,2)]';
        pos= find(a==max(a));
        right_plot(1,:) = rightp2(pos,:);
    end
    
    %Calculatting the slope of left and right lines
    
    slopeL = (left_plot(1,2)-left_plot(2,2))/(left_plot(1,1)-left_plot(2,1));
    slopeR = (right_plot(1,2)-right_plot(2,2))/(right_plot(1,1)-right_plot(2,1));
    
    %Finding coordinates of trapazium coorners
    
    %Bottom corners of the trapezium
    
    Y_bottom_left=r;
    X_bottom_left=floor(left_plot(2,1)+((Y_bottom_left-left_plot(2,2))*(1/slopeL)));
    Y_bottom_right=r;
    X_bottom_right=floor(right_plot(2,1)+((Y_bottom_right-right_plot(2,2))*(1/slopeR)));
    
    %Top corners of the trapezium
    if left_plot(2,2) < right_plot(2,2)        % Lower y coordinate
        Y_top_left=left_plot(2,2);
        X_top_left = left_plot(2,1);
        Y_top_right=left_plot(2,2);
        X_top_right=floor(right_plot(1,1)+((Y_top_right-right_plot(1,2))*(1/slopeR)));
    else
        Y_top_right = right_plot(2,2);
        X_top_right = right_plot(2,1);
        Y_top_left=right_plot(2,2);
        X_top_left=floor(left_plot(1,1)+((Y_top_left-left_plot(1,2))*(1/slopeL)));
    end
    
    %Finding the 4 coordinates of the patch that represent the detected lane 
    points = [X_bottom_left Y_bottom_left; X_bottom_right Y_bottom_right ; X_top_right Y_top_right; X_top_left Y_top_left ];
    %The order in which the coordinates are joined to form a polygon(patch)
    number = [1 2 3 4];
    
    % figure(1)
    % imshow(frame);
    % hold on
    % plot([X_bottom_right X_top_right], [Y_bottom_right, Y_top_right], 'LineWidth',5,'Color','red');
    % plot([X_bottom_left X_top_left], [Y_bottom_left, Y_top_left], 'LineWidth',5,'Color','red');
    % hold off
    
    
    
    %% Turn Prediction
    
    %Intersection Corrdinates of the two lines
    x_intersect=((slopeR*X_bottom_right) - (slopeL*X_bottom_left))/(slopeR - slopeL);
    y_intersect=(slopeL*(x_intersect - X_bottom_left))+Y_bottom_left;
    
    % center point of the base of the trapezium
    lane_center = (X_bottom_right - X_bottom_left)/2+ X_bottom_left;
    
    if (x_intersect/lane_center)<0.9
        predict_dir = " Turn Left ";
    elseif (x_intersect/lane_center)>1
        predict_dir = " Turn Right ";
    else
        predict_dir = " Keep Straight ";
    end
    
    
    
    %% Steering angle determination
    
    angle = acot((r-y_intersect)/abs( lane_center - x_intersect ));
    angle = rad2deg(angle)/12;                     %steering ratio is 12:1 (from online resource)
    
    %% Plotting onto the final frame
    
    figure(3)
    imshow(frame);
    hold on
    plot([X_bottom_left X_top_left], [Y_bottom_left, Y_top_left], 'LineWidth',5,'Color','red');
    plot([X_bottom_right X_top_right], [Y_bottom_right, Y_top_right], 'LineWidth',5,'Color','red');
    plot(x_intersect,Y_top_right,'x','LineWidth',2,'Color','red');
    plot(lane_center,r,'x','LineWidth',2,'Color','red');
    text(900, 60, "Direction:"+predict_dir ,'horizontalAlignment', 'center', 'Color','red','FontSize',20)
    text(900, 115,"Steering angle: "+ num2str(angle),'horizontalAlignment', 'center', 'Color','red','FontSize',20)
    
    patch('Faces', number, 'Vertices', points, 'FaceColor','green','Edgecolor','green','FaceAlpha',0.4)
    hold off
    
    %% Writing the frame to the output file
    writeVideo(Output_Video,getframe);
    frame_count=frame_count+1
    
%  sum = sum+toc;
 end
%% Closing Output Video object
avg= sum/VideoFile.NumFrames;
close(Output_Video)