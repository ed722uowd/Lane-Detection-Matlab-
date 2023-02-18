VideoFile = VideoReader('test_video.mp4') ;
%%
frame = readFrame(VideoFile);
imshow(frame)
roi_Y = roipoly
%%
frame = readFrame(VideoFile);
imshow(frame)
roi_W = roipoly
%%
figure(1)
imshow(roi_Y);
save('roi_Y');
%%
figure(2)
imshow(roi_W);
save('roi_W');
