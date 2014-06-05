function bbs = MBS_track(imgFileNames,bb,options)
% ----------------------------------------
% bbs = MBS_track(imgFileNames,bb,options)
% ----------------------------------------
% A moving-background subtraction based object tracker.
%
% Inputs:
%
%    imgFileNames ... a cell array with N elements of image filenames
%                     that represent individual frames of the input video
%
%    bb ............. bounding box of the target object's location in the
%                     first frame in format [x,y,w,h] where x, y are the
%                     coordinates of the upper-left corner and w, h are the
%                     width and height
%
%    options ........ a struct for defining options (see below for
%                     available options and their default values). To keep
%                     all options values default, options can be an empty
%                     struct
%
% Outputs:
%
%    bbs ............ the resulting matrix of bounding boxes that represent
%                     the tracked target object's positions. The matrix
%                     dimensions are [Nx4], where each row represents a
%                     single frame. Each bounding box is specified in the
%                     form of [x,y,w,h] where x, y are the coordinates of
%                     the upper-left corner and w, h are the width and
%                     height
%
% Otpions:
%
%    skipTracking ... [false] or true. If true, the last step (tracking)
%                     will be skipped. The function will return a NaN.
%
%    saveTD ......... [false] or <filename>. If defined, the original video
%                     with the tracked object's location in each frame will
%                     be saved to an mp4 file specified by the value of
%                     this option (ex. 'tracked.avi')
%
%    saveFG ......... [false] or <filename>. If defined, the extracted
%                     foreground will be saved to an mp4 file specified by
%                     the value of this option (ex. 'fore.avi')
%
%    saveBG ......... [false] or <filename>. If defined, the subtracted
%                     background will be saved to an mp4 file specified by
%                     the value of this option (ex. 'back.avi')
%
% Written by Alexander Besir (alex.besir@gmail.com)
% Faculty of Computer and Information Science Ljubljana
% May, 2014

    %% Options
    
    skipTracking = setOpt(options,'skipTracking',false);
    saveTD = setOpt(options,'saveTD',false);
    saveFG = setOpt(options,'saveFG',false);
    saveBG = setOpt(options,'saveBG',false);
    
    mp4fps = setOpt(options,'mp4fps',20);

    %% Initialization
    bb = round(bb);

    %% [STEP 1] Load images into memory
    
    % Display progres
    progress = 0;
    fprintf('   [STEP 1] Progress: ');
    
    % Load first image
    Imgs_first = cv.imread(imgFileNames{1});
    
    % Get width and height and total image number
    [Imgs_h,Imgs_w,~] = size(Imgs_first);
    Imgs_n = length(imgFileNames);
    
    % Prepare Imgs image data holder
    Imgs = zeros(Imgs_h,Imgs_w,3,Imgs_n);
    Imgs(:,:,:,1) = Imgs_first;
    
    % Load each image
    for i = 2 : Imgs_n
        
        Imgs(:,:,:,i) = imread(imgFileNames{i});
        
        % Display progres
        newProgress = round(i/Imgs_n*10);
        if newProgress > progress
            progress = newProgress;
            fprintf('*');
        end
        
    end
    
    % Convert to uint8
    Imgs = uint8(Imgs);
    
    % Store a copy of original images
    ImgsOrig = Imgs;
    
    % Display progres
    fprintf(' [OK] [Frames: %d]\n',Imgs_n);
    
    %% [STEP 2] Short-term image stabilization
    
    % Display progress
    progress = 0;
    fprintf('   [STEP 2] Progress: ');
    
    % Prepare data holder for transformation matrices
    T = cell(1,Imgs_n);
    
    % Set first image as keyframe
    keyframe = Imgs(:,:,:,1);
    keyframes_n = 1;
    
    for i = 2 : Imgs_n
        
        T{i} = cv.estimateRigidTransform(             ...
            keyframe,Imgs(:,:,:,i),'FullAffine',false ...
        );
        
        if isempty(T{i})
            
            keyframes_n = keyframes_n + 1;
            
            % Set current frame as new keyframe
            keyframe = Imgs(:,:,:,i);
            Imgs(:,:,:,i) = keyframe;
            
        else
            
            % Apply affine transformation
            Imgs(:,:,:,i) = cv.warpAffine(               ...
                Imgs(:,:,:,i), T{i}, 'WarpInverse', true ...
            );
        
        end
        
        % Display progress
        newProgress = round(i/Imgs_n*10);
        if newProgress > progress
            progress = newProgress;
            fprintf('*');
        end
        
    end
    
    % Convert to uint8
    Imgs = uint8(Imgs);
    
    % Display progress
    fprintf(' [OK] [Keyframes: %d]\n',keyframes_n);
    
    %% [STEP 3] Keyframe based background subtraction
    
    % Display progress
    progress = 0;
    fprintf('   [STEP 3] Progress: ');
    
    Medians = cell(0);
    Medians_i = 1;
    
    learningRate = 0;
    
    % Optional saveBG
    Background = NaN;
    if saveBG ~= 0
        Background = zeros(Imgs_h,Imgs_w,3,Imgs_n);
    end
    
    % Subtract background separately for each stabilized series of frames
    % between keyframes
    bs = cv.BackgroundSubtractorMOG2();
    
    for i = 1 : Imgs_n

        if isempty(T{i})
            
            % Find the index of next keyframe
            nextKeyframe_i = i+1;
            for j = i+1 : Imgs_n
                if isempty(T{j}) || j == Imgs_n
                    nextKeyframe_i = j;
                    break;
                end
            end
            
            % Compute the median image of current frame series
            medianBackground = uint8(median(     ...
                Imgs(:,:,:,i:nextKeyframe_i-1),4 ...
            ));
            
            % Find black region produced because of image stabilization in
            % median image
            blackRegion = (                      ...
                (medianBackground(:,:,1) == 0) & ...
                (medianBackground(:,:,2) == 0) & ...
                (medianBackground(:,:,3) == 0)   ...
            );
        
            % Use black region as mask to reconstruct missing values
            missingBackground = uint8(                             ...
                double(Imgs(:,:,:,i)).*repmat(blackRegion,[1,1,3]) ...
            );
            
            % Add missing background to median background image
            medianBackground = medianBackground + missingBackground;
            
            Medians{Medians_i} = medianBackground;
            Medians_i = Medians_i + 1;
            
            % Initialize background subtractor
            history = nextKeyframe_i-i;
            learningRate = 1/history;
            bs = cv.BackgroundSubtractorMOG2(history,16);
            bs.apply(medianBackground);

        end
        
        % Perform background subtraction
        fgmask = bs.apply(Imgs(:,:,:,i),'LearningRate',learningRate);
        
        if saveBG ~= 0
            Background(:,:,:,i) = bs.getBackgroundImage();
        end
        
        % Perform some filtering
        fgmask = bwmorph(fgmask,'open');
        fgmask = bwmorph(fgmask,'close');
        
        % Apply mask
        fgmask = double(fgmask);
        Imgs(:,:,:,i) = uint8(                            ...
            double(Imgs(:,:,:,i)).*repmat(fgmask,[1,1,3]) ...
        );
        
        % Display progress
        newProgress = round(i/Imgs_n*10);
        if newProgress > progress
            progress = newProgress;
            fprintf('*');
        end
        
    end
    
    % Display progress
    fprintf(' [OK]\n');
    
    %% [STEP 4] De-stabilization
    
    % Display progress
    progress = 0;
    fprintf('   [STEP 4] Progress: ');
    
    Medians_i = 0;
    
    for i = 1 : Imgs_n
        
        if ~isempty(T{i})
            
            % Apply affine transformation
            Imgs(:,:,:,i) = cv.warpAffine(                ...
                Imgs(:,:,:,i), T{i}, 'WarpInverse', false ...
            );
        
            % Optionally save background
            if saveBG ~= 0
                Background(:,:,:,i)  = cv.warpAffine( ...
                    uint8(Background(:,:,:,i)), T{i}, ...
                    'WarpInverse', false              ...
                );
            end
        
        else
            
            % Optionally save background
            if saveBG ~= 0
                Medians_i = Medians_i + 1;
                Background(:,:,:,i) = uint8(Medians{Medians_i});
            end
            
        end
        
        % Display progress
        newProgress = round(i/Imgs_n*10);
        if newProgress > progress
            progress = newProgress;
            fprintf('*');
        end
        
    end
    
    % Display progress
    fprintf(' [OK]\n');
  
    %% [******] Optionally save foreground to avi
    
    if saveFG ~= 0
        
        % Display progress
        progress = 0;
        fprintf('   [saveFG] Progress: ');
        
        ForegroundVideo = VideoWriter(fullfile(pwd,saveFG),'MPEG-4');
        ForegroundVideo.FrameRate = mp4fps;
        open(ForegroundVideo);
        
        for i = 1 : Imgs_n
            
            I = Imgs(:,:,:,i);
            I = addFrameData(I,i,Imgs_n,[0,0,0,0]);
            writeVideo(ForegroundVideo,uint8(I));
            
            % Display progress
            newProgress = round(i/Imgs_n*10);
            if newProgress > progress
                progress = newProgress;
                fprintf('*');
            end
            
        end
        
        close(ForegroundVideo);
        
        % Display progress
        fprintf(' [OK] [Saved to: %s]\n',saveFG);
        
    end
    
    %% [******] Optionally save background to avi
    
    if saveBG ~= 0
        
        % Display progress
        progress = 0;
        fprintf('   [saveBG] Progress: ');
        
        BackgroundVideo = VideoWriter(fullfile(pwd,saveBG),'MPEG-4');
        BackgroundVideo.FrameRate = mp4fps;
        open(BackgroundVideo);
        
        for i = 1 : Imgs_n
            
            % Reconstruct missing background from original video
            missingBackground = uint8(        ...
                (Background(:,:,1,i) +        ...
                 Background(:,:,2,i) +        ...
                 Background(:,:,3,i)) == 0    ...
            );
        
            missingBackground = ...
                repmat(missingBackground,[1,1,3]) .* ImgsOrig(:,:,:,i);
            
            Background(:,:,:,i) = ...
                uint8(Background(:,:,:,i)) + missingBackground;
            
            Background(:,:,:,i) = ...
                cv.medianBlur(uint8(Background(:,:,:,i)),'KSize',3);
            
            I = uint8(Background(:,:,:,i));
            I = addFrameData(I,i,Imgs_n,[0,0,0,0]);
            writeVideo(BackgroundVideo,uint8(I));
            
            % Display progress
            newProgress = round(i/Imgs_n*10);
            if newProgress > progress
                progress = newProgress;
                fprintf('*');
            end
            
        end
        
        close(BackgroundVideo);
        
        % Display progress
        fprintf(' [OK] [Saved to: %s]\n',saveBG);
        
    end
    
    %% [******] Optionally skip tracking
    
    if skipTracking
        bbs = NaN;
        return;
    end
    
    %% [STEP 5] CamShift + Kallman filter
    
    % Display progress
    progress = 0;
    fprintf('   [STEP 5] Progress: ');
    
    % Prepare data holder for object centers and sizes
    trckCenters = zeros(Imgs_n,2);
    trckCenters(1,:) = [bb(1)+bb(3)/2,bb(2)+bb(4)/2];
    trckSizes = zeros(Imgs_n,2);
    trckSizes(1,:) = [bb(3),bb(4)];
    
    % Prepare kalman filters for centers and sizes
    kfCenters = cv.KalmanFilter(4,2);
    kfCenters.statePre = [trckCenters(1,1);trckCenters(1,2);0;0];
    kfCenters.transitionMatrix = [1,0,1,0; 0,1,0,1; 0,0,1,0; 0,0,0,1];
    kfCenters.measurementMatrix([1,4]) = 1;
    kfCenters.processNoiseCov = eye(4) * 1e-5;
    kfCenters.measurementNoiseCov = eye(2) * 1e-4;
    kfCenters.errorCovPost = eye(4) * 0.1;
    
    kfSizes = cv.KalmanFilter(4,2);
    kfSizes.statePre = [trckSizes(1,1);trckSizes(1,2);0;0];
    kfSizes.transitionMatrix = [1,0,1,0; 0,1,0,1; 0,0,1,0; 0,0,0,1];
    kfSizes.measurementMatrix([1,4]) = 1;
    kfSizes.processNoiseCov = eye(4) * 1e-4;
    kfSizes.measurementNoiseCov = eye(2) * 1e-1;
    kfSizes.errorCovPost = eye(4) * 10;
    
    % Crop template from first frame
    template = imcrop(Imgs(:,:,:,1),bb);

    % Create a template mask that excludes black pixels
    mask = ~(                    ...
        template(:,:,1) == 0 &   ...
        template(:,:,2) == 0 &   ...
        template(:,:,3) == 0     ...
    );

    % Calculate mask ratio ratio
    mask_ratio = sum(sum(mask))/(size(template,1)*size(template,2));
    
    % Determine if template is good
    template_good = mask_ratio >= 0.5;
    
    % If template is bad, use template from first frame
    if ~template_good
        template = imcrop(Imgs_first,bb);
        mask = ones(size(template,1),size(template,2));
    end
    
    % Calculate template histogram
    bins = 9;
    edges = {                 ...
        linspace(0,256,bins), ...
        linspace(0,256,bins), ...
        linspace(0,256,bins)  ...
    };

    template_hist = cv.calcHist(template,edges,'Mask',mask);

    % Normalize histogram
    template_hist = minMaxNorm(template_hist,255);
    
    % Remember last good box index
    lastGood = 1;
    
    for i = 2 : Imgs_n
        
        % Calculate back projection
        backProjection = ...
            cv.calcBackProject(Imgs(:,:,:,i), template_hist, edges);
        
        % Remove black pixels from backprojection
        backProjection_mask = uint8(~( ...
            (Imgs(:,:,1,i) == 0) &     ...
            (Imgs(:,:,2,i) == 0) &     ...
            (Imgs(:,:,3,i) == 0)       ...
        ));
    
        backProjection = backProjection .* backProjection_mask;
        
        % Calculate search box from previous center and size
        searchBox = createBox(trckCenters(i-1,:),trckSizes(i-1,:));
        
        % Perform cam shift
        crit = struct();
        crit.type = 'Count+EPS';
        crit.maxCount = 10;
        crit.epsilon = 1;
        csEllipse = cv.CamShift(backProjection,searchBox,'Criteria',crit);
        
        % If CamShift returned an invalid box don't trust the measurement
        csEllipseGood = true;
        
        if sum(isnan(csEllipse.size)) > 0
            csEllipseGood = false;
        elseif abs(csEllipse.center(1)-trckCenters(i-1,1)) > trckSizes(i-1,1)
             csEllipseGood = false;
        elseif abs(csEllipse.center(2)-trckCenters(i-1,2)) > trckSizes(i-1,2)
             csEllipseGood = false;
        end
        
        if ~csEllipseGood
            
            % Predict
            kfCenters.predict();
            kfSizes.predict();
            
            % Get last good box
            lgBoxCenter = trckCenters(lastGood,:);
            lgBoxSize = trckSizes(lastGood,:);
            
            % Get pure Kalman predicted values
            kfCenters_s = kfCenters.correct(lgBoxCenter');
            kfSizes_s   = kfSizes.correct(lgBoxSize');
            
            % Save
            trckCenters(i,:) = [kfCenters_s(1),kfCenters_s(2)];
            trckSizes(i,:) = [kfSizes_s(1),kfSizes_s(2)];
            
        else
            
            % Get CamShift ellipse's center
            csBoxCenter = csEllipse.center;
            
            % Get CamShift ellipse's bounding box size
            csBox_img = zeros(Imgs_h,Imgs_w);
            csBox_img = cv.ellipse(                           ...
                csBox_img,csEllipse.center,csEllipse.size/2,  ...
                'Angle',csEllipse.angle,                      ...
                'Thickness',-1,                               ...
                'Color',1                                     ...
            );
            csBox_rp  = regionprops(csBox_img,'BoundingBox');
            csBox_bb  = csBox_rp.BoundingBox;
            csBoxSize = [csBox_bb(3),csBox_bb(4)];
            
            % Predict
            kfCenters.predict();
            kfSizes.predict();
            
            % Correct
            kfCenters_s = kfCenters.correct(csBoxCenter');
            kfSizes_s   = kfSizes.correct(csBoxSize');
            
            % Save
            trckCenters(i,:) = [kfCenters_s(1),kfCenters_s(2)];
            trckSizes(i,:) = [kfSizes_s(1),kfSizes_s(2)];
            
            % Remember this was a good box
            lastGood = i;
            
        end
        
        % Don't allow box to shrink below original size
        trckSizes(i,1) = max(trckSizes(i,1),bb(3));
        trckSizes(i,2) = max(trckSizes(i,2),bb(4));
        
        % Display progress
        newProgress = round(i/Imgs_n*10);
        if newProgress > progress
            progress = newProgress;
            fprintf('*');
        end
        
    end
    
    % Display progress
    fprintf(' [OK]\n');
    
    %% Create bounding boxes from tracking data
    bbs = zeros(Imgs_n,4);
    bbs(1,:) = bb;
    
    for i = 2 : Imgs_n
        bbs(i,:) = createBox(trckCenters(i,:),trckSizes(i,:));
    end
    
    %% [******] Optionally save foreground to avi
    
    if saveTD ~= 0
        
        % Display progress
        progress = 0;
        fprintf('   [saveTD] Progress: ');
        
        TrackedVideo = VideoWriter(fullfile(pwd,saveTD),'MPEG-4');
        TrackedVideo.FrameRate = mp4fps;
        open(TrackedVideo);
        
        for i = 1 : Imgs_n
            
            I = cv.rectangle(ImgsOrig(:,:,:,i),bbs(i,:),'Color',[0,255,0]);
            I = addFrameData(I,i,Imgs_n,bbs(i,:));
            writeVideo(TrackedVideo,uint8(I));
            
            % Display progress
            newProgress = round(i/Imgs_n*10);
            if newProgress > progress
                progress = newProgress;
                fprintf('*');
            end
            
        end
        
        close(TrackedVideo);
        
        % Display progress
        fprintf(' [OK] [Saved to: %s]\n',saveTD);
        
    end
    
end

function bb = createBox(boxCenter,boxSize)
    
    bb = round([                   ...
        boxCenter(1)-boxSize(1)/2, ...
        boxCenter(2)-boxSize(2)/2, ...
        boxSize(1), boxSize(2)     ...
    ]);

end

function histNormed = minMaxNorm(hist,topValue)

    histNormed = hist;
    [~,~,dims] = size(hist);
    
    for d = 1 : dims
        histNormed(:,:,d) = (hist(:,:,d)-min(min(hist(:,:,d)))) / ...
            (max(max(hist(:,:,d)))-min(min(hist(:,:,d))))*topValue;
    end

end

function value = setOpt(optionsStruct,fieldName, defaultValue)
    
   value = defaultValue;
   
   if isfield(optionsStruct,fieldName)
       value = optionsStruct.(fieldName);
   end
    
end

function ImgWithData = addFrameData(Img,currFrame,totalFrames,bb)

    [h,w,~] = size(Img);
    
    ImgWithData = zeros(h+30,w,3);
    ImgWithData(16:16+h-1,:,:) = Img;
    
    txt = ['Frame: ',                                               ...
        num2str(currFrame,'%03d'),' / ',num2str(totalFrames,'%03d') ...
    ];

    ImgWithData = cv.putText(     ...
        ImgWithData, txt, [5,10], ...
        'Color',[255,255,255],    ...
        'FontScale',0.33          ...
    );
    
    txt = [                                ...
        'x = ',sprintf('%03.0f',bb(1)),    ...
        '   y = ',sprintf('%03.0f',bb(2)), ...
        '   w = ',sprintf('%03.0f',bb(3)), ...
        '   h = ',sprintf('%03.0f',bb(4))  ...
    ];

    ImgWithData = cv.putText(       ...
        ImgWithData, txt, [5,h+25], ...
        'Color',[255,255,255],      ...
        'FontScale',0.33            ...
    );

end