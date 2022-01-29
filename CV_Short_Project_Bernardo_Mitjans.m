%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CAR LICENSE PLATE RECOGNITION %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% by Bruno Bernardo and Marc Mitjans          

close all;
warning('off');
clc;

%% Image initialization and preprocessing

% We start by converting the input image into a grayscale image and
% removing the noise with "medfilt".

[filename, filepath] = uigetfile('*.jpg','*.png');
I_orig = imread([filepath, filename]);

Ig = rgb2gray(I_orig);

% We store the number of rows and columns for further calculus.
[m,n] = size(Ig);

Ig = medfilt2(Ig,[3 3]); 

%% Image filtering - Binary edge map

% Using vertical edge detection preserves enough edge information in the
% plate area while it removes lots of horizontal edges around the License
% Plate.

BW = edge(Ig,'sobel',[],'vertical','nothinning');

%% Coarse candidate detection

% Taking into account that both horizontal and vertical translational steps
% are about 15% of the current width and height and the aspectio ratio is
% placed in 4.0:

aspect_ratio = 4.0;

% We compute the integral image. An integral image
% lets us rapidly calculate summations over image subregions(i.e.
% summations of pixels; and can be performed in constant time, regardless
% of the neighborhood size).

intIm = integralImage(BW); 


% To store the candidates we need to store for each window the following
% values: row, column, height and width.
candidates = zeros(10000,4);

% We slide the window through the image. It is important to know that
% i are rows, j are columns. But, on the other hand, x are columns (j) 
% and y are rows(i).

% Definition of the minimum and maximum height values of the sliding
% window:
H_min = 20;
H_max = 120;

figure; imshow(BW); hold on; title('Candidate detection');

% We store the labels for the 8-connected objects found in BW as well as
% the number of these connected objects.
[L, num] = bwlabel(BW);

% Now, we storage the measures of the Bounding Boxes of all the connected
% components:
regs = regionprops('table',L,'BoundingBox');

% We create a list of edges with Bounding Box coordinates as follows:
% BoundingBox = [number x y width height]
listOfEdges = [(1:num)' regs.BoundingBox]; 

LPmap = zeros(size(BW));

idx = 1;
localization = tic;
tot = tic;
for H = H_min:10:H_max
    % For each window size, the fine candidate detection is performed.
    
    W = aspect_ratio*H;
    
    % Threshold boundaries for fine detection.
    low_th = 0.1*H;
    high_th = H;
   
    mapOfEdges = zeros(size(BW));

    % Edges smaller than the lower threshold and bigger than the higher
    % threshold are removed from the mapOfEdges (we check the height of the
    % Bounding Box):
    
    save_edges = listOfEdges((listOfEdges(1:end,5) >= low_th) & (listOfEdges(1:end,5) <= high_th));
    mapOfEdges(ismember(L,save_edges(:,1))) = 1; % Creation of the mapOfEdges
    
    intIm2 = integralImage(mapOfEdges); % Find integral image I2
    
    % Edges only smaller than the lower threshold are removed from list 
    listOfEdges = listOfEdges(listOfEdges(:,5) >= low_th,:);
    
    % Definition of the thresholds for the fine candidate detection:
    T_min = 0.101;
    T_max = 0.063;
    T_mindiff = 0.021;
    T_maxdiff = 0.026;
    ed_min = T_min - ((T_max - T_min)/(H_max - H_min) * (H - H_min));
    ed_diffmax = T_mindiff - ((T_maxdiff - T_mindiff)/(H_max - H_min) * (H - H_min));
    
    for i = 1:floor(0.15*H):m-H         % Slide vertically. i are rows
        for j = 1:floor(0.15*W):n-W     % Slide horizontally. j are columns
            
            % For each window we need to compute the edge density. 
            % So i and j are ymin, xmin respectively.
            ed_1 = (intIm(i+H,j+floor(W)) - intIm(i,j+floor(W)) - intIm(i+H,j) + intIm(i,j))/(W*H);
           
            % Threshold for coarse candidate detection
            th = 0.17;
            
            if ed_1 > th % Coarse detection
                %% Fine candidate detection
                % If the coarse detection is fulfilled, we enter the fine
                % candidate detection.
                
                % Uncomment for coarse candidate detection:
                rectangle('Position',[j,i,W,H],'linewidth',0.5,'edgecolor','r');
                
                ed_2 = (intIm2(i+H,j+floor(W)) - intIm2(i,j+floor(W)) - intIm2(i+H,j) + intIm2(i,j))/(W*H);
                
                % 1st condition: Overall edge density conditions:
                if (ed_min <= ed_2) && (ed_1 - ed_2 <= ed_diffmax)
                % First condition is succeeded.
                    
                    % 2nd condition: Uniformity condition --> candidates
                    % are divided in four subcandidates and checked by parts

                    Wtemp = floor(W/2);
                    Htemp = floor(H/2);
                    w1 = [i, j];
                    w2 = [i, j + Wtemp];
                    w3 = [i + Htemp, j];
                    w4 = [i + Htemp, j + Wtemp];
                    
                    % Edge density for each subwindow is computed
                    ed1 = (intIm2(w1(1)+Htemp,w1(2)+Wtemp) - intIm2(w1(1),w1(2)+Wtemp) - intIm2(w1(1)+Htemp,w1(2)) + intIm2(w1(1),w1(2)))/(W*H/4);
                    ed2 = (intIm2(w2(1)+Htemp,w2(2)+Wtemp) - intIm2(w2(1),w1(2)+Wtemp) - intIm2(w2(1)+Htemp,w1(2)) + intIm2(w2(1),w1(2)))/(W*H/4);
                    ed3 = (intIm2(w3(1)+Htemp,w3(2)+Wtemp) - intIm2(w3(1),w3(2)+Wtemp) - intIm2(w3(1)+Htemp,w3(2)) + intIm2(w3(1),w3(2)))/(W*H/4);
                    ed4 = (intIm2(w4(1)+Htemp,w4(2)+Wtemp) - intIm2(w4(1),w4(2)+Wtemp) - intIm2(w4(1)+Htemp,w4(2)) + intIm2(w4(1),w4(2)))/(W*H/4);
                    
                    % 2nd condition: Uniformity condition (1st step)
                    if (ed1 >= ed_min && ed2 >= ed_min && ed3 >= ed_min && ed4 >= ed_min)
                        % The uniformity condition: 2nd step needs to be
                        % checked. Each window is divided in 20 horitzontal
                        % and 20 vertical subwindows. If 70% of all
                        % subwindows have an edge density higher than
                        % ed_min, the window is saved.
                        
                        count = 0;
                        
                        nv = 20;
                        Wtemp = floor(W/nv);
                        for it = 1:nv
                            temp_window = [i, j + (it-1)*Wtemp];
                            ed_w = (intIm2(temp_window(1)+H,temp_window(2)+Wtemp) - intIm2(temp_window(1),temp_window(2)+Wtemp)...
                                - intIm2(temp_window(1)+H,temp_window(2)) + intIm2(temp_window(1), temp_window(2)))/(H*Wtemp);
                            if ed_w >= ed_min
                                count = count + 1;
                            end
                        end
                        
                        nh = 20;
                        Htemp = floor(H/nh);
                        for it = 1:nh
                            temp_window = [i + (it-1)*Htemp, j];
                            ed_w = (intIm2(temp_window(1)+Htemp,temp_window(2)+W) - intIm2(temp_window(1),temp_window(2)+W)...
                                - intIm2(temp_window(1)+Htemp,temp_window(2)) + intIm2(temp_window(1), temp_window(2)))/(Htemp*W);
                            if ed_w >= ed_min
                                count = count + 1;
                            end
                        end
                        
                        % Uniformity condition: 2nd step
                        if count >= 0.7*(nv+nh) % higher than 70% of all the subcandidates
                            
                            % If a window reaches this point, it is saved
                            % as a potential LP region:                                                        
                            candidates(idx,:) = [i, j, H, W];
                            LPmap(i:i+H-1,j:j+W-1) = ones(H,W);
                            idx = idx + 1;
                            
                            % Uncomment for fine candidate detection:
                             rectangle('Position',[j,i,W,H],'linewidth',0.5,'edgecolor','r');
                           
                        end

                    end

                end
            end
        end
    end
end

% It computes the processing time of the algorithm.
fprintf('Localization time:\n')
toc(localization);
fprintf('\n');

% Obtaining the final list of candidates:
temp = candidates(candidates~=0);
candidates = reshape(temp,[length(temp)/4, 4]);

%% Character segmentation - Morphology based approach for number plate extraction 

[L2, n2] = bwlabel(LPmap);
props = regionprops('table',L2,'BoundingBox');

final_candidates = zeros(10,6,1);

final_image = {};
boxes = [];
im_idx = 1;

segmentation = tic;
for id = 1:n2
    % For each connected component in L2, we check whether it can be a LP
    % or not(i.e. for each potential LP region)
    
    % Binary map of component 'id'. A full white rectangle is used not to
    % lose any information around.
    box = props.BoundingBox(id,:);
    tempMap = ismember(L2,id);
    tempMap(ceil(box(2)):ceil(box(2))+ceil(box(4)),ceil(box(1)):ceil(box(1))+ceil(box(3))) = 1;
    
    newI = uint8(tempMap) .* Ig; 
    
    % We crop the image to facilitate the further computations:
    BW_new = imcrop(newI,box);
     %figure; imshow(BW_new);
    char_candidates = [];
    
    % The binarization loop starts:
    for ith_n = 0.9:-0.05:0.1

        % Convertion of BW_new to grayscale using the current threshold
        BW2 = not(im2bw(BW_new,ith_n));
         %figure; imshow(BW2); title(strjoin({'Threshold = ',num2str(ith_n)}));
        color = rand(1,3); % To plot each iteration with different color.

        % We perform a dilate and an erode with a disk as structural
        % element and we store the difference in the matrix 'Idiff':
        SE = strel('disk',1);

        imd = imdilate(BW2,SE);
        ime = imerode(BW2,SE);

        Idiff = imd - ime;

        se2 = strel('line',30,0);

        % Erosion with an horitzontal line to remove boundaries of the LP
        I2 = imerode(Idiff,se2);
        I2 = logical(Idiff) & not(logical(I2));
        
        [L8, n8] = bwlabel(I2,8);

        regs8 = regionprops('table',L8,'BoundingBox');

        [H, W] = size(I2);

        if n8~=0 % Now, if the image is not a black image in its 100% we
                 % sort from left to right all the bounding boxes:
            regs8.BoundingBox = sortrows(regs8.BoundingBox,1);
        end

        for ll = 1:n8
            bb = regs8.BoundingBox(ll,:);
            aspect_r = bb(4)/bb(3);

            if aspect_r >= 1.2 && aspect_r <= 4  && bb(4) >= 0.4*H && bb(4) <= 0.9*H
                % This connected component is considered for further
                % evaluation, stored in char_candidates.
                 %rectangle('Position',bb,'linewidth',1,'edgecolor',color);

                % In char_candidates we store in a fifth column the
                % gap until the next character, in
                % order to extract it during the OCR phase. Here they are
                % simply initialized to 1.
                char_candidates(end+1,:) = [bb 1];
                char_candidates = sortrows(char_candidates,1);
                
                % The following lines remove "repeated" bounding boxes,
                % which correspond to a same character. So each BB is
                % checked against the previous one and the next one. They
                % are merged using a weighted average.
                [~,index] = ismember(bb,char_candidates(:,1:4),'rows');
                if index > 1
                    ovrlp1 = bboxOverlapRatio(char_candidates(index,1:4), char_candidates(index-1,1:4));
                    if ovrlp1 >= 0.5 && char_candidates(index,3)*char_candidates(index,4) > char_candidates(index-1,3)*char_candidates(index-1,4)
                        weights = [2.5/ovrlp1 1];
                        char_candidates(index,1:4) = weights*[char_candidates(index,1:4); char_candidates(index-1,1:4)]/sum(weights);
                        char_candidates(index,5) = char_candidates(index,5) + char_candidates(index-1,5);
                        char_candidates(index-1,:) = [];
                        index = index - 1;
                        
                    elseif ovrlp1 >= 0.5
                        weights = [1 2.5/ovrlp1];
                        char_candidates(index,1:4) = weights*[char_candidates(index,1:4); char_candidates(index-1,1:4)]/sum(weights);
                        char_candidates(index,5) = char_candidates(index,5) + char_candidates(index-1,5);
                        char_candidates(index-1,:) = [];
                        index = index - 1;
                    end
                end
                if index < size(char_candidates,1)
                    ovrlp2 = bboxOverlapRatio(char_candidates(index,1:4), char_candidates(index+1,1:4));
                    if ovrlp2 >= 0.5 && char_candidates(index,3)*char_candidates(index,4) > char_candidates(index+1,3)*char_candidates(index+1,4)
                        weights = [2.5/ovrlp2 1];
                        char_candidates(index,1:4) = weights*[char_candidates(index,1:4); char_candidates(index+1,1:4)]/sum(weights);
                        char_candidates(index,5) = char_candidates(index,5) + char_candidates(index+1,5);
                        char_candidates(index+1,:) = [];
                    elseif ovrlp2 >= 0.5
                        weights = [1 2.5/ovrlp2];
                        char_candidates(index,1:4) = weights*[char_candidates(index,1:4); char_candidates(index+1,1:4)]/sum(weights);
                        char_candidates(index,5) = char_candidates(index,5) + char_candidates(index+1,5);
                        char_candidates(index+1,:) = [];
                    end
                end
                
                
            end 

        end

    end
    
    % Obtain a list of potential candidates:
    temp = char_candidates(char_candidates~=0);
    char_candidates = reshape(temp,[length(temp)/5, 5]);
    
%     for lop = 1:size(char_candidates,1)
%         rectangle('Position',char_candidates(lop,1:4),'linewidth',1,'edgecolor','g');
%     end

    % We obtain a list of bounding boxes that represent highly potential characters:
    sure_char_cand = char_candidates(char_candidates(:,5) >= 2,:);
    end_point = sure_char_cand(:,2) +sure_char_cand(:,4);
    sure_char_cand = sure_char_cand(sure_char_cand(:,4) < median(sure_char_cand(:,4))*1.15 & sure_char_cand(:,4) > median(sure_char_cand(:,4))*0.85... 
         & end_point > 0.8*mean(end_point) & end_point < 1.2*mean(end_point),:);

    end_point = sure_char_cand(:,2)+sure_char_cand(:,4);
    char_candidates = char_candidates(char_candidates(:,4) < median(sure_char_cand(:,4))*1.15 & char_candidates(:,4) > median(sure_char_cand(:,4))*0.85... 
         & char_candidates(:,2)+char_candidates(:,4) > 0.8*mean(end_point) & char_candidates(:,2)+char_candidates(:,4) < 1.2*mean(end_point),:);

    char_candidates = sortrows(char_candidates,1);
    
    if size(char_candidates,1) >= 6
        % Start depurating the chosen possible final candidates
        % If some bounding boxes interesct with a value higher than a
        % threshold, one of them is removed.
        new_idx = 2;
        while new_idx <= size(char_candidates,1)
            if new_idx > 1
                inters1 = rectint(char_candidates(new_idx,1:4), char_candidates(new_idx - 1,1:4));
                if inters1 >= 0.6*char_candidates(new_idx,3)*char_candidates(new_idx,4) 
                    char_candidates(new_idx,:) = [];
                    new_idx = new_idx - 1;
                else
                    new_idx = new_idx + 1;
                end
            elseif new_idx == 1
                new_idx = 2;
            else
                new_idx = new_idx + 1;
            end 
        end
        
        % final_candidates contains the coordinates with respect to its
        % cropped image. It is easier for processing.
        % Temporal final characters (for final depuration at the end)
        temp_final_candidates = zeros(size(char_candidates,1),size(char_candidates,2)+1);
        temp_final_candidates(1:size(char_candidates,1),1:size(char_candidates,2)) = char_candidates;
        
        % Now, we add the gaps inside temp_final_candidates and delete those
        % boxes which have high gaps (i.e. too extreme values).
        cand = 1;
        while cand <= size(char_candidates,1)-1
            gap_size = char_candidates(cand+1,1) - char_candidates(cand,1) - char_candidates(cand,3);
            if gap_size > 50
                char_candidates(cand,:) = [];
                temp_final_candidates = zeros(size(char_candidates,1), size(char_candidates,2)+1);
                temp_final_candidates(:,1:size(char_candidates,2)) = char_candidates;
                cand = 1;
            elseif gap_size > 0
                temp_final_candidates(cand,6) = gap_size;    
                cand = cand + 1;
            else
                temp_final_candidates(cand,6) = 1;    
                cand = cand + 1;
            end
        end
        
        % At this point, with these temp_final_candidates we need one last check to
        % remove outliners in the middle of the LP's. The differentiation
        % between the lower part of the BB is computed, and
        % checked against limit conditions.
        
        bottoms = temp_final_candidates(:,2) + temp_final_candidates(:,4);
        dif_bot = diff(bottoms);
        if not(all(dif_bot == 0))
            outl = (dif_bot >= 2.5*max(max(0.3,max(dif_bot(dif_bot<median(abs(dif_bot))))),min(0.3,max(dif_bot(dif_bot<median(abs(dif_bot))))))...
                | dif_bot <= -2.5*max(max(0.3,max(dif_bot(dif_bot<median(abs(dif_bot))))),min(0.3,max(dif_bot(dif_bot<median(abs(dif_bot)))))))...
                & ~(all(dif_bot > 0) | all(dif_bot < 0)) & ((max(dif_bot) - min(dif_bot) > 2.5) & (max(bottoms)-min(bottoms)) > 4.5);
            temp_final_candidates(strfind(outl',[1 1])+1,:) = [];
            if any(strfind(outl',[1 0]) == 1)
                temp_final_candidates(1,:) = [];
            end
            if any(strfind(outl',[0 1]) == length(outl))
                temp_final_candidates(end,:) = [];
            end
        end
        
%         for lop = 1:size(char_candidates,1)
%             rectangle('Position',char_candidates(lop,1:4),'linewidth',1,'edgecolor','g');
%         end

        % Finally, the final_candidates are stored, together with the
        % region where they are located.
        final_candidates(1:size(temp_final_candidates,1),1:size(temp_final_candidates,2),im_idx) = temp_final_candidates;
        final_image{im_idx} = BW_new;

        boxes(im_idx,:) = [box(1) box(2)]; % Store X and Y coordinates for the final window

        im_idx = im_idx + 1;
    end
    

end

fprintf('Segmentation time:\n')
toc(segmentation);
fprintf('\n');

%% Optical Character Recognition

% Show the characters with the binarized images that are going to be the
% input to the OCR system:

load('character');
global chars nums
nums = character(1:10);
chars  = character(11:36);
totals = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};

figure; imshow(I_orig); hold on

ocr = tic;

License_Plates = {};

for k = 1:size(final_candidates,3)
    letter_set = {};
    gaps = [];
    if size(final_candidates(final_candidates(:,1,k)~=0),1) <= 10
        for i = 1:size(final_candidates(final_candidates(:,1,k)~=0),1)
            % For each final_candidate we store the characters in a
            % letter_set.
            temp = final_image{k};
            temp2 = imcrop(temp,final_candidates(i,1:4,k));

            letter_set(end+1) = {temp2};
            gaps(end+1) = final_candidates(i,6,k);

        end
        
        % Flag that chooses whether to study spanish plates or general plates:
        gap_flag = 0;
        if max(gaps) > 3*median(gaps(1:end-1))
            % If there is a considerable high gap in between the
            % characters, we can use the general plate recognition.
            gap_flag = 1;
        end
        
        % Optical Character Recognition
        charact = OCR(letter_set,gaps, gap_flag);

        gaps = gaps > 3*median(gaps(1:end-1))

        if ~isempty(charact) && sum(charact(:,2) >= 0.375) > 4
            plate = [];
            new_box = [final_candidates(:,1,k) + boxes(k,1), final_candidates(:,2,k) + boxes(k,2), final_candidates(:,3:4,k)];
            for let = 1:size(charact,1)
                if (charact(let,1)~=33 && charact(let,2) > 0.375) || (charact(let,1) == 33 && charact(let,2) > 0.3)
                    plate = [plate, totals{1,charact(let,1)}];
                    if gap_flag && gaps(let)
                        plate = [plate, ' '];
                    elseif not(gap_flag) && let == 4
                        plate = [plate, ' '];
                    end
                    % Plot the plate in the image:
                    rectangle('Position',new_box(let,:),'linewidth',1,'edgecolor','r');
                end
            end
            License_Plates(end+1) = {plate};

        end
    end
    
end

fprintf('Optical Character Recognition time: \n');
toc(ocr);
fprintf('\n');

fprintf('Total License Plate Identification time: \n');
toc(tot);
fprintf('\n');

if not(isempty(License_Plates))
    fprintf('Found license plates:\n');
    for k = 1:length(License_Plates)
        display(License_Plates{k});
    end
else
    fprintf('No license plates found.\n');
end





