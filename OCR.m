function [ w_character ] = OCR(car_plate, gaps, gap_flag)
% This function computes the correlation between the character 
% template and the car plate of the vehicle coming from the binarized 
% image. The output is a string containing the character symbol. 
% The size of the input image must be 42 x 24 pixels in order to compute
% the comparison with the template.

format long g % To perform the comparison of correlations more precisely.

global chars nums
gaps = gaps > 3*median(gaps(1:end-1));

w_character = [];

% We divide the character array into two subarrays: one for the letters 
% and another for the numbers.
let_array = [];
num_array = [];

% Variable defined to count the number of gaps between sets of characters.
count = 0;

for element = 1:size(car_plate,2)
    % We start by computing different binarizations and extracting the best
    % one just before begin with the Optical Character Recognition
    temp_im = car_plate{1,element};
    
    is_one = false;
    
    % POST-PROCESSING:
    % It is aimed to removing any other black pixels than the main
    % letter (in case its bounding box interfered with other
    % obstacles). Also, it seeks to restrain the BB the closest to the
    % letter, to make the OCR comparison easier.
    
    thresh = graythresh(temp_im); 
    temp2 = im2bw(temp_im,thresh);
    temp2 = imdilate(not(temp2),strel('disk',1));
    [Ltemp, ntemp] = bwlabel(temp2,4);
    regs = regionprops('table',Ltemp,'Area','BoundingBox');
    table = [(1:ntemp)' regs.BoundingBox regs.Area];
    
    if not(isempty(table))
        
        table = sortrows(table,-6);
        
        letter = imcrop(ismember(Ltemp,table(1,1)),table(1,2:5));
        
        if size(letter,1)/size(letter,2) > 2.5
            % Checks whether the number can be a 1
            is_one = true;
        end
        letter2 = imresize(letter,[42 24]); % Finally, we rescale the image
        % so we can compare with the templates.
        
%         figure; imshow(not(letter2));
        
        % From now on, we perform the OCR analysis:
        if gap_flag && count == 0 % If there are no gaps, we perform
                                  % the OCR standard method (for spanish
                                  % LPs)
            
            index1 = 1;
            index2 = 1;
        
            correl1 = 0;
            correl2 = 0;
            
            % We have finally decided to separate the character-template 
            % comparisons into two different arrays in order to avoid some
            % extra confusions (i.e. 8 <--> B).
            
            % For each element in the array composed by letters, we 
            % calculate the correlation between the letter coming from the
            % image and the letter coming from the template. We storage 
            % this value as well as the template so we can finally print it.
            for new_element = 1:size(chars,2)

                r1 = corr2(chars{1,new_element},letter2);

                if r1 > correl1
                    let_array(element,:) = [index1, r1];
                    correl1 = r1;
                end
                
                index1 = index1 + 1;
            end
            
            % The same statement for the array of numbers. 
            for new_element = 1:size(nums,2)

                r2 = corr2(nums{1,new_element},letter2);

                if r2 > correl2
                    num_array(element,:) = [index2, r2];
                    correl2 = r2;
                    
                end
                
                index2 = index2 + 1;

            end
            
            % Arrived at this point, we have decided to apply heuristics because of
            % confusions that have appeared along the OCR process. 
            % Basically, we establish certain correlation thresholds that the
            % elements need to pass in order to get storaged. We decided to assign a
            % correlation of 0.6 as a good one. 
  
            % Heuristics to distinguish between 7, 3 and 1
            if is_one && ((num_array(element,2) > 0.3 && num_array(element,1) == 8) || num_array(element,1) == 4)
                num_array(element,:) = [2, 0.6]; % Reliable correlation
            end
            
            % Heuristics to distinguish between 5 and 6
            if (num_array(element,1) == 6 || num_array(element,1) == 7)
                if letter2(28,3) == 1
                    % It is a 6
                    num_array(element,:) = [7, 0.6]; % Reliable correlation
                else
                    % It is a 5
                    num_array(element,:) = [6, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between C and G
            if (let_array(element,1) == 3)
                if (letter2(25,14) == 1)
                    % It is a G
                    let_array(element,:) = [7, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between D and G
            if (let_array(element,1) == 4)
                if (letter2(14,20) == 0)
                    % It is a G
                    let_array(element,:) = [7, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between L and C
            if (let_array(element,1) == 12)
                if (letter2(4,16) == 1)
                    % It is a C
                    let_array(element,:) = [3, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between D and O.
            % In this case, we perform an erode in order to try to
            % discriminate in a better way between these two characters.
            if (let_array(element,1) == 15)
                letter3 = imerode(letter2,strel('disk',2));
                r_D = corr2(chars{1,4},letter3);
                r_O = corr2(chars{1,15},letter3);
                if r_D > r_O
                    % It is a D
                    let_array(element,:) = [4, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between B, S and H.
            if (let_array(element,1) == 19 || let_array(element,1) == 2 || let_array(element,1) == 8)
                if (letter2(13,20) == 1 || letter2(28,6) == 1) && (letter2(4,12) == 1 || letter2(42,12) == 1)
                    % It is a B
                    let_array(element,:) = [2, 0.6]; % Reliable correlation
                elseif (letter2(4,12) == 0 || letter2(42,12) == 0)
                    % It is a H
                    w_character(element,:) = [8, 0.6]; % Reliable correlation
                else
                    % It is a S
                    let_array(element,:) = [19, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between W and R.
            if(let_array(element,1) == 18)
                if (letter2(11,12) == 1)
                    % It is a W
                    let_array(element,:) = [23, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between M, W and V.
            if(let_array(element,1) == 22 || let_array(element,1) == 13)
                if (letter2(7,12) == 1)
                    % It is a W
                    let_array(element,:) = [23, 0.6]; % Reliable correlation
                elseif (letter2(39,5) == 1 || letter2 (39,18) == 1)
                    % It is a M
                    let_array(element,:) = [13, 0.6]; % Reliable correlation
                end
            end
            
            % Here we calculate if there is a gap or not between a
            % set of characters.
            if gaps(element) == 1
                
                % In the case there is at least one gap, we look at first
                % if the preliminary set of characters are letters or numbers
                conj = let_array(:,2) - num_array(:,2);
                conj = conj > 0;
                
                if sum(conj) > length(conj)/2
                    % If there exists a higher correlation with letters 
                    % in comparison to the numbers:
                    w_character = [let_array(:,1)+10 let_array(:,2)];
                    count = 2; % The next character set will be a number set.
                else
                    w_character = num_array;
                    count = 1; % The next character set will be a letter set.
                end
                
            end
        
        elseif (~gap_flag && (element > 4)) || count == 1
            % In the case the next set is a letter set:
        
            index1 = 1;
        
            correl1 = 0;

            for new_element = 1:size(chars,2)

                r1 = corr2(chars{1,new_element},letter2);

                if r1 > correl1
                    w_character(element,:) = [index1 + 10, r1];
                    correl1 = r1;
                end
                
                index1 = index1 + 1;
            end
            
            % Heuristics to distinguish between C and G
            if (w_character(element,1) == 13)
                if (letter2(25,14) == 1)
                    % It is a G
                    w_character(element,:) = [17, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between D and G
            if (w_character(element,1) == 14)
                if (letter2(14,20) == 0)
                    % It is a G
                    w_character(element,:) = [17, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between L and C.
            if (w_character(element,1) == 22)
                if (letter2(4,16) == 1)
                    % It is a C
                    w_character(element,:) = [13, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between D and O.
            % We return to perform an erode in order to try to
            % discriminate in a better way between these two characters.
            if (w_character(element,1) == 25)
                letter3 = imerode(letter2,strel('disk',2));
                r_D = corr2(chars{1,4},letter3);
                r_O = corr2(chars{1,15},letter3);
                if r_D > r_O
                    % It is a D
                    w_character(element,:) = [14, 0.6];
                else
                    % Further experimentation to verify that D is the
                    % pursued character.
                    if letter3(6,3) == 0
                        % It is a D
                        w_character(element,:) = [14, 0.6]; % Reliable correlation
                    end
                end
            end
            
            % Heuristics to distinguish between B, S and H.
            if (w_character(element,1) == 29 || w_character(element,1) == 12 || w_character(element,1) == 18)
                if (letter2(13,20) == 1 || letter2(28,6) == 1) && (letter2(4,12) == 1 || letter2(42,12) == 1)
                    % It is a B
                    w_character(element,:) = [12, 0.6]; % Reliable correlation
                elseif (letter2(4,12) == 0 || letter2(42,12) == 0)
                    % It is a H
                    w_character(element,:) = [18, 0.6]; % Reliable correlation
                else
                    % It is a S
                    w_character(element,:) = [29, 0.6]; % Reliable correlation
                end
            end
            
            % Heuristics to distinguish between W and R.
            if(w_character(element,1) == 28)
                if (letter2(11,12) == 1)
                    % It is a W
                    w_character(element,:) = [33, 0.6];
                end
            end
            
            % Heuristics to distinguish between M, W and V.
            if(w_character(element,1) == 32 || w_character(element,1) == 23)
                if (letter2(7,12) == 1)
                    % It is a W
                    w_character(element,:) = [33, 0.6]; % Reliable correlation
                elseif (letter2(39,5) == 1 || letter2 (39,18) == 1)
                    % It is an M
                    w_character(element,:) = [23, 0.6]; % Reliable correlation
                end
            end
    
            if gaps(element) == 1
                count = 2; % The next character set will be a number set.
            end
            
        elseif (~gap_flag && element <= 4) || count == 2
            % In the case the next set is a number set:
                        
            index2 = 1;
        
            correl2 = 0;
            
            for new_element = 1:size(nums,2)
                
                r2 = corr2(nums{1,new_element},letter2);

                if r2 > correl2
                    w_character(element,:) = [index2, r2];
                    correl2 = r2;
                end
                
                index2 = index2 + 1;
                                
            end
            
            % Heuristics to distinguish between 7, 3 and 1.
            if is_one && ((w_character(element,2) > 0.3 && w_character(element,1) == 8) || w_character(element,1) == 4)
                w_character(element,:) = [2, 0.6];
            end
            
            % Heuristics to distinguish between 5 and 6.
            if (w_character(element,1) == 6 || w_character(element,1) == 7)
                if letter2(28,3) == 1
                    % It is a 6
                    w_character(element,:) = [7, 0.6]; % Reliable correlation
                else
                    % It is a 5
                    w_character(element,:) = [6, 0.6]; % Reliable correlation
                end
            end

            if gaps(element) == 1
                count = 1; % The next character set will be a number set.
            end
            
        end
        
    end

   

end

return