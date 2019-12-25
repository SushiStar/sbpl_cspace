
% This file defines the motion primitive for only 4 angles and 6 motions.
% Those four angles are 0, pi/2, pi, pi*3/2
% Thos four movements are forward,backward,sidesteps, turn in places.
%



function[] = genmprim_fourdirections(outfilename)

%defines

LINESEGMENT_MPRIMS = 1; %set the desired type of motion primitives
UNICYCLE_MPRIMS = 0;



if LINESEGMENT_MPRIMS == 1
    % resolution = 0.02500;   %resolution of the map
    resolution = 0.025000;
    
    numberofangles = 8; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 12;

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 1;
    backwardcostmult = 1;
    sidestepcostmult = 1;
    turninplacecostmult = 1;
    
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts0_c(1,:) = [1 0  0  forwardcostmult];
    basemprimendpts0_c(2,:) = [0 1  0 sidestepcostmult];
    basemprimendpts0_c(3,:) = [0 -1 0 sidestepcostmult];
    basemprimendpts0_c(4,:) = [-1 0 0 backwardcostmult];
    %turn in place
    basemprimendpts0_c(5,:) = [0 0 1 turninplacecostmult];
    basemprimendpts0_c(6,:) = [0 0 2 turninplacecostmult];
    basemprimendpts0_c(7,:) = [0 0 3 turninplacecostmult];
    basemprimendpts0_c(8,:) = [0 0 4 turninplacecostmult];
    basemprimendpts0_c(9,:) = [0 0 8 turninplacecostmult];
    basemprimendpts0_c(10,:) = [0 0 6 turninplacecostmult];
    basemprimendpts0_c(11,:) = [0 0 7 turninplacecostmult];
    basemprimendpts0_c(12,:) = [0 0 8 turninplacecostmult];    
    

    
elseif UNICYCLE_MPRIMS == 1
    fprintf(1, 'ERROR: unsupported mprims type\n');
    return;
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
                % 4 angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle             % 16 prims per angle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);   % 8 angles

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        
        %amplify the current angle 0 9000 18000 27000 
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        
        
        %compute which template to use
        angle = currentangle;

            basemprimendpts_c = basemprimendpts0_c(primind,:);    
        
     
            
            
        
        %now figure out what action will be 
        %no hurts
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);
        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));
        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
       
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
     
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        %if (baseendpose_c(2) == 0 && baseendpose_c(3) == 0)
            fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        %end;
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 10;
        intermcells_m = zeros(numofsamples,3);
        if LINESEGMENT_MPRIMS == 1
            startpt = [0 0 currentangle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            for iind = 1:numofsamples
                intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                        startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                        0];
                rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
            end;
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;        
    end;
    grid;
    pause;
end;
        
fclose('all');