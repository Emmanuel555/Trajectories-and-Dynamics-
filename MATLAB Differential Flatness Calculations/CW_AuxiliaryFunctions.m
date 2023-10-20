classdef CW_AuxiliaryFunctions
    methods

        function [new_heading] = new_heading_input(obj,heading)
            phase_delay = pi/2 + pi/4;
            new_heading = heading + phase_delay;

            if new_heading > pi
                new_heading = (new_heading - pi) - pi;
            end

        end

        function [quadrant] = quadrant_output(obj,heading)
    
            quad = zeros(1,1); % unsigned, let the gain settle the sign
            
            % for experiment, its clockwise direction 
            if heading == 0
                quad = 0;
            
            elseif heading < 0 && heading > -pi/2
                quad = 1;
            
            elseif heading == -pi/2
                quad = 1.5;
            
            elseif heading < -pi/2 && heading > -pi
                quad = 2;
            
            elseif heading == -pi
                quad = 2.5;
            
            elseif heading > pi/2 && heading < pi
                quad = 3; 
            
            elseif heading == pi/2
                quad = 3.5;
            
            elseif heading < pi/2 && heading > 0
                quad = 4; 
            
            else 
                quad = -1;
            end
            
%             if toggle == 0
%                 quad = -1;
%             end
            
            quadrant = quad;
            
        end

        function [input] = flap_output(obj, azi, quadrant, gain_disk_pitch, desired_heading, body_rate_y)
            % rmb to put filter to prevent over actuation
            input = zeros(1,2);
            upper_bound = zeros(1,1);
            lower_bound = zeros(1,1);
            azimuth = azi;
            heading = desired_heading;
            pitch = zeros(1,1);
            Motor_Pulse = zeros(1,1);
            phase_delay = pi/2;
            
            if quadrant == 0 
                upper_bound = pi/2 + pi/4; 
                lower_bound = pi/2 - pi/4;
                if abs(azimuth) < upper_bound && abs(azimuth) > lower_bound
                    tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                    pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                    Motor_Pulse =  (pitch/abs(pitch)) * 1;
                else
                    pitch = 0;
                    Motor_Pulse = 0;
                end


            elseif quadrant == 1
                upper_bound = (-pi/2) + heading + pi/4; 
                lower_bound = (-pi/2) + heading - pi/4;
                activate_cos = zeros(1,1);
                if lower_bound < -pi
                    lower_bound = (lower_bound + pi) + pi;
                    %lower_bound = -pi;
                    activate_cos = 1;
                end
                if activate_cos == 0
                    if (azimuth < upper_bound && azimuth > lower_bound) || (azimuth < pi + upper_bound && azimuth > pi + lower_bound) % otherside
                        tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end
                else
                    if azimuth < upper_bound || azimuth > lower_bound
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction             
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;       
                    elseif azimuth < upper_bound + pi && azimuth > lower_bound - pi % otherside
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end
                end
            
                
            elseif quadrant == 1.5 
                upper_bound = pi + pi/4; 
                lower_bound = pi/4; 
                % no need for lower bound
                if abs(azimuth) > upper_bound || abs(azimuth) < lower_bound
                    tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                    pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading 
                    Motor_Pulse =  (pitch/abs(pitch)) * 1;
                else
                    pitch = 0;
                    Motor_Pulse = 0;
                end
            
            
            elseif quadrant == 2
                upper_bound = pi/2 + heading + pi/4; 
                lower_bound = pi/2 + heading - pi/4;
                activate_cos = zeros(1,1); 
            
                if upper_bound > 0
                    %lower_bound = (lower_bound + pi) + pi;
                    %lower_bound = 0;
                    activate_cos = 1;
                end
            
                if activate_cos == 0
                    if (azimuth < upper_bound && azimuth > lower_bound) || (azimuth > pi + lower_bound && azimuth < pi + upper_bound) % otherside first
                        tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end
                else
                    if azimuth > pi + lower_bound || azimuth < -(pi - upper_bound)
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    elseif azimuth < upper_bound && azimuth > lower_bound
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end
                end
            
            
            elseif quadrant == 2.5
                upper_bound = pi/2 + pi/4; 
                lower_bound = pi/2 - pi/4;
                if abs(azimuth) < upper_bound && abs(azimuth) > lower_bound
                    tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                    pitch = tilt_xw; % only for cyclic y
                    Motor_Pulse =  (pitch/abs(pitch)) * 1;
                else
                    pitch = 0;
                    Motor_Pulse = 0;
                end
            
            
            elseif quadrant == 3
                upper_bound = (-pi/2) + heading + pi/4; 
                lower_bound = (-pi/2) + heading - pi/4;
                activate_cos = zeros(1,1);  
                if lower_bound < 0
                    %lower_bound = (lower_bound + pi) + pi;
                    activate_cos = 1;
                end
            
                if activate_cos == 0
                    if (azimuth < upper_bound && azimuth > lower_bound) || (azimuth > -pi + lower_bound && azimuth < -pi + upper_bound) % otherside first
                        tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * gain_disk_pitch * -1; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end  
                else
                    if azimuth < upper_bound && azimuth > lower_bound
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * gain_disk_pitch * -1; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    elseif azimuth > pi + lower_bound || azimuth < -pi + upper_bound % otherside first
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * gain_disk_pitch * -1; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end 
                end
            
            
            elseif quadrant == 3.5
                upper_bound = pi + pi/4; 
                lower_bound = pi/4; 
                % no need for lower bound
                if abs(azimuth) > upper_bound || abs(azimuth) < lower_bound
                    tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                    pitch = tilt_xw;
                    Motor_Pulse =  (pitch/abs(pitch)) * 1;
                else
                    pitch = 0;
                    Motor_Pulse = 0;
                end
            
            
            elseif quadrant == 4
                upper_bound = pi/2 + heading + pi/4; 
                lower_bound = pi/2 + heading - pi/4;
                activate_cos = zeros(1,1); 
            
                if upper_bound > pi
                    %lower_bound = (lower_bound + pi) + pi;
                    %lower_bound = 0;
                    activate_cos = 1;
                end
            
                if activate_cos == 0
                    if (azimuth < upper_bound && azimuth > lower_bound) || (azimuth > -pi + lower_bound && azimuth < -pi + upper_bound) % otherside first
                        tilt_xw = sin(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw * -1; % only for cyclic y, -1 because craft is always pitching forward wrt to shifts in heading
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                    else
                        pitch = 0;
                        Motor_Pulse = 0;
                    end
                else
                     if azimuth > lower_bound || azimuth < -pi + upper_bound - pi
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                     elseif azimuth < upper_bound - pi && azimuth > -pi + lower_bound
                        tilt_xw = cos(azimuth); % tilt about yw heading in x direction 
                        pitch = tilt_xw; % only for cyclic y
                        Motor_Pulse =  (pitch/abs(pitch)) * 1;
                     else
                        pitch = 0;
                        Motor_Pulse = 0;
                     end
                end
            
            else
                Motor_Pulse =  0;
                pitch = 0;
            end
            
            
            % if Toggle == 1 % Front Motor    
            %     if abs(pitch) > 0.75
            %         pitch = (pitch/abs(pitch)) * 0.75;
            %     end
            % elseif Toggle == 0
            %     Motor_Pulse =  0;
            %     pitch = 0;
            % end
            
            input(:,1) = pitch * body_rate_y;
            input(:,2) = Motor_Pulse;
            
        end


        function [xunit,yunit] = circle(obj,x,y,r)
            % hold on
            x_origin = x;
            y_origin = y;
            radius = r;
            %th = 0:pi/50:2*pi; % 100 pts
            th = 0:pi/4:2*pi; % 8 pts
            xunit = radius * cos(th) + x_origin;
            yunit = radius * sin(th) + y_origin;
            z = ones([size(xunit)]);
            %overall = plot3(xunit, yunit, z, "red",'LineWidth',5);
            %xlabel('X')
            %ylabel('Y')
            %ylabel('Z')
            %axis equal
            % hold off
        end


        function [mag] = circle_setpoints_anti_cw(obj,speed,x_rad,y_rad,r)    
            %CIRCLE
            x_origin = x_rad;
            y_origin = y_rad;
            radius = r;
            %th = 0:pi/50:2*pi; % 100 pts
            th = 0:pi/4:2*pi; % 8 pts
            x = radius * cos(th) + x_origin;
            y = radius * sin(th) + y_origin;
           
            
            % wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
            %wpts_zy = [0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0; 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0]; 
            circle_wpts_xy = [x x(:,2:end) x(:,2:end) x(:,2:end); y y(:,2:end) y(:,2:end) y(:,2:end)]; % repeats for 4 times to ensure smoothness but we only take the middle points
            % tpts = 0:5;
            
            omega = speed / r; % 1 m/s = 1 rad/s based on v = r*omega
%             time_per_eigth = (pi/4)/speed; 
%             sample_per_loop = 100;
%             time_per_setpt = (time_per_eigth * 8) / sample_per_loop; 

            time_per_eigth = (pi/4)/omega; % 0.784
            hz = 360;
            time_per_setpt = 1/hz;
            sample_per_loop = (time_per_eigth * 8) / time_per_setpt;  % 2258
            sample_per_loop = ceil(sample_per_loop); % only need to change here...
            % 1 m/s = (pi/4) / (1 rad/s) = pi/4
            % 2 m/s = (pi/4) / (2 rad/s) = pi/8
            % 3 m/s = (pi/4) / (3 rad/s) = pi/12
            % 4 m/s = (pi/4) / (4 rad/s) = pi/16
            
            circle_tpts = 0:time_per_eigth:time_per_eigth*8*4; % intervals @ 1 m/s = 1 rad/s based on v = r*omega
            
            numsamples = sample_per_loop * 4;
            
            [circle_xy,circle_xyd,circle_xydd,circle_xyddd,circle_xydddd,circle_xypp,circle_xytimepoints,circle_xytsamples] = minsnappolytraj(circle_wpts_xy,circle_tpts,numsamples);
            %[zy,zyd,zydd,zyddd,zydddd,zypp,zytimepoints,zytsamples] = minsnappolytraj(wpts_zy,tpts,numsamples);
            
            circle_final_pose = zeros(2,(sample_per_loop*2)+1); % 565 per round, therefore (565 * 2) + 1 = 1131
            circle_final_vel = zeros(2,(sample_per_loop*2)+1);
            circle_final_acc = zeros(2,(sample_per_loop*2)+1);
            circle_final_jerk = zeros(2,(sample_per_loop*2)+1);
            circle_final_snap = zeros(2,(sample_per_loop*2)+1);
            
            % final_z_pose = zy(1,101:300);
            % final_z_vel = zyd(1,101:300);
            % final_z_acc = zydd(1,101:300);
            % final_z_jerk = zyddd(1,101:300);
            % final_z_snap = zydddd(1,101:300);
            
            for v = 1:2
                circle_final_pose(v,:) = circle_xy(v,sample_per_loop+1:(sample_per_loop*3)+1);
                circle_final_vel(v,:) = circle_xyd(v,sample_per_loop+1:(sample_per_loop*3)+1);
                circle_final_acc(v,:) = circle_xydd(v,sample_per_loop+1:(sample_per_loop*3)+1);
                circle_final_jerk(v,:) = circle_xyddd(v,sample_per_loop+1:(sample_per_loop*3)+1);
                circle_final_snap(v,:) = circle_xydddd(v,sample_per_loop+1:(sample_per_loop*3)+1);
            end
            
            % because the above solution starts from the same point but moves in a
            % clockwise fashion as compared to what I wanted which was anti-clockwise
            % from the same starting pt
            
            invert_pos = zeros(2,(sample_per_loop*2)+1);
            invert_pos(2,:) = (circle_final_pose(2,:));
            invert_pos(1,:) = (circle_final_pose(1,:));
            
            invert_vel = zeros(2,(sample_per_loop*2)+1);
            invert_vel(2,:) = (circle_final_vel(2,:));
            invert_vel(1,:) = (circle_final_vel(1,:));
            
            invert_acc = zeros(2,(sample_per_loop*2)+1);
            invert_acc(2,:) = (circle_final_acc(2,:));
            invert_acc(1,:) = (circle_final_acc(1,:));
            
            invert_jer = zeros(2,(sample_per_loop*2)+1);
            invert_jer(2,:) = (circle_final_jerk(2,:));
            invert_jer(1,:) = (circle_final_jerk(1,:));
            
            invert_sna = zeros(2,(sample_per_loop*2)+1);
            invert_sna(2,:) = (circle_final_snap(2,:));
            invert_sna(1,:) = (circle_final_snap(1,:));
            
            diff_pos = zeros(2,(sample_per_loop*2)); 
            diff_vel = zeros(2,(sample_per_loop*2));
            diff_acc = zeros(2,(sample_per_loop*2));
            
            % used in the later parts
            diff_jer = zeros(2,(sample_per_loop*2));
            diff_sna = zeros(2,(sample_per_loop*2));
            
            for v = 1:2
                for i = 1:(sample_per_loop*2)
                    diff_pos(v,i) = invert_pos(v,i+1) - invert_pos(v,i); 
                    diff_vel(v,i) = invert_vel(v,i+1) - invert_vel(v,i); 
                    diff_acc(v,i) = invert_acc(v,i+1) - invert_acc(v,i); 
                    diff_jer(v,i) = invert_jer(v,i+1) - invert_jer(v,i); 
                    diff_sna(v,i) = invert_sna(v,i+1) - invert_sna(v,i); 
                end
            end
            
            %mag = zeros(1,(sample_per_loop*2));
            
            mag(1,:) = sqrt((diff_pos(1,:)).^2 + (diff_pos(2,:)).^2); %pos
            mag(2,:) = sqrt((diff_vel(1,:)).^2 + (diff_vel(2,:)).^2); %vel
            mag(3,:) = sqrt((diff_acc(1,:)).^2 + (diff_acc(2,:)).^2); %acc
            mag(4,:) = sqrt((diff_jer(1,:)).^2 + (diff_jer(2,:)).^2); %jer
            mag(5,:) = sqrt((diff_sna(1,:)).^2 + (diff_sna(2,:)).^2); %sna
            
            direction = atan2(diff_pos(2,:),diff_pos(1,:));
            direction_deg = rad2deg(direction);

            mag(6,:) = direction; %direction
            mag(7,1) = time_per_setpt; %update rate
            mag(8,1:9) = x; 
            mag(9,:) = direction_deg; %direction
            mag(10,1) = sample_per_loop;
            mag(11,:) = invert_pos(1,1:sample_per_loop*2); % x
            mag(12,:) = invert_pos(2,1:sample_per_loop*2); % y
            mag(13,:) = invert_pos(2,1:sample_per_loop*2)-0.5; % z
            %circle_xy(2,1:1130)
        end


        












        
        %% 

        function [mag] = circle_setpoints_anti_cw_halved(obj,speed,x_rad,y_rad,r)    
            %CIRCLE
            x_origin = x_rad;
            y_origin = y_rad;
            radius = r;
            %th = 0:pi/50:2*pi; % 100 pts
            th = 0:pi/4:2*pi; % 8 pts
            x = radius * cos(th) + x_origin;
            y = radius * sin(th) + y_origin;
           
            
            % wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
            %wpts_zy = [0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0; 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0]; 
            circle_wpts_xy = [x x(:,2:end) x(:,2:end) x(:,2:end); y y(:,2:end) y(:,2:end) y(:,2:end)]; % repeats for 4 times to ensure smoothness but we only take the middle points
            % tpts = 0:5;
            
            omega = speed / r; % 1 m/s = 1 rad/s based on v = r*omega
%             time_per_eigth = (pi/4)/speed; 
%             sample_per_loop = 100;
%             time_per_setpt = (time_per_eigth * 8) / sample_per_loop; 

            time_per_eigth = (pi/4)/omega; % 0.784/2
            hz = 360;
            time_per_setpt = 1/hz;
            sample_per_loop = (time_per_eigth * 8) / time_per_setpt;  % 1130
            sample_per_loop = 1130;
            % 1 m/s = (pi/4) / (1 rad/s) = pi/4
            % 2 m/s = (pi/4) / (2 rad/s) = pi/8
            % 3 m/s = (pi/4) / (3 rad/s) = pi/12
            % 4 m/s = (pi/4) / (4 rad/s) = pi/16
            
            circle_tpts = 0:time_per_eigth:time_per_eigth*8*4; % intervals @ 1 m/s = 1 rad/s based on v = r*omega
            
            numsamples = sample_per_loop * 4;
            
            [circle_xy,circle_xyd,circle_xydd,circle_xyddd,circle_xydddd,circle_xypp,circle_xytimepoints,circle_xytsamples] = minsnappolytraj(circle_wpts_xy,circle_tpts,numsamples);
            %[zy,zyd,zydd,zyddd,zydddd,zypp,zytimepoints,zytsamples] = minsnappolytraj(wpts_zy,tpts,numsamples);
            
            circle_final_pose = zeros(2,2261); % 565 per round, therefore (565 * 2) + 1 = 1131
            circle_final_vel = zeros(2,2261);
            circle_final_acc = zeros(2,2261);
            circle_final_jerk = zeros(2,2261);
            circle_final_snap = zeros(2,2261);
            
            % final_z_pose = zy(1,101:300);
            % final_z_vel = zyd(1,101:300);
            % final_z_acc = zydd(1,101:300);
            % final_z_jerk = zyddd(1,101:300);
            % final_z_snap = zydddd(1,101:300);
            
            for v = 1:2
                circle_final_pose(v,:) = circle_xy(v,1131:3391);
                circle_final_vel(v,:) = circle_xyd(v,1131:3391);
                circle_final_acc(v,:) = circle_xydd(v,1131:3391);
                circle_final_jerk(v,:) = circle_xyddd(v,1131:3391);
                circle_final_snap(v,:) = circle_xydddd(v,1131:3391);
            end
            
            % because the above solution starts from the same point but moves in a
            % clockwise fashion as compared to what I wanted which was anti-clockwise
            % from the same starting pt
            
            invert_pos = zeros(2,2261);
            invert_pos(2,:) = (circle_final_pose(2,:));
            invert_pos(1,:) = (circle_final_pose(1,:));
            
            invert_vel = zeros(2,2261);
            invert_vel(2,:) = (circle_final_vel(2,:));
            invert_vel(1,:) = (circle_final_vel(1,:));
            
            invert_acc = zeros(2,2261);
            invert_acc(2,:) = (circle_final_acc(2,:));
            invert_acc(1,:) = (circle_final_acc(1,:));
            
            invert_jer = zeros(2,2261);
            invert_jer(2,:) = (circle_final_jerk(2,:));
            invert_jer(1,:) = (circle_final_jerk(1,:));
            
            invert_sna = zeros(2,2261);
            invert_sna(2,:) = (circle_final_snap(2,:));
            invert_sna(1,:) = (circle_final_snap(1,:));
            
            diff_pos = zeros(2,2260); 
            diff_vel = zeros(2,2260);
            diff_acc = zeros(2,2260);
            
            % used in the later parts
            diff_jer = zeros(2,2260);
            diff_sna = zeros(2,2260);
            
            for v = 1:2
                for i = 1:2260
                    diff_pos(v,i) = invert_pos(v,i+1) - invert_pos(v,i); 
                    diff_vel(v,i) = invert_vel(v,i+1) - invert_vel(v,i); 
                    diff_acc(v,i) = invert_acc(v,i+1) - invert_acc(v,i); 
                    diff_jer(v,i) = invert_jer(v,i+1) - invert_jer(v,i); 
                    diff_sna(v,i) = invert_sna(v,i+1) - invert_sna(v,i); 
                end
            end
            
            mag = zeros(11,2260);
            
            mag(1,:) = sqrt((diff_pos(1,:)).^2 + (diff_pos(2,:)).^2); %pos
            mag(2,:) = sqrt((diff_vel(1,:)).^2 + (diff_vel(2,:)).^2); %vel
            mag(3,:) = sqrt((diff_acc(1,:)).^2 + (diff_acc(2,:)).^2); %acc
            mag(4,:) = sqrt((diff_jer(1,:)).^2 + (diff_jer(2,:)).^2); %jer
            mag(5,:) = sqrt((diff_sna(1,:)).^2 + (diff_sna(2,:)).^2); %sna
            
            direction = atan2(diff_pos(2,:),diff_pos(1,:));
            direction_deg = rad2deg(direction);

            mag(6,:) = direction; %direction
            mag(7,1) = time_per_setpt; %update rate
            mag(8,1:9) = y; 
            mag(9,:) = direction_deg; %direction
            %mag(10,:) = circle_xy(1,1:1130);
            %circle_xy(2,1:1130)
        end


        function [mag] = circle_setpoints_cw(obj,speed,x_rad,y_rad,r)    
            %CIRCLE
            x_origin = x_rad;
            y_origin = y_rad;
            radius = r;
            %th = 0:pi/50:2*pi; % 100 pts
            th = 0:pi/4:2*pi; % 8 pts
            x = radius * cos(th) + x_origin;
            y = radius * sin(th) + y_origin;
            x = x*-1;
           
            
           % wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
            %wpts_zy = [0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0; 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0]; 
            circle_wpts_xy = [x x(:,2:end) x(:,2:end) x(:,2:end); y y(:,2:end) y(:,2:end) y(:,2:end)]; % repeats for 4 times to ensure smoothness but we only take the middle points
            % tpts = 0:5;
            
            %speed = 1; % 1 m/s = 1 rad/s based on v = r*omega
%             time_per_eigth = (pi/4)/speed; 
%             sample_per_loop = 100;
%             time_per_setpt = (time_per_eigth * 8) / sample_per_loop; 

            time_per_eigth = (pi/4)/speed; % 0.784
            hz = 360;
            time_per_setpt = 1/hz;
            sample_per_loop = (time_per_eigth * 8) / time_per_setpt;  % 2258
            sample_per_loop = 2260;
            % 1 m/s = (pi/4) / (1 rad/s) = pi/4
            % 2 m/s = (pi/4) / (2 rad/s) = pi/8
            % 3 m/s = (pi/4) / (3 rad/s) = pi/12
            % 4 m/s = (pi/4) / (4 rad/s) = pi/16
            
            circle_tpts = 0:time_per_eigth:time_per_eigth*8*4; % intervals @ 1 m/s = 1 rad/s based on v = r*omega
            
            numsamples = sample_per_loop * 4;
            
            [circle_xy,circle_xyd,circle_xydd,circle_xyddd,circle_xydddd,circle_xypp,circle_xytimepoints,circle_xytsamples] = minsnappolytraj(circle_wpts_xy,circle_tpts,numsamples);
            %[zy,zyd,zydd,zyddd,zydddd,zypp,zytimepoints,zytsamples] = minsnappolytraj(wpts_zy,tpts,numsamples);
            
            circle_final_pose = zeros(2,4521); % 565 per round, therefore (565 * 2) + 1 = 1131
            circle_final_vel = zeros(2,4521);
            circle_final_acc = zeros(2,4521);
            circle_final_jerk = zeros(2,4521);
            circle_final_snap = zeros(2,4521);
            
            % final_z_pose = zy(1,101:300);
            % final_z_vel = zyd(1,101:300);
            % final_z_acc = zydd(1,101:300);
            % final_z_jerk = zyddd(1,101:300);
            % final_z_snap = zydddd(1,101:300);
            
            for v = 1:2
                circle_final_pose(v,:) = circle_xy(v,2261:6781);
                circle_final_vel(v,:) = circle_xyd(v,2261:6781);
                circle_final_acc(v,:) = circle_xydd(v,2261:6781);
                circle_final_jerk(v,:) = circle_xyddd(v,2261:6781);
                circle_final_snap(v,:) = circle_xydddd(v,2261:6781);
            end
            
            % because the above solution starts from the same point but moves in a
            % clockwise fashion as compared to what I wanted which was anti-clockwise
            % from the same starting pt
            
            invert_pos = zeros(2,4521);
            invert_pos(2,:) = (circle_final_pose(2,:));
            invert_pos(1,:) = (circle_final_pose(1,:));
            
            invert_vel = zeros(2,4521);
            invert_vel(2,:) = (circle_final_vel(2,:));
            invert_vel(1,:) = (circle_final_vel(1,:));
            
            invert_acc = zeros(2,4521);
            invert_acc(2,:) = (circle_final_acc(2,:));
            invert_acc(1,:) = (circle_final_acc(1,:));
            
            invert_jer = zeros(2,4521);
            invert_jer(2,:) = (circle_final_jerk(2,:));
            invert_jer(1,:) = (circle_final_jerk(1,:));
            
            invert_sna = zeros(2,4521);
            invert_sna(2,:) = (circle_final_snap(2,:));
            invert_sna(1,:) = (circle_final_snap(1,:));
            
            diff_pos = zeros(2,4520); 
            diff_vel = zeros(2,4520);
            diff_acc = zeros(2,4520);
            
            % used in the later parts
            diff_jer = zeros(2,4520);
            diff_sna = zeros(2,4520);
            
            for v = 1:2
                for i = 1:4520
                    diff_pos(v,i) = invert_pos(v,i+1) - invert_pos(v,i); 
                    diff_vel(v,i) = invert_vel(v,i+1) - invert_vel(v,i); 
                    diff_acc(v,i) = invert_acc(v,i+1) - invert_acc(v,i); 
                    diff_jer(v,i) = invert_jer(v,i+1) - invert_jer(v,i); 
                    diff_sna(v,i) = invert_sna(v,i+1) - invert_sna(v,i); 
                end
            end
            
            mag = zeros(11,4520);
            
            mag(1,:) = sqrt((diff_pos(1,:)).^2 + (diff_pos(2,:)).^2); %pos
            mag(2,:) = sqrt((diff_vel(1,:)).^2 + (diff_vel(2,:)).^2); %vel
            mag(3,:) = sqrt((diff_acc(1,:)).^2 + (diff_acc(2,:)).^2); %acc
            mag(4,:) = sqrt((diff_jer(1,:)).^2 + (diff_jer(2,:)).^2); %jer
            mag(5,:) = sqrt((diff_sna(1,:)).^2 + (diff_sna(2,:)).^2); %sna
            
            direction = atan2(diff_pos(2,:),diff_pos(1,:));
            direction_deg = rad2deg(direction);

            mag(6,:) = direction; %direction
            mag(7,1) = time_per_setpt; %update rate
            mag(8,1:9) = y; 
            mag(9,:) = direction_deg; %direction
            %mag(10,:) = circle_xy(1,1:1130);
            %circle_xy(2,1:1130)
        end

        function [mag] = straight_line(obj,speed,x_rad,y_rad,r)    
            %straight_line
            x_origin = x_rad;
            radius = r;
            %th = 0:pi/50:2*pi; % 100 pts
            th = 0:pi/4:2*pi; % 8 pts
            x = radius * cos(th) + x_origin;
            x = x*-1;
            y = x;
            mm = sqrt(x.^2 + y.^2);
           
            
            % wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
            %wpts_zy = [0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0 0.5 1.0 0.5 0; 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0 -2 0 2 0]; 
            circle_wpts_xy = [mm mm(:,2:end) mm(:,2:end) mm(:,2:end); y y(:,2:end) y(:,2:end) y(:,2:end)]; % repeats for 4 times to ensure smoothness but we only take the middle points
            % tpts = 0:5;
            
            %speed = 1; % 1 m/s = 1 rad/s based on v = r*omega
            time_per_eigth = (pi/4)/speed; 
            sample_per_loop = 100;
            time_per_setpt = (time_per_eigth * 8) / sample_per_loop; 
            
            % 1 m/s = (pi/4) / (1 rad/s) = pi/4
            % 2 m/s = (pi/4) / (2 rad/s) = pi/8
            % 3 m/s = (pi/4) / (3 rad/s) = pi/12
            % 4 m/s = (pi/4) / (4 rad/s) = pi/16
            
            circle_tpts = 0:time_per_eigth:time_per_eigth*8*4; % intervals @ 1 m/s = 1 rad/s based on v = r*omega
            
            numsamples = sample_per_loop * 4;
            
            [circle_xy,circle_xyd,circle_xydd,circle_xyddd,circle_xydddd,circle_xypp,circle_xytimepoints,circle_xytsamples] = minsnappolytraj(circle_wpts_xy,circle_tpts,numsamples);
            %[zy,zyd,zydd,zyddd,zydddd,zypp,zytimepoints,zytsamples] = minsnappolytraj(wpts_zy,tpts,numsamples);
            
            circle_final_pose = zeros(2,201);
            circle_final_vel = zeros(2,201);
            circle_final_acc = zeros(2,201);
            circle_final_jerk = zeros(2,201);
            circle_final_snap = zeros(2,201);
            
            % final_z_pose = zy(1,101:300);
            % final_z_vel = zyd(1,101:300);
            % final_z_acc = zydd(1,101:300);
            % final_z_jerk = zyddd(1,101:300);
            % final_z_snap = zydddd(1,101:300);
            
            for v = 1:2
                circle_final_pose(v,:) = circle_xy(v,101:301);
                circle_final_vel(v,:) = circle_xyd(v,101:301);
                circle_final_acc(v,:) = circle_xydd(v,101:301);
                circle_final_jerk(v,:) = circle_xyddd(v,101:301);
                circle_final_snap(v,:) = circle_xydddd(v,101:301);
            end
            
            % because the above solution starts from the same point but moves in a
            % clockwise fashion as compared to what I wanted which was anti-clockwise
            % from the same starting pt
            
            invert_pos = zeros(2,201);
            invert_pos(2,:) = (circle_final_pose(2,:));
            invert_pos(1,:) = (circle_final_pose(1,:));
            
            invert_vel = zeros(2,201);
            invert_vel(2,:) = (circle_final_vel(2,:));
            invert_vel(1,:) = (circle_final_vel(1,:));
            
            invert_acc = zeros(2,201);
            invert_acc(2,:) = (circle_final_acc(2,:));
            invert_acc(1,:) = (circle_final_acc(1,:));
            
            invert_jer = zeros(2,201);
            invert_jer(2,:) = (circle_final_jerk(2,:));
            invert_jer(1,:) = (circle_final_jerk(1,:));
            
            invert_sna = zeros(2,201);
            invert_sna(2,:) = (circle_final_snap(2,:));
            invert_sna(1,:) = (circle_final_snap(1,:));
            
            diff_pos = zeros(2,200); 
            diff_vel = zeros(2,200);
            diff_acc = zeros(2,200);
            
            % used in the later parts
            diff_jer = zeros(2,200);
            diff_sna = zeros(2,200);
            
            for v = 1:2
                for i = 1:200
                    diff_pos(v,i) = invert_pos(v,i+1) - invert_pos(v,i); 
                    diff_vel(v,i) = invert_vel(v,i+1) - invert_vel(v,i); 
                    diff_acc(v,i) = invert_acc(v,i+1) - invert_acc(v,i); 
                    diff_jer(v,i) = invert_jer(v,i+1) - invert_jer(v,i); 
                    diff_sna(v,i) = invert_sna(v,i+1) - invert_sna(v,i); 
                end
            end
            
            mag = zeros(9,200);
            
            mag(1,:) = sqrt((diff_pos(1,:)).^2 + (diff_pos(2,:)).^2); %pos
            mag(2,:) = sqrt((diff_vel(1,:)).^2 + (diff_vel(2,:)).^2); %vel
            mag(3,:) = sqrt((diff_acc(1,:)).^2 + (diff_acc(2,:)).^2); %acc
            mag(4,:) = sqrt((diff_jer(1,:)).^2 + (diff_jer(2,:)).^2); %jer
            mag(5,:) = sqrt((diff_sna(1,:)).^2 + (diff_sna(2,:)).^2); %sna
            
            direction = atan2(diff_pos(2,:),diff_pos(1,:));
            direction_deg = rad2deg(direction);

            mag(6,:) = direction; %direction
            mag(7,1) = time_per_setpt; %update rate
            mag(8,:) = x; 
            
        end



    end
end