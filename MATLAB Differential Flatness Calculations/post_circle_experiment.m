%% Clear workspace, close all figures, clear command window
clear all
close all
clc

%% Variables
import body_obj.*

rate=1/360; %in 1/Hz, how fast the graph updates
bodyname=["gp"]; % multiple bodies allowed
%data_arr=["Mtime","Otime","name","x","y","z","qx","qy","qz","qw","euy","eup","eur","eury","eurp","eurr","vx","vy", "vz","pitch_norm"]; % Array to store to excel
data_arr=["Mtime","Otime","name","x","y","z","euy","eup","eur","vx","vy","vz","bod_rates","thrust","heading"]; % Array to store to excel

%% Create OptiTrack object
obj = OptiTrack;
Initialize(obj,'192.168.1.5','multicast'); %Ensure broadcast frame id is on, loop interface is set to this ip and transmission type is set to multicast

up = udpport("IPV4");
% up = udpport("datagram","OutputDatagramSize",3);
up.EnableBroadcast = true;

% Initalise all bodies at first
rb = obj.RigidBody;
% Create each motive defined rigid body
arr = blanks(numel(rb));

for i = 1:numel(rb)
    my_field = strcat(convertCharsToStrings(rb(i).Name));
    variable.(my_field) = body_obj;
    variable.(my_field).init(convertCharsToStrings(rb(i).Name));
end

computerip="192.168.1.137"; % ip of computer to be written to
port=1234; % port of the computer to be written to
%% Break loop if keypress to save to excel
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Close', ...
                    'Callback', 'delete(gcbf)');
drawnow


exp = ExpAuxiliaryFunctions;
% center for x and y (needa check again on optitrack)
center_x = 2.5;
center_y = 1.5;
% inverted for y and x 
mid_x = 2.0;
mid_y = 2.0;
radius = 0.5;
speed = 0.3;
derivatives = exp.circle_setpoints_anti_cw(speed,mid_x,mid_y,radius); % circle anti_cw setpoints, radius 0.5, speed 0.5
% derivatives = exp.circle_setpoints_cw(1,-2,2,1); % circle cw setpoints

% vel = load("invert_vel.mat");
% acc = load("invert_acc.mat");
% jer = load("invert_jer.mat");
% sna = load("invert_sna.mat");


%%% step 1 - attitude control:
%%  assign the heading at the period of the time interval which is pi/50 
%   take in measurements, assume the following for now as pseudo
opti_offset = 0.5; % original was 0.5
ideal_hgt = 1.5;
desired_alt = ideal_hgt - opti_offset;
mea_pos = zeros(3,1); % extract position measurements in real time from opti track 
mea_vel = zeros(3,1); % extract velocity measurements in real time from opti track
mea_acc = zeros(3,1); % extract acceleration measurements in real time from opti track 

mea_pitch = zeros(1,1); % euler angle for disk pitch which is body roll, anyway this is measured at the centre, so no diff
mea_precession_angle = zeros(1,1); % euler angle for disk roll, precession angle
mea_pitch_rate = zeros(1,1); % euler angle for disk pitch rate which is body roll rate

mea_rotation = zeros(1,1); % body yaw angle for azimuth, must be in RAD
mea_xy_pos_mag = zeros(1,1);
mea_x_pos = zeros(1,1);
mea_y_pos = zeros(1,1);
mea_z_pos = zeros(1,1);
mea_x_pos_past = zeros(1,1);
mea_y_pos_past = zeros(1,1);
mea_z_pos_past = zeros(1,1);
mea_xy_vel_mag = zeros(1,1);
trigger = 1; % temporary trigger for now to go into offboard mode

% gains
kpos = 55.0;
kvel = 65.0;
kpos_z = 10;
kd_z = 105;
prp = [1,1]; % bodyrate gain
ppq = 0.07; % body acc gain
dpp = 10;

% init a_des
a_des = zeros(3,1);
a_des_z = zeros(3,1);

% gravity
g = -9.81;

% linear drag coeff
Dx = 0.03;
Dy = 0.03;
Dz = 0.01;
linear_drag_coeff = [Dx,Dy,Dz];

% unit vectors
ey = [0,1,0]; % 1 x 3
ex = [1,0,0];
ez = [0,0,1];

% init error quaternion
error_quat = zeros(1,4);

% init body rates xy
body_rates = zeros(1,2);

% need to insert update rate, loop at 1/time_per_setpt freq which is currently 16 hz
update_rate = derivatives(7,1);

sample_per_loop = derivatives(10,1);
i = (sample_per_loop * 2) - 100; % counter
c = (sample_per_loop * 2) - 100; % counter
old_mag = 0;
old_precession_rate_angle = 0;
z_error_past = 0;
test = 1;
old_timestamp = 0;

% Logging
log_bod_rates = 0;
log_head = 0;
log_thrust = 0;

p_array = [];
t_array = [];

while ishandle(H)
%%
    % Get current rigid body information, this has to be recalled every time for new frame index
    rb = obj.RigidBody;

    % Output frame information
    %fprintf('\nFrame Index: %d\n',rb(1).FrameIndex);

    % Update each rigid body (data logging)
    for k = 1:numel(rb)

        % Check for correct body
        % This crashes if there is a glitching body, remove any unused body
        % from the asset tab, if not uncomment this line
        for j = 1:numel(bodyname)
            if strcmp(rb(k).Name,bodyname(j)) == 1

                % If body loses tracking, stop plotting
                if isempty(rb(k).Position)== 0
%                     fprintf('\t %s \n',string(rb(i).Name));
%                      fprintf('\t Position [%f,%f,%f]\n', [round(rb(i).Position(1)/1000,2),round(rb(i).Position(2)/1000,2),round(rb(i).Position(3)/1000,2)]);
%                     fprintf('\t Quaternion [%f,%f,%f,%f]\n',rb(i).Quaternion);
                    my_field = strcat(convertCharsToStrings(rb(k).Name));
                    variable.(my_field).updatepose(rb(k));
                    disp("timestamp");
                    %disp(rad2deg(variable.(my_field).euler));
                    disp(1/(rb(k).TimeStamp - old_timestamp));
                    old_timestamp = rb(k).TimeStamp; 
%                     disp(rad2deg(variable.("gp").euler));
%                   data_arr=[data_arr; [now rb(k).TimeStamp string(rb(k).Name) variable.(my_field).position(1) variable.(my_field).position(2) variable.(my_field).position(3) variable.(my_field).quarternion(1) variable.(my_field).quarternion(2) variable.(my_field).quarternion(3) variable.(my_field).quarternion(4) variable.(my_field).euler(1) variable.(my_field).euler(2) variable.(my_field).euler(3) variable.(my_field).euler_rate(1) variable.(my_field).euler_rate(2) variable.(my_field).euler_rate(3) variable.(my_field).velocity(1) variable.(my_field).velocity(2) variable.(my_field).velocity(3)]];
                    data_arr=[data_arr; [now rb(k).TimeStamp string(rb(k).Name) -1*variable.(my_field).position(2) variable.(my_field).position(1) variable.(my_field).position(3) variable.(my_field).euler(3) variable.(my_field).euler(2) variable.(my_field).euler(1) variable.(my_field).velocity(1) variable.(my_field).velocity(2) variable.(my_field).velocity(3) log_bod_rates log_thrust log_head]];
                    
                end
            end
        end
    end
   
%     disp("pitch");
%     disp(variable.gp.euler(2));
%    hold on
    
%     p_array(end) = variable.gp.euler(2);
%     t_array(end) = rb(k).TimeStamp;
%     if size(t_array) > 200
%         p_array(0)=[];
%         t_array(0)=[];
%     end
%         
%     plot(t_array,p_array,'g');
%     xlabel('counter');
%     ylabel('pitch');
%     title("Pitch vs Time");
    
    drawnow
    %hold off
    
    mea_pos = transpose(variable.gp.position); % extract position measurements in real time from opti track 
    mea_vel = transpose(variable.gp.velocity); % extract velocity measurements in real time from opti track
    mea_rotation = variable.gp.euler(3); 

    if variable.gp.euler(3) < 0.05 && variable.gp.euler(3) > -0.05  %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
        mea_pitch = variable.gp.euler(2);
    end

    %position assignment - "rotation matrix"
    
    %mea_xy_pos_mag = sqrt((mea_pos(1,:)).^2 + (mea_pos(2,:)).^2);
    
    % mea_pos(1,:) is positive X (along wall) and mea_pos(2,:) is negative Y (tangent to wall) => _| 
    mea_y_pos = mea_pos(1,:);
    mea_x_pos = -1*mea_pos(2,:);
    mea_z_pos = mea_pos(3,:);
    mea_xy_pos_mag = sqrt((mea_x_pos-mea_x_pos_past).^2 + (mea_y_pos-mea_y_pos_past).^2);
    mea_x_pos_past = mea_x_pos;
    mea_y_pos_past = mea_y_pos;
    mea_z_pos_past = mea_z_pos;
    
    mea_xy_vel_mag = sqrt((mea_vel(1,:)).^2 + (mea_vel(2,:)).^2);
    mea_euler = [0,mea_pitch,0]; % default seq is about ZYX
    mea_pitch_rate = variable.gp.euler_rate(2);
    
%%  reset
 
    if i > sample_per_loop*2
        i = 1;
        c = 1;
    end 
    
    %%%% (Precession Rate Tracking)
    omega_mea = mea_xy_vel_mag / radius; % omega tracking of the body  
    latest_mag = mea_xy_pos_mag - sqrt(mid_x^2 + mid_y^2);
    latest_precession_rate_angle = acos(latest_mag/old_mag);

    precession_rate = -1 * ((latest_mag-old_mag)/abs((latest_mag-old_mag))) * (latest_precession_rate_angle/update_rate); % if negative in opti-track means correct direction, I multiply by -1 to make it positive so its in the correct direction

    old_mag = latest_mag;
    old_precession_rate_angle = latest_precession_rate_angle;
    
    %%%% (Test)
    % xy
%     a_fb_xy = abs((kpos*(derivatives(1,test) - mea_xy_pos_mag)) + (kvel*(derivatives(2,test) - mea_xy_vel_mag))); % xy_magnitude plane since yaw can be easily taken care of 
%     gain = kpos*(derivatives(1,test) - mea_xy_pos_mag)/abs(kpos*(derivatives(1,test) - mea_xy_pos_mag));
%     a_rd = derivatives(2,test) * linear_drag_coeff(1,1);
%     a_des(1,:) = a_fb_xy + derivatives(3,test) - a_rd; % fits into the x axis of ades

    % z (can be used to test, needs to activate hover flaps mode)
%     a_fb_z = kpos_z*(desired_alt - mea_pos(3,1)); % z
%     a_des_z(3,:) = a_fb_z + g;
%     zd = a_des_z / norm(a_des_z); % 3 x 1 = z_desired, if empty it would be 0 0 1  

    % direction (testing) 
%     desired_heading = derivatives(6,test);



    %%%% (Actual)
    %% xy 
    a_fb_xy = abs((kpos*(derivatives(1,i) - mea_xy_pos_mag)) + (kvel*(derivatives(2,i) - mea_xy_vel_mag))); % xy_magnitude plane since yaw can be easily taken care of 
    gain = kpos*(derivatives(1,i) - mea_xy_pos_mag)/abs(kpos*(derivatives(1,i) - mea_xy_pos_mag)); % always negative cos derivatives default value simply too small (negligible)
    a_rd = derivatives(2,i) * linear_drag_coeff(1,1);
    a_des(1,:) = a_fb_xy + derivatives(3,i) - a_rd; % fits into the x axis of ades 

    %% z (can be used to test, needs to activate hover flaps mode)

    %z_error = mea_pos(3,1)-derivatives(13,c);
    z_error = mea_pos(3,1)-desired_alt; % this one is with the fixed height
    a_rd_z = mea_vel(3) * Dz;
    a_fb_z = kpos_z*z_error + kd_z*(z_error-z_error_past); % z
    % disp ("alt: ");
    disp ("Pos & Att: X,Y,Z,Pitch ");
    disp([mea_x_pos mea_y_pos mea_z_pos mea_pitch]);
    a_des(3,:) = a_fb_z + g - a_rd_z;
    a_des_z(3,:) = a_fb_z + g - a_rd_z;


    zd = a_des / norm(a_des); % consists of all 3 axis, this was segregated due to collective and cyclic thrust decoupling  
    zd_z = a_des_z / norm(a_des_z); % 3 x 1 = z_desired, if empty it would be 0 0 1 
    z_error_past = z_error;

    % direction (actual)
    %desired_heading = atan2((derivatives(12,i)-mea_y_pos),(derivatives(11,i)-mea_x_pos));
    desired_heading = derivatives(6,i);
    true_heading = desired_heading;
    log_head = true_heading;
    
    %% Bodyrates (for collective thrust test, this entire section can be disabled)
    qz = eul2quat(mea_euler); % default seq is q = [w x y z]
    disk_vector = quatrotate(qz,ez); % vector of 1 x 3
    angle = acos((dot(disk_vector,transpose(zd))/(norm(disk_vector)*norm(zd)))); % will nvr catch up one
    n = cross(disk_vector,transpose(zd)) / norm(cross(disk_vector,transpose(zd)));
    B = quatrotate((quatinv(qz)),n);

    % if angle = 0, it would just be an identity quat matrix
    error_quat = [cos(angle/2),B*sin(angle/2)];

    % can alwways break here to make sure shit is running correctly
    
    if error_quat(:,1) < 0
        body_rates = -2 * prp.* error_quat(:,2:3);
    else
        body_rates = 2 * prp.* error_quat(:,2:3);
    end
    
%     disp("body_rates");
%     disp(body_rates);

    %% Collective thrust (can be used to test)
    % cmd_z = dot(transpose(zd),transpose(a_des)); %% command sent to motor, need to include filter to make sure negative cmds dun go thru
    cmd_z = dot(transpose(zd_z),transpose(a_des_z)); %% command sent to motor, need to include filter to make sure negative cmds dun go thru
    cmd_z = 0.05*cmd_z;
    if cmd_z > 0.7
        cmd_z = 0.7;
    end
    log_thrust = cmd_z;
    disp("cmd_z: ");
    disp(cmd_z);  

    %% Diff Flat Feedforward component (actual)
    ff_n = (derivatives(4,i) + linear_drag_coeff(:,1)*derivatives(3,i));
    v = [derivatives(4,i),0,0];
    ff_d = cmd_z + linear_drag_coeff(:,1)*dot(zd,v) + linear_drag_coeff(:,1)*dot(ex,v) - linear_drag_coeff(:,3)*dot(zd,v);
    if ff_d == 0
        body_rate_ref = 0;
    else
        body_rate_ref = ff_n/ff_d;
    end
    
    %% testing
    % body_rate_ref = 0;
    
%     if i > 561 && i < 1130
%         ppq = 0.07;
%     else 
%         ppq = 0.07;
%     end
   
    %% precession controller
    pc = 0.01 * (omega_mea - precession_rate); 
    disp("precession signal");
    disp(pc);

    %% inclusion of diff flatness component
    cmd_bodyrate = (ppq * (body_rates(:,2) - mea_pitch_rate)) + body_rate_ref; % now bod rate ref is separate from the gain, gain for cyclic, multiply this to azimuth sin or cos from quadrant, the other value is the desired heading  
    log_bod_rates = cmd_bodyrate;

%     bod_rate_cap = 0.117;
%     if abs(cmd_bodyrate) > bod_rate_cap
%         cmd_bodyrate = 0.117;
%     end    

    desired_heading = exp.new_heading_input(desired_heading);
    quadrant = exp.quadrant_output(desired_heading); 
    init_input = exp.flap_output(mea_rotation,quadrant,gain,desired_heading,-1*abs(cmd_bodyrate));   % -1 for pitching backwards 
    final_flap_input = init_input(:,1) * 15;
    disp("quadrant");
    disp(quadrant);
    disp("heading");
    disp(true_heading);

    %% trigger
%     trigger = trigger + 1;
%     if mod(trigger,16) == 0

    %% Disk yawing controller 

    % need to run 2 experiments: 
    % 1 - static rig to see the efx of increasing counter and the turning rate of the disk
    % 2 - dynamic experiment to see the efx of it in flight and the real time data achieved
    % 3 - both will be under the efx of the new changes to pos-mag as well as the bod rate ref being separate from the eqn that has gain ppq
   
    r_x = mea_x_pos - center_x;
    r_y = mea_y_pos - center_y;
    rad_data = sqrt((r_x).^2 + (r_y).^2) - radius;


    i = i + 50 + (dpp * ceil(rad_data)); % 50 is the number to update
    c = c + 1;
%     end
%     trigger = trigger + update_rate; % temporary holding

    
    input = [true_heading,final_flap_input,cmd_z,mea_rotation]; %heading, flap, motor, yaw
    fprintf('Input [%f,%f,%f]\n', input);
    disp("counter");
    disp(i);
    write(up,input,"double", computerip,port);


    pause(rate);
end

%% Write to excel
filename = 'C:\Users\area_\OneDrive\Desktop\testdata.xlsx';
disp("FINISH")
writematrix(data_arr,filename,'Sheet',1,'Range','B2'); % ("Array",filename,~,sheetname,~,range of cells to paste in 'E1:I5')