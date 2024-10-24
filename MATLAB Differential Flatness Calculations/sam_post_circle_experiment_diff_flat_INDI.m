%% Clear workspace, close all figures, clear command window
clear all
close all
clc

%% Variables
% import body_obj.*
hz = 300; % original value here was 300; % 100 hz for optitrack, matlab rate is at 60 
rate= 1/hz; % in 1/Hz, how fast the graph updates in terms of period (time)
bodyname= "STSAR"; % multiple bodies allowed
%data_arr=["Mtime","Otime","name","x","y","z","qx","qy","qz","qw","euy","eup","eur","eury","eurp","eurr","vx","vy", "vz","pitch_norm"]; % Array to store to excel
%data_arr=["Mtime","Otime","name","x","y","z","euy","eup","eur","vx","vy","vz","ax","ay","az","bod_ang_acc","thrust","no. of laps","ref_x","ref_y","ref_z","ref_vx","ref_vy","ref_vz"]; % Array to store to excel
data_arr=["timestamp","x","y","z","eur","eup","euy","vx","vy","vz","ax","ay","az","thrust","no. of laps","ref_x","ref_y","ref_z","ref_vx","ref_vy","ref_vz"]; % Array to store to excel

%% Create OptiTrack object
obj = OptiTrack;
Initialize(obj,'192.168.65.4','multicast'); % Dun touch ** IP of the optitrack com, Ensure broadcast frame id is on, loop interface is set to this ip and transmission type is set to multicast

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

computerip="192.168.65.227";% ip of monocopter to be written to (this can change from time to time) ***
port=1234; % port of the computer to be written to
%% Break loop if keypress to save to excel
DlgH = figure;
H = uicontrol('Style', 'PushButton', ...
                    'String', 'Close', ...
                    'Callback', 'delete(gcbf)');
%drawnow


exp = test_ExpAuxiliaryFunctions;
% center for x and y (needa check again on optitrack)
center_x = 2.85-0.5;
center_y = 1.85;
% inverted for y and x 
mid_x = 2.0;
mid_y = 2.0;
radius = 0.5;
speed = 0.1;
monocopter_rotation = "c"; % clockwise
z_enabled = "n"; 
% derivatives = exp.circle_setpoints_anti_cw(speed,mid_x,mid_y,radius,hz,monocopter_rotation); % circle anti_cw setpoints, radius 0.5, speed 0.5
derivatives = exp.point();
% derivatives = exp.trochoid(1,monocopter_rotation);

% vel = load("invert_vel.mat");
% acc = load("invert_acc.mat");
% jer = load("invert_jer.mat");
% sna = load("invert_sna.mat");


%%  Collective ref assignment wo increments
%   take in measurements, assume the following for now as pseudo
opti_offset = 0.5; % original was 0.5
ideal_hgt = 1.5;
desired_alt = ideal_hgt - opti_offset;

% XYZ
mea_pos = zeros(3,1); % extract position measurements in real time from opti track 
mea_vel = zeros(3,1); % extract velocity measurements in real time from opti track
mea_acc = zeros(3,1); % extract acceleration measurements in real time from opti track 

% RPY
mea_precession_angle = zeros(2,1); % euler angle for disk roll, precession angle
mea_angles = zeros(3,1); % selected disk angles (disk frame)
mea_angular_rate = zeros(3,1); % selected disk angles rate (disk frame)
mea_angular_rate_rate = zeros(3,1); % selected disk angles rate rate (disk frame)
mea_rotation = zeros(1,1); % body yaw angle for azimuth, must be in RAD
inertia_frame_angles = zeros(3,1); % world frame angles
inertia_frame_angular_rate = zeros(3,1); % world frame angular rate
inertia_frame_angular_rate_rate = zeros(3,1); % world frame angular rate rate
body_frame_angular_rate = zeros(3,1); % body frame angular rate
body_frame_angular_rate_rate = zeros(3,1); % body frame angular rate rate
inverted_body_frame_angular_rate = zeros(3,1); % inverted body frame angular rate
inverted_body_frame_angular_rate_rate = zeros(3,1); % inverted body frame angular rate rate

mea_xyz_pos_mag = zeros(3,1);

if monocopter_rotation == "cc"
    mea_xyz_pos_past = [mid_x - radius; mid_y; 0];
else
    mea_xyz_pos_past = [mid_x + radius; mid_y; 0];
end
mea_xyz_vel_past = zeros(3,1);
mea_xyz_acc_past = zeros(3,1);
mea_disk_yaw_rate = zeros(1,1);
mea_disk_yaw_past = derivatives(6,1); 
trigger = 1; % temporary trigger for now to go into offboard mode
mea_xyz_pos = zeros(3,1);
mea_xyz_vel = zeros(3,1);
mea_xyz_acc = zeros(3,1);
mea_bod_pitch_deg = zeros(1,1); % must be in deg


% gains
kpos = [0.075;0.075;120];
kvel = [0.1;0.1;10];
kdpos = [12;12;210];
%kpos_z = 10;
%kd_z = 105;
kpa = [0.1;0.1;0]; % att p gain - angle 2 x 1 0.25
kpr = [0.15;0.15;0]; % bodyrate p gain - jerk 3 x 1
kpang = [0.1;0.1;0]; % bodyangacc p gain - 3 x 1
kdang = [0.005;0.005;0]; % bodyangacc d gain - 3 x 1
kiang = [30;30;0]; % bodyangacc i gain - 3 x 1
%kdr = [1;1;1]; % bodyrate d gain - jerk
ppq = 0.40; % body acc gain
ppc = 0.10; % centrifugal gain, alternatively, ppc = 1 - ppq
dpp = 30;

% normalized motor gains 
kt = -200;

% init a_des
a_des = zeros(3,1);
a_des_z = zeros(3,1);
a_past = zeros(3,1);

% gravity
g = -9.81;

% linear drag coeff
Dx = 0.0;
Dy = 0.0;
Dz = 0.0;
linear_drag_coeff = [Dx,Dy,Dz];

% unit vectors
ey = [0,1,0]; % 1 x 3
ex = [1,0,0];
ez = [0,0,1];

% init error quaternion
error_quat = zeros(1,4);

% init body rates xy
body_rates = zeros(1,2);

% need to insert update rate, loop at 1/time_per_setpt freq which is currently 300 hz
update_rate = derivatives(7,1);

% points per loop
sample_per_loop = derivatives(10,1);
ref_counter = 1;
i = 1; % counter, used to be -100
c = 1; % counter, used to be -100

% error matrices
error = zeros(3,1);
error_ang = zeros(3,1);
error_past = zeros(3,1);
error_ang_past = zeros(3,1);
test = 1;
old_timestamp = 0;

% Logging
log_bod_acc = zeros(3,1);
log_head = 0;
log_motor_input = 0;
log_laps = 0;
ref_x = derivatives(11,1);
ref_y = derivatives(12,1);
ref_z = ideal_hgt;
ref_vx = 0.0;
ref_vy = 0.0;
ref_vz = 0.0;


p_array = [];
t_array = [];

% Motor - might be done on wj side (ωdes - wmeas) = Kv ∗ V where kv = motor constant
motor_feedback = zeros(1,1);

% Flap = (flap_angle) = Kv ∗ V
flap_feedback = zeros(1,1);

% Moment of inertia (I) - ixx, iyy, izz
Ixx = 0.0005764953; % 0.0005764953
Iyy = 0.0005764953; % 0.0005764953
Izz = 0.0005975766; % 0.0005975766
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz]; % tentative values 

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
        
        if strcmp(rb(k).Name,bodyname) == 1

            % If body loses tracking, stop plotting
            if isempty(rb(k).Position)== 0
%                     fprintf('\t %s \n',string(rb(i).Name));
%                      fprintf('\t Position [%f,%f,%f]\n', [round(rb(i).Position(1)/1000,2),round(rb(i).Position(2)/1000,2),round(rb(i).Position(3)/1000,2)]);
%                     fprintf('\t Quaternion [%f,%f,%f,%f]\n',rb(i).Quaternion);
                my_field = strcat(convertCharsToStrings(rb(k).Name));
                variable.(my_field).updatepose(rb(k));
                fprintf('\t Frequency %f\n', 1/(rb(k).TimeStamp - old_timestamp));
                old_timestamp = rb(k).TimeStamp;
%                     disp(rad2deg(variable.("gp").euler));
%                   data_arr=[data_arr; [now rb(k).TimeStamp string(rb(k).Name) variable.(my_field).position(1) variable.(my_field).position(2) variable.(my_field).position(3) variable.(my_field).quarternion(1) variable.(my_field).quarternion(2) variable.(my_field).quarternion(3) variable.(my_field).quarternion(4) variable.(my_field).euler(1) variable.(my_field).euler(2) variable.(my_field).euler(3) variable.(my_field).euler_rate(1) variable.(my_field).euler_rate(2) variable.(my_field).euler_rate(3) variable.(my_field).velocity(1) variable.(my_field).velocity(2) variable.(my_field).velocity(3)]];
                %data_arr=[data_arr; [now rb(k).TimeStamp string(rb(k).Name) variable.(my_field).position(1) variable.(my_field).position(2) variable.(my_field).position(3) variable.(my_field).euler(3) variable.(my_field).euler(2) variable.(my_field).euler(1) variable.(my_field).velocity(1) variable.(my_field).velocity(2) variable.(my_field).velocity(3) variable.(my_field).acceleration(1) variable.(my_field).acceleration(2) variable.(my_field).acceleration(3) log_bod_acc log_motor_input log_laps ref_x ref_y ref_z ref_vx ref_vy ref_vz]];
                data_arr=[data_arr; [rb(k).TimeStamp variable.(my_field).position(1) variable.(my_field).position(2) variable.(my_field).position(3) variable.(my_field).euler(1) variable.(my_field).euler(2) variable.(my_field).euler(3) variable.(my_field).velocity(1) variable.(my_field).velocity(2) variable.(my_field).velocity(3) variable.(my_field).acceleration(1) variable.(my_field).acceleration(2) variable.(my_field).acceleration(3) log_motor_input log_laps ref_x ref_y ref_z ref_vx ref_vy ref_vz]];

            end
        end
    end
   
%     disp("pitch");
%     disp(variable.gp.euler(2));
%    hold on
        
    mea_pos = transpose(variable.STSAR.position); % extract position measurements in real time from opti track 
    mea_vel = transpose(variable.STSAR.velocity); % extract velocity measurements in real time from opti track
    mea_acc = transpose(variable.STSAR.acceleration); % extract acceleration measurements in real time from opti track ( need to do ***)
    inertia_frame_angles = transpose(variable.STSAR.euler); % in body obj it shud be [2,1,3] inertia frame rpy [2,1,3] from euler [x,y,z] 
    inertia_frame_angular_rate = transpose(variable.STSAR.euler_rate); % in body obj it shud be [2,1,3] inertia frame rpy [2,1,3] from euler [x,y,z] 
    inertia_frame_angular_rate_rate = transpose(variable.STSAR.euler_rate_rate); % % in body obj it shud be [2,1,3] inertia frame rpy [2,1,3] from euler [x,y,z] 
    
    %% Body
    phi = inertia_frame_angles(1,1); % _| roll - from inertia eul_xyz(2) about y, body pitch
    theta = inertia_frame_angles(2,1); % _| pitch - from inertia eul_xyz(1) about x, body roll, body obj it shud be [2,1,3] inertia frame rpy [2,1,3] from euler [x,y,z] 

    W = [ 1,  0,        -sin(theta);
          0,  cos(phi),  cos(theta)*sin(phi);
          0, -sin(phi),  cos(theta)*cos(phi) ]; % inertia to body frame

    % W = [ 1,  0,  0;
    %       0,  1,  0;
    %       0, 0,  1 ]; % inertia to body frame

    body_frame_angular_rate = W*inertia_frame_angular_rate;
    body_frame_angular_rate_rate = W*inertia_frame_angular_rate_rate;
    mea_bod_pitch_deg = rad2deg(-1*phi); % it would be negative y angle pitching up hence the negation 
    mea_rotation = inertia_frame_angles(3,1); % negative
    % J = W.'*I*W; % back portion of the INDI

    
    %% Disk & Gyro (in the event of we need to invert the craft upside down)
    % d_phi = pi + inertia_frame_angles(1,1); % roll - from inertia eul_xyz(2) about y, body pitch
    % d_theta = inertia_frame_angles(2,1); % pitch - from inertia eul_xyz(1) about x, bo
    % 
    % D = [ 1,  0,        -sin(d_theta);
    %       0,  cos(phi),  cos(d_theta)*sin(d_phi);
    %       0, -sin(phi),  cos(d_theta)*cos(d_phi) ]; % inertia to disk frame
    % 
    % inverted_body_frame_angular_rate = D*body_frame_angular_rate; % not really disk frame per se, more of body frame inverted 
    % inverted_body_frame_angular_rate_rate = D*body_frame_angular_rate_rate;
    

    %% Disk & Gyro

    mea_angular_rate(3,1) = 0;    
    % need to test w opti track coordinate system to see if it can output this results 
    if abs(mea_rotation) > deg2rad(90) %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
    %if abs(variable.gp.euler(3)) < abs(derivatives(6,i) + deg2rad(10)) && variable.gp.euler(3) > -0.05  %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
        mea_angles(1,1) = theta; % disk pitch, body roll, can still follow body convention cos its jus a negative of disk 
        mea_angular_rate(1,1) = body_frame_angular_rate(2,1); % disk pitch rate, body roll
        mea_angular_rate_rate(1,1) = body_frame_angular_rate_rate(2,1);
        %mea_precession_angle(1,1) = mea_angular_rate_rate(2,1)/body_frame_angular_rate(3,1); % needa check if euler rate(3) is negative for our craft, lets hope it is since its spins cw, anti-cw is positive
    elseif abs(mea_rotation) < deg2rad(90) %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
    %if abs(variable.gp.euler(3)) < abs(derivatives(6,i) + deg2rad(10)) && variable.gp.euler(3) > -0.05  %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
        mea_angles(1,1) = -1*theta; % disk pitch, body roll, can still follow body convention cos its jus a negative of disk 
        mea_angular_rate(1,1) = -1*body_frame_angular_rate(2,1); % disk pitch rate, body roll
        mea_angular_rate_rate(1,1) = -1*body_frame_angular_rate_rate(2,1);
        %mea_precession_angle(1,1) = mea_angular_rate_rate(2,1)/body_frame_angular_rate(3,1); % needa check if euler rate(3) is negative for our craft, lets hope it is since its spins cw, anti-cw is positive
    end


    if mea_rotation > deg2rad(0) %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
    %if abs(variable.gp.euler(3)) < abs(derivatives(6,i) + deg2rad(10)) && variable.gp.euler(3) > -0.05  %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
        mea_angles(2,1) = -1*theta; % becomes disk pitch cos of phase shift, body roll
        mea_angular_rate(2,1) = -1*body_frame_angular_rate(2,1); % disk pitch rate, body roll
        mea_angular_rate_rate(2,1) = -1*body_frame_angular_rate_rate(2,1);
        %mea_precession_angle(2,1) = mea_angular_rate_rate(1,1)/body_frame_angular_rate(3,1);
    elseif mea_rotation < deg2rad(0) %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
    %if abs(variable.gp.euler(3)) < abs(derivatives(6,i) + deg2rad(10)) && variable.gp.euler(3) > -0.05  %% needa check if this will be logged at zero, if not mea_pitch will always be zero and we need a range
        mea_angles(2,1) = theta; % becomes disk pitch cos of phase shift, body roll
        mea_angular_rate(2,1) = body_frame_angular_rate(2,1); % disk pitch rate, body roll
        mea_angular_rate_rate(2,1) = body_frame_angular_rate_rate(2,1);
        %mea_precession_angle(2,1) = mea_angular_rate_rate(1,1)/body_frame_angular_rate(3,1);    
    end
    
    %position assignment
    % mea_pos(1,:) is tangent to wall (X) and mea_pos(2,:) is along wall (Y) -- updated 
    mea_xyz_pos(2,1) = mea_pos(2,:);
    mea_xyz_pos(1,1) = mea_pos(1,:);
    mea_xyz_pos(3,1) = mea_pos(3,:);
    mea_xyz_pos_mag = mea_xyz_pos - mea_xyz_pos_past;
    

    %disk yaw assignment
    mea_disk_yaw_rate = ((atan2(mea_xyz_pos_mag(2,:),mea_xyz_pos_mag(1,:))) - mea_disk_yaw_past)/update_rate;
    mea_disk_yaw_past = atan2(mea_xyz_pos_mag(2,:),mea_xyz_pos_mag(1,:));
    
    %velocity assignment
    mea_xyz_vel(2,1) = mea_vel(2,:);
    mea_xyz_vel(1,1) = mea_vel(1,:);
    mea_xyz_vel(3,1) = mea_vel(3,:);

    %acceleration assignment
    mea_xyz_acc(2,1) = mea_acc(2,:);
    mea_xyz_acc(1,1) = mea_acc(1,:);
    mea_xyz_acc(3,1) = mea_acc(3,:);
    
    %euler angles of the disk 
    mea_euler = [0,mea_angles(1,1),mea_angles(2,1)]; % default seq is about ZYX
    
%%  reset
 
    if ref_counter == sample_per_loop
        i = sample_per_loop;
        c = sample_per_loop;
    end 
   
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
    %% xyz 
    % delta_pos = sqrt((derivatives(11,i) - mea_x_pos).^2 + (derivatives(12,i) - mea_y_pos).^2 );
    % delta_vel = sqrt((derivatives(14,i) - (mea_vel(1,:))).^2 + (derivatives(15,i) - (mea_vel(2,:))).^2);
    % a_fb_xy = abs((kpos*delta_pos) + (kvel*(delta_vel)));

    if z_enabled == "y"
        desired_alt = derivatives(13,i) - opti_offset;
        z_cmd_vel_acc = (kvel(3,1)*(derivatives(16,i) - mea_xyz_vel(3,1))) + derivatives(19,i); 
    else
        z_cmd_vel_acc = 0;
    end

    error = [derivatives(11,i) - mea_xyz_pos(1,1);derivatives(12,i) - mea_xyz_pos(2,1);mea_xyz_pos(3,1)-desired_alt]; 
    a_rd = [mea_xyz_vel(1,1)*linear_drag_coeff(1,1);mea_xyz_vel(2,1)*linear_drag_coeff(1,2);mea_xyz_vel(3,1)*linear_drag_coeff(1,3)]; 
    a_des(1,:) = (kpos(1,1)*error(1,1)) + (kdpos(1,1)*(error(1,1) - error_past(1,1))) + (kvel(1,1)*(derivatives(14,i) - mea_xyz_vel(1,1))); %+ derivatives(17,i) - a_rd(1,1); % x
    a_des(2,:) = (kpos(2,1)*error(2,1)) + (kdpos(2,1)*(error(2,1) - error_past(2,1))) + (kvel(2,1)*(derivatives(15,i) - mea_xyz_vel(2,1))); %+ derivatives(18,i) - a_rd(2,1); % y
    a_des(3,:) = (kpos(3,1)*error(3,1)) + (kdpos(3,1)*(error(3,1) - error_past(3,1))) + g - a_rd(3,1) - z_cmd_vel_acc; % z, ellipse needa add the derivatives for z, a_ref is negative 
    %% z (can be used to test, needs to activate hover flaps mode)
    a_des_z(3,:) = a_des(3,:); 
    error_past = error; 
    mea_xyz_pos_past = mea_xyz_pos;
    ref_x = derivatives(11,i);
    ref_y = derivatives(12,i);
    ref_z = desired_alt;
    ref_vx = derivatives(14,i);
    ref_vy = derivatives(15,i);
    ref_vz = derivatives(16,i);

    %% old code
    % a_fb_xy = abs((kpos*(derivatives(1,i) - mea_xy_pos_mag)) + (kvel*(derivatives(2,i) - mea_xy_vel_mag))); % xy_magnitude plane since yaw can be easily taken care of 
    % gain = kpos*(derivatives(1,i) - mea_xy_pos_mag)/abs(kpos*(derivatives(1,i) - mea_xy_pos_mag)); % always negative cos derivatives default value simply too small (negligible)
    % a_rd = sqrt((derivatives(14,i).^2) + (derivatives(15,i).^2)) * linear_drag_coeff(1,1);
    % a_rd = derivatives(2,i) * linear_drag_coeff(1,1);
    % a_des(1,:) = a_fb_xy + sqrt((derivatives(16,i).^2) + (derivatives(17,i).^2)) - a_rd; % fits into the x axis of ades 

    % z_error = mea_pos(3,1)-derivatives(13,c);
    % z_error = mea_xyz_pos(3,1)-desired_alt; % this one is with the fixed height
    % a_rd_z = mea_vel(3) * Dz;
    % a_fb_z = kpos_z*z_error + kd_z*(z_error-z_error_past); % z
    % disp ("alt: ");
    % disp ("Pos & Att: X,Y,Z,Pitch ");
    % disp([mea_x_pos mea_y_pos mea_z_pos mea_pitch]);
    % a_des(3,:) = a_fb_z + g - a_rd_z;
    % a_des_z(3,:) = a_fb_z + g - a_rd_z;
 
    % direction (actual)
    % desired_heading = atan2((derivatives(12,i)-mea_y_pos),(derivatives(11,i)-mea_x_pos));
    % desired_heading = derivatives(6,i);
    % true_heading = desired_heading;
    % log_head = true_heading;

    %% Collective thrust (can be used to test)
    zd = a_des / norm(a_des); % consists of all 3 axis, this was segregated due to collective and cyclic thrust decoupling  
    zd_z = a_des_z / norm(a_des_z); % 3 x 1 = z_desired, if empty it would be 0 0 1 
    % cmd_z = dot(transpose(zd),transpose(a_des)); %% command sent to motor, need to include filter to make sure negative cmds dun go thru
    cmd_z = dot(transpose(zd_z),transpose(a_des_z)); %% command sent to motor, need to include filter to make sure negative cmds dun go thru
    rho = 1.225;
    radius = 0.61;
    cl = 0.11 * mea_bod_pitch_deg; % gradient for cl taken from naca 0006, pitch must be in deg
    cd = 0.023 * mea_bod_pitch_deg; % gradient for cd taken from naca 0006, pitch must be in deg
    chord_length = 0.1;
    mass = 0.16;
    Fz_wo_mass = -1*(cl*rho*chord_length*(radius^3))/(6*mass);
    Fd_wo_mass = 1*(cd*rho*chord_length*(radius^3))/(6*mass);
    omega_z = cmd_z/(Fz_wo_mass + Fd_wo_mass); % omega z as motor input
    %omega_z = sqrt(cmd_z/(Fz_wo_mass + Fd_wo_mass)); % omega z for body rotation rate
    
    % (INDI Component for Collective Thrust) %%% continue from here
    % omega_z = omega_z - abs(body_frame_angular_rate(3,1)); % this is for body rotation which we aint doing this time round
    
    % Mixing
    omega_z = omega_z/kt; % this is for motor input where kt*omega_z as in the paper

    % omega_z = 0.05*omega_z;
    % if omega_z > 0.7
    %     omega_z = 0.7;
    % end

    log_motor_input = omega_z;
    %fprintf('Desired (%d) vs Actual (%d)', counter_des, counter_actl);
    %fprintf('Omega z is : %d', omega_z);

    %% 1. Attitude controller
    
    % Bodyrates (for collective thrust test, this entire section can be disabled)
    qz = eul2quat(mea_euler); % default seq is q = [w x y z]
    disk_vector = quatrotate(qz,ez); % vector of 1 x 3
    angle = acos((dot(disk_vector,transpose(zd))/(norm(disk_vector)*norm(zd)))); % will nvr catch up one
    n = cross(disk_vector,transpose(zd)) / norm(cross(disk_vector,transpose(zd)));
    B = quatrotate((quatinv(qz)),n);

    % if angle = 0, it would just be an identity quat matrix
    error_quat = [cos(angle/2),B*sin(angle/2)];

    % can always break here to make sure shit is running correctly
    
    if error_quat(:,1) < 0
        body_rates = -2 * error_quat(:,2:3); % 2 and 3 refers to omega x and y
    else
        body_rates = 2 * error_quat(:,2:3);
    end

%     disp("body_rates");
%     disp(body_rates);

    %cmd_att = kpa.*([body_rates(1,1);body_rates(1,2);0]/3); % rp - wy, wx for moving along x y, want to compare purely against this need to switch to /1
    % cmd_att = kpa.*([body_rates(1,1);body_rates(1,2);0]); % solution is abt x and abt y 
    bod_test = exp.test_attitude(mea_euler,a_des); % solution is x and y 
    cmd_att = kpa.*([body_rates(1,2);body_rates(1,1);0]); %rpy

    %% 2. Diff Flat Feedforward component (for jerk) 20 21 22 is jerk xyz 

    % Jerk
    des_jerk_to_ang_rates = [derivatives(20,i)/(-1*cmd_z);derivatives(21,i)/cmd_z;0]; % rp - wy, wx for moving along x y 
    cmd_bod_rate = kpr.*((des_jerk_to_ang_rates - mea_angular_rate));

    %% 3. Diff Flat Feedforward component (for snap) 23 24 25 is snap xyz

    % Snap
    ff_snap = [derivatives(23,i)/(-1*cmd_z);derivatives(24,i)/cmd_z;0]; % rp - wy, wx for moving along x y 
    
    %% 4. Angular acc combine all 3
    %cmd_bod_acc = (cmd_att + cmd_bod_rate + ff_snap); % reverted (now bod rate ref is separate from the gain) gain for cyclic, multiply this to azimuth sin or cos from quadrant, the other value is the desired heading
    cmd_bod_acc = cmd_att;

    % (INDI Component for body ang acc) - needa include j = moment of inertia
    error_ang = cmd_bod_acc - mea_angular_rate_rate;
    cmd_bod_acc = kpang.*error_ang + kdang.*(error_ang-error_ang_past);
    % cmd_bod_acc = I*cmd_bod_acc; %% cont tmr...
    log_bod_acc = cmd_bod_acc;
    error_ang_past = error_ang;
    % cmd_bodyrate = flap_feedback + j*cmd_bodyrate;
        
    %     bod_rate_cap = 0.117;
    %     if abs(cmd_bodyrate) > bod_rate_cap
    %         cmd_bodyrate = 0.117;
    %     end  

    %% Pitch
    % desired_pitch_heading = 0; % changed to zero cos of alignment issues
    % desired_pitch_heading = exp.sam_new_heading_input(desired_pitch_heading); % to account for phase delay 
    % quadrant_pitch = exp.quadrant_output(desired_pitch_heading); 
    
    %% Roll
    % desired_roll_heading = -pi/2; % pi/2 heading is facing forward
    % desired_roll_heading = exp.sam_new_heading_input(desired_roll_heading); % to account for phase delay 
    % quadrant_roll = exp.quadrant_output(desired_roll_heading); 

    %% Flap inputs
    %centri_input = exp.flap_output(mea_rotation,centri_quadrant,centri_heading,-1*ppc*abs(cmd_bodyrate));      
    %init_input = exp.flap_output(mea_rotation,quadrant,desired_heading,-1*abs(cmd_bod_acc));% conservation of angular momentum thats why need -1     
    
    % pitch_input = exp.flap_output(mea_rotation,quadrant_pitch,desired_pitch_heading,-1*cmd_bod_acc(2,1));% conservation of angular momentum thats why need -1     
    % roll_input = exp.flap_output(mea_rotation,quadrant_roll,desired_roll_heading,-1*cmd_bod_acc(1,1));% conservation of angular momentum thats why need -1  
    
    % Mixing
    %final_flap_input = deg2rad((pitch_input(:,1) + roll_input(:,1)) * 5000); % tried braking and heading compensation, not very good
  
    %% trigger
    %     trigger = trigger + 1;
    %     if mod(trigger,16) == 0
    
    %i = i + 50; % 50 or 30 is the number to update
    %c = c + 1;
    %     end
    %     trigger = trigger + update_rate; % temporary holding

    if ref_counter == sample_per_loop
        ref_counter = 1;
        log_laps = log_laps + 1;
    elseif ref_counter < sample_per_loop
        ref_counter = ref_counter + 1;
    end
        
    i = i + 1; % 50 or 30 is the number to update
    c = c + 1;
    
    %input = [0,final_flap_input,omega_z,mea_rotation]; % heading, flap, motor, yaw
    
    % plot(ref_x, ref_y, '-o')
    % hold on
    % plot(mea_xyz_pos(1,1), mea_xyz_pos(2,1), '-*')
    % hold off
    % xlabel('X-Pose')
    % ylabel('Y-Pose')
    % xlim([0,4])
    % ylim([0,4])
    % title('Live plotting')
    % legend('Ref','Actual')
    % input = [round(mea_rotation,2),round(0,2),round(0,2), abs(round(omega_z,2))]; % heading, flap, motor, yaw
    % input = [input_x, input_y, input_z, exp_daa];


   
    %input_a = (a_des(1,:)/abs(a_des(1,:))) * abs(round(cmd_bod_acc(1,:),2)); % signs occasionally can be inconsistent, need to use the signs from error
    input_a = a_des(1,:);
    if abs(input_a) > 1
        input_a = input_a/abs(input_a);
    end
    %input_b = (a_des(2,:)/abs(a_des(2,:))) * abs(round(cmd_bod_acc(2,:),2));
    input_b = a_des(2,:);
    if abs(input_b) > 1
        input_b = input_b/abs(input_b);
    end
    input_c = round(a_des_z(3,:)/-100,2);
    %input_d = round(final_flap_input,2) + abs(round(omega_z*10500,2));
    input_d = round(0,2);

    %input = [round(0,2),round(0,2),round(a_des_z(3,:)/-100,2),round(0,2)]; % to wj cyclic
    input = [input_a,input_b,input_c,input_d]; % my cyclic
    %fprintf('\t flap pulse input, flap omega_z input %f,%f\n', final_flap_input, abs(round(omega_z,2)));
    fprintf('\t  input [%f,%f,%f,%f]\n',input);
    %fprintf('\t bod_rates [%f,%f,%f]\n', transpose(cmd_att));
    %fprintf('\t bod_rates [%f,%f]\n', bod_test);
    fprintf('\t cmd bod acc [%f,%f,%f]\n', transpose(cmd_bod_acc));
    %fprintf('\t pitch input %f\n', pitch_input(:,1));
    %fprintf('\t roll input %f\n', roll_input(:,1));
    %fprintf('\t Counter is %f\n', i);
    fprintf('\t Pos [%f,%f,%f]\n', transpose(mea_pos));
    %fprintf('\t Error [%f,%f,%f]\n', transpose(error_past));
    %fprintf('\t Ref Pos [%f,%f,%f]\n', ref_x,ref_y,ref_z);
    %fprintf('\t Inertia Euler angles [%f,%f,%f]\n', transpose(inertia_frame_angles));
    fprintf('\t Disk Euler angles [%f,%f,%f]\n', mea_angles(1,1), mea_angles(2,1), 0);
    
    %fprintf('\t cl and cd %f,%f\n', cl, cd);
    write(up,input,"double", computerip,port);


    %pause(rate);
end

%% Write to excel
filename = 'C:\Users\WJ-AIR\OneDrive - Singapore University of Technology and Design\Emma RAL25\ral_sam_data\Sam0pos_hold.xlsx';
disp("FINISH")
writematrix(data_arr,filename,'Sheet',1,'Range','B2'); % ("Array",filename,~,sheetname,~,range of cells to paste in 'E1:I5')