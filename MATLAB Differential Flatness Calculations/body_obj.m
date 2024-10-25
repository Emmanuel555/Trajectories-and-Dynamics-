%% Rigid Body object
% Used to store variables for a rigid body
classdef body_obj < handle
   properties
      bodyname="";
      position=[0,0,0];
      velocity=[0 0 0];
      acceleration=[0 0 0];
      quaternion=[0,0,0,0];
      euler=[0,0,0];
      euler_rate=[0 0 0];
      quat_euler=[0 0 0];
      quat_euler_rate=[0 0 0];
      quat_euler_rate_rate=[0 0 0];
      euler_rate_rate=[0 0 0];
      past_position=[0,0,0];
      past_velocity=[0,0,0];
      past_quaternion=[0,0,0,0];
      past_quaternion_rate=[0,0,0,0];
      past_euler=[0,0,0];
      past_euler_rate=[0 0 0];
      past_time=0;
      pitch_norm=0;
      body_roll;
      roll_rate;
      body_rotation;
   end
   methods
      function init(obj,name)
        obj.bodyname=name;
      end

      function updatepose(obj,rb)
        q0 = rb.Quaternion(1);
        q1 = rb.Quaternion(2);
        q2 = rb.Quaternion(3);
        q3 = rb.Quaternion(4);

        obj.quaternion = [q0,q1,q2,q3]; % w, x, y, z  1x4
        w = [-q1,q0,-q3,q2; -q2,q3,q0,-q1; -q3,-q2,q1,q0]; % 3x4
        dq = (obj.quaternion-obj.past_quaternion)/((rb.TimeStamp-obj.past_time)*1000);
        dqq = (dq-obj.past_quaternion_rate)/((rb.TimeStamp-obj.past_time)*1000);

        eul_quat = 2*w*obj.quaternion;
        eul_quat_rate = 2*w*dq; %3x1
        eul_quat_rate_rate = 2*w*dqq; %3x1

        obj.quat_euler = [eul_quat(2),eul_quat(1),eul_quat(3)]; % may need to swap 
        obj.quat_euler_rate = [eul_quat_rate(2),eul_quat_rate(1),eul_quat_rate(3)];
        obj.quat_euler_rate_rate = [eul_quat_rate_rate(2),eul_quat_rate_rate(1),eul_quat_rate_rate(3)];
        
        %obj.position = [rb.Position(1)/1000 rb.Position(2)/1000 rb.Position(3)/1000];
        obj.position = [rb.Position(1)/1000 rb.Position(2)/1000 rb.Position(3)/1000];

        % eul_zyz = quat2eul(obj.quarternion,'ZYZ'); % zyz, yaw pitch roll
        eul_xyz = quat2eul(obj.quaternion,'XYZ'); %  xyz, roll pitch yaw for body |_, must initialise with this form in optitrack
        % eul = [eul_xyz(2),eul_xyz(1),eul_xyz(3)]; %  xyz, roll pitch yaw - switch to  _| 
        offset = pi/2;
        eul = [-1*eul_xyz(1),eul_xyz(2),eul_xyz(3)];
        % eul(3) = -eul(3);   % (3 = yaw, 2 = pitch, 1 = roll) for the disk 
        obj.euler = eul;

        obj.euler_rate = (obj.euler-obj.past_euler)/((rb.TimeStamp-obj.past_time)*1000);
        obj.euler_rate_rate = (obj.euler_rate-obj.past_euler_rate)/((rb.TimeStamp-obj.past_time)*1000);
%          obj.pitch_norm=(((obj.euler(3)*sin(obj.euler(1))))+((obj.euler(2)*cos(obj.euler(1)))));

        %obj.euler=rad2deg(eul);
        if obj.position-obj.past_position == 0 %Reject 0 division
            obj.velocity=[0 0 0];
        else
            obj.velocity=(obj.position-obj.past_position)/((rb.TimeStamp-obj.past_time)*1000);
        end

        if obj.velocity-obj.past_velocity == 0 %Reject 0 division
            obj.acceleration=[0 0 0];
        else
            obj.acceleration=(obj.velocity-obj.past_velocity)/((rb.TimeStamp-obj.past_time)*1000);
        end

%         obj.pitch_norm=((-rb.Position(2)*cos(obj.euler(1)))/sin(obj.euler(1)));
        %disp("yaw     pitch    roll");
        %disp([obj.euler(3) obj.euler(2) obj.euler(1)]);
        %fprintf('\t Pos [%f,%f,%f]\n', obj.position);
        %fprintf('\t Euler angles [%f,%f,%f]\n', eul);
        %disp([eul_zyz(1) eul_zyz(2) eul_zyz(3)]);
        %disp([eul_xyz(1) eul_xyz(2) eul_xyz(3)]);
        %disp([obj.euler_rate(3) obj.euler_rate(2) obj.euler_rate(1)])
         %disp(obj.euler(2))
         %disp(obj.euler(1))
        obj.past_velocity = obj.velocity;
        obj.past_position = obj.position;
        obj.past_euler = obj.euler;
        obj.past_euler_rate = obj.euler_rate;
        obj.past_quaternion = obj.quaternion;
        obj.past_quaternion_rate = dq;
        obj.past_time = rb.TimeStamp;
      end

   end
end
