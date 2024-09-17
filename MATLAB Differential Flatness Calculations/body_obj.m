%% Rigid Body object
% Used to store variables for a rigid body
classdef body_obj < handle
   properties
      bodyname="";
      position=[0,0,0];
      quarternion=[0,0,0,0];
      euler=[0,0,0];
      past_euler=[0,0,0];
      euler_rate=[0 0 0];
      velocity=[0 0 0];
      acceleration=[0 0 0];
      past_position=[0,0,0];
      past_velocity=[0,0,0];
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
        obj.quarternion = [rb.Quaternion(1) rb.Quaternion(2) rb.Quaternion(3) rb.Quaternion(4)]; % w, x, y, z
        obj.position = [rb.Position(1)/1000 rb.Position(2)/1000 rb.Position(3)/1000];
        eul_zyz = quat2eul(obj.quarternion,'ZYZ'); % zyz, yaw pitch roll
        eul_xyz = quat2eul(obj.quarternion,'XYZ'); %  xyz, roll pitch yaw
        eul = [0,eul_xyz(1),eul_xyz(3)];
        eul(3) = -eul(3);   % (3 = yaw, 2 = pitch, 1 = roll) for the disk 
        obj.euler = eul;
        obj.euler_rate = (obj.euler-obj.past_euler)/((rb.TimeStamp-obj.past_time)*1000);
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
        disp("yaw     pitch    roll");
        disp([obj.euler(3) obj.euler(2) obj.euler(1)]);
        %disp([eul_zyz(1) eul_zyz(2) eul_zyz(3)]);
        %disp([eul_xyz(1) eul_xyz(2) eul_xyz(3)]);
        %disp([obj.euler_rate(3) obj.euler_rate(2) obj.euler_rate(1)])
         %disp(obj.euler(2))
         %disp(obj.euler(1))
        obj.past_velocity = obj.velocity;
        obj.past_position = obj.position;
        obj.past_euler=obj.euler;
        obj.past_time=rb.TimeStamp;
      end

   end
end
