with Ada.Text_IO; use Ada.Text_IO;

package body Motion.Logger is

   function Total_Time (Times : Acceleration_Profile_Times) return Time is
   begin
      return 8.0 * Times (1) + 4.0 * Times (2) + 2.0 * Times (3) + Times (4);
   end Total_Time;

   --  TODO: Add caching here
   function Point_Along_Curve (Start, Finish : Curve_Point_Set; Distance : Length) return Scaled_Position is
      Distance_To_Go : Length          := Distance;
      Last_Point     : Scaled_Position := Start (0);
   begin
      -- return Last_Point;
      --  TODO: Maybe find point on bezier instead of on line between points.
      for I in 1 .. Start'Last loop
         --  TODO: Remove magic constant.
         if Distance_To_Go < 0.000_1 * mm then
            -- Put_Line ("S0" & Curve_Point_Set_Index'Image (I));
            return Last_Point;
         elsif abs (Start (I) - Last_Point) >= Distance_To_Go then
            -- Put_Line ("S+" & Curve_Point_Set_Index'Image (I));
            return Last_Point + (Start (I) - Last_Point) * (Distance_To_Go / abs (Start (I) - Last_Point));
         else
            Distance_To_Go := Distance_To_Go - abs (Start (I) - Last_Point);
            Last_Point     := Start (I);
         end if;
      end loop;

      for I in Finish'First .. 0 loop
         --  TODO: Remove magic constant.
         if Distance_To_Go < 0.000_1 * mm then
            -- Put_Line ("F0" & Curve_Point_Set_Index'Image (I));
            return Last_Point;
         elsif abs (Finish (I) - Last_Point) >= Distance_To_Go then
            -- Put_Line ("F+" & Curve_Point_Set_Index'Image (I));
            return Last_Point + (Finish (I) - Last_Point) * (Distance_To_Go / abs (Finish (I) - Last_Point));
         else
            Distance_To_Go := Distance_To_Go - abs (Finish (I) - Last_Point);
            Last_Point     := Finish (I);
         end if;
      end loop;

      -- Put_Line ("PAST END");
      return Last_Point;
   end Point_Along_Curve;

   task body Runner is
      Config : Config_Parameters;

      function Fast_Velocity_At_Time (Profile : Acceleration_Profile_Times; Start_Vel : Velocity) return Velocity is
         T1 : constant Time     := Profile (1);
         T2 : constant Time     := Profile (2);
         T3 : constant Time     := Profile (3);
         T4 : constant Time     := Profile (4);
         Cm : constant Crackle  := Config.Crackle_Limit;
         Vs : constant Velocity := Start_Vel;
      begin
         return Vs + Cm * T1 * (T1 + T2) * (2.0 * T1 + T2 + T3) * (4.0 * T1 + 2.0 * T2 + T3 + T4);
      end Fast_Velocity_At_Time;

      function Fast_Distance_At_Time (Profile : Acceleration_Profile_Times; Start_Vel : Velocity) return Length is
         T1 : constant Time     := Profile (1);
         T2 : constant Time     := Profile (2);
         T3 : constant Time     := Profile (3);
         T4 : constant Time     := Profile (4);
         Cm : constant Crackle  := Config.Crackle_Limit;
         Vs : constant Velocity := Start_Vel;
      begin
         return
           (Vs + Cm * T1 * (T1 + T2) * (2.0 * T1 + T2 + T3) * (4.0 * T1 + 2.0 * T2 + T3 + T4) / 2.0) *
           (8.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4);
      end Fast_Distance_At_Time;

      procedure Processor (Data : in out Block_Data) is
         Last_End_Time : Time := 0.0 * s;
         Current_Time  : Time := 0.0 * s;
      begin
         pragma Assert (Data.Last_Stage = Acceleration_Profile_Generator_Stage);

         for I in Data.Segment_Acceleration_Profiles'Range loop
            declare
               Corner_Distance : Length :=
                 Curve_Corner_Distance (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I));
               Accel_Distance  : Length :=
                 Fast_Distance_At_Time
                   (Data.Segment_Acceleration_Profiles (I).Accel, Data.Corner_Velocity_Limits (I - 1));
            begin
               while Current_Time < Last_End_Time + Total_Time (Data.Segment_Acceleration_Profiles (I).Accel) loop
                  declare
                     Distance : Length          :=
                       Distance_At_Time
                         (Current_Time - Last_End_Time,
                          Data.Segment_Acceleration_Profiles (I).Accel,
                          Config.Crackle_Limit,
                          Data.Corner_Velocity_Limits (I - 1));
                     Point    : Scaled_Position :=
                       Point_Along_Curve (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I), Distance);
                  begin
                     Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)));
                     null;
                  end;
                  Current_Time := Current_Time + Logger_Interpolation_Time;
               end loop;

               Last_End_Time := Last_End_Time + Total_Time (Data.Segment_Acceleration_Profiles (I).Accel);

               --  TODO: Coasting.

               Last_End_Time := Last_End_Time + Data.Segment_Acceleration_Profiles (I).Coast;

               while Current_Time < Last_End_Time + Total_Time (Data.Segment_Acceleration_Profiles (I).Decel) loop
                  declare
                     Distance : Length          :=
                       Distance_At_Time
                         (Current_Time - Last_End_Time,
                          Data.Segment_Acceleration_Profiles (I).Accel,
                          Config.Crackle_Limit,
                          Data.Corner_Velocity_Limits (I - 1));
                     Point    : Scaled_Position :=
                       Point_Along_Curve
                         (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I), Distance + Accel_Distance);
                  begin
                     Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)));
                     null;
                  end;
                  Current_Time := Current_Time + Logger_Interpolation_Time;
               end loop;

               Last_End_Time := Last_End_Time + Total_Time (Data.Segment_Acceleration_Profiles (I).Decel);
            end;
         end loop;
      end Processor;

   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Wait (Logger_Stage);
            Block_Queue (Block_Index).Process (Logger_Stage, Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Logger;
