with Motion.Planner; use Motion.Planner;
with Ada.Text_IO;    use Ada.Text_IO;
with Ada.Exceptions;

package body Motion.Executor is

   Config  : Config_Parameters;
   Working : Execution_Block;

   function Curve_Corner_Distance (Start, Finish : Corners_Index) return Length is
   begin
      return
        Working.Curve_Point_Sets (Start).Outgoing_Length + Working.Curve_Point_Sets (Finish).Incoming_Length +
        abs
        (Working.Curve_Point_Sets (Start).Outgoing (Working.Curve_Point_Sets (Start).Outgoing'Last) -
         Working.Curve_Point_Sets (Finish).Incoming (Working.Curve_Point_Sets (Finish).Incoming'First));
   end Curve_Corner_Distance;

   --  TODO: Add caching.
   function Point_Along_Curve (Start, Finish : Curve_Point_Set_Values; Distance : Length) return Scaled_Position is
      Distance_To_Go : Length          := Distance;
      Last_Point     : Scaled_Position := Start (Start'First);
   begin
      for I in Start'First + 1 .. Start'Last loop
         if abs (Start (I) - Last_Point) >= Distance_To_Go and abs (Start (I) - Last_Point) /= 0.0 * mm then
            return Last_Point + (Start (I) - Last_Point) * (Distance_To_Go / abs (Start (I) - Last_Point));
         else
            Distance_To_Go := Distance_To_Go - abs (Start (I) - Last_Point);
            Last_Point     := Start (I);
         end if;
      end loop;

      for I in Finish'First .. Finish'Last loop
         if abs (Finish (I) - Last_Point) >= Distance_To_Go and abs (Finish (I) - Last_Point) /= 0.0 * mm then
            return Last_Point + (Finish (I) - Last_Point) * (Distance_To_Go / abs (Finish (I) - Last_Point));
         else
            Distance_To_Go := Distance_To_Go - abs (Finish (I) - Last_Point);
            Last_Point     := Finish (I);
         end if;
      end loop;
      
      return Last_Point;
   end Point_Along_Curve;

   procedure Logger is
      Last_End_Time : Time := 0.0 * s;
      Current_Time  : Time := 0.0 * s;
   begin
      for I in Working.Feedrate_Profiles'Range loop
         declare
            Corner_Distance : constant Length   := Curve_Corner_Distance (I - 1, I);
            Accel_Distance  : constant Length   :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Accel,
                 Working.Segment_Limits (I).Crackle_Max,
                 Working.Corner_Velocity_Limits (I - 1));
            -- Coast_Velocity  : constant Velocity :=
            --   Fast_Velocity_At_Max_Time
            --     (Working.Feedrate_Profiles (I).Accel,
            --      Working.Segment_Limits (I).Crackle_Max,
            --      Working.Corner_Velocity_Limits (I - 1));
            Decel_Distance  : constant Length   :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Decel, Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I));
            -- Accel_Distance : constant Length := Distance_At_Time (Working.Feedrate_Profiles (I).Accel, Total_Time (Working.Feedrate_Profiles (I).Accel), Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            -- Decel_Distance : constant Length := Distance_At_Time (Working.Feedrate_Profiles (I).Decel, Total_Time (Working.Feedrate_Profiles (I).Decel), Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I));
            Coast_Distance : constant Length := Corner_Distance - Accel_Distance - Decel_Distance;
         begin
            -- Put_Line ("ACCEL" & I'Image);
            while Current_Time - Last_End_Time < Total_Time (Working.Feedrate_Profiles (I).Accel) loop
               declare
                  Distance : Length          :=
                    Distance_At_Time
                      (Working.Feedrate_Profiles (I).Accel,
                       Current_Time - Last_End_Time,
                       Working.Segment_Limits (I).Crackle_Max,
                       Working.Corner_Velocity_Limits (I - 1));
                  Point    : Scaled_Position :=
                    Point_Along_Curve
                      (Working.Curve_Point_Sets (I - 1).Outgoing, Working.Curve_Point_Sets (I).Incoming, Distance);
               begin
                  Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)) & "," & Distance'Image);
                  null;
               end;
               Current_Time := Current_Time + Logger_Interpolation_Time;
            end loop;

            Last_End_Time := Last_End_Time + Total_Time (Working.Feedrate_Profiles (I).Accel);

            -- Put_Line ("COAST" & I'Image & Working.Feedrate_Profiles (I)'Image);
            -- Put_Line ("Coast dist: " & Coast_Distance'Image);
            while Current_Time - Last_End_Time < Working.Feedrate_Profiles (I).Coast loop
               declare
                  Distance : Length          :=
                    Coast_Distance * ((Current_Time - Last_End_Time) / Working.Feedrate_Profiles (I).Coast);
                  Point    : Scaled_Position :=
                    Point_Along_Curve
                      (Working.Curve_Point_Sets (I - 1).Outgoing,
                       Working.Curve_Point_Sets (I).Incoming,
                       Accel_Distance + Distance);
               begin
                  Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)) & "," & Velocity'Image(Accel_Distance + Distance));
                  null;
               end;
               Current_Time := Current_Time + Logger_Interpolation_Time;
            end loop;

            Last_End_Time := Last_End_Time + Working.Feedrate_Profiles (I).Coast;

            -- Put_Line ("DECEL" & I'Image);
            while Current_Time - Last_End_Time < Total_Time (Working.Feedrate_Profiles (I).Decel) loop
               declare
                  Distance : Length          :=
                    Distance_At_Time
                      (Working.Feedrate_Profiles (I).Decel,
                       Last_End_Time + Total_Time (Working.Feedrate_Profiles (I).Decel) - Current_Time,
                       Working.Segment_Limits (I).Crackle_Max,
                       Working.Corner_Velocity_Limits (I));
                  Point    : Scaled_Position :=
                    Point_Along_Curve
                      (Working.Curve_Point_Sets (I - 1).Outgoing,
                       Working.Curve_Point_Sets (I).Incoming,
                       Corner_Distance - Distance);
               begin
                  Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)) & "," & Velocity'Image (Accel_Distance + Coast_Distance + Distance));
                  null;
               end;
               Current_Time := Current_Time + Logger_Interpolation_Time;
            end loop;
            
            -- while Current_Time < Last_End_Time + Total_Time (Working.Feedrate_Profiles (I).Decel) loop
            --    declare
            --       Distance : Length          :=
            --         Distance_At_Time
            --           (Working.Feedrate_Profiles (I).Decel,
            --            Current_Time - Last_End_Time,
            --            -Working.Segment_Limits (I).Crackle_Max,
            --            Coast_Velocity);
            --       Point    : Scaled_Position :=
            --         Point_Along_Curve
            --           (Working.Curve_Point_Sets (I - 1).Outgoing,
            --            Working.Curve_Point_Sets (I).Incoming,
            --            Accel_Distance + Coast_Distance + Distance);
            --    begin
            --       Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)) & "," & Velocity'Image (Accel_Distance + Coast_Distance + Distance));
            --       null;
            --    end;
            --    Current_Time := Current_Time + Logger_Interpolation_Time;
            -- end loop;

            Last_End_Time := Last_End_Time + Total_Time (Working.Feedrate_Profiles (I).Decel);
            
            Current_Time := Current_Time - Last_End_Time;
            Last_End_Time := 0.0 * s;
         end;
      end loop;
   end Logger;

   task body Runner is
   begin
      accept Init (Conf : Config_Parameters) do
         Config := Conf;
      end Init;

      loop
         Execution_Block_Queue.Dequeue (Working);
         Logger;
      end loop;
   exception
      when E : others =>
         Put_Line ("Exception in Motion.Executor:");
         Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

end Motion.Executor;
