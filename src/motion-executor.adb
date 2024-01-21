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

   package Curve_Walker is
      procedure Reset_For_Segment (Current_Segment : Corners_Index);
      function Walk_To (Distance : Length) return Scaled_Position;
   end Curve_Walker;

   package body Curve_Walker is
      Segment         : Corners_Index;
      In_Second_Curve : Boolean;
      Last_Point      : Scaled_Position;
      I               : Curve_Point_Set_Index;
      Last_Distance   : Length;

      procedure Reset_For_Segment (Current_Segment : Corners_Index) is
      begin
         Segment         := Current_Segment;
         In_Second_Curve := False;
         I               := Working.Curve_Point_Sets (Segment - 1).Outgoing'First;
         Last_Point      := Working.Curve_Point_Sets (Segment - 1).Outgoing (I);
         Last_Distance   := 0.0 * mm;
      end Reset_For_Segment;

      function Walk_To (Distance : Length) return Scaled_Position is
         Distance_To_Go : Length := Distance - Last_Distance;
      begin
         pragma Assert (Distance >= Last_Distance);

         if not In_Second_Curve then
            loop
               if abs (Working.Curve_Point_Sets (Segment - 1).Outgoing (I) - Last_Point) >= Distance_To_Go and
                 abs (Working.Curve_Point_Sets (Segment - 1).Outgoing (I) - Last_Point) /= 0.0 * mm
               then
                  return
                    Last_Point +
                    (Working.Curve_Point_Sets (Segment - 1).Outgoing (I) - Last_Point) *
                      (Distance_To_Go / abs (Working.Curve_Point_Sets (Segment - 1).Outgoing (I) - Last_Point));
               else
                  Last_Distance  :=
                    Last_Distance + abs (Working.Curve_Point_Sets (Segment - 1).Outgoing (I) - Last_Point);
                  Distance_To_Go := Distance - Last_Distance;
                  Last_Point     := Working.Curve_Point_Sets (Segment - 1).Outgoing (I);
                  exit when I = Working.Curve_Point_Sets (Segment - 1).Points_Per_Side;
                  I := I + 1;
               end if;
            end loop;

            In_Second_Curve := True;
            I               := Working.Curve_Point_Sets (Segment).Incoming'First;
         end if;

         loop
            if abs (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point) >= Distance_To_Go and
              abs (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point) /= 0.0 * mm
            then
               return
                 Last_Point +
                 (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point) *
                   (Distance_To_Go / abs (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point));
            else
               Last_Distance := Last_Distance + abs (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point);
               Distance_To_Go := Distance - Last_Distance;
               Last_Point     := Working.Curve_Point_Sets (Segment).Incoming (I);
               exit when I = Working.Curve_Point_Sets (Segment).Points_Per_Side;
               I := I + 1;
            end if;
         end loop;

         return Last_Point;
      end Walk_To;
   end Curve_Walker;

   procedure Logger is
      Last_End_Time : Time := 0.0 * s;
      Current_Time  : Time := 0.0 * s;
   begin
      for I in 2 .. Working.N_Corners loop
         Curve_Walker.Reset_For_Segment (I);
         while Current_Time < Total_Time (Working.Feedrate_Profiles (I)) loop
            declare
               Distance : Length          :=
                 Distance_At_Time
                   (Working.Feedrate_Profiles (I),
                    Current_Time,
                    Working.Segment_Limits (I).Crackle_Max,
                    Working.Corner_Velocity_Limits (I - 1));
               Point    : Scaled_Position := Curve_Walker.Walk_To (Distance);
            begin
               Put_Line (Length'Image (Point (X_Axis)) & "," & Length'Image (Point (Y_Axis)) & "," & Distance'Image);
            end;
            Current_Time := Current_Time + Logger_Interpolation_Time;
         end loop;

         Current_Time := Current_Time - Total_Time (Working.Feedrate_Profiles (I));
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
