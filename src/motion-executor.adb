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
                  exit when I = Working.Curve_Point_Sets (Segment - 1).Outgoing'Last;
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
               Last_Distance  := Last_Distance + abs (Working.Curve_Point_Sets (Segment).Incoming (I) - Last_Point);
               Distance_To_Go := Distance - Last_Distance;
               Last_Point     := Working.Curve_Point_Sets (Segment).Incoming (I);
               exit when I = Working.Curve_Point_Sets (Segment).Incoming'Last;
               I := I + 1;
            end if;
         end loop;

         return Last_Point;
      end Walk_To;
   end Curve_Walker;

   package Running_Average is
      procedure Reset (Pos : Scaled_Position);
      procedure Push (Pos : Scaled_Position);
      function Get return Scaled_Position;
   end Running_Average;

   package body Running_Average is
      type Index is mod 8;
      Entries       : array (Index) of Scaled_Position;
      Current_Index : Index := 0;

      procedure Reset (Pos : Scaled_Position) is
      begin
         Entries := [others => Pos];
      end Reset;

      procedure Push (Pos : Scaled_Position) is
      begin
         Current_Index           := Current_Index + 1;
         Entries (Current_Index) := Pos;
      end Push;

      function Get return Scaled_Position is
         Pos_Sum   : Scaled_Position := [others => 0.0 * mm];
         Index_Sum : Dimensionless   := 0.0;
      begin
         for I in Index loop
            Pos_Sum   := Pos_Sum + Scaled_Position_Offset (Entries (Current_Index + 1 + I)) * 1.0;
            Index_Sum := Index_Sum + 1.0;
         end loop;

         return Pos_Sum / Index_Sum;
      end Get;
   end Running_Average;

   procedure Logger is
      Current_Time : Time := 0.0 * s;
   begin
      for I in Working.Feedrate_Profiles'Range loop
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
               -- Running_Average.Push (Point);
               -- Put_Line (Running_Average.Get (X_Axis)'Image & "," & Running_Average.Get (Y_Axis)'Image);
               Put_Line (Point (X_Axis)'Image & "," & Point (Y_Axis)'Image);
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
      -- Running_Average.Reset (Config.Initial_Position * Config.Limit_Scaler);

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
