with Motion.Planner; use Motion.Planner;
with Ada.Text_IO;    use Ada.Text_IO;
with Ada.Exceptions;

package body Motion.Executor is

   Config  : Config_Parameters;
   Working : Execution_Block;

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

   function Compute_Bezier_Point (Bez : Bezier; T : Dimensionless) return Scaled_Position is
      Bez_2 : Bezier := Bez;
   begin
      for J in reverse Bez_2'First .. Bez_2'Last - 1 loop
         for I in Bez_2'First .. J loop
            Bez_2 (I) := Bez_2 (I) + (Bez_2 (I + 1) - Bez_2 (I)) * T;
         end loop;
      end loop;

      return Bez_2 (Bez_2'First);
   end Compute_Bezier_Point;

   procedure Logger is
      Current_Time : Time := 0.0 * s;
   begin
      for I in Working.Feedrate_Profiles'Range loop
         declare
            Start_Curve_Half_Distance : constant Length := Distance_At_T (Working.Beziers (I - 1), 0.5);
            End_Curve_Half_Distance   : constant Length := Distance_At_T (Working.Beziers (I), 0.5);
            Mid_Distance              : constant Length :=
              abs (Working.Beziers (I) (Bezier_Index'First) - Working.Beziers (I - 1) (Bezier_Index'Last));
         begin
            while Current_Time < Total_Time (Working.Feedrate_Profiles (I)) loop
               declare
                  Distance : Length :=
                    Distance_At_Time
                      (Working.Feedrate_Profiles (I),
                       Current_Time,
                       Working.Segment_Limits (I).Crackle_Max,
                       Working.Corner_Velocity_Limits (I - 1));
                  Point    : Scaled_Position;
               begin
                  if Distance < Start_Curve_Half_Distance then
                     Point :=
                       Compute_Bezier_Point
                         (Working.Beziers (I - 1),
                          T_At_Distance (Working.Beziers (I - 1), Distance + Start_Curve_Half_Distance));
                  elsif Distance < Start_Curve_Half_Distance + Mid_Distance then
                     Point :=
                       Working.Beziers (I - 1) (Bezier_Index'Last) +
                       (Working.Beziers (I) (Bezier_Index'First) - Working.Beziers (I - 1) (Bezier_Index'Last)) *
                         ((Distance - Start_Curve_Half_Distance) / Mid_Distance);
                  else
                     Point :=
                       Compute_Bezier_Point
                         (Working.Beziers (I),
                          T_At_Distance (Working.Beziers (I), Distance - Start_Curve_Half_Distance - Mid_Distance));
                  end if;
                  -- Running_Average.Push (Point);
                  -- Put_Line (Running_Average.Get (X_Axis)'Image & "," & Running_Average.Get (Y_Axis)'Image);
                  Put_Line (Point (X_Axis)'Image & "," & Point (Y_Axis)'Image);
               end;
               Current_Time := Current_Time + Logger_Interpolation_Time;
            end loop;

            Current_Time := Current_Time - Total_Time (Working.Feedrate_Profiles (I));
         end;
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
