with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Unchecked_Conversion;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Exceptions;

package body Motion.Planner is

   Config            : Config_Parameters;
   Working           : aliased Execution_Block;
   PP_Last_Pos       : Position;
   PP_Corners        : Block_Plain_Corners (1 .. Corners_Index'Last);
   PP_Segment_Limits : Block_Segment_Limits (2 .. Corners_Index'Last);

   package Elementary_Functions is new Ada.Numerics.Generic_Elementary_Functions (Dimensioned_Float);
   use Elementary_Functions;

   function Curve_Corner_Distance (Start, Finish : Corners_Index) return Length is
   begin
      return
        Working.Curve_Point_Sets (Start).Outgoing_Length + Working.Curve_Point_Sets (Finish).Incoming_Length +
        abs
        (Working.Curve_Point_Sets (Start).Outgoing (Working.Curve_Point_Sets (Start).Outgoing'Last) -
         Working.Curve_Point_Sets (Finish).Incoming (Working.Curve_Point_Sets (Finish).Incoming'First));
   end Curve_Corner_Distance;

   procedure Preprocessor is
      Next_Master       : Master_Manager.Master := Master_Manager.Motion_Master;
      N_Corners         : Corners_Index         := 1;
      Working_N_Corners : Corners_Index with
        Address => Working.N_Corners'Address;
   begin
      PP_Corners (1)      := PP_Last_Pos * Config.Limit_Scaler;
      Working.Next_Master := Master_Manager.Motion_Master;

      loop
         exit when N_Corners = Corners_Index'Last;
         N_Corners := N_Corners + 1;

         declare
            Next_Command : Command;
         begin
            Command_Queue.Dequeue (Next_Command);
            case Next_Command.Kind is
               when Flush_Kind =>
                  Next_Master := Next_Command.Next_Master;
                  N_Corners   := N_Corners - 1;
                  exit;
               when Move_Kind =>
                  if abs (PP_Last_Pos - Next_Command.Pos) < Preprocessor_Minimum_Move_Distance then
                     N_Corners := N_Corners - 1;
                  else
                     PP_Last_Pos                   := Next_Command.Pos;
                     PP_Corners (N_Corners)        := Next_Command.Pos * Config.Limit_Scaler;
                     PP_Segment_Limits (N_Corners) :=
                       (Velocity_Max     =>
                          Velocity'Min (Next_Command.Limits.Velocity_Max, Config.Max_Limits.Velocity_Max),
                        Acceleration_Max =>
                          Acceleration'Min (Next_Command.Limits.Acceleration_Max, Config.Max_Limits.Acceleration_Max),
                        Jerk_Max         => Jerk'Min (Next_Command.Limits.Jerk_Max, Config.Max_Limits.Jerk_Max),
                        Snap_Max         => Snap'Min (Next_Command.Limits.Snap_Max, Config.Max_Limits.Snap_Max),
                        Crackle_Max => Crackle'Min (Next_Command.Limits.Crackle_Max, Config.Max_Limits.Crackle_Max),
                        Chord_Error_Max  =>
                          Length'Min (Next_Command.Limits.Chord_Error_Max, Config.Max_Limits.Chord_Error_Max));
                  end if;
            end case;
         end;
      end loop;

      --  This is hacky and not portable, but if we try to assign to the entire record as you normally would then GCC
      --  insists on creating a whole Working_Data on the stack.
      Working_N_Corners      := N_Corners;
      Working.Corners        := PP_Corners (1 .. N_Corners);
      Working.Segment_Limits := PP_Segment_Limits (2 .. N_Corners);
      Working.Next_Master    := Next_Master;
   end Preprocessor;

   --  TODO: A lot of the equations here are transcribed almost directly from
   --  https://doi.org/10.1007/s00170-022-09463-y, we should go through them and see if we can optimise them to be more
   --  precise and faster as this is all being done with limited precision floats rather than a CAS.

   procedure Corner_Blender is

      --  function Compute_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Angle is
      --     function Clamped_Arccos (X : Dimensionless) return Angle is
      --     begin
      --        if X < -1.0 then
      --           return Ada.Numerics.Pi;
      --        elsif X > 1.0 then
      --           return 0.0;
      --        else
      --           return Arccos (X);
      --        end if;
      --     end Clamped_Arccos;
      --
      --     V1              : constant Scaled_Position_Offset := Start - Corner;
      --     V2              : constant Scaled_Position_Offset := Finish - Corner;
      --     Dot_Product     : constant Dimensionless          := Dot (V1 / abs V1, V2 / abs V2);
      --     Primary_Angle   : constant Angle                  := Clamped_Arccos (Dot_Product);
      --     Secondary_Angle : constant Angle                  := (Ada.Numerics.Pi - Primary_Angle) / 2.0;
      --  begin
      --     return Secondary_Angle;
      --  end Compute_Secondary_Angle;

      function Compute_Sine_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless is
         V1 : constant Scaled_Position_Offset := Start - Corner;
         V2 : constant Scaled_Position_Offset := Finish - Corner;
         A  : constant Area                   := Dot (V1, V2);
         B  : constant Area                   := 2.0 * (abs V1) * (abs V2);
      begin
         if 0.5 + A / B < 0.0 then
            return 0.0;
         elsif (0.5 + A / B)**(1 / 2) > 1.0 then
            return 1.0;
         else
            return (0.5 + A / B)**(1 / 2);
         end if;
      end Compute_Sine_Secondary_Angle;

      function Compute_Cosine_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Dimensionless is
         V1 : constant Scaled_Position_Offset := Corner - Start;
         V2 : constant Scaled_Position_Offset := Finish - Corner;
         A  : constant Area                   := Dot (V1, V2);
         B  : constant Area                   := 2.0 * (abs V1) * (abs V2);
      begin
         if 0.5 + A / B < 0.0 then
            return 0.0;
         elsif (0.5 + A / B)**(1 / 2) > 1.0 then
            return 1.0;
         else
            return (0.5 + A / B)**(1 / 2);
         end if;
      end Compute_Cosine_Secondary_Angle;

      function Compute_Bezier_Base_Length
        (Start, Corner, Finish : Scaled_Position; Chord_Error_Max : Length) return Length
      is
         Sine_Secondary_Angle   : constant Dimensionless := Compute_Sine_Secondary_Angle (Start, Corner, Finish);
         Cosine_Secondary_Angle : constant Dimensionless := Compute_Cosine_Secondary_Angle (Start, Corner, Finish);
         Incoming_Length        : constant Length        := abs (Start - Corner);
         Outgoing_Length        : constant Length        := abs (Finish - Corner);

         Deviation_Limit_Numerator   : constant Length        := Chord_Error_Max * 2.0**14;
         Deviation_Limit_Denominator : constant Dimensionless :=
           Sine_Secondary_Angle * (4_072_849.0 / 429.0 + 714.0 + 2.0**14 * 1_225.0 / (858.0 * Cosine_Secondary_Angle));
         Incoming_Limit              : constant Length        :=
           (0.49 * 858.0 * Incoming_Length * Cosine_Secondary_Angle) / (5_210.0 * Cosine_Secondary_Angle + 1_225.0);
         Outgoing_Limit              : constant Length        :=
           (0.49 * 858.0 * Outgoing_Length * Cosine_Secondary_Angle) / (5_210.0 * Cosine_Secondary_Angle + 1_225.0);
      begin
         --  TODO: Do we need a small error margin here?
         if Deviation_Limit_Denominator = 0.0 then
            --  Collinear points.
            return Length'Min (Incoming_Limit, Outgoing_Limit);
         else
            return
              Length'Min
                (Deviation_Limit_Numerator / Deviation_Limit_Denominator, Length'Min (Incoming_Limit, Outgoing_Limit));
         end if;
      end Compute_Bezier_Base_Length;

      function Compute_Unit_Bisector (Start, Corner, Finish : Scaled_Position) return Position_Scale is
         A        : constant Scaled_Position_Offset := Start - Corner;
         B        : constant Scaled_Position_Offset := Finish - Corner;
         Bisector : constant Position_Scale         := A / abs A + B / abs B;
      begin
         if abs Bisector = 0.0 then
            return Bisector;
         else
            return Bisector / abs Bisector;
         end if;
      end Compute_Unit_Bisector;

      function Compute_Control_Points (Start, Corner, Finish : Scaled_Position; Chord_Error_Max : Length) return Bezier
      is
         Cosine_Secondary_Angle : constant Dimensionless := Compute_Cosine_Secondary_Angle (Start, Corner, Finish);
         Base_Length : constant Length := Compute_Bezier_Base_Length (Start, Corner, Finish, Chord_Error_Max);
         Incoming_Unit          : constant Position_Scale         := (Start - Corner) / abs (Start - Corner);
         Outgoing_Unit          : constant Position_Scale         := (Finish - Corner) / abs (Finish - Corner);
         M                      : constant Scaled_Position_Offset :=
           ((Outgoing_Unit - Incoming_Unit) / abs (Outgoing_Unit - Incoming_Unit)) * Base_Length;
         Result                 : Bezier;
      begin
         Result (Result'First)     :=
           Corner + Incoming_Unit * ((4.0 + 889.0 / 429.0 + 1_225.0 / (858.0 * Cosine_Secondary_Angle)) * Base_Length);
         Result (Result'First + 1) := Result (Result'First) - Incoming_Unit * Base_Length;
         Result (Result'First + 2) := Result (Result'First + 1) - Incoming_Unit * Base_Length;
         Result (Result'First + 3) := Result (Result'First + 2) - Incoming_Unit * Base_Length;
         Result (Result'First + 4) := Result (Result'First + 3) - Incoming_Unit * Base_Length;
         Result (Result'First + 5) :=
           Result (Result'First + 4) + M * (10.0 / 143.0) - Incoming_Unit * ((133.0 / 143.0) * Base_Length);
         Result (Result'First + 6) :=
           Result (Result'First + 5) + M * (38.0 / 143.0) - Incoming_Unit * ((105.0 / 143.0) * Base_Length);
         Result (Result'First + 7) :=
           Result (Result'First + 6) + M * (254.0 / 429.0) - Incoming_Unit * ((175.0 / 429.0) * Base_Length);
         Result (Result'First + 8) := Result (Result'First + 7) + M;

         Result (Result'Last)     :=
           Corner + Outgoing_Unit * ((4.0 + 889.0 / 429.0 + 1_225.0 / (858.0 * Cosine_Secondary_Angle)) * Base_Length);
         Result (Result'Last - 1) := Result (Result'Last) - Outgoing_Unit * Base_Length;
         Result (Result'Last - 2) := Result (Result'Last - 1) - Outgoing_Unit * Base_Length;
         Result (Result'Last - 3) := Result (Result'Last - 2) - Outgoing_Unit * Base_Length;
         Result (Result'Last - 4) := Result (Result'Last - 3) - Outgoing_Unit * Base_Length;
         Result (Result'Last - 5) :=
           Result (Result'Last - 4) - M * (10.0 / 143.0) - Outgoing_Unit * ((133.0 / 143.0) * Base_Length);
         Result (Result'Last - 6) :=
           Result (Result'Last - 5) - M * (38.0 / 143.0) - Outgoing_Unit * ((105.0 / 143.0) * Base_Length);
         Result (Result'Last - 7) :=
           Result (Result'Last - 6) - M * (254.0 / 429.0) - Outgoing_Unit * ((175.0 / 429.0) * Base_Length);
         Result (Result'Last - 8) := Result (Result'Last - 7) - M;

         --  for R of Result loop
         --     Put_Line (R (X_Axis)'Image & "," & R (Y_Axis)'Image);
         --  end loop;
         --  Put_Line ("");

         return Result;
      end Compute_Control_Points;

      function Compute_Curve_Mid_Point
        (Start, Corner, Finish : Scaled_Position; Chord_Error_Max : Length) return Scaled_Position
      is
         Sine_Secondary_Angle   : constant Dimensionless  := Compute_Sine_Secondary_Angle (Start, Corner, Finish);
         Cosine_Secondary_Angle : constant Dimensionless  := Compute_Cosine_Secondary_Angle (Start, Corner, Finish);
         Base_Length : constant Length         := Compute_Bezier_Base_Length (Start, Corner, Finish, Chord_Error_Max);
         Bisector               : constant Position_Scale := Compute_Unit_Bisector (Start, Corner, Finish);
         Deviation              : constant Length         :=
           (Sine_Secondary_Angle / 2.0**14) * Base_Length *
           ((397.0 / 429.0) + 10_207.0 + (2.0**14 * 1_225.0) / (858.8 * Cosine_Secondary_Angle));
      begin
         return Corner + Bisector * Deviation;
      end Compute_Curve_Mid_Point;

      Last_Comp_Error : Length := 0.0 * mm;
   begin
      for I in Working.Corners'Range loop
         Working.Shifted_Corners (I) := Working.Corners (I);
      end loop;

      for I in Working.Shifted_Corner_Error_Limits'First + 1 .. Working.Shifted_Corner_Error_Limits'Last - 1 loop
         Working.Shifted_Corner_Error_Limits (I) :=
           Length'Min (Working.Segment_Limits (I).Chord_Error_Max, Working.Segment_Limits (I + 1).Chord_Error_Max);
      end loop;
      Working.Shifted_Corner_Error_Limits (Working.Shifted_Corner_Error_Limits'First) := 0.0 * mm;
      Working.Shifted_Corner_Error_Limits (Working.Shifted_Corner_Error_Limits'Last)  := 0.0 * mm;

      Working.Midpoints (Working.Midpoints'First) := Working.Corners (Working.Corners'First);
      Working.Midpoints (Working.Midpoints'Last)  := Working.Corners (Working.Corners'Last);

      if Corner_Blender_Do_Shifting then
         loop
            Last_Comp_Error := 0.0 * mm;

            for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
               declare
                  Start  : constant Scaled_Position := Working.Shifted_Corners (I - 1);
                  Corner : constant Scaled_Position := Working.Shifted_Corners (I);
                  Finish : constant Scaled_Position := Working.Shifted_Corners (I + 1);
               begin
                  if Sin (Corner_Blender_Max_Secondary_Angle_To_Blend) <
                    Compute_Sine_Secondary_Angle
                      (Working.Corners (I - 1), Working.Corners (I), Working.Corners (I + 1))
                  then
                     Working.Midpoints (I) := Working.Corners (I);
                  else
                     Working.Midpoints (I) :=
                       Compute_Curve_Mid_Point (Start, Corner, Finish, Working.Shifted_Corner_Error_Limits (I));
                     Last_Comp_Error       := Length'Max (@, abs (Working.Midpoints (I) - Working.Corners (I)));
                  end if;
               end;
            end loop;

            exit when Last_Comp_Error <= Corner_Blender_Max_Computational_Error;

            for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
               Working.Shifted_Corners (I) := @ + (Working.Corners (I) - Working.Midpoints (I));
            end loop;

            for I in Working.Corners'First + 1 .. Working.Corners'Last - 1 loop
               declare
                  Start  : constant Scaled_Position := Working.Shifted_Corners (I - 1);
                  Corner : constant Scaled_Position := Working.Shifted_Corners (I);
                  Finish : constant Scaled_Position := Working.Shifted_Corners (I + 1);
               begin
                  Working.Shifted_Corner_Error_Limits (I) :=
                    abs Dot
                      (Working.Corners (I) - Working.Shifted_Corners (I),
                       Compute_Unit_Bisector (Start, Corner, Finish));
               end;
            end loop;
         end loop;
      end if;

      for I in Working.Beziers'First + 1 .. Working.Beziers'Last - 1 loop
         declare
            Start   : constant Scaled_Position := Working.Shifted_Corners (I - 1);
            Corner  : constant Scaled_Position := Working.Shifted_Corners (I);
            Finish  : constant Scaled_Position := Working.Shifted_Corners (I + 1);
            Sin_Sec : constant Dimensionless   := Compute_Sine_Secondary_Angle (Start, Corner, Finish);
         begin
            if Sin (Corner_Blender_Max_Secondary_Angle_To_Blend) <
              Compute_Sine_Secondary_Angle (Working.Corners (I - 1), Working.Corners (I), Working.Corners (I + 1))
            then
               Working.Beziers (I)            := [others => Working.Corners (I)];
               Working.Inverse_Curvatures (I) := 0.0 * mm;
            else
               Working.Beziers (I)            :=
                 Compute_Control_Points (Start, Corner, Finish, Working.Shifted_Corner_Error_Limits (I));
               Working.Inverse_Curvatures (I) :=
                 (12.0 / 7.0) *
                 Compute_Bezier_Base_Length (Start, Corner, Finish, Working.Shifted_Corner_Error_Limits (I)) /
                 (Sin_Sec / (1.0 - Sin_Sec**2)**(1 / 2));
            end if;
         end;
      end loop;

      Working.Beziers (Working.Beziers'First) := [others => Working.Corners (Working.Beziers'First)];
      Working.Beziers (Working.Beziers'Last)  := [others => Working.Corners (Working.Beziers'Last)];

      -- Put_Line
      --   ("BASE = " &
      --    Length'Image
      --      (Compute_Bezier_Base_Length
      --         (Working.Shifted_Corners (5), Working.Shifted_Corners (6), Working.Shifted_Corners (7),
      --          Working.Shifted_Corner_Error_Limits (6))));
      -- Put_Line ("B4B5 = " & Length'Image (abs (Working.Beziers (6) (4) - Working.Beziers (6) (5))));
   end Corner_Blender;

   procedure Curve_Splitter is

      function Curve_Length (Curve : Curve_Point_Set_Values) return Length is
         Sum : Length := 0.0 * mm;
      begin
         for I in Curve'First .. Curve'Last - 1 loop
            Sum := Sum + abs (Curve (I) - Curve (I + 1));
         end loop;

         return Sum;
      end Curve_Length;

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

      function Compute_Bezier_Point_V2 (Bez : Bezier; T : Dimensionless) return Scaled_Position is
         Bez_2 : Bezier := Bez;
      begin
         return
           Bez (0) * ((1.0 - T)**15) + Scaled_Position_Offset (Bez (1)) * (15.0 * T * (1.0 - T)**14) +
           Scaled_Position_Offset (Bez (2)) * (105.0 * T**2 * (1.0 - T)**13) +
           Scaled_Position_Offset (Bez (3)) * (455.0 * T**3 * (1.0 - T)**12) +
           Scaled_Position_Offset (Bez (4)) * (1_365.0 * T**4 * (1.0 - T)**11) +
           Scaled_Position_Offset (Bez (5)) * (3_003.0 * T**5 * (1.0 - T)**10) +
           Scaled_Position_Offset (Bez (6)) * (5_005.0 * T**6 * (1.0 - T)**9) +
           Scaled_Position_Offset (Bez (7)) * (6_435.0 * T**7 * (1.0 - T)**8) +
           Scaled_Position_Offset (Bez (8)) * (6_435.0 * T**8 * (1.0 - T)**7) +
           Scaled_Position_Offset (Bez (9)) * (5_005.0 * T**9 * (1.0 - T)**6) +
           Scaled_Position_Offset (Bez (10)) * (3_003.0 * T**10 * (1.0 - T)**5) +
           Scaled_Position_Offset (Bez (11)) * (1_365.0 * T**11 * (1.0 - T)**4) +
           Scaled_Position_Offset (Bez (12)) * (455.0 * T**12 * (1.0 - T)**3) +
           Scaled_Position_Offset (Bez (13)) * (105.0 * T**13 * (1.0 - T)**2) +
           Scaled_Position_Offset (Bez (14)) * (15.0 * T**14 * (1.0 - T)) +
           Scaled_Position_Offset (Bez (15)) * (T**15);
      end Compute_Bezier_Point_V2;

      function Solve_Points_Per_Side (Bez : Bezier) return Curve_Point_Set_Index is
         Lower : Curve_Point_Set_Index := 10;
         Upper : Curve_Point_Set_Index := Curve_Point_Set_Index'Last;
         Mid   : Curve_Point_Set_Index;
      begin
         --  TODO: remove
         return 5_000;
         loop
            Mid := Lower + (Upper - Lower) / 2;
            exit when Lower = Upper;
            if Curve_Splitter_Target_Step <=
              abs (Compute_Bezier_Point (Bez, 0.0) - Compute_Bezier_Point (Bez, 0.5 / Dimensionless (Mid)))
            then
               Lower := Mid + 1;
            else
               Upper := Mid;
            end if;
         end loop;

         return Mid;
      end Solve_Points_Per_Side;

      function Compute_Inverse_Curvature (A, B, C : Scaled_Position) return Length is
         Side_AB  : Length := abs (A - B) / 2.0;
         Side_AC  : Length := abs (A - C) / 2.0;
         Side_BC  : Length := abs (B - C) / 2.0;
         Tri_Area : Area   :=
           0.25 *
           (Hypervolume'Max
                ((Side_AB + Side_AC + Side_BC) * (-Side_AB + Side_AC + Side_BC) * (Side_AB - Side_AC + Side_BC) *
                 (Side_AB + Side_AC - Side_BC),
                 0.0 * mm**4))**
             (1 / 2);
      begin
         if A = C then
            return 0.0 * mm;
         end if;

         if Tri_Area = 0.0 * mm**2 then
            if Side_AC > Side_AB then
               return Length'Last;
            else
               return 0.0 * mm;
            end if;
         else
            return (Side_AB * Side_AC * Side_BC) / (4.0 * Tri_Area);
         end if;
      end Compute_Inverse_Curvature;

   begin
      for I in Working.Curve_Point_Sets'Range loop
         --  declare
         --     Points_Per_Side : Curve_Point_Set_Index with
         --       Address => Working.Curve_Point_Sets (I).Points_Per_Side'Address;
         --  begin
         --     Points_Per_Side := Solve_Points_Per_Side (Working.Beziers (I));
         --  end;
         Working.Curve_Point_Sets (I) :=
           (Points_Per_Side => Solve_Points_Per_Side (Working.Beziers (I)), others => <>);

         for J in Working.Curve_Point_Sets (I).Incoming'Range loop
            Working.Curve_Point_Sets (I).Incoming (J) :=
              Compute_Bezier_Point
                (Working.Beziers (I),
                 Dimensionless (J) / Dimensionless (Working.Curve_Point_Sets (I).Points_Per_Side * 2));
         end loop;

         for J in Working.Curve_Point_Sets (I).Outgoing'Range loop
            Working.Curve_Point_Sets (I).Outgoing (J) :=
              Compute_Bezier_Point
                (Working.Beziers (I),
                 Dimensionless (J) / Dimensionless (Working.Curve_Point_Sets (I).Points_Per_Side * 2) + 0.5);
         end loop;

         --  These should be identical, but we could change the curve in the future.
         Working.Curve_Point_Sets (I).Incoming_Length := Curve_Length (Working.Curve_Point_Sets (I).Incoming);
         Working.Curve_Point_Sets (I).Outgoing_Length := Curve_Length (Working.Curve_Point_Sets (I).Outgoing);

         -- Working.Inverse_Curvatures (I) :=
         --   Compute_Inverse_Curvature
         --     (Working.Curve_Point_Sets (I).Incoming (Working.Curve_Point_Sets (I).Incoming'Last - 1),
         --      Working.Curve_Point_Sets (I).Incoming (Working.Curve_Point_Sets (I).Incoming'Last),
         --      Working.Curve_Point_Sets (I).Outgoing (Working.Curve_Point_Sets (I).Outgoing'First + 1));
      end loop;

      -- Put_Line ("LEN = " & Length'Image (Curve_Length (Working.Curve_Point_Sets (6).Incoming) * 2.0));
   end Curve_Splitter;

   procedure Kinematic_Limiter is

      function Optimal_Accel_For_Distance
        (Start_Vel        : Velocity;
         Distance         : Length;
         Acceleration_Max : Acceleration;
         Jerk_Max         : Jerk;
         Snap_Max         : Snap;
         Crackle_Max      : Crackle)
         return Feedrate_Profile_Times
      is
         D     : constant Length       := Distance;
         Vs    : constant Velocity     := Start_Vel;
         Am    : constant Acceleration := Acceleration_Max;
         Jm    : constant Jerk         := Jerk_Max;
         Sm    : constant Snap         := Snap_Max;
         Cm    : constant Crackle      := Crackle_Max;
         Cases : array (Feedrate_Profile_Times_Index) of Feedrate_Profile_Times;

         function Solve_Distance_At_Time
           (Profile : Feedrate_Profile_Times; Variable : Feedrate_Profile_Times_Index) return Feedrate_Profile_Times
         is
            Result : Feedrate_Profile_Times := Profile;

            Lower : Time := 0.0 * s;
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            --  accelerators.
            Upper : Time := 86_400.0 * s;

            type Casted_Time is mod 2**64;
            function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
            function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
         begin
            --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
            --  that.
            pragma Assert (Time'Size = 64);
            pragma Assert (Casted_Time'Size = 64);
            pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
            pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

            loop
               Result (Variable) := Cast_Time (Cast_Time (Lower) + (Cast_Time (Upper) - Cast_Time (Lower)) / 2);
               exit when Lower = Result (Variable) or Upper = Result (Variable);
               if Fast_Distance_At_Max_Time (Result, Cm, Vs) <= D then
                  Lower := Result (Variable);
               else
                  Upper := Result (Variable);
               end if;
            end loop;

            return Result;
         end Solve_Distance_At_Time;

      begin
         --!pp off
         if Sm**2 < Jm * Cm then
            if Am >= Jm * (Jm / Sm + Sm / Cm) then
               Cases :=
               [
                  --  Reachable: Sm, Jm, Am
                  4 => [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, 0.0 * s],
                  --  Reachable: Sm, Jm
                  3 => [Sm / Cm, Jm / Sm - Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Reachable: Sm
                  2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            elsif Am >= 2.0 * Sm**3 / Cm**2 then
               Cases :=
               [
                  --  Reachable: Sm, Am
                  4 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [Sm / Cm, (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm, 0.0 * s, 0.0 * s],
                  --  Reachable: Sm
                  2 => [Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            else
               Cases :=
               [
                  --  Reachable: Am
                  4 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            end if;
         else
            if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
               Cases :=
               [
                  --  Reachable: Jm, Am
                  4 => [(Jm / Cm)**(1 / 2), 0.0 * s, Am / Jm - 2.0 * (Jm / Cm)**(1 / 2), 0.0 * s],
                  --  Reachable: Jm
                  3 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(Jm / Cm)**(1 / 2), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            else
               Cases :=
               [
                  --  Reachable: Am
                  4 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  3 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Impossible case.
                  2 => [(Am / (2.0 * Cm))**(1 / 3), 0.0 * s, 0.0 * s, 0.0 * s],
                  --  Reachable: None
                  1 => [0.0 * s, 0.0 * s, 0.0 * s, 0.0 * s]
               ];
            end if;
         end if;
         --!pp on

         for I in reverse Cases'Range loop
            if I = Cases'First or D > Fast_Distance_At_Max_Time (Cases (I), Cm, Vs) then
               --  There are simple analytical solutions for a lot of these, but this is already fast so there is no
               --  reason to optimise it.
               return Solve_Distance_At_Time (Cases (I), I);
            end if;
         end loop;

         --  Unreachable.
         raise Program_Error;
      end Optimal_Accel_For_Distance;

   begin
      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'First) := 0.0 * mm / s;
      Working.Corner_Velocity_Limits (Working.Corner_Velocity_Limits'Last)  := 0.0 * mm / s;

      for I in Working.Corner_Velocity_Limits'First + 1 .. Working.Corner_Velocity_Limits'Last - 1 loop
         declare
            Limit           : Velocity;
            Optimal_Profile : Feedrate_Profile_Times;

            Inverse_Curvature : constant Length       := Working.Inverse_Curvatures (I);
            Velocity_Max      : constant Velocity     :=
              Velocity'Min (Working.Segment_Limits (I).Velocity_Max, Working.Segment_Limits (I + 1).Velocity_Max);
            Acceleration_Max  : constant Acceleration :=
              Acceleration'Min
                (Working.Segment_Limits (I).Acceleration_Max, Working.Segment_Limits (I + 1).Acceleration_Max);
            Jerk_Max          : constant Jerk         :=
              Jerk'Min (Working.Segment_Limits (I).Jerk_Max, Working.Segment_Limits (I + 1).Jerk_Max);
            Snap_Max          : constant Snap         :=
              Snap'Min (Working.Segment_Limits (I).Snap_Max, Working.Segment_Limits (I + 1).Snap_Max);
            Crackle_Max       : constant Crackle      :=
              Crackle'Min (Working.Segment_Limits (I).Crackle_Max, Working.Segment_Limits (I + 1).Crackle_Max);
         begin
            Limit := Velocity_Max;
            --  Inverse curvature range is 0..Length'Last, make sure to avoid overflow here.
            --  GCC with optimisation enabled may transform sqrt(x)*sqrt(y) to sqrt(x*y) etc., but that should be
            --  fine in optimised builds with Ada's checks disabled as the Velocity'Min call will immediately
            --  discard the resulting infinity.
            Limit := Velocity'Min (Limit, Acceleration_Max**(1 / 2) * Inverse_Curvature**(1 / 2));
            Limit := Velocity'Min (Limit, Jerk_Max**(1 / 3) * Inverse_Curvature**(2 / 3));
            Limit := Velocity'Min (Limit, Snap_Max**(1 / 4) * Inverse_Curvature**(3 / 4));
            Limit := Velocity'Min (Limit, Crackle_Max**(1 / 5) * Inverse_Curvature**(4 / 5));

            --  TODO: Add limit based on interpolation time.
            --  TODO: Snap and crackle limits currently do not match paper and are likely overly conservative.

            --  The 0.97 here ensures that no feedrate profiles end up with a very small accel/decel part which can
            --  lead to numerical errors that cause kinematic limits to be greatly exceeded for a single interpolation
            --  period. If this is removed, then the sanity check in Feedrate_Profile_Generator also needs to be
            --  removed.
            --  TODO: Check whether this actually matters in practice.
            Optimal_Profile :=
              Optimal_Accel_For_Distance
                (Working.Corner_Velocity_Limits (I - 1),
                 Curve_Corner_Distance (I - 1, I),
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Limit           :=
              Velocity'Min
                (Limit,
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile,
                    0.97 * Working.Segment_Limits (I).Crackle_Max,
                    Working.Corner_Velocity_Limits (I - 1)));

            Working.Corner_Velocity_Limits (I) := Limit;
         end;
      end loop;

      for I in reverse Working.Corner_Velocity_Limits'First + 1 .. Working.Corner_Velocity_Limits'Last - 1 loop
         declare
            Optimal_Profile : Feedrate_Profile_Times;
         begin
            Optimal_Profile                    :=
              Optimal_Accel_For_Distance
                (Working.Corner_Velocity_Limits (I + 1),
                 Curve_Corner_Distance (I, I + 1),
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Working.Corner_Velocity_Limits (I) :=
              Velocity'Min
                (Working.Corner_Velocity_Limits (I),
                 Fast_Velocity_At_Max_Time
                   (Optimal_Profile,
                    0.97 * Working.Segment_Limits (I).Crackle_Max,
                    Working.Corner_Velocity_Limits (I + 1)));
         end;
      end loop;
   end Kinematic_Limiter;

   procedure Feedrate_Profile_Generator is

      function Optimal_Accel_For_Delta_V
        (Delta_V          : Velocity;
         Acceleration_Max : Acceleration;
         Jerk_Max         : Jerk;
         Snap_Max         : Snap;
         Crackle_Max      : Crackle)
         return Feedrate_Profile_Times
      is
         Vd : constant Velocity     := abs Delta_V;
         Am : constant Acceleration := Acceleration_Max;
         Jm : constant Jerk         := Jerk_Max;
         Sm : constant Snap         := Snap_Max;
         Cm : constant Crackle      := Crackle_Max;

         function Solve_Velocity_At_Time
           (Profile  : Feedrate_Profile_Times;
            Variable : Feedrate_Profile_Times_Index;
            Target   : Velocity)
            return Feedrate_Profile_Times
         is
            Result : Feedrate_Profile_Times := Profile;

            Lower : Time := 0.0 * s;
            --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or
            --  a particle accelerator. It is not recommended to install Prunt on space probes or particle
            -- accelerators.
            Upper : Time := 86_400.0 * s;

            type Casted_Time is mod 2**64;
            function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
            function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
         begin
            --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
            --  that.
            pragma Assert (Time'Size = 64);
            pragma Assert (Casted_Time'Size = 64);
            pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
            pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

            loop
               Result (Variable) := Cast_Time (Cast_Time (Lower) + (Cast_Time (Upper) - Cast_Time (Lower)) / 2);
               exit when Lower = Result (Variable) or Upper = Result (Variable);
               if Fast_Velocity_At_Max_Time (Result, Cm, 0.0 * mm / s) <= Target then
                  Lower := Result (Variable);
               else
                  Upper := Result (Variable);
               end if;
            end loop;

            return Result;
         end Solve_Velocity_At_Time;

      begin
         --  This function is called a lot more than Optimal_Accel_For_Distance, so we use simple analytical solutions
         --  where they exist. In the one case where we resort to Solve_Velocity_At_Time, the analytical solution that
         --  Mathematica outputs involves a Cm**18, which is far outside the range of Dimensioned_Float for reasonable
         --  values of Cm.
         --
         --  For reference:
         --  ToRadicals[
         --    Solve[
         --      With[
         --        {T1 = Sm/Cm, T3 = 0, T4 = 0},
         --        v == Cm*T1*(T1 + T2)*(2*T1 + T2 + T3)*(4*T1 + 2*T2 + T3 + T4)
         --      ],
         --      T2,
         --      NonNegativeReals
         --    ]
         --  ]
         if Sm**2 < Jm * Cm then
            if Am >= Jm * (Jm / Sm + Sm / Cm) then
               if Vd > Am * (Am / Jm + Jm / Sm + Sm / Cm) then
                  --  Reachable: Sm, Jm, Am
                  return
                    [Sm / Cm, Jm / Sm - Sm / Cm, Am / Jm - Jm / Sm - Sm / Cm, Vd / Am - Am / Jm - Jm / Sm - Sm / Cm];
               elsif Vd > 2.0 * Jm * (Jm / Sm + Sm / Cm)**2 then
                  --  Reachable: Sm, Jm
                  return
                    [Sm / Cm,
                    Jm / Sm - Sm / Cm,
                    0.5 * ((Jm / Sm + Sm / Cm)**2 + 4.0 * Vd / Jm)**(1 / 2) - 1.5 * (Jm / Sm + Sm / Cm),
                    0.0 * s];
               elsif Vd > 8.0 * Sm**4 / Cm**3 then
                  --  Reachable: Sm
                  return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            elsif Am >= 2.0 * Sm**3 / Cm**2 then
               if Vd > Am * (2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) + Sm / Cm) then
                  --  Reachable: Sm, Am
                  return
                    [Sm / Cm,
                    (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2) - 1.5 * Sm / Cm,
                    0.0 * s,
                    Vd / Am - Sm / Cm - 2.0 * (0.25 * Sm**2 / Cm**2 + Am / Sm)**(1 / 2)];
               elsif Vd > 8.0 * Sm**4 / Cm**3 then
                  --  Reachable: Sm
                  return Solve_Velocity_At_Time ([Sm / Cm, 0.0 * s, 0.0 * s, 0.0 * s], 2, Vd);
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            else
               if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
                  --  Reachable: Am
                  return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            end if;
         else
            if Am > 2.0 * Jm * (Jm / Cm)**(1 / 2) then
               if Vd > Am * (Am / Jm + 2.0 * (Jm / Cm)**(1 / 2)) then
                  --  Reachable: Jm, Am
                  return
                    [(Jm / Cm)**(1 / 2),
                    0.0 * s,
                    Am / Jm - 2.0 * (Jm / Cm)**(1 / 2),
                    Vd / Am - Am / Jm - 2.0 * (Jm / Cm)**(1 / 2)];
               elsif Vd > 8.0 * Jm**2 / Cm then
                  --  Reachable: Jm
                  return
                    [(Jm / Cm)**(1 / 2), 0.0 * s, (Jm / Cm + Vd / Jm)**(1 / 2) - 3.0 * (Jm / Cm)**(1 / 2), 0.0 * s];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            else
               if Vd > 8.0 * Cm * (0.5 * Am / Cm)**(4 / 3) then
                  --  Reachable: Am
                  return [(0.5 * Am / Cm)**(1 / 3), 0.0 * s, 0.0 * s, Vd / Am - 4.0 * (0.5 * Am / Cm)**(1 / 3)];
               else
                  --  Reachable: None
                  return [(0.125 * Vd / Cm)**(1 / 4), 0.0 * s, 0.0 * s, 0.0 * s];
               end if;
            end if;
         end if;
      end Optimal_Accel_For_Delta_V;

   begin
      for I in Working.Feedrate_Profiles'Range loop
         declare
            Profile                : constant Feedrate_Profile_Times :=
              Optimal_Accel_For_Delta_V
                (Working.Corner_Velocity_Limits (I - 1) - Working.Corner_Velocity_Limits (I),
                 Working.Segment_Limits (I).Acceleration_Max,
                 Working.Segment_Limits (I).Jerk_Max,
                 Working.Segment_Limits (I).Snap_Max,
                 Working.Segment_Limits (I).Crackle_Max);
            Accel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time
                (Profile, Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Decel_Profile_Distance : constant Length                 :=
              Fast_Distance_At_Max_Time
                (Profile, -Working.Segment_Limits (I).Crackle_Max, Working.Corner_Velocity_Limits (I - 1));
            Curve_Distance         : constant Length                 := Curve_Corner_Distance (I - 1, I);
         begin
            pragma Assert (Curve_Distance < Length'Min (Accel_Profile_Distance, Decel_Profile_Distance));
         end;

         Working.Feedrate_Profiles (I).Accel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I - 1) - Working.Segment_Limits (I).Velocity_Max,
              Working.Segment_Limits (I).Acceleration_Max,
              Working.Segment_Limits (I).Jerk_Max,
              Working.Segment_Limits (I).Snap_Max,
              Working.Segment_Limits (I).Crackle_Max);
         Working.Feedrate_Profiles (I).Decel :=
           Optimal_Accel_For_Delta_V
             (Working.Corner_Velocity_Limits (I) - Working.Segment_Limits (I).Velocity_Max,
              Working.Segment_Limits (I).Acceleration_Max,
              Working.Segment_Limits (I).Jerk_Max,
              Working.Segment_Limits (I).Snap_Max,
              Working.Segment_Limits (I).Crackle_Max);

         declare
            Accel_Distance : Length          :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Accel,
                 Working.Segment_Limits (I).Crackle_Max,
                 Working.Corner_Velocity_Limits (I - 1));
            Coast_Velocity : Velocity        := Working.Segment_Limits (I).Velocity_Max;
            Decel_Distance : Length          :=
              Fast_Distance_At_Max_Time
                (Working.Feedrate_Profiles (I).Decel, -Working.Segment_Limits (I).Crackle_Max, Coast_Velocity);
            Curve_Distance : constant Length := Curve_Corner_Distance (I - 1, I);
         begin
            if Accel_Distance + Decel_Distance <= Curve_Distance then
               Working.Feedrate_Profiles (I).Coast :=
                 (Curve_Distance - Accel_Distance - Decel_Distance) / Coast_Velocity;
            else
               Working.Feedrate_Profiles (I).Coast := 0.0 * s;
               declare
                  type Casted_Vel is mod 2**64;
                  function Cast_Vel is new Ada.Unchecked_Conversion (Velocity, Casted_Vel);
                  function Cast_Vel is new Ada.Unchecked_Conversion (Casted_Vel, Velocity);
                  Upper : Velocity := Working.Segment_Limits (I).Velocity_Max;
                  -- Lower : Velocity := Working.Corner_Velocity_Limits (I);
                  Lower : Velocity :=
                    Velocity'Max (Working.Corner_Velocity_Limits (I - 1), Working.Corner_Velocity_Limits (I));
                  Mid   : Velocity;
               begin
                  --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check
                  --  for that.
                  pragma Assert (Velocity'Size = 64);
                  pragma Assert (Casted_Vel'Size = 64);
                  pragma Assert (Cast_Vel (86_400.0 * mm / s) = 4_680_673_776_000_565_248);
                  pragma Assert (Cast_Vel (0.123_45 * mm / s) = 4_593_559_930_647_147_132);

                  loop
                     Mid := Cast_Vel (Cast_Vel (Lower) + (Cast_Vel (Upper) - Cast_Vel (Lower)) / 2);
                     exit when Lower = Mid or Upper = Mid;

                     Working.Feedrate_Profiles (I).Accel :=
                       Optimal_Accel_For_Delta_V
                         (Working.Corner_Velocity_Limits (I - 1) - Mid,
                          Working.Segment_Limits (I).Acceleration_Max,
                          Working.Segment_Limits (I).Jerk_Max,
                          Working.Segment_Limits (I).Snap_Max,
                          Working.Segment_Limits (I).Crackle_Max);
                     Working.Feedrate_Profiles (I).Decel :=
                       Optimal_Accel_For_Delta_V
                         (Working.Corner_Velocity_Limits (I) - Mid,
                          Working.Segment_Limits (I).Acceleration_Max,
                          Working.Segment_Limits (I).Jerk_Max,
                          Working.Segment_Limits (I).Snap_Max,
                          Working.Segment_Limits (I).Crackle_Max);

                     Accel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Accel,
                          Working.Segment_Limits (I).Crackle_Max,
                          Working.Corner_Velocity_Limits (I - 1));
                     Decel_Distance :=
                       Fast_Distance_At_Max_Time
                         (Working.Feedrate_Profiles (I).Decel,
                          Working.Segment_Limits (I).Crackle_Max,
                          Working.Corner_Velocity_Limits (I));

                     if Accel_Distance + Decel_Distance <= Curve_Distance then
                        Lower := Mid;
                     else
                        Upper := Mid;
                     end if;
                  end loop;
               end;
            end if;
         end;
      end loop;
   end Feedrate_Profile_Generator;

   task body Runner is
   begin
      accept Init (Conf : Config_Parameters) do
         Config := Conf;
      end Init;
      PP_Last_Pos := Config.Initial_Position;

      loop
         Preprocessor;
         Corner_Blender;
         Curve_Splitter;
         Kinematic_Limiter;
         Feedrate_Profile_Generator;
         Execution_Block_Queue.Enqueue (Working);
      end loop;
   exception
      when E : others =>
         Put_Line ("Exception in Motion.Planner:");
         Put_Line (Ada.Exceptions.Exception_Information (E));
   end Runner;

end Motion.Planner;
