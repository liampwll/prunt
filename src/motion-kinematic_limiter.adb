with Ada.Unchecked_Conversion;

package body Motion.Kinematic_Limiter is

   task body Runner is
      Config : Config_Parameters;

      --  This is just a simplified version of Distance_At_Time where T = T1.
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

      function Solve_Distance_At_Time
        (Profile   : Acceleration_Profile_Times;
         Start_Vel : Velocity;
         Variable  : Acceleration_Profile_Times_Index;
         Target    : Length)
         return Acceleration_Profile_Times
      is
         Result : Acceleration_Profile_Times := Profile;

         Lower : Time := 0.0 * s;
         --  A maximum of 24 hours should be more than enough unless you are using Prunt to control a space probe or a
         --  particle accelerator. It is not recommended to install Prunt on space probes or particle accelerators.
         Upper : Time := 86_400.0 * s;

         type Casted_Time is mod 2**64;
         function Cast_Time is new Ada.Unchecked_Conversion (Time, Casted_Time);
         function Cast_Time is new Ada.Unchecked_Conversion (Casted_Time, Time);
      begin
         --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check that here.
         pragma Assert (Time'Size = 64);
         pragma Assert (Casted_Time'Size = 64);
         pragma Assert (Cast_Time (86_400.0 * s) = 4_680_673_776_000_565_248);
         pragma Assert (Cast_Time (0.123_45 * s) = 4_593_559_930_647_147_132);

         loop
            Result (Variable) := Cast_Time ((Cast_Time (Lower) + Cast_Time (Upper)) / 2);
            exit when Lower = Result (Variable) or Upper = Result (Variable);
            if Fast_Distance_At_Time (Result, Start_Vel) <= Target then
               Lower := Result (Variable);
            else
               Upper := Result (Variable);
            end if;
         end loop;

         return Result;
      end Solve_Distance_At_Time;

      --!pp off
      function Optimal_Accel_For_Distance (Start_Vel : Velocity; Distance : Length) return Acceleration_Profile_Times
      is
         D     : constant Length       := Distance;
         Vs    : constant Velocity     := Start_Vel;
         Am    : constant Acceleration := Config.Acceleration_Limit;
         Jm    : constant Jerk         := Config.Jerk_Limit;
         Sm    : constant Snap         := Config.Snap_Limit;
         Cm    : constant Crackle      := Config.Crackle_Limit;
         Cases : array (Acceleration_Profile_Times_Index) of Acceleration_Profile_Times;
      begin
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

         for I in reverse Cases'Range loop
            if I = Cases'First or
              D > Distance_At_Time (Cases (I) (Acceleration_Profile_Times_Index'First), Cases (I), Cm, Vs)
            then
               --  There are simple analytical solutions for a lot of these, but this is already fast so there is no
               --  reason to optimise it.
               return Solve_Distance_At_Time (Cases (I), Vs, I, D);
            end if;
         end loop;

         --  Unreachable.
         raise Program_Error;
      end Optimal_Accel_For_Distance;
      --!pp on

      procedure Processor (Data : in out Block_Data) is
      begin
         pragma Assert (Data.Last_Stage = Curve_Splitter_Stage);

         Data.Corner_Velocity_Limits (Data.Corner_Velocity_Limits'First) := 0.0 * mm / s;
         Data.Corner_Velocity_Limits (Data.Corner_Velocity_Limits'Last)  := 0.0 * mm / s;

         for I in Data.Curve_Point_Sets'First + 1 .. Data.Curve_Point_Sets'Last - 1 loop
            declare
               Limit           : Velocity := Config.Velocity_Limit;
               Optimal_Profile : Acceleration_Profile_Times;
            begin
               Limit := Velocity'Min (Limit, Data.Segment_Velocity_Limits (I));
               Limit := Velocity'Min (Limit, Data.Segment_Velocity_Limits (I + 1));
               --  Inverse curvature range is 0..Length'Last, make sure to avoid overflow here.
               --  GCC with optimisation enabled may transform sqrt(x)*sqrt(y) to sqrt(x*y) etc., but that should be
               --  fine in optimised builds with Ada's checks disabled as the Velocity'Min call will immediately
               --  discard the resulting infinity.
               Limit :=
                 Velocity'Min (Limit, Config.Acceleration_Limit**(1 / 2) * Data.Inverse_Curvatures (I)**(1 / 2));
               Limit := Velocity'Min (Limit, Config.Jerk_Limit**(1 / 3) * Data.Inverse_Curvatures (I)**(2 / 3));
               Limit := Velocity'Min (Limit, Config.Snap_Limit**(1 / 4) * Data.Inverse_Curvatures (I)**(3 / 4));
               Limit := Velocity'Min (Limit, Config.Crackle_Limit**(1 / 5) * Data.Inverse_Curvatures (I)**(4 / 5));

               --  TODO: Add limit based on interpolation time.
               --  TODO: Snap and crackle limits currently do not match paper and are likely overly conservative.

               --  TODO: Is a small error margin needed here and on the reverse pass to ensure the full acceleration
               --  profile generator will not fail?
               Optimal_Profile :=
                 Optimal_Accel_For_Distance
                   (Data.Corner_Velocity_Limits (I - 1),
                    Curve_Corner_Distance (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I)));
               Limit           :=
                 Velocity'Min
                   (Limit,
                    Velocity_At_Time
                      (Optimal_Profile (Acceleration_Profile_Times'First),
                       Optimal_Profile,
                       Config.Crackle_Limit,
                       Data.Corner_Velocity_Limits (I - 1)));

               Data.Corner_Velocity_Limits (I) := Limit;
            end;
         end loop;

         for I in reverse Data.Curve_Point_Sets'First + 1 .. Data.Curve_Point_Sets'Last - 1 loop
            declare
               Optimal_Profile : Acceleration_Profile_Times;
            begin
               Optimal_Profile                 :=
                 Optimal_Accel_For_Distance
                   (Data.Corner_Velocity_Limits (I + 1),
                    Curve_Corner_Distance (Data.Curve_Point_Sets (I), Data.Curve_Point_Sets (I + 1)));
               Data.Corner_Velocity_Limits (I) :=
                 Velocity'Min
                   (Data.Corner_Velocity_Limits (I),
                    Velocity_At_Time
                      (Optimal_Profile (Acceleration_Profile_Times'First),
                       Optimal_Profile,
                       Config.Crackle_Limit,
                       Data.Corner_Velocity_Limits (I - 1)));
            end;
         end loop;
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Wait (Kinematic_Limiter_Stage);
            Block_Queue (Block_Index).Process (Kinematic_Limiter_Stage, Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Kinematic_Limiter;
