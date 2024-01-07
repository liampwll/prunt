with Ada.Unchecked_Conversion;
with Ada.Text_IO; use Ada.Text_IO;

package body Motion.Acceleration_Profile_Generator is

   task body Runner is
      Config : Config_Parameters;

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

      function Fast_Delta_V_At_Time (Profile : Acceleration_Profile_Times) return Velocity is
         T1 : constant Time    := Profile (1);
         T2 : constant Time    := Profile (2);
         T3 : constant Time    := Profile (3);
         T4 : constant Time    := Profile (4);
         Cm : constant Crackle := Config.Crackle_Limit;
      begin
         return Cm * T1 * (T1 + T2) * (2.0 * T1 + T2 + T3) * (4.0 * T1 + 2.0 * T2 + T3 + T4);
      end Fast_Delta_V_At_Time;

      function Solve_Velocity_At_Time
        (Profile  : Acceleration_Profile_Times;
         Variable : Acceleration_Profile_Times_Index;
         Target   : Velocity)
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
            if Fast_Delta_V_At_Time (Result) <= Target then
               Lower := Result (Variable);
            else
               Upper := Result (Variable);
            end if;
         end loop;

         return Result;
      end Solve_Velocity_At_Time;

      function Optimal_Accel_For_Delta_V (Delta_V : Velocity) return Acceleration_Profile_Times is
         Vd : constant Velocity     := abs Delta_V;
         Am : constant Acceleration := Config.Acceleration_Limit;
         Jm : constant Jerk         := Config.Jerk_Limit;
         Sm : constant Snap         := Config.Snap_Limit;
         Cm : constant Crackle      := Config.Crackle_Limit;
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

      procedure Processor (Data : in out Block_Data) is
      begin
         pragma Assert (Data.Last_Stage = Kinematic_Limiter_Stage);

         for I in Data.Segment_Acceleration_Profiles'Range loop
            -- declare
            --    Acc             : Acceleration_Profile_Times :=
            --      Optimal_Accel_For_Delta_V (Data.Corner_Velocity_Limits (I - 1) - Data.Corner_Velocity_Limits (I));
            --    Corner_Distance : constant Length            :=
            --      Curve_Corner_Distance (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I));
            -- begin
            --    null;
            --    Put (Length'Image(Fast_Distance_At_Time (Acc, Data.Corner_Velocity_Limits (I - 1))) & Length'Image (Corner_Distance));
            -- end;

            Data.Segment_Acceleration_Profiles (I).Accel :=
              Optimal_Accel_For_Delta_V (Data.Corner_Velocity_Limits (I - 1) - Data.Segment_Velocity_Limits (I));
            Data.Segment_Acceleration_Profiles (I).Decel :=
              Optimal_Accel_For_Delta_V (Data.Corner_Velocity_Limits (I) - Data.Segment_Velocity_Limits (I));
            declare
               Accel_Distance  : Length          :=
                 Fast_Distance_At_Time
                   (Data.Segment_Acceleration_Profiles (I).Accel, Data.Corner_Velocity_Limits (I - 1));
               Decel_Distance  : Length          :=
                 Fast_Distance_At_Time (Data.Segment_Acceleration_Profiles (I).Decel, Data.Corner_Velocity_Limits (I));
               Corner_Distance : constant Length :=
                 Curve_Corner_Distance (Data.Curve_Point_Sets (I - 1), Data.Curve_Point_Sets (I));
            begin
               if Accel_Distance + Decel_Distance <= Corner_Distance then
                  Data.Segment_Acceleration_Profiles (I).Coast :=
                    (Corner_Distance - Accel_Distance - Decel_Distance) / Data.Segment_Velocity_Limits (I);
               else
                  Data.Segment_Acceleration_Profiles (I).Coast := 0.0 * s;
                  declare
                     type Casted_Vel is mod 2**64;
                     function Cast_Vel is new Ada.Unchecked_Conversion (Velocity, Casted_Vel);
                     function Cast_Vel is new Ada.Unchecked_Conversion (Casted_Vel, Velocity);
                     Upper : Velocity := Data.Segment_Velocity_Limits (I);
                     Lower : Velocity := Data.Corner_Velocity_Limits (I);
                     Mid   : Velocity;
                  begin
                     --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to
                     --  check that here.
                     pragma Assert (Velocity'Size = 64);
                     pragma Assert (Casted_Vel'Size = 64);
                     pragma Assert (Cast_Vel (86_400.0 * mm / s) = 4_680_673_776_000_565_248);
                     pragma Assert (Cast_Vel (0.123_45 * mm / s) = 4_593_559_930_647_147_132);

                     loop
                        Mid := Cast_Vel ((Cast_Vel (Lower) + Cast_Vel (Upper)) / 2);
                        exit when Lower = Mid or Upper = Mid;

                        Data.Segment_Acceleration_Profiles (I).Accel :=
                          Optimal_Accel_For_Delta_V (Data.Corner_Velocity_Limits (I - 1) - Mid);
                        Data.Segment_Acceleration_Profiles (I).Decel :=
                          Optimal_Accel_For_Delta_V (Data.Corner_Velocity_Limits (I) - Mid);

                        Accel_Distance :=
                          Fast_Distance_At_Time
                            (Data.Segment_Acceleration_Profiles (I).Accel, Data.Corner_Velocity_Limits (I - 1));
                        Decel_Distance :=
                          Fast_Distance_At_Time
                            (Data.Segment_Acceleration_Profiles (I).Decel, Data.Corner_Velocity_Limits (I));

                        if Accel_Distance + Decel_Distance <= Corner_Distance then
                           Lower := Mid;
                        else
                           Upper := Mid;
                        end if;
                     end loop;
                     -- Put_Line
                     --   (Length'Image (Accel_Distance) & Length'Image (Decel_Distance) &
                     --    Length'Image ((Accel_Distance + Decel_Distance) / Corner_Distance) & Velocity'Image (Data.Corner_Velocity_Limits (I)));
                  end;
               end if;
            end;
         end loop;
      end Processor;

   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Wait (Acceleration_Profile_Generator_Stage);
            Block_Queue (Block_Index).Process (Acceleration_Profile_Generator_Stage, Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Acceleration_Profile_Generator;
