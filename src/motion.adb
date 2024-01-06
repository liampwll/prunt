with Motion.Preprocessor;
with Motion.Curvifier;
with Motion.Curve_Splitter;
with Motion.Kinematic_Limiter;
with Motion.Acceleration_Profile_Generator;
with Motion.Logger;
with Ada.Exceptions;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Task_Identification;

package body Motion is

   procedure Init (Config : Config_Parameters) is
   begin
      Motion.Preprocessor.Runner.Init (Config);
      Motion.Curvifier.Runner.Init (Config);
      Motion.Curve_Splitter.Runner.Init (Config);
      Motion.Kinematic_Limiter.Runner.Init (Config);
      Motion.Acceleration_Profile_Generator.Runner.Init (Config);
      Motion.Logger.Runner.Init (Config);
   end Init;

   procedure Enqueue (Pos : Position; Velocity_Limit : Velocity) is
   begin
      Motion.Preprocessor.Runner.Enqueue (Pos, Velocity_Limit);
   end Enqueue;

   procedure Flush (Next_Master : Master_Manager.Master) is
   begin
      Motion.Preprocessor.Runner.Flush (Next_Master);
   end Flush;

   protected body Block is

      entry Process
        (for Stage in Preprocessor_Stage .. Logger_Stage) (Processor : access procedure (Data : in out Block_Data))
        when Block_Pipeline_Stages'Pred (Stage) = Data.Last_Stage
      is
      begin
         Processor (Data);
         Data.Last_Stage := Stage;
      exception
         when E : others =>
            Put_Line ("Exception in motion pipeline:");
            Put_Line (Ada.Exceptions.Exception_Information (E));
            Put_Line ("Terminating task " & Ada.Task_Identification.Image (Process'Caller));
            Ada.Task_Identification.Abort_Task (Process'Caller);
      end Process;

   end Block;

   function Compute_Bezier_Point (Bez : Bezier; T : Dimensionless) return Scaled_Position is
      type Partial_Bezier is array (Bezier_Index range <>) of Scaled_Position;

      function Recur (Bez : Partial_Bezier) return Scaled_Position is
         Next_Bez : constant Partial_Bezier :=
           [for I in Bez'First .. Bez'Last - 1 => Bez (I) + (Bez (I + 1) - Bez (I)) * T];
      begin
         if Next_Bez'Length = 1 then
            return Next_Bez (Bezier_Index'First);
         else
            return Recur (Next_Bez);
         end if;
      end Recur;

   begin
      return Recur ([for I in Bezier_Index => Bez (I)]);
   end Compute_Bezier_Point;

   function Curve_Corner_Distance (Start, Finish : Curve_Point_Set) return Length is
      Sum : Length := abs (Start (Start'Last) - Finish (Finish'First));
   begin
      for I in 0 .. Start'Last - 1 loop
         Sum := Sum + abs (Start (I) - Start (I + 1));
      end loop;

      for I in Finish'First + 1 .. 0 loop
         Sum := Sum + abs (Finish (I - 1) - Finish (I));
      end loop;

      return Sum;
   end Curve_Corner_Distance;

   type Acceleration_Profile_Stage_Index is range 1 .. 15;

   --  The recursive nature of these functions is significantly more inefficient than directly computing the value at
   --  each stage. However, this method avoids any discontinuity due to floating point errors. Whether this matters in
   --  practice has not been analysed.

   function Crackle_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Crackle is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;
   begin
      if T < T1 then
         return Cm;
      elsif T < T1 + T2 then
         return 0.0 * mm / s**5;
      elsif T < 2.0 * T1 + T2 then
         return -Cm;
      elsif T < 2.0 * T1 + T2 + T3 then
         return 0.0 * mm / s**5;
      elsif T < 3.0 * T1 + T2 + T3 then
         return -Cm;
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return 0.0 * mm / s**5;
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Cm;
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return 0.0 * mm / s**5;
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return -Cm;
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return 0.0 * mm / s**5;
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Cm;
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return 0.0 * mm / s**5;
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Cm;
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return 0.0 * mm / s**5;
      else
         return -Cm;
      end if;
   end Crackle_At_Time;

   function Snap_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Snap is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;

      function Snap_At_Stage (DT : Time; Stage : Acceleration_Profile_Stage_Index) return Snap is
      begin
         case Stage is
            when 1 =>
               return Cm * DT;
            when 2 =>
               return Snap_At_Stage (T1, 1);
            when 3 =>
               return Snap_At_Stage (T1 + T2, 2) - Cm * DT;
            when 4 =>
               return Snap_At_Stage (2.0 * T1 + T2, 3);
            when 5 =>
               return Snap_At_Stage (2.0 * T1 + T2 + T3, 4) - Cm * DT;
            when 6 =>
               return Snap_At_Stage (3.0 * T1 + T2 + T3, 5);
            when 7 =>
               return Snap_At_Stage (3.0 * T1 + 2.0 * T2 + T3, 6) + Cm * DT;
            when 8 =>
               return Snap_At_Stage (4.0 * T1 + 2.0 * T2 + T3, 7);
            when 9 =>
               return Snap_At_Stage (4.0 * T1 + 2.0 * T2 + T3 + T4, 8) - Cm * DT;
            when 10 =>
               return Snap_At_Stage (5.0 * T1 + 2.0 * T2 + T3 + T4, 9);
            when 11 =>
               return Snap_At_Stage (5.0 * T1 + 3.0 * T2 + T3 + T4, 10) + Cm * DT;
            when 12 =>
               return Snap_At_Stage (6.0 * T1 + 3.0 * T2 + T3 + T4, 11);
            when 13 =>
               return Snap_At_Stage (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 12) + Cm * DT;
            when 14 =>
               return Snap_At_Stage (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 13);
            when 15 =>
               return Snap_At_Stage (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4, 14) - Cm * DT;
         end case;
      end Snap_At_Stage;

   begin
      if T < T1 then
         return Snap_At_Stage (T, 1);
      elsif T < T1 + T2 then
         return Snap_At_Stage (T - (T1), 2);
      elsif T < 2.0 * T1 + T2 then
         return Snap_At_Stage (T - (T1 + T2), 3);
      elsif T < 2.0 * T1 + T2 + T3 then
         return Snap_At_Stage (T - (2.0 * T1 + T2), 4);
      elsif T < 3.0 * T1 + T2 + T3 then
         return Snap_At_Stage (T - (2.0 * T1 + T2 + T3), 5);
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return Snap_At_Stage (T - (3.0 * T1 + T2 + T3), 6);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Snap_At_Stage (T - (3.0 * T1 + 2.0 * T2 + T3), 7);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Snap_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3), 8);
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Snap_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3 + T4), 9);
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Snap_At_Stage (T - (5.0 * T1 + 2.0 * T2 + T3 + T4), 10);
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Snap_At_Stage (T - (5.0 * T1 + 3.0 * T2 + T3 + T4), 11);
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Snap_At_Stage (T - (6.0 * T1 + 3.0 * T2 + T3 + T4), 12);
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Snap_At_Stage (T - (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 13);
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return Snap_At_Stage (T - (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 14);
      else
         return Snap_At_Stage (T - (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4), 15);
      end if;
   end Snap_At_Time;

   function Jerk_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Jerk is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;

      function Jerk_At_Stage (DT : Time; Stage : Acceleration_Profile_Stage_Index) return Jerk is
      begin
         case Stage is
            when 1 =>
               return Cm * DT**2 / 2.0;
            when 2 =>
               return Jerk_At_Stage (T1, 1) + Cm * DT * T1;
            when 3 =>
               return Jerk_At_Stage (T1 + T2, 2) + Cm * DT * (-DT + 2.0 * T1) / 2.0;
            when 4 =>
               return Jerk_At_Stage (2.0 * T1 + T2, 3);
            when 5 =>
               return Jerk_At_Stage (2.0 * T1 + T2 + T3, 4) - Cm * DT**2 / 2.0;
            when 6 =>
               return Jerk_At_Stage (3.0 * T1 + T2 + T3, 5) - Cm * DT * T1;
            when 7 =>
               return Jerk_At_Stage (3.0 * T1 + 2.0 * T2 + T3, 6) + Cm * DT * (DT - 2.0 * T1) / 2.0;
            when 8 =>
               return Jerk_At_Stage (4.0 * T1 + 2.0 * T2 + T3, 7);
            when 9 =>
               return Jerk_At_Stage (4.0 * T1 + 2.0 * T2 + T3 + T4, 8) - Cm * DT**2 / 2.0;
            when 10 =>
               return Jerk_At_Stage (5.0 * T1 + 2.0 * T2 + T3 + T4, 9) - Cm * DT * T1;
            when 11 =>
               return Jerk_At_Stage (5.0 * T1 + 3.0 * T2 + T3 + T4, 10) + Cm * DT * (DT - 2.0 * T1) / 2.0;
            when 12 =>
               return Jerk_At_Stage (6.0 * T1 + 3.0 * T2 + T3 + T4, 11);
            when 13 =>
               return Jerk_At_Stage (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 12) + Cm * DT**2 / 2.0;
            when 14 =>
               return Jerk_At_Stage (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 13) + Cm * DT * T1;
            when 15 =>
               return Jerk_At_Stage (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4, 14) + Cm * DT * (-DT + 2.0 * T1) / 2.0;
         end case;
      end Jerk_At_Stage;

   begin
      if T < T1 then
         return Jerk_At_Stage (T, 1);
      elsif T < T1 + T2 then
         return Jerk_At_Stage (T - (T1), 2);
      elsif T < 2.0 * T1 + T2 then
         return Jerk_At_Stage (T - (T1 + T2), 3);
      elsif T < 2.0 * T1 + T2 + T3 then
         return Jerk_At_Stage (T - (2.0 * T1 + T2), 4);
      elsif T < 3.0 * T1 + T2 + T3 then
         return Jerk_At_Stage (T - (2.0 * T1 + T2 + T3), 5);
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return Jerk_At_Stage (T - (3.0 * T1 + T2 + T3), 6);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Jerk_At_Stage (T - (3.0 * T1 + 2.0 * T2 + T3), 7);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Jerk_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3), 8);
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Jerk_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3 + T4), 9);
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Jerk_At_Stage (T - (5.0 * T1 + 2.0 * T2 + T3 + T4), 10);
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Jerk_At_Stage (T - (5.0 * T1 + 3.0 * T2 + T3 + T4), 11);
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Jerk_At_Stage (T - (6.0 * T1 + 3.0 * T2 + T3 + T4), 12);
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Jerk_At_Stage (T - (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 13);
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return Jerk_At_Stage (T - (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 14);
      else
         return Jerk_At_Stage (T - (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4), 15);
      end if;
   end Jerk_At_Time;

   function Acceleration_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Acceleration
   is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;

      function Acceleration_At_Stage (DT : Time; Stage : Acceleration_Profile_Stage_Index) return Acceleration is
      begin
         case Stage is
            when 1 =>
               return Cm * DT**3 / 6.0;
            when 2 =>
               return Acceleration_At_Stage (T1, 1) + Cm * DT * T1 * (DT + T1) / 2.0;
            when 3 =>
               return
                 Acceleration_At_Stage (T1 + T2, 2) +
                 Cm * DT * (-DT**2 + 3.0 * DT * T1 + 3.0 * T1 * (T1 + 2.0 * T2)) / 6.0;
            when 4 =>
               return Acceleration_At_Stage (2.0 * T1 + T2, 3) + Cm * DT * T1 * (T1 + T2);
            when 5 =>
               return Acceleration_At_Stage (2.0 * T1 + T2 + T3, 4) + Cm * DT * (-DT**2 + 6.0 * T1 * (T1 + T2)) / 6.0;
            when 6 =>
               return Acceleration_At_Stage (3.0 * T1 + T2 + T3, 5) + Cm * DT * T1 * (-DT + T1 + 2.0 * T2) / 2.0;
            when 7 =>
               return
                 Acceleration_At_Stage (3.0 * T1 + 2.0 * T2 + T3, 6) +
                 Cm * DT * (DT**2 - 3.0 * DT * T1 + 3.0 * T1**2) / 6.0;
            when 8 =>
               return Acceleration_At_Stage (4.0 * T1 + 2.0 * T2 + T3, 7);
            when 9 =>
               return Acceleration_At_Stage (4.0 * T1 + 2.0 * T2 + T3 + T4, 8) - Cm * DT**3 / 6.0;
            when 10 =>
               return Acceleration_At_Stage (5.0 * T1 + 2.0 * T2 + T3 + T4, 9) + Cm * DT * T1 * (-DT - T1) / 2.0;
            when 11 =>
               return
                 Acceleration_At_Stage (5.0 * T1 + 3.0 * T2 + T3 + T4, 10) +
                 Cm * DT * (DT**2 - 3.0 * DT * T1 - 3.0 * T1 * (T1 + 2.0 * T2)) / 6.0;
            when 12 =>
               return Acceleration_At_Stage (6.0 * T1 + 3.0 * T2 + T3 + T4, 11) - Cm * DT * T1 * (T1 + T2);
            when 13 =>
               return
                 Acceleration_At_Stage (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 12) +
                 Cm * DT * (DT**2 - 6.0 * T1 * (T1 + T2)) / 6.0;
            when 14 =>
               return
                 Acceleration_At_Stage (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 13) +
                 Cm * DT * T1 * (DT - T1 - 2.0 * T2) / 2.0;
            when 15 =>
               return
                 Acceleration_At_Stage (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4, 14) +
                 Cm * DT * (-DT**2 + 3.0 * DT * T1 - 3.0 * T1**2) / 6.0;
         end case;
      end Acceleration_At_Stage;

   begin
      if T < T1 then
         return Acceleration_At_Stage (T, 1);
      elsif T < T1 + T2 then
         return Acceleration_At_Stage (T - (T1), 2);
      elsif T < 2.0 * T1 + T2 then
         return Acceleration_At_Stage (T - (T1 + T2), 3);
      elsif T < 2.0 * T1 + T2 + T3 then
         return Acceleration_At_Stage (T - (2.0 * T1 + T2), 4);
      elsif T < 3.0 * T1 + T2 + T3 then
         return Acceleration_At_Stage (T - (2.0 * T1 + T2 + T3), 5);
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return Acceleration_At_Stage (T - (3.0 * T1 + T2 + T3), 6);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Acceleration_At_Stage (T - (3.0 * T1 + 2.0 * T2 + T3), 7);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Acceleration_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3), 8);
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Acceleration_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3 + T4), 9);
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Acceleration_At_Stage (T - (5.0 * T1 + 2.0 * T2 + T3 + T4), 10);
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Acceleration_At_Stage (T - (5.0 * T1 + 3.0 * T2 + T3 + T4), 11);
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Acceleration_At_Stage (T - (6.0 * T1 + 3.0 * T2 + T3 + T4), 12);
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Acceleration_At_Stage (T - (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 13);
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return Acceleration_At_Stage (T - (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 14);
      else
         return Acceleration_At_Stage (T - (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4), 15);
      end if;
   end Acceleration_At_Time;

   function Velocity_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle; Start_Vel : Velocity) return Velocity
   is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;

      function Velocity_At_Stage (DT : Time; Stage : Acceleration_Profile_Stage_Index) return Velocity is
      begin
         case Stage is
            when 1 =>
               return Start_Vel + Cm * DT**4 / 24.0;
            when 2 =>
               return Velocity_At_Stage (T1, 1) + Cm * DT * T1 * (2.0 * DT**2 + 3.0 * DT * T1 + 2.0 * T1**2) / 12.0;
            when 3 =>
               return
                 Velocity_At_Stage (T1 + T2, 2) +
                 Cm * DT *
                   (-DT**3 + 4.0 * DT**2 * T1 + 6.0 * DT * T1 * (T1 + 2.0 * T2) +
                    4.0 * T1 * (T1**2 + 3.0 * T1 * T2 + 3.0 * T2**2)) /
                   24.0;
            when 4 =>
               return
                 Velocity_At_Stage (2.0 * T1 + T2, 3) +
                 Cm * DT * T1 * (DT * (T1 + T2) + 2.0 * T1**2 + 3.0 * T1 * T2 + T2**2) / 2.0;
            when 5 =>
               return
                 Velocity_At_Stage (2.0 * T1 + T2 + T3, 4) +
                 Cm * DT *
                   (-DT**3 + 12.0 * DT * T1 * (T1 + T2) +
                    12.0 * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + 2.0 * T1 * T3 + T2**2 + 2.0 * T2 * T3)) /
                   24.0;
            when 6 =>
               return
                 Velocity_At_Stage (3.0 * T1 + T2 + T3, 5) +
                 Cm * DT * T1 *
                   (-2.0 * DT**2 + 3.0 * DT * (T1 + 2.0 * T2) + 22.0 * T1**2 + 30.0 * T1 * T2 + 12.0 * T1 * T3 +
                    6.0 * T2**2 + 12.0 * T2 * T3) /
                   12.0;
            when 7 =>
               return
                 Velocity_At_Stage (3.0 * T1 + 2.0 * T2 + T3, 6) +
                 Cm * DT *
                   (DT**3 - 4.0 * DT**2 * T1 + 6.0 * DT * T1**2 +
                    4.0 * T1 * (11.0 * T1**2 + 18.0 * T1 * T2 + 6.0 * T1 * T3 + 6.0 * T2**2 + 6.0 * T2 * T3)) /
                   24.0;
            when 8 =>
               return
                 Velocity_At_Stage (4.0 * T1 + 2.0 * T2 + T3, 7) +
                 Cm * DT * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + T1 * T3 + T2**2 + T2 * T3);
            when 9 =>
               return
                 Velocity_At_Stage (4.0 * T1 + 2.0 * T2 + T3 + T4, 8) +
                 Cm * DT * (-DT**3 + 24.0 * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + T1 * T3 + T2**2 + T2 * T3)) / 24.0;
            when 10 =>
               return
                 Velocity_At_Stage (5.0 * T1 + 2.0 * T2 + T3 + T4, 9) +
                 Cm * DT * T1 *
                   (-2.0 * DT**2 - 3.0 * DT * T1 + 22.0 * T1**2 + 36.0 * T1 * T2 + 12.0 * T1 * T3 + 12.0 * T2**2 +
                    12.0 * T2 * T3) /
                   12.0;
            when 11 =>
               return
                 Velocity_At_Stage (5.0 * T1 + 3.0 * T2 + T3 + T4, 10) +
                 Cm * DT *
                   (DT**3 - 4.0 * DT**2 * T1 - 6.0 * DT * T1 * (T1 + 2.0 * T2) +
                    4.0 * T1 * (11.0 * T1**2 + 15.0 * T1 * T2 + 6.0 * T1 * T3 + 3.0 * T2**2 + 6.0 * T2 * T3)) /
                   24.0;
            when 12 =>
               return
                 Velocity_At_Stage (6.0 * T1 + 3.0 * T2 + T3 + T4, 11) +
                 Cm * DT * T1 *
                   (-DT * (T1 + T2) + 2.0 * T1**2 + 3.0 * T1 * T2 + 2.0 * T1 * T3 + T2**2 + 2.0 * T2 * T3) / 2.0;
            when 13 =>
               return
                 Velocity_At_Stage (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 12) +
                 Cm * DT * (DT**3 - 12.0 * DT * T1 * (T1 + T2) + 12.0 * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + T2**2)) /
                   24.0;
            when 14 =>
               return
                 Velocity_At_Stage (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 13) +
                 Cm * DT * T1 *
                   (2.0 * DT**2 - 3.0 * DT * (T1 + 2.0 * T2) + 2.0 * T1**2 + 6.0 * T1 * T2 + 6.0 * T2**2) / 12.0;
            when 15 =>
               return
                 Velocity_At_Stage (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4, 14) +
                 Cm * DT * (-DT**3 + 4.0 * DT**2 * T1 - 6.0 * DT * T1**2 + 4.0 * T1**3) / 24.0;
         end case;
      end Velocity_At_Stage;

   begin
      if T < T1 then
         return Velocity_At_Stage (T, 1);
      elsif T < T1 + T2 then
         return Velocity_At_Stage (T - (T1), 2);
      elsif T < 2.0 * T1 + T2 then
         return Velocity_At_Stage (T - (T1 + T2), 3);
      elsif T < 2.0 * T1 + T2 + T3 then
         return Velocity_At_Stage (T - (2.0 * T1 + T2), 4);
      elsif T < 3.0 * T1 + T2 + T3 then
         return Velocity_At_Stage (T - (2.0 * T1 + T2 + T3), 5);
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return Velocity_At_Stage (T - (3.0 * T1 + T2 + T3), 6);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Velocity_At_Stage (T - (3.0 * T1 + 2.0 * T2 + T3), 7);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Velocity_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3), 8);
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Velocity_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3 + T4), 9);
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Velocity_At_Stage (T - (5.0 * T1 + 2.0 * T2 + T3 + T4), 10);
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Velocity_At_Stage (T - (5.0 * T1 + 3.0 * T2 + T3 + T4), 11);
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Velocity_At_Stage (T - (6.0 * T1 + 3.0 * T2 + T3 + T4), 12);
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Velocity_At_Stage (T - (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 13);
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return Velocity_At_Stage (T - (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 14);
      else
         return Velocity_At_Stage (T - (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4), 15);
      end if;
   end Velocity_At_Time;

   function Distance_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle; Start_Vel : Velocity) return Length
   is
      T1 : constant Time    := Profile (1);
      T2 : constant Time    := Profile (2);
      T3 : constant Time    := Profile (3);
      T4 : constant Time    := Profile (4);
      Cm : constant Crackle := Crackle_Limit;

      function Distance_At_Stage (DT : Time; Stage : Acceleration_Profile_Stage_Index) return Length is
      begin
         case Stage is
            when 1 =>
               return Start_Vel * T + Cm * DT**5 / 120.0;
            when 2 =>
               return
                 Distance_At_Stage (T1, 1) +
                 Cm * DT * T1 * (DT**3 + 2.0 * DT**2 * T1 + 2.0 * DT * T1**2 + T1**3) / 24.0;
            when 3 =>
               return
                 Distance_At_Stage (T1 + T2, 2) +
                 Cm * DT *
                   (-DT**4 + 5.0 * DT**3 * T1 + 10.0 * DT**2 * T1 * (T1 + 2.0 * T2) +
                    10.0 * DT * T1 * (T1**2 + 3.0 * T1 * T2 + 3.0 * T2**2) +
                    5.0 * T1 * (T1**3 + 4.0 * T1**2 * T2 + 6.0 * T1 * T2**2 + 4.0 * T2**3)) /
                   120.0;
            when 4 =>
               return
                 Distance_At_Stage (2.0 * T1 + T2, 3) +
                 Cm * DT * T1 *
                   (2.0 * DT**2 * (T1 + T2) + 3.0 * DT * (2.0 * T1**2 + 3.0 * T1 * T2 + T2**2) + 7.0 * T1**3 +
                    14.0 * T1**2 * T2 + 9.0 * T1 * T2**2 + 2.0 * T2**3) /
                   12.0;
            when 5 =>
               return
                 Distance_At_Stage (2.0 * T1 + T2 + T3, 4) +
                 Cm * DT *
                   (-DT**4 + 20.0 * DT**2 * T1 * (T1 + T2) +
                    30.0 * DT * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + 2.0 * T1 * T3 + T2**2 + 2.0 * T2 * T3) +
                    10.0 * T1 *
                      (7.0 * T1**3 + 14.0 * T1**2 * T2 + 12.0 * T1**2 * T3 + 9.0 * T1 * T2**2 + 18.0 * T1 * T2 * T3 +
                       6.0 * T1 * T3**2 + 2.0 * T2**3 + 6.0 * T2**2 * T3 + 6.0 * T2 * T3**2)) /
                   120.0;
            when 6 =>
               return
                 Distance_At_Stage (3.0 * T1 + T2 + T3, 5) +
                 Cm * DT * T1 *
                   (-DT**3 + 2.0 * DT**2 * (T1 + 2.0 * T2) +
                    2.0 * DT * (11.0 * T1**2 + 15.0 * T1 * T2 + 6.0 * T1 * T3 + 3.0 * T2**2 + 6.0 * T2 * T3) +
                    49.0 * T1**3 + 76.0 * T1**2 * T2 + 48.0 * T1**2 * T3 + 30.0 * T1 * T2**2 + 60.0 * T1 * T2 * T3 +
                    12.0 * T1 * T3**2 + 4.0 * T2**3 + 12.0 * T2**2 * T3 + 12.0 * T2 * T3**2) /
                   24.0;
            when 7 =>
               return
                 Distance_At_Stage (3.0 * T1 + 2.0 * T2 + T3, 6) +
                 Cm * DT *
                   (DT**4 - 5.0 * DT**3 * T1 + 10.0 * DT**2 * T1**2 +
                    10.0 * DT * T1 * (11.0 * T1**2 + 18.0 * T1 * T2 + 6.0 * T1 * T3 + 6.0 * T2**2 + 6.0 * T2 * T3) +
                    5.0 * T1 *
                      (49.0 * T1**3 + 120.0 * T1**2 * T2 + 48.0 * T1**2 * T3 + 96.0 * T1 * T2**2 +
                       84.0 * T1 * T2 * T3 + 12.0 * T1 * T3**2 + 24.0 * T2**3 + 36.0 * T2**2 * T3 +
                       12.0 * T2 * T3**2)) /
                   120.0;
            when 8 =>
               return
                 Distance_At_Stage (4.0 * T1 + 2.0 * T2 + T3, 7) +
                 Cm * DT * T1 *
                   (DT * (2.0 * T1**2 + 3.0 * T1 * T2 + T1 * T3 + T2**2 + T2 * T3) + 8.0 * T1**3 + 16.0 * T1**2 * T2 +
                    6.0 * T1**2 * T3 + 10.0 * T1 * T2**2 + 9.0 * T1 * T2 * T3 + T1 * T3**2 + 2.0 * T2**3 +
                    3.0 * T2**2 * T3 + T2 * T3**2) /
                   2.0;
            when 9 =>
               return
                 Distance_At_Stage (4.0 * T1 + 2.0 * T2 + T3 + T4, 8) +
                 Cm * DT *
                   (-DT**4 + 60.0 * DT * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + T1 * T3 + T2**2 + T2 * T3) +
                    60.0 * T1 *
                      (8.0 * T1**3 + 16.0 * T1**2 * T2 + 6.0 * T1**2 * T3 + 4.0 * T1**2 * T4 + 10.0 * T1 * T2**2 +
                       9.0 * T1 * T2 * T3 + 6.0 * T1 * T2 * T4 + T1 * T3**2 + 2.0 * T1 * T3 * T4 + 2.0 * T2**3 +
                       3.0 * T2**2 * T3 + 2.0 * T2**2 * T4 + T2 * T3**2 + 2.0 * T2 * T3 * T4)) /
                   120.0;
            when 10 =>
               return
                 Distance_At_Stage (5.0 * T1 + 2.0 * T2 + T3 + T4, 9) +
                 Cm * DT * T1 *
                   (-DT**3 - 2.0 * DT**2 * T1 +
                    2.0 * DT * (11.0 * T1**2 + 18.0 * T1 * T2 + 6.0 * T1 * T3 + 6.0 * T2**2 + 6.0 * T2 * T3) +
                    143.0 * T1**3 + 264.0 * T1**2 * T2 + 96.0 * T1**2 * T3 + 48.0 * T1**2 * T4 + 144.0 * T1 * T2**2 +
                    132.0 * T1 * T2 * T3 + 72.0 * T1 * T2 * T4 + 12.0 * T1 * T3**2 + 24.0 * T1 * T3 * T4 +
                    24.0 * T2**3 + 36.0 * T2**2 * T3 + 24.0 * T2**2 * T4 + 12.0 * T2 * T3**2 + 24.0 * T2 * T3 * T4) /
                   24.0;
            when 11 =>
               return
                 Distance_At_Stage (5.0 * T1 + 3.0 * T2 + T3 + T4, 10) +
                 Cm * DT *
                   (DT**4 - 5.0 * DT**3 * T1 - 10.0 * DT**2 * T1 * (T1 + 2.0 * T2) +
                    10.0 * DT * T1 * (11.0 * T1**2 + 15.0 * T1 * T2 + 6.0 * T1 * T3 + 3.0 * T2**2 + 6.0 * T2 * T3) +
                    5.0 * T1 *
                      (143.0 * T1**3 + 308.0 * T1**2 * T2 + 96.0 * T1**2 * T3 + 48.0 * T1**2 * T4 +
                       210.0 * T1 * T2**2 + 156.0 * T1 * T2 * T3 + 72.0 * T1 * T2 * T4 + 12.0 * T1 * T3**2 +
                       24.0 * T1 * T3 * T4 + 44.0 * T2**3 + 60.0 * T2**2 * T3 + 24.0 * T2**2 * T4 + 12.0 * T2 * T3**2 +
                       24.0 * T2 * T3 * T4)) /
                   120.0;
            when 12 =>
               return
                 Distance_At_Stage (6.0 * T1 + 3.0 * T2 + T3 + T4, 11) +
                 Cm * DT * T1 *
                   (-2.0 * DT**2 * (T1 + T2) +
                    3.0 * DT * (2.0 * T1**2 + 3.0 * T1 * T2 + 2.0 * T1 * T3 + T2**2 + 2.0 * T2 * T3) + 89.0 * T1**3 +
                    178.0 * T1**2 * T2 + 60.0 * T1**2 * T3 + 24.0 * T1**2 * T4 + 111.0 * T1 * T2**2 +
                    90.0 * T1 * T2 * T3 + 36.0 * T1 * T2 * T4 + 6.0 * T1 * T3**2 + 12.0 * T1 * T3 * T4 + 22.0 * T2**3 +
                    30.0 * T2**2 * T3 + 12.0 * T2**2 * T4 + 6.0 * T2 * T3**2 + 12.0 * T2 * T3 * T4) /
                   12.0;
            when 13 =>
               return
                 Distance_At_Stage (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 12) +
                 Cm * DT *
                   (DT**4 - 20.0 * DT**2 * T1 * (T1 + T2) + 30.0 * DT * T1 * (2.0 * T1**2 + 3.0 * T1 * T2 + T2**2) +
                    10.0 * T1 *
                      (89.0 * T1**3 + 178.0 * T1**2 * T2 + 72.0 * T1**2 * T3 + 24.0 * T1**2 * T4 + 111.0 * T1 * T2**2 +
                       108.0 * T1 * T2 * T3 + 36.0 * T1 * T2 * T4 + 12.0 * T1 * T3**2 + 12.0 * T1 * T3 * T4 +
                       22.0 * T2**3 + 36.0 * T2**2 * T3 + 12.0 * T2**2 * T4 + 12.0 * T2 * T3**2 +
                       12.0 * T2 * T3 * T4)) /
                   120.0;
            when 14 =>
               return
                 Distance_At_Stage (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4, 13) +
                 Cm * DT * T1 *
                   (DT**3 - 2.0 * DT**2 * (T1 + 2.0 * T2) + 2.0 * DT * (T1**2 + 3.0 * T1 * T2 + 3.0 * T2**2) +
                    191.0 * T1**3 + 380.0 * T1**2 * T2 + 144.0 * T1**2 * T3 + 48.0 * T1**2 * T4 + 234.0 * T1 * T2**2 +
                    216.0 * T1 * T2 * T3 + 72.0 * T1 * T2 * T4 + 24.0 * T1 * T3**2 + 24.0 * T1 * T3 * T4 +
                    44.0 * T2**3 + 72.0 * T2**2 * T3 + 24.0 * T2**2 * T4 + 24.0 * T2 * T3**2 + 24.0 * T2 * T3 * T4) /
                   24.0;
            when 15 =>
               return
                 Distance_At_Stage (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4, 14) +
                 Cm * DT *
                   (-DT**4 + 5.0 * DT**3 * T1 - 10.0 * DT**2 * T1**2 + 10.0 * DT * T1**3 +
                    5.0 * T1 *
                      (191.0 * T1**3 + 384.0 * T1**2 * T2 + 144.0 * T1**2 * T3 + 48.0 * T1**2 * T4 +
                       240.0 * T1 * T2**2 + 216.0 * T1 * T2 * T3 + 72.0 * T1 * T2 * T4 + 24.0 * T1 * T3**2 +
                       24.0 * T1 * T3 * T4 + 48.0 * T2**3 + 72.0 * T2**2 * T3 + 24.0 * T2**2 * T4 + 24.0 * T2 * T3**2 +
                       24.0 * T2 * T3 * T4)) /
                   120.0;
         end case;
      end Distance_At_Stage;

   begin
      if T < T1 then
         return Distance_At_Stage (T, 1);
      elsif T < T1 + T2 then
         return Distance_At_Stage (T - (T1), 2);
      elsif T < 2.0 * T1 + T2 then
         return Distance_At_Stage (T - (T1 + T2), 3);
      elsif T < 2.0 * T1 + T2 + T3 then
         return Distance_At_Stage (T - (2.0 * T1 + T2), 4);
      elsif T < 3.0 * T1 + T2 + T3 then
         return Distance_At_Stage (T - (2.0 * T1 + T2 + T3), 5);
      elsif T < 3.0 * T1 + 2.0 * T2 + T3 then
         return Distance_At_Stage (T - (3.0 * T1 + T2 + T3), 6);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 then
         return Distance_At_Stage (T - (3.0 * T1 + 2.0 * T2 + T3), 7);
      elsif T < 4.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Distance_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3), 8);
      elsif T < 5.0 * T1 + 2.0 * T2 + T3 + T4 then
         return Distance_At_Stage (T - (4.0 * T1 + 2.0 * T2 + T3 + T4), 9);
      elsif T < 5.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Distance_At_Stage (T - (5.0 * T1 + 2.0 * T2 + T3 + T4), 10);
      elsif T < 6.0 * T1 + 3.0 * T2 + T3 + T4 then
         return Distance_At_Stage (T - (5.0 * T1 + 3.0 * T2 + T3 + T4), 11);
      elsif T < 6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Distance_At_Stage (T - (6.0 * T1 + 3.0 * T2 + T3 + T4), 12);
      elsif T < 7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4 then
         return Distance_At_Stage (T - (6.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 13);
      elsif T < 7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4 then
         return Distance_At_Stage (T - (7.0 * T1 + 3.0 * T2 + 2.0 * T3 + T4), 14);
      else
         return Distance_At_Stage (T - (7.0 * T1 + 4.0 * T2 + 2.0 * T3 + T4), 15);
      end if;
   end Distance_At_Time;
end Motion;
