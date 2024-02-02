with Ada.Unchecked_Conversion;

package body Motion.PH_Beziers is

   function Distance_At_T (Bez : PH_Bezier; T : Dimensionless) return Length is
      L : constant Length := abs (Bez.Control_Points (0) - Bez.Control_Points (1));
      B : constant Length := abs (Bez.Control_Points (4) - Bez.Control_Points (5));
   begin
      if L = 0.0 then
         return 0.0 * mm;
      else
         return
           T *
           (23_940.0 * L**2 + 9_815_520.0 * T**14 * (-B**2 + L**2) + 73_616_400.0 * T**13 * (B**2 - L**2) +
            233_873_640.0 * T**12 * (-B**2 + L**2) + 403_663_260.0 * T**11 * (B**2 - L**2) +
            400_071_672.0 * T**10 * (-B**2 + L**2) + 216_432_216.0 * T**9 * (B**2 - L**2) +
            50_100_050.0 * T**8 * (-B**2 + L**2) + 920_205.0 * T**7 * (-B**2 + L**2) +
            3_680_820.0 * T**6 * (B**2 - L**2) + 5_153_148.0 * T**5 * (-B**2 + L**2) +
            2_576_574.0 * T**4 * (B**2 - L**2)) /
           (1_596.0 * L);
      end if;
   end Distance_At_T;

   function T_At_Distance (Bez : PH_Bezier; Distance : Length) return Dimensionless is
      Result : Dimensionless;
      Lower  : Dimensionless := 0.0;
      Upper  : Dimensionless := 1.0;

      type Casted_Dimensionless is mod 2**64;
      function Cast_Dimensionless is new Ada.Unchecked_Conversion (Dimensionless, Casted_Dimensionless);
      function Cast_Dimensionless is new Ada.Unchecked_Conversion (Casted_Dimensionless, Dimensionless);
   begin
      --  This probably breaks when not using IEEE 754 floats or on other weird systems, so try to check for
      --  that.
      pragma Assert (Dimensionless'Size = 64);
      pragma Assert (Casted_Dimensionless'Size = 64);
      pragma Assert (Cast_Dimensionless (86_400.0) = 4_680_673_776_000_565_248);
      pragma Assert (Cast_Dimensionless (0.123_45) = 4_593_559_930_647_147_132);

      pragma Assert (Distance <= Distance_At_T (Bez, 1.0));

      loop
         Result :=
           Cast_Dimensionless
             (Cast_Dimensionless (Lower) + (Cast_Dimensionless (Upper) - Cast_Dimensionless (Lower)) / 2);
         exit when Lower = Result or Upper = Result;
         if Distance_At_T (Bez, Result) <= Distance then
            Lower := Result;
         else
            Upper := Result;
         end if;
      end loop;

      return Result;
   end T_At_Distance;

   function Inverse_Curvature (Bez : PH_Bezier) return Length is
   begin
      return Bez.Inverse_Curvature;
   end Inverse_Curvature;

   function Midpoint (Bez : PH_Bezier) return Scaled_Position is
   begin
      --  It is possible to compute the midpoint by multiplying the corner deviation by the unit bisector of the two
      --  vectors from the corner to the start/finish. The corner deviation may be computed by the following equation:
      --  Midpoint = (Sine_Secondary_Angle / 2.0**14) * Base_Length *
      --    ((397.0 / 429.0) + 10_207.0 + (2.0**14 * 1_225.0) / (858.8 * Cosine_Secondary_Angle))
      --
      --  This method may be used if a speed improvement is needed.

      return Point_At_T (Bez, 0.5);
   end Midpoint;

   function Point_At_T (Bez : PH_Bezier; T : Dimensionless) return Scaled_Position is
      Bez_2 : PH_Control_Points := Bez.Control_Points;
   begin
      for J in reverse Bez_2'First .. Bez_2'Last - 1 loop
         for I in Bez_2'First .. J loop
            Bez_2 (I) := Bez_2 (I) + (Bez_2 (I + 1) - Bez_2 (I)) * T;
         end loop;
      end loop;

      return Bez_2 (Bez_2'First);
   end Point_At_T;

   --  This method is slower than Point_At_T on most CPUs, but may be useful if this code is ported to a GPU or FPGA.
   --  It may also be faster for cases where T is known at compile time, but I am not aware of any methods to detect
   --  that with GCC.
   function Point_At_T_V2 (Bez : PH_Bezier; T : Dimensionless) return Scaled_Position is
   begin
      return
        Bez.Control_Points (0) * ((1.0 - T)**15) +
        Scaled_Position_Offset (Bez.Control_Points (1)) * (15.0 * T * (1.0 - T)**14) +
        Scaled_Position_Offset (Bez.Control_Points (2)) * (105.0 * T**2 * (1.0 - T)**13) +
        Scaled_Position_Offset (Bez.Control_Points (3)) * (455.0 * T**3 * (1.0 - T)**12) +
        Scaled_Position_Offset (Bez.Control_Points (4)) * (1_365.0 * T**4 * (1.0 - T)**11) +
        Scaled_Position_Offset (Bez.Control_Points (5)) * (3_003.0 * T**5 * (1.0 - T)**10) +
        Scaled_Position_Offset (Bez.Control_Points (6)) * (5_005.0 * T**6 * (1.0 - T)**9) +
        Scaled_Position_Offset (Bez.Control_Points (7)) * (6_435.0 * T**7 * (1.0 - T)**8) +
        Scaled_Position_Offset (Bez.Control_Points (8)) * (6_435.0 * T**8 * (1.0 - T)**7) +
        Scaled_Position_Offset (Bez.Control_Points (9)) * (5_005.0 * T**9 * (1.0 - T)**6) +
        Scaled_Position_Offset (Bez.Control_Points (10)) * (3_003.0 * T**10 * (1.0 - T)**5) +
        Scaled_Position_Offset (Bez.Control_Points (11)) * (1_365.0 * T**11 * (1.0 - T)**4) +
        Scaled_Position_Offset (Bez.Control_Points (12)) * (455.0 * T**12 * (1.0 - T)**3) +
        Scaled_Position_Offset (Bez.Control_Points (13)) * (105.0 * T**13 * (1.0 - T)**2) +
        Scaled_Position_Offset (Bez.Control_Points (14)) * (15.0 * T**14 * (1.0 - T)) +
        Scaled_Position_Offset (Bez.Control_Points (15)) * (T**15);
   end Point_At_T_V2;

   function Point_At_Distance (Bez : PH_Bezier; Distance : Length) return Scaled_Position is
   begin
      return Point_At_T (Bez, T_At_Distance (Bez, Distance));
   end Point_At_Distance;

   function Create_Bezier (Start, Corner, Finish : Scaled_Position; Deviation_Limit : Length) return PH_Bezier is
      function Real_Create_Bezier return PH_Bezier is
         function Sine_Secondary_Angle return Dimensionless is
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
         end Sine_Secondary_Angle;

         function Cosine_Secondary_Angle return Dimensionless is
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
         end Cosine_Secondary_Angle;

         function Base_Length return Length is
            Incoming_Length : constant Length := abs (Start - Corner);
            Outgoing_Length : constant Length := abs (Finish - Corner);

            Deviation_Base_Length_Numerator   : constant Length        := Deviation_Limit * 2.0**14;
            Deviation_Base_Length_Denominator : constant Dimensionless :=
              Sine_Secondary_Angle *
              (4_072_849.0 / 429.0 + 714.0 + 2.0**14 * 1_225.0 / (858.0 * Cosine_Secondary_Angle));
            Incoming_Limit                    : constant Length        :=
              (0.49 * 858.0 * Incoming_Length * Cosine_Secondary_Angle) / (5_210.0 * Cosine_Secondary_Angle + 1_225.0);
            Outgoing_Limit                    : constant Length        :=
              (0.49 * 858.0 * Outgoing_Length * Cosine_Secondary_Angle) / (5_210.0 * Cosine_Secondary_Angle + 1_225.0);
         begin
            --  TODO: Do we need a small error margin here?
            if Deviation_Base_Length_Denominator = 0.0 then
               --  Collinear points.
               return Length'Min (Incoming_Limit, Outgoing_Limit);
            else
               return
                 Length'Min
                   (Deviation_Base_Length_Numerator / Deviation_Base_Length_Denominator,
                    Length'Min (Incoming_Limit, Outgoing_Limit));
            end if;
         end Base_Length;

         Incoming_Unit : constant Position_Scale         := (Start - Corner) / abs (Start - Corner);
         Outgoing_Unit : constant Position_Scale         := (Finish - Corner) / abs (Finish - Corner);
         M             : constant Scaled_Position_Offset :=
           ((Outgoing_Unit - Incoming_Unit) / abs (Outgoing_Unit - Incoming_Unit)) * Base_Length;
         Points        : PH_Control_Points;
      begin
         Points (0) :=
           Corner + Incoming_Unit * ((4.0 + 889.0 / 429.0 + 1_225.0 / (858.0 * Cosine_Secondary_Angle)) * Base_Length);
         Points (1) := Points (0) - Incoming_Unit * Base_Length;
         Points (2) := Points (1) - Incoming_Unit * Base_Length;
         Points (3) := Points (2) - Incoming_Unit * Base_Length;
         Points (4) := Points (3) - Incoming_Unit * Base_Length;
         Points (5) := Points (4) + M * (10.0 / 143.0) - Incoming_Unit * ((133.0 / 143.0) * Base_Length);
         Points (6) := Points (5) + M * (38.0 / 143.0) - Incoming_Unit * ((105.0 / 143.0) * Base_Length);
         Points (7) := Points (6) + M * (254.0 / 429.0) - Incoming_Unit * ((175.0 / 429.0) * Base_Length);
         Points (8) := Points (7) + M;

         Points (15) :=
           Corner + Outgoing_Unit * ((4.0 + 889.0 / 429.0 + 1_225.0 / (858.0 * Cosine_Secondary_Angle)) * Base_Length);
         Points (14) := Points (15) - Outgoing_Unit * Base_Length;
         Points (13) := Points (14) - Outgoing_Unit * Base_Length;
         Points (12) := Points (13) - Outgoing_Unit * Base_Length;
         Points (11) := Points (12) - Outgoing_Unit * Base_Length;
         Points (10) := Points (11) - M * (10.0 / 143.0) - Outgoing_Unit * ((133.0 / 143.0) * Base_Length);
         Points (9)  := Points (10) - M * (38.0 / 143.0) - Outgoing_Unit * ((105.0 / 143.0) * Base_Length);

         return
           (Control_Points    => Points,
            Inverse_Curvature =>
              (12.0 / 7.0) * Base_Length / (Sine_Secondary_Angle / (1.0 - Sine_Secondary_Angle**2)**(1 / 2)));
      end Real_Create_Bezier;
   begin
      if Start = Corner or Finish = Corner then
         return (Control_Points => [others => Corner], Inverse_Curvature => 0.0 * mm);
      else
         return Real_Create_Bezier;
      end if;
   end Create_Bezier;

end Motion.PH_Beziers;
