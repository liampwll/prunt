with Ada.Numerics.Generic_Elementary_Functions;
with Ada.Text_IO; use Ada.Text_IO;

package body Motion.Curvifier is

   package Elementary_Functions is new Ada.Numerics.Generic_Elementary_Functions (Dimensioned_Float);
   use Elementary_Functions;

   function Compute_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Angle is
      function Clamped_Arccos (X : Dimensionless) return Angle is
      begin
         if X < -1.0 then
            return Ada.Numerics.Pi;
         elsif X > 1.0 then
            return 0.0;
         else
            return Arccos (X);
         end if;
      end Clamped_Arccos;

      V1              : constant Scaled_Position_Offset := Start - Corner;
      V2              : constant Scaled_Position_Offset := Finish - Corner;
      Dot_Product     : constant Dimensionless          := Dot (V1 / abs V1, V2 / abs V2);
      Primary_Angle   : constant Angle                  := Clamped_Arccos (Dot_Product);
      Secondary_Angle : constant Angle                  := (Ada.Numerics.Pi - Primary_Angle) / 2.0;
   begin
      return Secondary_Angle;
   end Compute_Secondary_Angle;

   function Compute_Control_Point_Ratio (Start, Corner, Finish : Scaled_Position) return Control_Point_Ratio is
      Secondary_Angle : constant Angle := Compute_Secondary_Angle (Start, Corner, Finish);
   begin
      if Secondary_Angle < Ada.Numerics.Pi / 6.0 then
         return [2.5, 1.0, 0.7, 0.4];
      elsif Secondary_Angle < Ada.Numerics.Pi / 4.0 then
         return [1.0, 0.8, 0.3, 0.3];
      elsif Secondary_Angle < Ada.Numerics.Pi / 3.0 then
         return [1.0, 0.5, 0.5, 0.5];
      else
         return [0.7, 0.3, 0.2, 0.1];
      end if;
   end Compute_Control_Point_Ratio;

   function Compute_Inverse_Curvature
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Length
   is
      CPR             : constant Control_Point_Ratio := Compute_Control_Point_Ratio (Start, Corner, Finish);
      Secondary_Angle : constant Angle               := Compute_Secondary_Angle (Start, Corner, Finish);
      X               : constant Dimensionless       := 14.0 * CPR (1) + 14.0 * CPR (2) + 6.0 * CPR (3) + CPR (4);
      Y               : constant Dimensionless := 130.0 * CPR (1) + 46.0 * CPR (2) + 10.0 * CPR (3) + CPR (4) + 256.0;
      Z               : constant Dimensionless := 56.0 * CPR (1) + 28.0 * CPR (2) + 8.0 * CPR (3) + CPR (4) + 70.0;
   begin
      if Secondary_Angle = Ada.Numerics.Pi / 2.0 then
         --  This is probably not needed for most Ada implementations and is likely not good enough when it is needed.
         --  We can fix it later if problems occur.
         return 0.0 * mm;
      elsif Secondary_Angle = 0.0 then
         return Length'Last;
      end if;
      return (9.0 * Z**2.0 * Max_Chord_Error) / (8.0 * X * Y * Tan (Secondary_Angle)**2.0);
   end Compute_Inverse_Curvature;

   function Sum_Control_Point_Ratio (CPR : Control_Point_Ratio) return Dimensionless is
      Sum : Dimensionless := 0.0;
   begin
      for X of CPR loop
         Sum := @ + X;
      end loop;
      return Sum;
   end Sum_Control_Point_Ratio;

   function Compute_Bezier_Base_Length
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Length
   is
      Secondary_Angle : constant Angle               := Compute_Secondary_Angle (Start, Corner, Finish);
      CPR             : constant Control_Point_Ratio := Compute_Control_Point_Ratio (Start, Corner, Finish);
      CPR_Sum         : constant Dimensionless       := Sum_Control_Point_Ratio (CPR);
      Incoming_Length : constant Length              := abs (Start - Corner);
      Outgoing_Length : constant Length              := abs (Finish - Corner);

      Deviation_Limit_Numerator   : constant Length        := Max_Chord_Error * 256.0;
      Deviation_Limit_Denominator : constant Dimensionless :=
        (130.0 * CPR (1) + 46.0 * CPR (2) + 10.0 * CPR (3) + CPR (4) + 256.0) * Sin (Secondary_Angle);

      --  TODO: The following 0.5 is not needed for the first move in a block.
      Incoming_Limit  : constant Length := 0.5 * Incoming_Length / (CPR_Sum + 1.0);
      --  TODO: The following 0.5 is not needed for the last move in a block.
      Outgoing_Limit  : constant Length := 0.5 * Outgoing_Length / (CPR_Sum + 1.0);
   begin
      if Deviation_Limit_Denominator = 0.0 then
         return 0.0 * mm;
      else
         return Length'Min
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

   function Compute_Control_Points (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Bezier is
      CPR           : constant Control_Point_Ratio := Compute_Control_Point_Ratio (Start, Corner, Finish);
      Base_Length   : constant Length := Compute_Bezier_Base_Length (Start, Corner, Finish, Max_Chord_Error);
      Incoming_Unit : constant Position_Scale      := (Start - Corner) / abs (Start - Corner);
      Outgoing_Unit : constant Position_Scale      := (Finish - Corner) / abs (Finish - Corner);
      Result        : Bezier;
      CPR_Sum       : Dimensionless                := 1.0;
   begin
      Result (Result'First + 4) := Corner + Incoming_Unit * Base_Length;
      Result (Result'First + 5) := Corner + Outgoing_Unit * Base_Length;

      for I in 1 .. 4 loop
         CPR_Sum                                      := @ + CPR (I);
         Result (Result'First + 4 - Bezier_Index (I)) := Corner + Incoming_Unit * (CPR_Sum * Base_Length);
         Result (Result'First + 5 + Bezier_Index (I)) := Corner + Outgoing_Unit * (CPR_Sum * Base_Length);
      end loop;

      return Result;
   end Compute_Control_Points;

   function Compute_Curve_Mid_Point
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Scaled_Position
   is
      Secondary_Angle : constant Angle               := Compute_Secondary_Angle (Start, Corner, Finish);
      CPR             : constant Control_Point_Ratio := Compute_Control_Point_Ratio (Start, Corner, Finish);
      Base_Length     : constant Length := Compute_Bezier_Base_Length (Start, Corner, Finish, Max_Chord_Error);
      Bisector        : constant Position_Scale      := Compute_Unit_Bisector (Start, Corner, Finish);
      Magic           : constant Dimensionless       :=
        (130.0 * CPR (1) + 46.0 * CPR (2) + 10.0 * CPR (3) + CPR (4) + 256.0) / 256.0 * Sin (Secondary_Angle) *
        Base_Length;
   begin
      return Corner + Bisector * Magic;
   end Compute_Curve_Mid_Point;

   task body Runner is
      Config : Config_Parameters;

      type Midpoints_Type is array (Corners_Index) of Scaled_Position;
      type Midpoints_Type_Access is access Midpoints_Type;

      type Shifted_Corners_Type is array (Corners_Index) of Scaled_Position;
      type Shifted_Corners_Type_Access is access Shifted_Corners_Type;

      type Chord_Error_Limits_Type is array (Corners_Index) of Length;
      type Chord_Error_Limits_Type_Access is access Chord_Error_Limits_Type;

      Midpoints          : constant Midpoints_Type_Access          := new Midpoints_Type;
      Shifted_Corners    : constant Shifted_Corners_Type_Access    := new Shifted_Corners_Type;
      Chord_Error_Limits : constant Chord_Error_Limits_Type_Access := new Chord_Error_Limits_Type;

      procedure Processor (Data : in out Block_Data) is
         Max_Computational_Error : Length;
      begin
         pragma Assert (Data.Last_Stage = Preprocessor_Stage);

         for I in Data.Corners'Range loop
            Shifted_Corners (I)    := Data.Corners (I);
            Chord_Error_Limits (I) := Config.Chord_Error_Limit;
         end loop;

         Shifted_Corners (Data.Corners'First) := Data.Corners (Data.Corners'First);
         Shifted_Corners (Data.Corners'Last)  := Data.Corners (Data.Corners'Last);

         loop
            Max_Computational_Error := 0.0 * mm;

            for I in Data.Corners'First + 1 .. Data.Corners'Last - 1 loop
               declare
                  Start  : constant Scaled_Position := Shifted_Corners (I - 1);
                  Corner : constant Scaled_Position := Shifted_Corners (I);
                  Finish : constant Scaled_Position := Shifted_Corners (I + 1);
               begin
                  Midpoints (I)           := Compute_Curve_Mid_Point (Start, Corner, Finish, Chord_Error_Limits (I));
                  Max_Computational_Error := Length'Max (@, abs (Midpoints (I) - Data.Corners (I)));
               end;
            end loop;

            exit when Max_Computational_Error <= Curvifier_Max_Computational_Error;

            for I in Data.Corners'First + 1 .. Data.Corners'Last - 1 loop
               Shifted_Corners (I) := @ + (Data.Corners (I) - Midpoints (I));
            end loop;

            for I in Data.Corners'First + 1 .. Data.Corners'Last - 1 loop
               Chord_Error_Limits (I) :=
                 abs Dot
                   (Data.Corners (I) - Shifted_Corners (I),
                    Compute_Unit_Bisector (Shifted_Corners (I - 1), Shifted_Corners (I), Shifted_Corners (I + 1)));
            end loop;
         end loop;

         for I in Data.Beziers'First + 1 .. Data.Beziers'Last - 1 loop
            Data.Beziers (I) :=
               Compute_Control_Points
                  (Shifted_Corners (I - 1), Shifted_Corners (I), Shifted_Corners (I + 1), Chord_Error_Limits (I));
            Data.Inverse_Curvatures (I) :=
               Compute_Inverse_Curvature
                  (Shifted_Corners (I - 1), Shifted_Corners (I), Shifted_Corners (I + 1), Chord_Error_Limits (I));
         end loop;

         Data.Beziers (Data.Beziers'First) := [for I in Bezier_Index => Shifted_Corners (Data.Beziers'First)];
         Data.Beziers (Data.Beziers'Last)  := [for I in Bezier_Index => Shifted_Corners (Data.Beziers'Last)];
         Data.Inverse_Curvatures (Data.Inverse_Curvatures'First) := 0.0 * mm;
         Data.Inverse_Curvatures (Data.Inverse_Curvatures'Last)  := 0.0 * mm;
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Process (Curvifier_Stage) (Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Curvifier;
