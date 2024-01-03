private package Motion.Curvifier is

   task Runner is
      entry Init (In_Config : Config_Parameters);
   end Runner;

private

   type Control_Point_Ratio is array (1 .. 4) of Dimensionless;

   function Compute_Secondary_Angle (Start, Corner, Finish : Scaled_Position) return Angle;

   function Compute_Control_Point_Ratio (Start, Corner, Finish : Scaled_Position) return Control_Point_Ratio;

   function Compute_Inverse_Curvature
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Length;

   function Sum_Control_Point_Ratio (CPR : Control_Point_Ratio) return Dimensionless;

   function Compute_Bezier_Base_Length
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Length;

   function Compute_Unit_Bisector (Start, Corner, Finish : Scaled_Position) return Position_Scale;

   function Compute_Control_Points (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Bezier;

   function Compute_Curve_Mid_Point
     (Start, Corner, Finish : Scaled_Position; Max_Chord_Error : Length) return Scaled_Position;

end Motion.Curvifier;
