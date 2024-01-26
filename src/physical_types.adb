with Ada.Numerics.Generic_Elementary_Functions;

package body Physical_Types with
  --  Disable SPARK here as proving all this math is hard.
  SPARK_Mode => Off
is

   package Math is new Ada.Numerics.Generic_Elementary_Functions (Dimensioned_Float);
   use Math;

   function "*" (Left : Position; Right : Position_Scale) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) * Right (I)];
   end "*";

   function "*" (Left : Position_Offset; Right : Position_Scale) return Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) * Right (I)];
   end "*";

   function "*" (Left : Position_Scale; Right : Dimensionless) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) * Right];
   end "*";

   function "*" (Left : Position_Scale; Right : Length) return Scaled_Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) * Right];
   end "*";

   function "*" (Left : Scaled_Position; Right : Position_Scale) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) * Right (I)];
   end "*";

   function "*" (Left : Scaled_Position; Right : Dimensionless) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) * Right];
   end "*";

   function "*" (Left : Scaled_Position_Offset; Right : Position_Scale) return Scaled_Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) * Right (I)];
   end "*";

   function "*" (Left : Scaled_Position_Offset; Right : Dimensionless) return Scaled_Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) * Right];
   end "*";

   function "+" (Left : Scaled_Position; Right : Scaled_Position_Offset) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) + Right (I)];
   end "+";

   function "+" (Left, Right : Position_Scale) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) + Right (I)];
   end "+";

   function "-" (Left, Right : Position) return Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) - Right (I)];
   end "-";

   function "-" (Left, Right : Position_Scale) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) - Right (I)];
   end "-";

   function "-" (Left, Right : Scaled_Position) return Scaled_Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) - Right (I)];
   end "-";

   function "-" (Left, Right : Scaled_Position_Offset) return Scaled_Position_Offset is
   begin
      return [for I in Axis_Name => Left (I) - Right (I)];
   end "-";

   function "-" (Left : Scaled_Position; Right : Scaled_Position_Offset) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) - Right (I)];
   end "-";

   function "/" (Left : Position_Offset; Right : Length) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) / Right];
   end "/";

   function "/" (Left : Position_Scale; Right : Dimensionless) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) / Right];
   end "/";

   function "/" (Left : Scaled_Position_Offset; Right : Length) return Position_Scale is
   begin
      return [for I in Axis_Name => Left (I) / Right];
   end "/";

   function "/" (Left : Scaled_Position; Right : Dimensionless) return Scaled_Position is
   begin
      return [for I in Axis_Name => Left (I) / Right];
   end "/";

   function "abs" (Left : Position_Offset) return Length is
      Square_Sum : Area := 0.0 * mm**2;
   begin
      for X of Left loop
         Square_Sum := Square_Sum + X * X;
      end loop;

      return Sqrt (Square_Sum);
   end "abs";

   function "abs" (Left : Position_Scale) return Dimensionless is
      Square_Sum : Dimensionless := 0.0;
   begin
      for X of Left loop
         Square_Sum := Square_Sum + X * X;
      end loop;

      return Sqrt (Square_Sum);
   end "abs";

   function "abs" (Left : Scaled_Position_Offset) return Length is
      Square_Sum : Area := 0.0 * mm**2;
   begin
      for X of Left loop
         Square_Sum := Square_Sum + X * X;
      end loop;

      return Sqrt (Square_Sum);
   end "abs";

   function Dot (Left, Right : Position_Scale) return Dimensionless is
      Sum : Dimensionless := 0.0;
   begin
      for I in Axis_Name loop
         Sum := Sum + Left (I) * Right (I);
      end loop;

      return Sum;
   end Dot;

   function Dot (Left : Scaled_Position_Offset; Right : Position_Scale) return Length is
      Sum : Length := 0.0 * mm;
   begin
      for I in Axis_Name loop
         Sum := Sum + Left (I) * Right (I);
      end loop;

      return Sum;
   end Dot;

   function Dot (Left, Right : Scaled_Position_Offset) return Area is
      Sum : Area := 0.0 * mm**2;
   begin
      for I in Axis_Name loop
         Sum := Sum + Left (I) * Right (I);
      end loop;

      return Sum;
   end Dot;

end Physical_Types;
