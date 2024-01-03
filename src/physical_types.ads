package Physical_Types is
   type Dimensioned_Float is new Long_Float with
     Dimension_System =>
      ((Unit_Name => Millimeter, Unit_Symbol => "mm", Dim_Symbol => "Length"),
       (Unit_Name => Second, Unit_Symbol => "s", Dim_Symbol => "Time"),
       (Unit_Name => Celcius, Unit_Symbol => "°C", Dim_Symbol => "Temperature"));

   subtype Length is Dimensioned_Float with
       Dimension => (Symbol => "mm", Millimeter => 1, others => 0);

   subtype Time is Dimensioned_Float with
       Dimension => (Symbol => "s", Second => 1, others => 0);

   subtype Temperature is Dimensioned_Float with
       Dimension => (Symbol => "°C", Celcius => 1, others => 0);

   subtype Angle is Dimensioned_Float with
       Dimension => (Symbol => "rad", others => 0);

   subtype Dimensionless is Dimensioned_Float with
       Dimension => (Symbol => "×", others => 0);

   pragma Warnings (Off, "assumed to be");
   mm      : constant Length      := 1.0;
   s       : constant Time        := 1.0;
   celcius : constant Temperature := 1.0;
   radian  : constant Angle       := 1.0;
   pragma Warnings (On, "assumed to be");

   ms  : constant Time := s / 1_000.0;
   min : constant Time := s * 60.0;

   subtype Velocity is Dimensioned_Float with
       Dimension => (Symbol => "mm/s", Millimeter => 1, Second => -1, others => 0);
   subtype Acceleration is Dimensioned_Float with
       Dimension => (Symbol => "mm/s²", Millimeter => 1, Second => -2, others => 0);
   subtype Jerk is Dimensioned_Float with
       Dimension => (Symbol => "mm/s³", Millimeter => 1, Second => -3, others => 0);
   subtype Snap is Dimensioned_Float with
       Dimension => (Symbol => "mm/s⁴", Millimeter => 1, Second => -4, others => 0);
   subtype Crackle is Dimensioned_Float with
       Dimension => (Symbol => "mm/s⁵", Millimeter => 1, Second => -5, others => 0);

   subtype Area is Dimensioned_Float with
       Dimension => (Symbol => "mm²", Millimeter => 2, others => 0);

   subtype Curvature is Dimensioned_Float with
       Dimension => (Symbol => "mm⁻¹", Millimeter => -1, others => 0);
   subtype Curvature_To_2 is Dimensioned_Float with
       Dimension => (Symbol => "mm⁻²", Millimeter => -2, others => 0);
   subtype Curvature_To_3 is Dimensioned_Float with
       Dimension => (Symbol => "mm⁻³", Millimeter => -3, others => 0);
   subtype Curvature_To_4 is Dimensioned_Float with
       Dimension => (Symbol => "mm⁻⁴", Millimeter => -4, others => 0);

   type Axis_Name is (X_Axis, Y_Axis, Z_Axis, E_Axis);

   type Position is array (Axis_Name) of Length;
   --  A Scaled_Position is an absolute position that does not represent real machine coordinates.
   type Scaled_Position is array (Axis_Name) of Length;
   type Position_Offset is array (Axis_Name) of Length;
   type Scaled_Position_Offset is array (Axis_Name) of Length;
   type Position_Scale is array (Axis_Name) of Dimensionless;

   function "*" (Left : Position; Right : Position_Scale) return Scaled_Position;
   function "*" (Left : Position_Offset; Right : Position_Scale) return Position_Offset;
   function "*" (Left : Position_Scale; Right : Dimensionless) return Position_Scale;
   function "*" (Left : Position_Scale; Right : Length) return Scaled_Position_Offset;
   function "*" (Left : Scaled_Position; Right : Position_Scale) return Scaled_Position;
   function "*" (Left : Scaled_Position_Offset; Right : Position_Scale) return Scaled_Position_Offset;
   function "*" (Left : Scaled_Position_Offset; Right : Dimensionless) return Scaled_Position_Offset;
   function "+" (Left : Scaled_Position; Right : Scaled_Position_Offset) return Scaled_Position;
   function "+" (Left, Right : Position_Scale) return Position_Scale;
   function "-" (Left, Right : Position) return Position_Offset;
   function "-" (Left, Right : Scaled_Position) return Scaled_Position_Offset;
   function "/" (Left : Position_Offset; Right : Length) return Position_Scale;
   function "/" (Left : Position_Scale; Right : Dimensionless) return Position_Scale;
   function "/" (Left : Scaled_Position_Offset; Right : Length) return Position_Scale;
   function "abs" (Left : Position_Offset) return Length;
   function "abs" (Left : Position_Scale) return Dimensionless;
   function "abs" (Left : Scaled_Position_Offset) return Length;
   function Dot (Left, Right : Position_Scale) return Dimensionless;
   function Dot (Left : Scaled_Position_Offset; Right : Position_Scale) return Length;

end Physical_Types;
