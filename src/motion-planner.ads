with Ada.Containers.Synchronized_Queue_Interfaces;
with Ada.Containers.Bounded_Synchronized_Queues;

private package Motion.Planner is

   type Command_Kind is (Move_Kind, Flush_Kind);

   type Command (Kind : Command_Kind := Move_Kind) is record
      case Kind is
         when Flush_Kind =>
            Next_Master : Master_Manager.Master;
         when Move_Kind =>
            Pos    : Position;
            Limits : Kinematic_Limits;
      end case;
   end record;

   package Command_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Command);
   package Command_Queues is new Ada.Containers.Bounded_Synchronized_Queues (Command_Queues_Interface, 100);

   Command_Queue : Command_Queues.Queue;

   --  TODO: Change back to 2000
   type Corners_Index is range 0 .. 200;

   type Curve_Point_Set_Index is range 0 .. 1_000;
   type Curve_Point_Set_Values is array (Curve_Point_Set_Index range <>) of Scaled_Position;
   type Curve_Point_Set (Points_Per_Side : Curve_Point_Set_Index := 0) is record
      Incoming : Curve_Point_Set_Values (0 .. Points_Per_Side);
      Outgoing : Curve_Point_Set_Values (0 .. Points_Per_Side);
   end record;

   --  Curve_Splitter
   type Block_Curve_Point_Sets is array (Corners_Index range <>) of Curve_Point_Set;

   --  Feedrate_Profile_Generator
   type Block_Feedrate_Profiles is array (Corners_Index range <>) of Feedrate_Profile;

   --  N_Corners may be 0 or 1, in which case no movement should occur.
   type Execution_Block (N_Corners : Motion.Planner.Corners_Index := 0) is record
      --  Preprocessor
      Next_Master       : Master_Manager.Master;
      --  Curve_Splitter
      Curve_Point_Sets  : Block_Curve_Point_Sets (1 .. N_Corners);
      --  Feedrate_Profile_Generator
      Feedrate_Profiles : Block_Feedrate_Profiles (2 .. N_Corners);
   end record;

   package Execution_Block_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Execution_Block);
   package Execution_Block_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Execution_Block_Queues_Interface, 10);
   use Execution_Block_Queues;

   Execution_Block_Queue : Execution_Block_Queues.Queue;

   task Runner is
      entry Init (Conf : Config_Parameters);
   end Runner;

private

   --  Preprocessor
   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Limits is array (Corners_Index range <>) of Kinematic_Limits;

   --  Corner_Blender
   Corner_Blender_Max_Computational_Error : Length := 0.000_1 * mm;

   type Bezier_Index is range 1 .. 10;
   type Bezier is array (Bezier_Index) of Scaled_Position;
   type Block_Beziers is array (Corners_Index range <>) of Bezier;

   type Block_Inverse_Curvatures is array (Corners_Index range <>) of Length;
   type Block_Midpoints is array (Corners_Index range <>) of Scaled_Position;
   type Block_Shifted_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Shifted_Corner_Error_Limits is array (Corners_Index range <>) of Length;

   --  Kinematic_Limiter
   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  N_Corners may be 0 or 1, in which case no movement should occur.
   type Working_Block (N_Corners : Corners_Index := 0) is record
      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type with a discriminant to indicate the stage causes a stack overflow when trying to change
      --  the discriminant without making a copy as GCC tries to copy the whole thing to the stack. In the future we
      --  could possibly use SPARK to ensure stages do not touch fields that are not yet assigned.
      
      Execution : Execution_Block (N_Corners);

      --  Preprocessor
      Corners        : Block_Plain_Corners (1 .. N_Corners);
      Segment_Limits : Block_Segment_Limits (2 .. N_Corners);

      --  Corner_Blender
      Beziers                     : Block_Beziers (1 .. N_Corners);
      Inverse_Curvatures          : Block_Inverse_Curvatures (1 .. N_Corners);
      Midpoints                   : Block_Midpoints (1 .. N_Corners);
      Shifted_Corners             : Block_Shifted_Corners (1 .. N_Corners);
      Shifted_Corner_Error_Limits : Block_Shifted_Corner_Error_Limits (1 .. N_Corners);

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits (1 .. N_Corners);
   end record;
   
   function Compute_Bezier_Point (Bez : Bezier; T : Dimensionless) return Scaled_Position;

   function Curve_Corner_Distance (Start, Finish : Curve_Point_Set_Values) return Length;

end Motion.Planner;
