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

   type Corners_Index is range 1 .. 2000;

   --  Preprocessor
   type Block_Plain_Corners is array (Corners_Index) of Scaled_Position;
   type Block_Segment_Limits is array (Corners_Index range 2 .. Corners_Index'Last) of Kinematic_Limits;

   --  Corner_Blender
   Corner_Blender_Max_Computational_Error : Length := 0.000_1 * mm;

   type Bezier_Index is range 1 .. 10;
   type Bezier is array (Bezier_Index) of Scaled_Position;
   type Block_Beziers is array (Corners_Index) of Bezier;

   type Block_Inverse_Curvatures is array (Corners_Index) of Length;
   type Block_Midpoints is array (Corners_Index) of Scaled_Position;
   type Block_Shifted_Corners is array (Corners_Index) of Scaled_Position;
   type Block_Shifted_Corner_Error_Limits is array (Corners_Index) of Length;

   --  Curve_Splitter
   Curve_Splitter_Target_Step : Length := 0.000_1 * mm;

   type Curve_Point_Set_Index is range 1 .. 1_000;
   type Curve_Point_Set_Values is array (Curve_Point_Set_Index) of Scaled_Position;
   type Curve_Point_Set is record
      Points_Per_Side : Curve_Point_Set_Index;
      Incoming_Length : Length;
      Outgoing_Length : Length;
      Incoming        : Curve_Point_Set_Values;
      Outgoing        : Curve_Point_Set_Values;
   end record;

   type Block_Curve_Point_Sets is array (Corners_Index) of Curve_Point_Set;

   --  Feedrate_Profile_Generator
   type Block_Feedrate_Profiles is array (Corners_Index range 2 .. Corners_Index'Last) of Feedrate_Profile;

   --  Kinematic_Limiter
   type Block_Corner_Velocity_Limits is array (Corners_Index) of Velocity;

   --  N_Corners may be 0 or 1, in which case no movement should occur.
   type Execution_Block is record
      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type with a discriminant to indicate the stage causes a stack overflow when trying to change
      --  the discriminant without making a copy as GCC tries to copy the whole thing to the stack. In the future we
      --  could possibly use SPARK to ensure stages do not touch fields that are not yet assigned.
      N_Corners : Motion.Planner.Corners_Index;

      --  Preprocessor
      Next_Master    : Master_Manager.Master;
      Corners        : Block_Plain_Corners;
      Foo : Boolean;
      Segment_Limits : Block_Segment_Limits;

      --  Corner_Blender
      Beziers                     : Block_Beziers;
      Inverse_Curvatures          : Block_Inverse_Curvatures;
      Midpoints                   : Block_Midpoints;
      Shifted_Corners             : Block_Shifted_Corners;
      Shifted_Corner_Error_Limits : Block_Shifted_Corner_Error_Limits;

      --  Curve_Splitter
      Curve_Point_Sets : Block_Curve_Point_Sets;

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits;

      --  Feedrate_Profile_Generator
      Feedrate_Profiles : Block_Feedrate_Profiles;
   end record;

   package Execution_Block_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Execution_Block);
   package Execution_Block_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Execution_Block_Queues_Interface, 5);
   use Execution_Block_Queues;

   Execution_Block_Queue : Execution_Block_Queues.Queue;

   task Runner is
      entry Init (Conf : Config_Parameters);
   end Runner;
   
end Motion.Planner;
