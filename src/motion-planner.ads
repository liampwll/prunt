with Ada.Containers.Synchronized_Queue_Interfaces;
with Ada.Containers.Bounded_Synchronized_Queues;
with Motion.PH_Beziers; use Motion.PH_Beziers;

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

   type Corners_Index is range 0 .. 50_000;

   --  Preprocessor
   Preprocessor_Minimum_Move_Distance : constant Length := 0.001 * mm;
   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Limits is array (Corners_Index range <>) of Kinematic_Limits;

   --  Corner_Blender
   Corner_Blender_Do_Shifting                  : constant Boolean := True;
   Corner_Blender_Max_Computational_Error      : constant Length  := 0.001 * mm;
   Corner_Blender_Max_Secondary_Angle_To_Blend : constant Angle   := 89.5 * deg;

   type Block_Beziers is array (Corners_Index range <>) of PH_Bezier;

   type Block_Inverse_Curvatures is array (Corners_Index range <>) of Length;
   type Block_Midpoints is array (Corners_Index range <>) of Scaled_Position;
   type Block_Shifted_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Shifted_Corner_Error_Limits is array (Corners_Index range <>) of Length;

   --  Feedrate_Profile_Generator
   type Block_Feedrate_Profiles is array (Corners_Index range <>) of Feedrate_Profile;

   --  Kinematic_Limiter
   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  N_Corners may be 0 or 1, in which case no movement should occur.
   type Execution_Block (N_Corners : Motion.Planner.Corners_Index := 0) is record
      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type with a discriminant to indicate the stage causes a stack overflow when trying to change
      --  the discriminant without making a copy as GCC tries to copy the whole thing to the stack. In the future we
      --  could possibly use SPARK to ensure stages do not touch fields that are not yet assigned.

      --  Having so many discriminated types here may seem like it will cause performance issues, but in practice it is
      --  faster than the same code without discriminated types (refer to the no-discriminated-records branch).

      --  Preprocessor
      Next_Master    : Master_Manager.Master;
      Corners        : Block_Plain_Corners (1 .. N_Corners);
      Segment_Limits : Block_Segment_Limits (2 .. N_Corners);

      --  Corner_Blender
      Beziers                     : Block_Beziers (1 .. N_Corners);
      Shifted_Corners             : Block_Shifted_Corners (1 .. N_Corners);
      Shifted_Corner_Error_Limits : Block_Shifted_Corner_Error_Limits (1 .. N_Corners);

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits (1 .. N_Corners);

      --  Feedrate_Profile_Generator
      Feedrate_Profiles : Block_Feedrate_Profiles (2 .. N_Corners);
   end record;

   --  TODO: It might make sense to create a custom type here that can hold n bytes rather than n records so lots of
   --  small blocks may be queued if the planner is flushed often.
   package Execution_Block_Queues_Interface is new Ada.Containers.Synchronized_Queue_Interfaces (Execution_Block);
   package Execution_Block_Queues is new Ada.Containers.Bounded_Synchronized_Queues
     (Execution_Block_Queues_Interface, 3);
   use Execution_Block_Queues;

   Execution_Block_Queue : Execution_Block_Queues.Queue;

   task Runner is
      entry Init (Conf : Config_Parameters);
   end Runner;

end Motion.Planner;
