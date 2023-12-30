with Physical_Types; use Physical_Types;
with Master_Manager;

package Motion is

   type Kinematics_Kind is (Cartesian_Kind, CoreXY_Kind);

   type Config_Parameters (Kinematics : Kinematics_Kind := Cartesian_Kind) is record
      Velocity_Limit      : Velocity;
      Acceleration_Limit  : Acceleration;
      Jerk_Limit          : Jerk;
      Snap_Limit          : Snap;
      Crackle_Limit       : Crackle;
      Limit_Scaler        : Position_Scale;
      Chord_Error_Limit   : Length;
      --  TODO: Replace with homing.
      Initial_Position    : Position;
      E_Distance_Per_Step : Length;
      Z_Distance_Per_Step : Length;
      case Kinematics is
         when Cartesian_Kind =>
            X_Distance_Per_Step : Length;
            Y_Distance_Per_Step : Length;
         when CoreXY_Kind =>
            A_Distance_Per_Step : Length;
            B_Distance_Per_Step : Length;
      end case;
   end record;

   procedure Init (Config : Config_Parameters);
   procedure Enqueue (Pos : Position; Velocity_Limit : Velocity);
   procedure Flush (Next_Master : Master_Manager.Master);

private

   type Block_Pipeline_Stages is
     (None_Stage,
      Preprocessor_Stage,
      Curvifier_Stage,
      Step_Splitter_Stage,
      Kinematic_Limiter_Stage,
      Acceleration_Profile_Generator_Stage,
      --  TODO: Replace with step generation.
      Logger_Stage);

   type Corners_Index is range 0 .. 2**8 - 1;

   --  Preprocessor

   --  Approximate distance between corners in steps. This is used so we can have a bounded max buffer size.
   Max_Corner_Distance : constant := 100_000;

   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  Curvifier

   Curvifier_Max_Computational_Error : Length := 0.000_1 * mm;

   type Bezier_Index is range 1 .. 10;
   type Bezier is array (Bezier_Index) of Scaled_Position;

   type Block_Beziers is array (Corners_Index range <>) of Bezier;
   type Block_Inverse_Curvatures is array (Corners_Index range <>) of Length;

   --  Step_Splitter

   --  TODO: Work out the worst case for this value.
   Max_Steps_Per_Segment : constant := 4 * Max_Corner_Distance;

   type Motor_Name is (A_Or_X_Motor, B_Or_Y_Motor, E_Motor, Z_Motor);
   type Motor_Direction is (Forward, Backward);

   type Step is record
      Motor              : Motor_Name;
      Direction          : Motor_Direction;
      --  The distance measured here is the straight-line distances between points on the curve where a step was
      --  generated. This seems to be a good compromise between the real length of the curve and the Manhattan distance
      --  of the actual steps.
      Distance_From_Last : Length;
   end record;

   type Step_Segment_Steps_Index is range 0 .. Max_Steps_Per_Segment;
   type Step_Segment_Steps is array (Step_Segment_Steps_Index range <>) of Step;

   type Step_Segment (N_Steps : Step_Segment_Steps_Index := 0) is record
      Steps : Step_Segment_Steps (1 .. N_Steps);
   end record;

   type Block_Step_Segments is array (Corners_Index range <>) of Step_Segment;

   --  Kinematic_Limiter

   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  Acceleration_Profile_Generator

   type Step_Time_Deltas_Times is array (Step_Segment_Steps_Index range <>) of Time;

   type Step_Time_Deltas_Segment (N_Steps : Step_Segment_Steps_Index := 0) is record
      Times : Step_Segment_Steps (1 .. N_Steps);
   end record;

   type Block_Step_Time_Deltas_Segments is array (Corners_Index range <>) of Step_Time_Deltas_Segment;

   --  End of pipeline stage types.

   --!pp off
   type Block_Data (Last_Stage : Block_Pipeline_Stages := None_Stage; N_Corners : Corners_Index := 0) is record
      case Last_Stage is
         when Preprocessor_Stage .. Block_Pipeline_Stages'Last =>
            Next_Master             : Master_Manager.Master;
            Corners                 : Block_Plain_Corners (1 .. N_Corners);
            Segment_Velocity_Limits : Block_Segment_Velocity_Limits (2 .. N_Corners);

      case Last_Stage is
         when Curvifier_Stage .. Block_Pipeline_Stages'Last =>
            Beziers            : Block_Beziers (1 .. N_Corners);
            Inverse_Curvatures : Block_Inverse_Curvatures (1 .. N_Corners);

      case Last_Stage is
         when Step_Splitter_Stage .. Block_Pipeline_Stages'Last =>
            Step_Segments : Block_Step_Segments (2 .. N_Corners);

      case Last_Stage is
         when Kinematic_Limiter_Stage .. Block_Pipeline_Stages'Last =>
            Corner_Velocity_Limits : Block_Corner_Velocity_Limits (2 .. N_Corners);

      case Last_Stage is
         when Acceleration_Profile_Generator_Stage .. Block_Pipeline_Stages'Last =>
            Step_Time_Deltas_Segments : Block_Step_Time_Deltas_Segments (2 .. N_Corners);

      case Last_Stage is
         when Logger_Stage .. Block_Pipeline_Stages'Last =>
            null;

         when others =>
            null;
      end case;
         when others =>
            null;
      end case;
         when others =>
            null;
      end case;
         when others =>
            null;
      end case;
         when others =>
            null;
      end case;
         when others =>
            null;
      end case;
   end record;
   --!pp on

   protected type Block is
      entry Process (Preprocessor_Stage .. Logger_Stage) (Processor : access procedure (Data : in out Block_Data));
   private
      Data : Block_Data := (Last_Stage => None_Stage, N_Corners => 0);
   end Block;

   type Block_Queues_Index is range 0 .. 3;
   type Block_Queues is array (Block_Queues_Index) of Block;
   type Block_Queues_Access is access Block_Queues;
   Block_Queue : constant Block_Queues_Access := new Block_Queues;

end Motion;
