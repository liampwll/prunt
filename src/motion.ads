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
      Curve_Splitter_Stage,
      Kinematic_Limiter_Stage,
      Acceleration_Profile_Generator_Stage,
      --  TODO: Replace with step generation.
      Logger_Stage);

   type Corners_Index is range 0 .. 2_000;

   --  Preprocessor

   type Block_Plain_Corners is array (Corners_Index range <>) of Scaled_Position;
   type Block_Segment_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  Curvifier

   Curvifier_Max_Computational_Error : Length := 0.000_1 * mm;

   type Bezier_Index is range 1 .. 10;
   type Bezier is array (Bezier_Index) of Scaled_Position;

   type Block_Beziers is array (Corners_Index range <>) of Bezier;
   type Block_Inverse_Curvatures is array (Corners_Index range <>) of Length;

   --  Curve_Splitter

   Curve_Points_Per_Side : constant := 1_000;
   type Curve_Point_Set_Index is range -Curve_Points_Per_Side .. Curve_Points_Per_Side;
   type Curve_Point_Set is array (Curve_Point_Set_Index) of Scaled_Position;

   type Block_Curve_Point_Sets is array (Corners_Index range <>) of Curve_Point_Set;

   --  Kinematic_Limiter

   type Block_Corner_Velocity_Limits is array (Corners_Index range <>) of Velocity;

   --  Acceleration_Profile_Generator

   type Acceleration_Profile_Times_Index is range 1 .. 4;
   type Acceleration_Profile_Times is array (Acceleration_Profile_Times_Index) of Time;

   type Segment_Acceleration_Profile is record
      Accel : Acceleration_Profile_Times;
      Coast : Time;
      Decel : Acceleration_Profile_Times;
   end record;

   type Block_Segment_Acceleration_Profiles is array (Corners_Index range <>) of Segment_Acceleration_Profile;

   --  End of pipeline stage types.

   type Block_Data (N_Corners : Corners_Index := 0) is record
      Last_Stage : Block_Pipeline_Stages := None_Stage;

      --  TODO: Having all these fields accessible before the relevant stage is called is not ideal, but using a
      --  discriminated type causes a stack overflow when trying to change the discriminant without making a copy as
      --  GCC tries to copy the whole thing to the stack. In the future we could possibly use SPARK to ensure stages do
      --  not touch fields that are not yet assigned.

      --  Preprocessor
      Next_Master             : Master_Manager.Master;
      Corners                 : Block_Plain_Corners (1 .. N_Corners);
      Segment_Velocity_Limits : Block_Segment_Velocity_Limits (2 .. N_Corners);

      --  Curvifier
      Beziers            : Block_Beziers (1 .. N_Corners);
      Inverse_Curvatures : Block_Inverse_Curvatures (1 .. N_Corners);

      --  Curve_Splitter
      Curve_Point_Sets : Block_Curve_Point_Sets (1 .. N_Corners);

      --  Kinematic_Limiter
      Corner_Velocity_Limits : Block_Corner_Velocity_Limits (1 .. N_Corners);

      --  Acceleration_Profile_Generator
      Segment_Acceleration_Profiles : Block_Segment_Acceleration_Profiles (2 .. N_Corners);
   end record;

   protected type Block is
      entry Wait (Preprocessor_Stage .. Logger_Stage);
      procedure Process (Stage : Block_Pipeline_Stages; Processor : access procedure (Data : in out Block_Data));
   private
      Data : Block_Data := (Last_Stage => None_Stage, N_Corners => 0, others => <>);
   end Block;

   type Block_Queues_Index is range 1 .. 8;
   type Block_Queues is array (Block_Queues_Index) of Block;
   type Block_Queues_Access is access Block_Queues;
   Block_Queue : constant Block_Queues_Access := new Block_Queues;

   function Compute_Bezier_Point (Bez : Bezier; T : Dimensionless) return Scaled_Position;

   function Curve_Corner_Distance (Start, Finish : Curve_Point_Set) return Length;

   function Crackle_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Crackle;
   function Snap_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Snap;
   function Jerk_At_Time (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Jerk;
   function Acceleration_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle) return Acceleration;
   function Velocity_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle; Start_Vel : Velocity) return Velocity;
   function Distance_At_Time
     (T : Time; Profile : Acceleration_Profile_Times; Crackle_Limit : Crackle; Start_Vel : Velocity) return Length;

end Motion;
