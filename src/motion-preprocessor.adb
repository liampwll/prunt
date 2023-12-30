with Ada.Containers.Bounded_Vectors;
with Ada.Containers;
use type Ada.Containers.Count_Type;
with Ada.Numerics.Generic_Elementary_Functions;
with Master_Manager; use Master_Manager;
with Ada.Text_IO;    use Ada.Text_IO;

package body Motion.Preprocessor is
   package Corner_Vectors is new Ada.Containers.Bounded_Vectors (Corners_Index, Scaled_Position);
   pragma Warnings (Off, "dimensions mismatch");
   --  Disabling this warning does not disable errors on dimension mismatch during assignment.
   package Velocity_Vectors is new Ada.Containers.Bounded_Vectors (Corners_Index, Velocity);
   pragma Warnings (Off, "dimensions mismatch");

   package Dimensioned_Float_Math is new Ada.Numerics.Generic_Elementary_Functions (Dimensioned_Float);
   use Dimensioned_Float_Math;

   task body Runner is
      Config                 : Config_Parameters;
      Queued_Corners         : Corner_Vectors.Vector (Ada.Containers.Count_Type (Corners_Index'Last));
      Queued_Velocities      : Velocity_Vectors.Vector (Ada.Containers.Count_Type (Corners_Index'Last));
      Current_Corner         : Scaled_Position;
      Target_Corner          : Scaled_Position;
      Target_Velocity        : Velocity;
      Next_Block_Next_Master : Master_Manager.Master;
      Flush_Requested        : Boolean;

      function Compute_Steps_Upper_Bound (V : Scaled_Position_Offset) return Dimensionless is
      begin
         case Config.Kinematics is
            when Cartesian_Kind =>
               return
                 Sqrt
                   ((V (X_Axis) / Config.X_Distance_Per_Step)**2 + (V (Y_Axis) / Config.Y_Distance_Per_Step)**2 +
                    (V (E_Axis) / Config.E_Distance_Per_Step)**2 + (V (Z_Axis) / Config.Z_Distance_Per_Step)**2);
            when CoreXY_Kind =>
               --  TODO: Do this properly.
               return
                 Sqrt
                   ((V (X_Axis) / Length'Min (Config.A_Distance_Per_Step, Config.B_Distance_Per_Step))**2 +
                    (V (Y_Axis) / Length'Min (Config.A_Distance_Per_Step, Config.B_Distance_Per_Step))**2 +
                    (V (E_Axis) / Config.E_Distance_Per_Step)**2 + (V (Z_Axis) / Config.Z_Distance_Per_Step)**2);
         end case;
      end Compute_Steps_Upper_Bound;

      procedure Processor (Data : in out Block_Data) is
      begin
         Data :=
           (Last_Stage              => Preprocessor_Stage,
            N_Corners               => Corners_Index (Queued_Corners.Length),
            Next_Master             => Next_Block_Next_Master,
            Corners                 => <>,
            Segment_Velocity_Limits => <>);

         for I in Queued_Corners.First_Index .. Queued_Corners.Last_Index loop
            Data.Corners (Data.Corners'First - Queued_Corners.First_Index + I) := Queued_Corners.Element (I);
         end loop;

         for I in Queued_Velocities.First_Index .. Queued_Velocities.Last_Index loop
            Data.Segment_Velocity_Limits (Data.Segment_Velocity_Limits'First - Queued_Velocities.First_Index + I) :=
              Queued_Velocities.Element (I);
         end loop;
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      Current_Corner  := Config.Initial_Position * Config.Limit_Scaler;
      Target_Corner   := Current_Corner;
      Target_Velocity := 0.0 * mm / s;

      loop
         for Block_Index in Block_Queues_Index loop
            Next_Block_Next_Master := Motion_Master;
            Queued_Corners.Clear;
            Queued_Velocities.Clear;
            Flush_Requested := False;
            Queued_Corners.Append (Current_Corner);

            while not Flush_Requested and then Queued_Corners.Length /= Queued_Corners.Capacity loop
               if Current_Corner /= Target_Corner then
                  if Compute_Steps_Upper_Bound (Current_Corner - Target_Corner) > Dimensionless (Max_Corner_Distance)
                  then
                     --  Move Current_Corner towards Target_Corner by Max_Corner_Distance.
                     Current_Corner :=
                       @ +
                       (Current_Corner - Target_Corner) *
                         (Dimensionless (Max_Corner_Distance) /
                          Compute_Steps_Upper_Bound (Current_Corner - Target_Corner));
                  else
                     Current_Corner := Target_Corner;
                  end if;
                  Queued_Corners.Append (Current_Corner);
                  Queued_Velocities.Append (Target_Velocity);
               else
                  select
                     accept Enqueue (Pos : Position; Velocity_Limit : Velocity) do
                        Target_Corner   := Pos * Config.Limit_Scaler;
                        Target_Velocity := Velocity_Limit;
                        if Velocity_Limit > Config.Velocity_Limit then
                           Target_Velocity := Config.Velocity_Limit;
                        end if;
                     end Enqueue;
                  or
                     accept Flush (Next_Master : Master_Manager.Master) do
                        Next_Block_Next_Master := Next_Master;
                        Flush_Requested        := True;
                     end Flush;
                  end select;
               end if;
            end loop;

            Block_Queue (Block_Index).Process (Preprocessor_Stage) (Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Preprocessor;
