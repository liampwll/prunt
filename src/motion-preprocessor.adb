with Ada.Containers.Bounded_Vectors;
with Ada.Containers;
use type Ada.Containers.Count_Type;
with Master_Manager; use Master_Manager;

package body Motion.Preprocessor is
   package Corner_Vectors is new Ada.Containers.Bounded_Vectors (Corners_Index, Scaled_Position);
   pragma Warnings (Off, "dimensions mismatch");
   --  Disabling this warning does not disable errors on dimension mismatch during assignment.
   package Velocity_Vectors is new Ada.Containers.Bounded_Vectors (Corners_Index, Velocity);
   pragma Warnings (Off, "dimensions mismatch");

   task body Runner is
      Config                 : Config_Parameters;
      Queued_Corners         : Corner_Vectors.Vector (Ada.Containers.Count_Type (Corners_Index'Last));
      Queued_Velocities      : Velocity_Vectors.Vector (Ada.Containers.Count_Type (Corners_Index'Last));
      Current_Corner         : Scaled_Position;
      Next_Block_Next_Master : Master_Manager.Master;
      Force_Early_End        : Boolean;

      procedure Processor (Data : in out Block_Data) is
      begin
         Data := (N_Corners => Corners_Index (Queued_Corners.Length), others => <>);

         for I in Queued_Corners.First_Index .. Queued_Corners.Last_Index loop
            Data.Corners (Data.Corners'First - Queued_Corners.First_Index + I) := Queued_Corners.Element (I);
         end loop;

         for I in Queued_Velocities.First_Index .. Queued_Velocities.Last_Index loop
            Data.Segment_Velocity_Limits (Data.Segment_Velocity_Limits'First - Queued_Velocities.First_Index + I) :=
              Queued_Velocities.Element (I);
         end loop;

         Data.Next_Master := Next_Block_Next_Master;
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      Current_Corner  := Config.Initial_Position * Config.Limit_Scaler;
      Force_Early_End := False;

      loop
         for Block_Index in Block_Queues_Index loop
            Next_Block_Next_Master := Motion_Master;
            Queued_Corners.Clear;
            Queued_Velocities.Clear;
            Queued_Corners.Append (Current_Corner);
            Force_Early_End := False;

            while not Force_Early_End and Queued_Corners.Length /= Queued_Corners.Capacity loop
               select
                  accept Enqueue (Pos : Position; Velocity_Limit : Velocity) do
                     declare
                        Next_Corner : Scaled_Position        := Pos * Config.Limit_Scaler;
                        Diff        : Scaled_Position_Offset := Next_Corner - Current_Corner;
                     begin
                        if Diff (X_Axis) = 0.0 and Diff (Y_Axis) = 0.0 and Diff (E_Axis) = 0.0 then
                           --  Z-only moves are a good time to allow pauses.
                           Force_Early_End := True;
                        end if;
                        Current_Corner := Next_Corner;
                     end;
                     Queued_Corners.Append (Current_Corner);
                     if Velocity_Limit > Config.Velocity_Limit then
                        Queued_Velocities.Append (Config.Velocity_Limit);
                     else
                        Queued_Velocities.Append (Velocity_Limit);
                     end if;
                  end Enqueue;
               or
                  accept Flush (Next_Master : Master_Manager.Master) do
                     Next_Block_Next_Master := Next_Master;
                     Force_Early_End        := True;
                  end Flush;
               end select;
            end loop;

            Block_Queue (Block_Index).Wait (Preprocessor_Stage);
            Block_Queue (Block_Index).Process (Preprocessor_Stage, Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Preprocessor;
