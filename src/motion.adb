with Motion.Preprocessor;
with Motion.Curvifier;
with Motion.Step_Splitter;
with Motion.Kinematic_Limiter;
with Motion.Acceleration_Profile_Generator;
with Motion.Logger;
with Ada.Exceptions;
with Ada.Text_IO; use Ada.Text_IO;
with Ada.Task_Identification;

package body Motion is

   procedure Init (Config : Config_Parameters) is
   begin
      Motion.Preprocessor.Runner.Init (Config);
      Motion.Curvifier.Runner.Init (Config);
      Motion.Step_Splitter.Runner.Init (Config);
      Motion.Kinematic_Limiter.Runner.Init (Config);
      Motion.Acceleration_Profile_Generator.Runner.Init (Config);
      Motion.Logger.Runner.Init (Config);
   end Init;

   procedure Enqueue (Pos : Position; Velocity_Limit : Velocity) is
   begin
      Motion.Preprocessor.Runner.Enqueue (Pos, Velocity_Limit);
   end Enqueue;

   procedure Flush (Next_Master : Master_Manager.Master) is
   begin
      Motion.Preprocessor.Runner.Flush (Next_Master);
   end Flush;

   protected body Block is

      entry Process
        (for Stage in Preprocessor_Stage .. Logger_Stage) (Processor : access procedure (Data : in out Block_Data))
        when Block_Pipeline_Stages'Pred (Stage) = Data.Last_Stage
      is
      begin
         Processor (Data);
      exception
         when E : others =>
            Put_Line ("Exception in motion pipeline:");
            Put_Line (Ada.Exceptions.Exception_Information (E));
            Put_Line ("Terminating task " & Ada.Task_Identification.Image (Process'Caller));
            Ada.Task_Identification.Abort_Task (Process'Caller);
      end Process;

   end Block;

end Motion;
