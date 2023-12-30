with Ada.Text_IO; use Ada.Text_IO;

package body Motion.Step_Splitter is

   task body Runner is
      Config : Config_Parameters;

      function Compute_Bezier_Point (Bez : Bezier; T : Dimensionless) return Scaled_Position is
         type Partial_Bezier is array (Bezier_Index range <>) of Scaled_Position;

         function Recur (Bez : Partial_Bezier) return Scaled_Position is
            Next_Bez : constant Partial_Bezier :=
              [for I in Bez'First .. Bez'Last - 1 => Bez (I) + (Bez (I + 1) - Bez (I)) * T];
         begin
            if Next_Bez'Length = 1 then
               return Next_Bez (Bezier_Index'First);
            else
               return Recur (Next_Bez);
            end if;
         end Recur;

      begin
         return Recur ([for I in Bezier_Index => Bez (I)]);
      end Compute_Bezier_Point;

      procedure Processor (Data : in out Block_Data) is
      begin
         pragma Assert (Data.Last_Stage = Curvifier_Stage);

         for I in Data.Beziers'Range loop
            for T in 0 .. 100 loop
               Put (Length'Image (Compute_Bezier_Point (Data.Beziers (I), Dimensionless (T) / 100.0) (X_Axis)));
               Put (",");
               Put_Line (Length'Image (Compute_Bezier_Point (Data.Beziers (I), Dimensionless (T) / 100.0) (Y_Axis)));
            end loop;
         end loop;
         
         Data := (Last_Stage => None_Stage, N_Corners => 0);
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Process (Step_Splitter_Stage) (Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Step_Splitter;
