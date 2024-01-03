with Ada.Text_IO; use Ada.Text_IO;

package body Motion.Curve_Splitter is

   task body Runner is
      Config : Config_Parameters;

      procedure Processor (Data : in out Block_Data) is
      begin
         pragma Assert (Data.Last_Stage = Curvifier_Stage);

         for I in Data.Curve_Point_Sets'Range loop
            for J in Curve_Point_Set'Range loop
               Data.Curve_Point_Sets (I) (J) :=
                 Compute_Bezier_Point
                   (Data.Beziers (I),
                    (Dimensionless (J) + Dimensionless (Curve_Points_Per_Side)) /
                    Dimensionless (Curve_Points_Per_Side * 2));
            end loop;
         end loop;
      end Processor;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;

      loop
         for Block_Index in Block_Queues_Index loop
            Block_Queue (Block_Index).Process (Curve_Splitter_Stage) (Processor'Access);
         end loop;
      end loop;
   end Runner;

end Motion.Curve_Splitter;
