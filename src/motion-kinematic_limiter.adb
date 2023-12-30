package body Motion.Kinematic_Limiter is

   task body Runner is
      Config : Config_Parameters;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;
   end Runner;

end Motion.Kinematic_Limiter;
