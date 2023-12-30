package body Motion.Logger is

   task body Runner is
      Config : Config_Parameters;
   begin
      accept Init (In_Config : Config_Parameters) do
         Config := In_Config;
      end Init;
   end Runner;

end Motion.Logger;
