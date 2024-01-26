private package Motion.Executor is

   task Runner is
      entry Init (Conf : Config_Parameters);
   end Runner;

   Logger_Interpolation_Time : constant Time := 0.000_2 * s;

end Motion.Executor;
