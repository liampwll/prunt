private package Motion.Preprocessor is

   task Runner is
      entry Init (In_Config : Config_Parameters);
      entry Enqueue (Pos : Position; Velocity_Limit : Velocity);
      entry Flush (Next_Master : Master_Manager.Master);
   end Runner;

end Motion.Preprocessor;
