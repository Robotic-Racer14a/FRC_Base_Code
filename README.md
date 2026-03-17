This is the base code used by FRC Team 1322 the GRAYT Leviathons created by me (Alex Clute, FRC 2337 Alumni, FRC 1322 and FTC 9933, 17064, and 25312 Mentor, FTC and FRC CSA)

Important notes
  - Built off of CTRE Swerve Generator, with simplifications made for clarity
  - Uses a SystemVariables class for handshaking and for holding Constants classes
  - Includes custom DriveToPose Command which has two modes, continous and fine
    - Pure-Pursuit Model, simplifes process of teaching and executing, but is worse at object avoidance
    - Created by allowing for an array of DriveToPoseObjects,
      which each include a pose, as well as optional parameters for max speed of that move
      and distance until bypass, where it will move towards a target until it gets within that range (or will stop at that pose if distance is zero)
