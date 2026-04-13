MATLAB 3DOF TRAJECTORY OPTIMIZATION PACKAGE

Files included:
1. runOptimization.m
2. runNominalSimulation.m
3. getDefaultParameters.m
4. simulateTrajectory.m
5. objectiveTrajectory.m
6. nonlinearConstraints.m
7. pointMass3DOF_opt.m
8. controlSchedule_opt.m
9. aeroModel_opt.m
10. atmosphereDensity_opt.m
11. speedOfSound_opt.m
12. thrustProfile_opt.m
13. massFlowRate_opt.m
14. groundEvent_opt.m
15. plotTrajectoryResults.m

How to use:
1. Put all .m files in one MATLAB folder.
2. Run:
   runNominalSimulation
   to validate the corrected model first.
3. Then run:
   runOptimization
   to perform trajectory optimization.

Main fixes relative to the uploaded code:
- qbar, L, D are now always computed consistently
- speed of sound is altitude-dependent instead of constant
- duplicated / inconsistent plotting variables removed
- simulation is wrapped into a clean optimizer-facing function
- objective and nonlinear constraint functions added
- decision variables exposed for fmincon
