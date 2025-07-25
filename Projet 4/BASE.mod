MODULE BASE (SYSMODULE, NOSTEPIN, VIEWONLY)

 ! System module with basic predefined system data
 !**

 ! System data tool0, wobj0 and load0
 ! Do not translate or delete tool0, wobj0, load0
 PERS tooldata tool0 := [TRUE, [[0, 0, 0], [1, 0, 0, 0]],
                        [0.001, [0, 0, 0.001],[1, 0, 0, 0], 0, 0, 0]];

 PERS wobjdata wobj0 := [FALSE, TRUE, "", [[0, 0, 0],[1, 0, 0, 0]],
                        [[0, 0, 0],[1, 0, 0, 0]]];

 PERS loaddata load0 := [0.001, [0, 0, 0.001],[1, 0, 0, 0], 0, 0, 0];

ENDMODULE
