
MODULE user (SYSMODULE)

  ! Predefined user data
 !*********************

 ! Declaration of numeric registers reg1...reg5
 VAR num reg1 := 0;
 VAR num reg2 := 0;
 VAR num reg3 := 0;
 VAR num reg4 := 0;
 VAR num reg5 := 0;

 ! Declaration of stopwatch clock1
 VAR clock clock1;
 PERS tooldata tPince_bout:=[TRUE,[[0,0,233.37],[0.707107,0,0,-0.707107]],[1.7,[12.2,0,158],[1,0,0,0],0.009,0.003,0.012]];
 PERS tooldata tPince_bloc:=[TRUE,[[0,0,220],[0.707107,0,0,-0.707107]],[1.7,[12.2,0,158],[1,0,0,0],0.009,0.003,0.012]];
 PERS tooldata tSuceG:=[TRUE,[[-38.1,42.545,222.174],[0.653281,-0.270598,-0.270598,-0.653281]],[1.7,[12.2,0,158],[1,0,0,0],0,0,0]];
 PERS tooldata tSuceD:=[TRUE,[[38.1,42.545,222.174],[0.653281,-0.270598,-0.270598,-0.653281]],[1.7,[12.2,0,158],[1,0,0,0],0,0,0]];
 PERS tooldata tsuceCentre:=[TRUE,[[0,42.545,222.174],[0.653281,-0.270598,-0.270598,-0.653281]],[1.7,[12.2,0,158],[1,0,0,0],0,0,0]];
 PERS tooldata tCrayon:=[TRUE,[[0,0,283.37],[0.707107,0,0,-0.707107]],[1.7,[12.2,0,158],[1,0,0,0],0.009,0.003,0.012]];
 
 PERS tooldata toolDataEnCours:=[TRUE,[[0,0,220],[0.707107,0,0,-0.707107]],[1.7,[12.2,0,158],[1,0,0,0],0.009,0.003,0.012]];
 
 !PERS tooldata tCrayon:=[TRUE,[[0,0,283.37],[0.707107,0,0,-0.707107]],[1.7,[12.2,0,158],[1,0,0,0],0.009,0.003,0.012]];

 ! Template for declaration of workobject wobj1
 !TASK PERS wobjdata wobj1 := [FALSE, TRUE, "", [[0, 0, 0],[1, 0, 0, 0]],[[0, 0, 0],[1, 0, 0, 0]]];


ENDMODULE
