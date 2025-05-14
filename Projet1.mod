MODULE Lab1
! ----------------------------------------------------------------------------
! Programme : GPA546Lab1
! Auteurs : _______________ ET ______________
! Date : _________________
! R�vision : ________________
!
! Description :
! Ce programme permet de prendre deux blocs dans une glissoire et de les
! superposer sur une table de travail.
! ----------------------------------------------------------------------------

  ! Donnees de type position enseignees
  PERS robtarget rGlissoire:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rDepot:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rRetrait:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  ! Donnees de type position calculees
  VAR robtarget rDepot2;

  ! Donnees de type constante
  CONST num Epaisseur:=_____; ! �paisseur d'un bloc (en pouces)
  CONST num PouceToMM:=25.4; ! Facteur de conversion
  CONST num Decalage:=_____; ! Distance d'approche ou de retrait (mm)

  ! Donn�es de type Variable
  VAR num EpaisMM:=0;

  ! Vitesse d'approche et de retrait (mm/sec)
  CONST speeddata LowSpeed:=[250,500,5000,1000];
  
  ! Vitesse maximale du robot (mm/sec)
  CONST speeddata HighSpeed:=[1000,500,5000,1000];

  ! Etat des entrees/sorties
  CONST dionum Ouverte:=0;
  CONST dionum Fermee:=1;
  CONST dionum Retracte:=0;
  CONST dionum Extension:=1;

  ! Variable pour limiter la zone de travail du robot
  VAR wztemporary EspaceRestreint;

! ----------------------------------------------------------------------------
! Procedure : main
! Auteurs : Yanick Noiseux
! Date : 4-08-2011
! R�vision : 1.1
! R�vision par: Martin Gaudreault
! Description :
! Routine qui est executee par le robot initialement.
! **** elle doit �tre en mode "cycle"
!
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC main()

	! 1) Initialisation :
	init;

	! Deplacement a la position de repos
	MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;

	! 2) Calculs :
	EpaisMM := Epaisseur * PouceToMM; ! Conversion en mm
	rDepot2 := Offs(rDepot,0,0,______);

	! 3) D�placement du premier bloc :
	! Prise du bloc dans le glissoire
	Prise;
	! D�p�t du bloc 1 � la localisation rDepot :
	Depot(______);

	! 4) D�placement du deuxi�me bloc :
	! Prise du bloc dans le glissoire
	! ************* A compl�ter... *************
	! D�p�t du bloc 2 � la localisation rDepot :
	! ************* A compl�ter... *************

	! 5) Retourner le robot a la position de repos
	MoveJ ____, ____, fine, tPince_bloc______;
	
	Pince\Ouvert; ! Ouvrir la pince

ENDPROC
!**************************************************************************************

! ----------------------------------------------------------------------------
! Procedure : init
! Auteurs : Yanick Noiseux
! Date : 4-08-2011
! R�vision : 1.1
! R�vision par: Martin Gaudreault
! Description :
! Routine qui initialise les variables syst�mes et les �tats des sorties
!
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC init()

	! volume pour limiter les articulation du robot
	VAR shapedata joint_space;
	
	! Ne garder que les deux constantes correspondant a votre robot
	! Ces valeurs ne doivent pas �tre chang�es afin d'aider a la securite et repr�sentent
    ! les limites logicielles pour chacun des axes
	
	! *** d�commentez ces deux lignes si vous �tes sur le robot 1
	!CONST jointtarget low_pos:=[[ 54, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ 128,  55,  55,  90,  103,  109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** d�commentez ces deux lignes si vous �tes sur le robot 2
	!CONST jointtarget low_pos:=[[ -134, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ -65,  55,  55,  90,  103,  109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** d�commentez ces deux lignes si vous �tes sur le robot 3
	!CONST jointtarget low_pos:=[[ 44, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ 118,  55,  55,  90,  103,  109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** d�commentez ces deux lignes si vous �tes sur le robot 4
	!CONST jointtarget low_pos:=[[ -104, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ -28,  55,  55,  90,  103,  109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];

	! activation de la limitation, obligatoire
	WZFree EspaceRestreint;
	WZLimJointDef \Outside, joint_space, low_pos, high_pos;
	WZLimSup \Temp, EspaceRestreint, joint_space;

	VelSet 25,1000; ! Limitation impos�e pour la s�curit�, obligatoire

	SetDO DO09_FV0101, Retracte; ! Rentrer le v�rin
	Pince\Ouvert; ! Ouvrir la pince

ENDPROC
!**************************************************************************************

! ----------------------------------------------------------------------------
! Procedure : Prise
! Auteurs : Yanick Noiseux
! Date : 4-08-2011
! R�vision : 1.0
!
! Description :
! Routine qui s'approche du glissoire, atends une piece, 
! l'indexe et la prends de maniere securitaire
!
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Prise()

! V�rification de pr�sence de bloc dans la glissoire
	IF DI09_ZS0101 = 0 THEN
		TPErase;
		TPWrite "Aucun bloc dans la glissoire";
		TPWrite "Placer deux bloc et remettre en mode AUTO";
		WaitDI DI09_ZS0101, 1; ! Attente d'un bloc
		Waittime 1; ! attente que le bloc se stabilise
	ENDIF

	! Prehension du bloc
	MoveJ RelTool(rGlissoire,0,0,Decalage),HighSpeed,z50,tPince_bloc\wobj:=wobj0;
	SetDO DO09_FV0101, Extension; ! Indexer le bloc durant le mouvement
	WAITDI DI14_ZS0104,1; ! Attendre pour le v�rin sortie
	MoveL rGlissoire, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
	Pince\Fermer; ! Fermer la pince
	SetDO DO09_FV0101, Retracte;
	WAITDI DI14_ZS0104,0;
	MoveL RelTool(rGlissoire,0,0,Decalage),LowSpeed,z50,tPince_bloc\wobj:=wobj0;
	
ENDPROC
!**************************************************************************************

! ----------------------------------------------------------------------------
! Procedure : Depot
! Auteurs : Yanick Noiseux
! Date : 4-08-2011
! R�vision : 1.1
! R�vision par: Martin Gaudreault
! Description :
! Routine qui initialise les variables syst�mes et les �tats des sorties
!
! Parametre: recoit un robtarget o� aller d�poser le bloc avec aproche et degagement
!
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Depot(robtarget rPosDepot)

	MoveJ RelTool(___,___,___,___), ____, z50, tPince_bloc\wobj:=wobj0;
	MoveL rPosDepot, ____, fine, tPince_bloc;
	Pince\Ouvert; ! Ouvrir la pince
	MoveL RelTool(___,___,___,___), ____, z50, tPince_bloc\wobj:=wobj0;

ENDPROC
!**************************************************************************************

! ----------------------------------------------------------------------------
! Procedure : Pince
! Auteurs : Yanick Noiseux
! Date : 4-08-2011
! R�vision : 1.1
! R�vision par: Martin Gaudreault
! Description :
! Routine qui D�fini l'�tat de l'outil pince du robot
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Pince(\switch Ouvert | switch Fermer)

    ! Action sur la pince
    IF Present(Fermer) THEN
		! verifie si la sortie doit-�tre chang� d'�tat
		IF Doutput(DO01_EE_PINCE01)=0 THEN
       		Set DO01_EE_PINCE01; ! Ferme la pince
			WAITTIME 1;  !  Attente que la pince bouge physiquement
		ENDIF
    ELSE
		! verifie si la sortie doit-�tre chang� d'�tat
		IF Doutput(DO01_EE_PINCE01)=1 THEN
       		Reset DO01_EE_PINCE01; ! Ferme la pince
			WAITTIME 1;  !  Attente que la pince bouge physiquement
		ENDIF
    ENDIF
	
ENDPROC
!**************************************************************************************
ENDMODULE
