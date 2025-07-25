MODULE Lab_4
! -------------------------------------------------------------------------------------
! Programme : GPA546Lab4
! Auteurs   : Olivier Millette ET Camilo Serna
! Date      : 4 août 2025
! Révision  : V0.0
!
! Description :
! -------------------------------------------------------------------------------------

  ! Positions pré-définies VRAI ROBOT
!  PERS robtarget rPriseGli:=[[-660.73,-1058.86,419.28],[0.208833,0.621189,-0.717118,-0.237182],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!  PERS robtarget rDepotGli:=[[-851.42,-914.68,568.33],[0.211183,0.614,-0.723282,-0.235094],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!  PERS robtarget rDepot:=[[1.48,-964.60,368.18],[0.0173768,0.643389,-0.765217,0.0138353],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!  PERS robtarget rRetrait:=[[-380.03,-976.73,740.33],[0.0571998,0.646818,-0.758565,-0.0541672],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!  PERS robtarget rCrayon:=[[-460.90,-711.35,486.39],[0.00396694,-0.645432,0.763775,0.00698067],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
!  PERS robtarget rSoudure:=[[-66.69,1008.93,496.08],[0.0114044,-0.709903,-0.704128,-0.0105808],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  PERS robtarget rPriseGli:=[[-248.62,671.10,350.13],[0.204826,-0.685379,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rDepotGli:=[[-432.49,814.95,513.48],[0.204826,-0.68538,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rDepot:=[[201.58,790.42,312.21],[0.00756763,-0.709954,-0.704175,-0.00677525],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rRetrait:=[[201.58,790.42,648.19],[0.0075676,-0.709954,-0.704175,-0.0067752],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rCrayon:=[[-66.69,1008.93,431.12],[0.0114044,-0.709903,-0.704128,-0.0105808],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rFeuille_x:=[[0.11,669.58,341.32],[0.0154746,0.916956,0.0408178,0.396593],[1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rFeuille_y:=[[0.11,883.12,341.32],[0.0154747,0.916956,0.0408176,0.396593],[1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  ! Orientation Crayon insiede this one:
  ! PERS robtarget rDepot:=[[290.71,780.85,391.27],[0.00116785,-0.919386,0.00812619,-0.39327],[0,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  ! Position calculée
  VAR robtarget rDepot_new;
  VAR robtarget rFeuille_z;

  ! Constantes
  CONST num Epaisseur   := 1;     ! Épaisseur d'un bloc (en pouces)
  CONST num PouceToMM   := 25.4;  ! Facteur de conversion pouce ? mm
  CONST num Decalage    := -200;  ! Distance d'approche/retrait (mm) (+X pour marge de sécurité)

  ! Variables
  VAR num EpaisMM := Epaisseur * PouceToMM;
  VAR num angleDeg := 45;
  VAR pos pos_rRetrait;
  VAR pos pos_Actuelle;
  VAR num calcDistance;

  ! Vitesses d'approche/retrait (mm/s)
  CONST speeddata LowSpeed:=[250,500,5000,1000];
  ! Vitesse max robot (mm/s)
  CONST speeddata HighSpeed:=[1000,500,5000,1000];

  ! États I/O
  CONST dionum Ouverte   := 0;
  CONST dionum Fermee    := 1;
  CONST dionum Retracte  := 0;
  CONST dionum Extension := 1;

  ! Zone de travail restreinte du robot
  VAR wztemporary EspaceRestreint;
  
 ! ----------------------------------------------------------------------------
 ! ----------------------------------------------------------------------------

 ! Sorties numériques (DO) ALIASES
 VAR signaldo pinceFermer;          ! DO01_EE_PINCE01  - Commande de fermeture/ouverture de la pince
 VAR signaldo ventouse1Succion;     ! DO02_EE_VENT01   - Vide (ventouse 1)
 VAR signaldo ventouse2Succion;     ! DO03_EE_VENT02   - Vide (ventouse 2)
 VAR signaldo lampeBleue;           ! DO04_EE_LampBlu  - Lampe bleue (statut)
 VAR signaldo lampeOrange;          ! DO05_EE_LampOr   - Lampe orange (avertissement)
 VAR signaldo verinExtension;       ! DO09_FV0101      - Commande d'extension/rétraction du vérin glissoire

! Entrées numériques (DI) ALIASES
 VAR signaldi objetSurVentouse1;    ! DI01_EE_VAC01    - Objet détecté sur ventouse 1
 VAR signaldi objetSurVentouse2;    ! DI02_EE_VAC02    - Objet détecté sur ventouse 2
 VAR signaldi blocPresentPoussoir;  ! DI09_ZS0101      - Bloc présent dans la glissoire (poussoir)
 VAR signaldi blocOriente;          ! DI10_ZS0102      - Bloc correctement orienté
 VAR signaldi blocBasNiveau;        ! DI11_LSL0101     - Bas niveau de blocs dans la glissoire
 VAR signaldi blocHautNiveau;       ! DI12_LSH0101     - Haut niveau de blocs dans la glissoire
 VAR signaldi verinRetracte;        ! DI13_ZS0103      - Fin de course : vérin rétracté
 VAR signaldi verinEtendu;          ! DI14_ZS0104      - Fin de course : vérin étendu
 VAR signaldi boutonSoudureDI;      ! DI_Virtuel1_Bouton1 - bouton programmable 1
 
!*************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
VAR intnum soudureInterrupt;   ! Any unused interrupt number
VAR bool soudureDemandee := FALSE;
VAR bool lampeBleueActive := FALSE;

TRAP DemandeSoudure
    soudureDemandee := TRUE;
    ISleep soudureInterrupt;
    TPWrite "Soudure demandée! Sera exécutée après ce bloc.";
ENDTRAP

PROC SimulerSoudure()
    VAR num angle := 60 * 3.14159265 / 180; ! 60° en radians
    VAR pos dirRot;
    VAR pos vecXY;
    VAR pos dirXY;
    VAR robtarget rFeuille_1;
    VAR robtarget rFeuille_2;
    VAR num vecNorm;

    LeCrayon\Prise;

    MoveJ RelTool(rFeuille_x,0,0,Decalage), HighSpeed, z50, tCrayon\wobj:=wobj0;
    MoveJ RelTool(rFeuille_x,0,0,Decalage \Rx := angleDeg), LowSpeed, z50, tCrayon\wobj:=wobj0;
    SetDO lampeOrange, 1;
    WaitTime 1;

    ! ------- Triangle motion begins here -------
    vecXY := [rFeuille_y.trans.x - rFeuille_x.trans.x,
              rFeuille_y.trans.y - rFeuille_x.trans.y,
              rFeuille_y.trans.z - rFeuille_x.trans.z];

    vecNorm := Sqrt(Pow(vecXY.x,2) + Pow(vecXY.y,2) + Pow(vecXY.z,2));
    dirXY.x := vecXY.x / vecNorm;
    dirXY.y := vecXY.y / vecNorm;
    dirXY.z := vecXY.z / vecNorm;

    rFeuille_1 := rFeuille_x;
    rFeuille_1.trans := rFeuille_x.trans + 50 * dirXY;

    dirRot.x := dirXY.x * Cos(angle) - dirXY.y * Sin(angle);
    dirRot.y := dirXY.x * Sin(angle) + dirXY.y * Cos(angle);
    dirRot.z := dirXY.z;

    rFeuille_2 := rFeuille_1;
    rFeuille_2.trans := rFeuille_1.trans + 50 * dirRot;

    rFeuille_z := rFeuille_2;

    MoveL rFeuille_1, LowSpeed, z10, tCrayon\WObj:=wobj0;
    MoveL rFeuille_2, LowSpeed, z10, tCrayon\WObj:=wobj0;
    MoveL rFeuille_x, LowSpeed, z10, tCrayon\WObj:=wobj0;
    ! ------- Triangle motion ends here -------

    LeCrayon\Deposer;
    soudureDemandee := FALSE;
    SetDO lampeOrange, 0;
    IWatch soudureInterrupt;
ENDPROC


PROC LeCrayon(\switch Prise | switch Deposer)
    MoveJ RelTool(rCrayon,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
    MoveL rCrayon, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 0.5;
    IF Present(Prise) THEN
        Pince\Fermer;
    ELSE
        Pince\Ouvert;
    ENDIF
    MoveL RelTool(rCrayon,0,0,Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;
ENDPROC

PROC SurveillanceProximite()
    pos_rRetrait := rRetrait.trans;
    pos_Actuelle := CPos(\Tool:=tPince_bloc, \WObj:=wobj0);
    calcDistance := Distance(pos_Actuelle, pos_rRetrait);

    IF calcDistance < 150 THEN
        IF NOT lampeBleueActive THEN
            SetDO lampeBleue, 1;
            lampeBleueActive := TRUE;
            TPWrite "Trop proche de rRetrait (<150 mm)";
        ENDIF
    ELSE
        IF lampeBleueActive THEN
            SetDO lampeBleue, 0;
            lampeBleueActive := FALSE;
            TPWrite "Éloigné de rRetrait (>150 mm)";
        ENDIF
    ENDIF
ENDPROC


!*************************************************************************************
! ------------------------------------------------------------------------------
!**************************************************************************************
PROC main()
    
	! Initialisation
    configIO;           ! Mappage I/O + contrôles
	init;               ! Configurations initiales
!    Start SurveillanceProximite;
    CONNECT soudureInterrupt WITH DemandeSoudure;
    ISignalDI boutonSoudureDI, low, soudureInterrupt;

    verPositionAxes;    ! Vérification de positionnement des axes
    
    ! Calculs
	EpaisMM := Epaisseur * PouceToMM;          ! Conversion en mm
    
    WHILE TRUE DO
        SimulerSoudure;
    ENDWHILE
    
!    ! Movement des blocs
!    WHILE TRUE DO
!        Deplacement_blocs;
!    ENDWHILE
    
	! Aller en position « home »
	MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;

	Pince\Ouvert;                              ! Ouvrir la pince
ENDPROC
!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC configIO()

    VAR string sigName;   ! Nom du signal en cours de mappage

    ! ==================== SORTIES (DO) ====================
    sigName := "DO01_EE_PINCE01";
    AliasIO sigName, pinceFermer;

    sigName := "DO02_EE_VENT01";
    AliasIO sigName, ventouse1Succion;

    sigName := "DO03_EE_VENT02";
    AliasIO sigName, ventouse2Succion;

    sigName := "DO04_EE_LampBlu";
    AliasIO sigName, lampeBleue;

    sigName := "DO05_EE_LampOr";
    AliasIO sigName, lampeOrange;

    sigName := "DO09_FV0101";
    AliasIO sigName, verinExtension;


    ! ==================== ENTRÉES (DI) ====================
    sigName := "DI01_EE_VAC01";
    AliasIO sigName, objetSurVentouse1;

    sigName := "DI02_EE_VAC02";
    AliasIO sigName, objetSurVentouse2;

    sigName := "DI09_ZS0101";
    AliasIO sigName, blocPresentPoussoir;

    sigName := "DI10_ZS0102";
    AliasIO sigName, blocOriente;

    sigName := "DI11_LSL0101";
    AliasIO sigName, blocBasNiveau;

    sigName := "DI12_LSH0101";
    AliasIO sigName, blocHautNiveau;

    sigName := "DI13_ZS0103";
    AliasIO sigName, verinRetracte;

    sigName := "DI14_ZS0104";
    AliasIO sigName, verinEtendu;
    
    sigName := "DI_Virtuel1_Bouton1";  
    AliasIO sigName, boutonSoudureDI;

    RETURN;                       ! Tout s'est bien passé ? on sort
ERROR
    TPWrite "AliasIO échoué pour : " + sigName;
    TPWrite "ERRNO = " \Num:=ERRNO;
    Exit;                         ! Arrêt du programme pour investigation
ENDPROC
!*************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC init()
	! Volume limitant les articulations du robot
	VAR shapedata joint_space;  
    VAR jointtarget low_pos;
    VAR jointtarget high_pos;
	
	! Conserver seulement le jeu adapté à votre robot
	! Ces limites logicielles garantissent la sécurité
	
	! *** Robot 1
	! low_pos:=[[ 54, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	! high_pos:=[[ 128, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    
    IF RobOS() THEN
        ! *** Robot 2
    	low_pos:=[[ -134, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    	high_pos:=[[ -65, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    ELSE
        ! Code for Virtual Controller
        TPWrite "Utilisation de la cellule virtuelle dans Robostudio";
        ! *** Robot 3
    	low_pos:=[[ 44, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    	high_pos:=[[ 118, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    ENDIF
	
	! *** Robot 4
	! low_pos:=[[ -104, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	! high_pos:=[[ -28, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];

	! Activation de la limitation (obligatoire)
	WZFree EspaceRestreint;
	WZLimJointDef \Outside, joint_space, low_pos, high_pos;
	WZLimSup \Temp, EspaceRestreint, joint_space;

    ! Limitation de vitesse pour la sécurité
	VelSet 25,1000;
    ! Rétracter le vérin
	SetDO verinExtension, Retracte; ! ? alias au lieu de DO09_FV0101
	! Ouvrir la pince
    Pince\Ouvert;

ENDPROC
!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
!**************************************************************************************
PROC verPositionAxes()
    ! Déclaration des variables
    VAR jointtarget posActuelle;          ! Position articulaire actuelle du robot
    VAR jointtarget posReference;         ! Position articulaire de référence (calculée à partir de rRetrait)
    VAR num delta;                        ! Écart angulaire entre la position actuelle et la référence
    VAR bool erreurDetectee := FALSE;     ! Indicateur s'il y a une erreur sur au moins une articulation
    VAR string msgErreur := "Écart articulaire : ";  ! Message à afficher en cas d'erreur
    VAR num i;
    VAR string tmp;
    VAR num seuil;

    ! Obtenir la position actuelle des articulations du robot
    posActuelle := CJointT();

    ! Calculer la position articulaire cible à partir du robtarget rRetrait
    posReference := CalcJointT(rRetrait, tPince_bloc\wobj:=wobj0);

    ! Comparaison articulation par articulation (de J1 à J6)
    FOR i FROM 1 TO 6 DO
        IF i = 1 THEN
            delta := fun_abs_val(posActuelle.robax.rax_1 - posReference.robax.rax_1);
        ELSEIF i = 2 THEN
            delta := fun_abs_val(posActuelle.robax.rax_2 - posReference.robax.rax_2);
        ELSEIF i = 3 THEN
            delta := fun_abs_val(posActuelle.robax.rax_3 - posReference.robax.rax_3);
        ELSEIF i = 4 THEN
            delta := fun_abs_val(posActuelle.robax.rax_4 - posReference.robax.rax_4);
        ELSEIF i = 5 THEN
            delta := fun_abs_val(posActuelle.robax.rax_5 - posReference.robax.rax_5);
        ELSEIF i = 6 THEN
            delta := fun_abs_val(posActuelle.robax.rax_6 - posReference.robax.rax_6);
        ENDIF

        ! calcul du seuil selon l'articulation
        IF i <= 3 THEN
            seuil := 1;
        ELSE
            seuil := 5;
        ENDIF
        
        ! si dépassement du seuil, on construit le message et on signale l'erreur
        IF delta > seuil THEN
            tmp := "J" + numToStr(i,0) + " (" + numToStr(delta,1);
            tmp := tmp + "°) ";
            msgErreur := msgErreur + tmp;
            erreurDetectee := TRUE;
        ENDIF
    ENDFOR

    ! Si une ou plusieurs erreurs ont été détectées :
    IF erreurDetectee THEN
        TPWrite msgErreur;                              ! Afficher les articulations fautives avec écarts
        TPWrite "Position initiale invalide. Programme arrêté.";
        ! Exit;                                           ! Arrêter l'exécution du programme
    ELSE
        TPWrite "Position initiale validée.";           ! Aucun problème détecté
    ENDIF
ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Prise_Glissoire()

    ! Vérifier la présence d'un bloc dans la glissoire
    IF DInput(blocPresentPoussoir) = 0 THEN
        TPErase;
        TPWrite "Aucun bloc dans la glissoire";
        TPWrite "Placer deux blocs et remettre en mode AUTO";
        WaitDI blocPresentPoussoir, 1;      ! Attendre qu'un bloc soit détecté
        WaitTime 1;                         ! Stabilisation du bloc
    ENDIF

    ! Saisir le bloc
    MoveJ RelTool(rPriseGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
    SetDO verinExtension, Extension;       ! Indexer le bloc pendant le mouvement
    WaitDI verinEtendu, 1;                 ! Attendre l'extension du vérin
    MoveL rPriseGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    Pince\Fermer;                          ! Fermer la pince
    SetDO verinExtension, Retracte;        ! Rétracter le vérin
    WaitDI verinEtendu, 0;                 ! Attendre la rétraction complète
    MoveL RelTool(rPriseGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

PROC Depot_Glissoire()

    ! Saisir le bloc
    MoveJ RelTool(rDepotGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
    MoveL rDepotGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    Pince\Ouvert;                          ! Ouvrir la pince
    MoveL RelTool(rDepotGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
! Procédure : Depot
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Depot(robtarget rPosDepot)

	! 1) Approche rapide offset Z pour éviter les collisions
	MoveJ RelTool(rPosDepot,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;

	! 2) Descente linéaire à vitesse réduite pour un positionnement précis
	MoveL rPosDepot, LowSpeed, fine, tPince_bloc;

	! 3) Ouvrir la pince pour relâcher le bloc
	Pince\Ouvert;

	! 4) Remontée linéaire vers la position d'approche (offset Z)
	MoveL RelTool(rPosDepot,0,0,Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

PROC Prise_en_Depot(robtarget rPrisDepot)

	! 1) Approche rapide offset Z pour éviter les collisions
	MoveJ RelTool(rPrisDepot,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;

	! 2) Descente linéaire à vitesse réduite pour un positionnement précis
	MoveL rPrisDepot, LowSpeed, fine, tPince_bloc;

	! 3) Ouvrir la pince pour relâcher le bloc
	Pince\Fermer;

	! 4) Remontée linéaire vers la position d'approche (offset Z)
	MoveL RelTool(rPrisDepot,0,0,Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Pince(\switch Ouvert | switch Fermer)
    ! Action sur la pince
    IF Present(Fermer) THEN
        ! Vérifier si la sortie doit être modifiée
        IF DOutput(pinceFermer) = 0 THEN
            Set pinceFermer;              ! Fermer la pince
            WaitTime 1;                   ! Attendre le mouvement physique
        ENDIF
    ELSE
        ! Vérifier si la sortie doit être modifiée
        IF DOutput(pinceFermer) = 1 THEN
            Reset pinceFermer;            ! Ouvrir la pince
            WaitTime 1;                   ! Attendre le mouvement physique
        ENDIF
    ENDIF
ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Deplacement_blocs()
    VAR num bloc;
    VAR num b := 2;
    VAR num distance_rDepot;
    VAR num distance_rPrise;
    
    FOR bloc FROM 0 TO (b - 1) DO
        distance_rDepot := -((EpaisMM) * bloc);
        ! Calcul de la nouvelle position rDepot_new par rapport à rDepot et au bloc qui correspond
        rDepot_new := RelTool(rDepot, 0, 0, distance_rDepot \Rz := 0);
        ! Prendre le bloc au Glissoire
        Prise_Glissoire;
    	! Dépôt du bloc dans rDepot_new
    	Depot(rDepot_new);
        IF soudureDemandee = TRUE THEN
            SimulerSoudure;
        ENDIF
    ENDFOR
    
    ! Retour à la position de retrait
    MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 3;
    IF soudureDemandee = TRUE THEN
        SimulerSoudure;
    ENDIF
    
    FOR bloc FROM (b - 1) TO 0 STEP -1 DO
        distance_rPrise := -((EpaisMM) * bloc);
        ! Calcul de la nouvelle position rDepot_new par rapport à rDepot et au bloc qui correspond
        rDepot_new := RelTool(rDepot, 0, 0, distance_rPrise \Rz := 0);
        ! Pris le Bloc le plus a haut
        Prise_en_Depot(rDepot_new);
        ! Depot le bloc au haut du Glissoire
        Depot_Glissoire;
        IF soudureDemandee = TRUE THEN
            SimulerSoudure;
        ENDIF
    ENDFOR
    
    ! Retour à la position de retrait
    MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 3;
    IF soudureDemandee = TRUE THEN
        SimulerSoudure;
    ENDIF
    
ENDPROC

!**************************************************************************************
!**************************************************************************************
!**************************************************************************************

 FUNC num fun_abs_val(num x)
     IF x >= 0 THEN
         RETURN x;
     ELSE
         RETURN -x;
     ENDIF
 ENDFUNC

!**************************************************************************************
!**************************************************************************************
!**************************************************************************************
ENDMODULE
